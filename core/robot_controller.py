"""
core/robot_controller.py
------------------------
Loop principale del robot. Coordina:
  - LineDetector  → errore linea
  - PIDController → sterzo
  - Engines       → motori

Gira in un thread separato e condivide lo stato con il web server
tramite l'oggetto RobotState (thread-safe).
"""

import threading
import time
from CFGReader import CFGReader
from Engines import Engines
from core.line_detector import LineDetector
from core.pid_controller import PIDController
from utils.logger import get_logger

log = get_logger("RobotController")


# ── Stato condiviso ────────────────────────────────────────────────────────────

class RobotState:
    """Oggetto thread-safe che contiene lo stato corrente del robot."""

    MODES = ("idle", "following", "searching", "right_angle", "stopped")

    def __init__(self):
        self._lock    = threading.Lock()
        self.mode     = "idle"
        self.error    = 0.0
        self.steering = 0.0
        self.speed    = 0.0
        self.running  = False

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "mode":     self.mode,
                "error":    round(self.error, 1),
                "steering": round(self.steering, 1),
                "speed":    round(self.speed, 1),
                "running":  self.running,
            }


# ── Controller principale ──────────────────────────────────────────────────────

class RobotController:
    """
    Controlla il robot nel loop principale.

    Uso:
        rc = RobotController()
        rc.start()
        ...
        rc.stop()
    """

    def __init__(self,
                 settings_file: str = "settings.cfg",
                 motors_file:   str = "motors.cfg"):

        # Config
        self._scfg = CFGReader(settings_file)
        if not self._scfg:
            raise FileNotFoundError(f"'{settings_file}' non trovato!")

        # Moduli
        self._detector = LineDetector(settings_file)
        self._pid      = PIDController(
            kp           = self._scfg.read("pid", "kp"),
            ki           = self._scfg.read("pid", "ki"),
            kd           = self._scfg.read("pid", "kd"),
            dead_zone    = self._scfg.read("pid", "dead_zone"),
            max_integral = self._scfg.read("pid", "max_integral"),
        )
        self._engines  = Engines(motors_file)
        self._state    = RobotState()

        # Parametri di ricerca e angolo retto (dal motors.cfg via Engines)
        self._search_time    = self._engines.search_time
        self._search_dir     = 1    # +1=destra, -1=sinistra (ultima direzione persa)
        self._search_start   = None
        self._ra_enabled     = self._scfg.read("line", "right_angle_enabled")

        # Thread
        self._running = False
        self._thread  = None

    # ── Proprietà pubbliche ────────────────────────────────────────────────────

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def detector(self) -> LineDetector:
        return self._detector

    @property
    def pid(self) -> PIDController:
        return self._pid

    @property
    def engines(self) -> Engines:
        return self._engines

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        """Avvia camera e loop di controllo."""
        if self._running:
            return
        self._detector.start()
        self._running = True
        self._state.update(running=True, mode="following")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("RobotController avviato")

    def stop(self):
        """Ferma il loop, i motori e la camera."""
        self._running = False
        self._state.update(running=False, mode="stopped")
        if self._thread:
            self._thread.join(timeout=2)
        self._engines.stop()
        self._detector.stop()
        log.info("RobotController fermato")

    def cleanup(self):
        """Stop + cleanup GPIO."""
        self.stop()
        self._engines.cleanup()

    # ── Loop principale ────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            try:
                if self._detector.is_right_angle() and self._ra_enabled:
                    self._handle_right_angle()
                elif self._detector.is_line_lost():
                    self._handle_search()
                else:
                    self._handle_follow()
            except Exception as e:
                log.error(f"Errore nel loop: {e}")
                self._engines.stop()
                time.sleep(0.1)

    def _handle_follow(self):
        """Modalità line following normale con PID."""
        self._search_start = None   # reset timer ricerca
        self._pid.reset() if self._state.mode != "following" else None

        error    = self._detector.get_error()
        steering = self._pid.compute(error)

        # Velocità adattiva: rallenta se la curva è brusca
        speed = self._adaptive_speed(abs(steering))
        self._engines.curve(speed=speed, steering=steering)

        # Memorizza ultima direzione di perdita linea
        if error != 0:
            self._search_dir = 1 if error > 0 else -1

        self._state.update(
            mode="following",
            error=error,
            steering=steering,
            speed=speed,
        )

    def _handle_search(self):
        """Modalità ricerca linea: ruota nell'ultima direzione vista."""
        if self._search_start is None:
            self._search_start = time.monotonic()
            self._pid.reset()
            log.info("Linea persa — avvio ricerca")

        elapsed = time.monotonic() - self._search_start

        if elapsed > self._search_time:
            # Linea non trovata: ferma il robot
            self._engines.stop()
            self._state.update(mode="stopped", speed=0, steering=0)
            log.warning("Linea non trovata dopo search_time — robot fermo")
            time.sleep(0.5)
            return

        speed = self._engines.search_turn_speed
        if self._search_dir > 0:
            self._engines.set_speed(speed, -speed)   # gira a destra
        else:
            self._engines.set_speed(-speed, speed)   # gira a sinistra

        self._state.update(
            mode="searching",
            error=self._detector.get_error(),
            steering=float(self._search_dir * speed),
            speed=0,
        )

    def _handle_right_angle(self):
        """Manovra angolo retto: gira nella direzione della linea."""
        log.info(f"Angolo retto rilevato — svolta {'destra' if self._search_dir > 0 else 'sinistra'}")
        self._engines.stop()
        time.sleep(0.1)

        self._state.update(mode="right_angle", steering=0, speed=0)

        if self._search_dir > 0:
            self._engines.right_angle_right()
        else:
            self._engines.right_angle_left()

        self._pid.reset()

    def _adaptive_speed(self, abs_steering: float) -> float:
        """
        Riduce la velocità proporzionalmente alla curvatura.
        Sterzo alto → velocità bassa (fino a min_speed).
        Sterzo basso → velocità alta (fino a max_speed).
        """
        eng = self._engines
        # Normalizza sterzo 0..100 → fattore 0..1
        factor = min(abs_steering / 100.0, 1.0)
        speed = eng.max_speed - factor * (eng.max_speed - eng.min_speed)
        return max(eng.min_speed, min(eng.max_speed, speed))

    # ── Aggiornamento live settings ───────────────────────────────────────────

    def update_pid(self, **kwargs):
        """Aggiorna parametri PID a runtime."""
        self._pid.update_params(**kwargs)

    def update_vision(self, **kwargs):
        """Aggiorna parametri visione a runtime."""
        self._detector.update_settings(**kwargs)

    def update_speeds(self, **kwargs):
        """Aggiorna velocità motori a runtime."""
        eng = self._engines
        if "base_speed"  in kwargs: eng.base_speed  = kwargs["base_speed"]
        if "max_speed"   in kwargs: eng.max_speed   = kwargs["max_speed"]
        if "min_speed"   in kwargs: eng.min_speed   = kwargs["min_speed"]
        if "turn_speed"  in kwargs: eng.turn_speed  = kwargs["turn_speed"]
        log.info(f"Speed settings aggiornati: {kwargs}")
