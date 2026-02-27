"""
core/pid_controller.py
----------------------
Controllore PID puro. Riceve l'errore (px dal centro) e restituisce
il valore di sterzo da passare a Engines.curve().
"""

import time
from utils.logger import get_logger

log = get_logger("PID")


class PIDController:
    """
    Controllore PID con:
    - zona morta (dead zone)
    - anti-windup sull'integrale
    - clamping dell'output a [-100, +100]
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 dead_zone: float = 0, max_integral: float = 100):
        self.kp          = kp
        self.ki          = ki
        self.kd          = kd
        self.dead_zone   = dead_zone
        self.max_integral = max_integral

        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = None

    # ── Aggiornamento parametri live ──────────────────────────────────────────

    def update_params(self, kp: float = None, ki: float = None,
                      kd: float = None, dead_zone: float = None,
                      max_integral: float = None):
        """Aggiorna i parametri PID a runtime (es. dalla web interface)."""
        if kp          is not None: self.kp          = kp
        if ki          is not None: self.ki          = ki
        if kd          is not None: self.kd          = kd
        if dead_zone   is not None: self.dead_zone   = dead_zone
        if max_integral is not None: self.max_integral = max_integral
        log.info(f"Parametri aggiornati → Kp={self.kp} Ki={self.ki} Kd={self.kd}")

    def reset(self):
        """Azzera integrale e stato interno (es. dopo una perdita di linea)."""
        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = None
        log.debug("PID reset")

    # ── Calcolo ───────────────────────────────────────────────────────────────

    def compute(self, error: float) -> float:
        """
        Calcola l'output PID dato l'errore corrente.

        error:  distanza dal centro in pixel (negativo=sx, positivo=dx)
        return: valore di sterzo -100..+100
                positivo = sterza a destra, negativo = sterza a sinistra
        """
        now = time.monotonic()
        if self._last_time is None:
            dt = 0.033   # primo frame: assume ~30fps
        else:
            dt = now - self._last_time
            dt = max(dt, 0.001)  # evita divisione per zero
        self._last_time = now

        # Zona morta
        if abs(error) < self.dead_zone:
            error = 0.0

        # Proporzionale
        p = self.kp * error

        # Integrale con anti-windup
        self._integral += error * dt
        self._integral = max(-self.max_integral,
                             min(self.max_integral, self._integral))
        i = self.ki * self._integral

        # Derivativo
        derivative = (error - self._last_error) / dt
        d = self.kd * derivative
        self._last_error = error

        output = p + i + d

        # Clamp output
        output = max(-100.0, min(100.0, output))

        log.debug(f"err={error:+.1f} P={p:+.2f} I={i:+.2f} D={d:+.2f} → {output:+.2f}")
        return output

    # ── Proprietà di stato ────────────────────────────────────────────────────

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def last_error(self) -> float:
        return self._last_error
