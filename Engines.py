"""
Engines.py
----------
Classe per il controllo dei motori e del servo di un robot RoboCup Jr Rescue Line.
Legge tutta la configurazione (pin, mappatura, inversioni, velocità, tempi) da motors.cfg.

Uso base:
    from Engines import Engines
    import time

    with Engines() as eng:
        eng.forward()          # avanti a velocità base
        time.sleep(2)
        eng.turn_left()        # svolta a sinistra sul posto
        time.sleep(0.8)
        eng.stop()

Uso avanzato (PID line following):
    eng.curve(speed=30, steering=-25)   # curva a sinistra con sterzo PID
"""

import RPi.GPIO as GPIO
import time
from CFGReader import CFGReader


class Engines:
    """
    Controlla 4 motori DC (2x driver TB6612) + 1 servo.
    Tutti i parametri vengono letti da motors.cfg tramite CFGReader.

    Velocità: float -100..+100
        +100 = massima velocità avanti
        -100 = massima velocità indietro
           0 = stop
    """

    FRONT_LEFT  = "front_left"
    FRONT_RIGHT = "front_right"
    REAR_LEFT   = "rear_left"
    REAR_RIGHT  = "rear_right"

    def __init__(self, config_file: str = "motors.cfg"):
        self._cfg = CFGReader(config_file)
        if not self._cfg:
            raise FileNotFoundError(f"File di configurazione '{config_file}' non trovato!")

        GPIO.setmode(GPIO.BCM)
        self._setup_channels()
        self._load_params()
        self._setup_gpio()

    # ══════════════════════════════════════════════════════════════════════════
    #  Setup interno
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_channels(self):
        """Legge i pin dei 4 canali motore dal config."""
        cfg = self._cfg
        self._channels = {
            "driver1_a": {
                "in1": cfg.read("driver1", "ain1"),
                "in2": cfg.read("driver1", "ain2"),
                "pwm_pin": cfg.read("driver1", "pwma"),
            },
            "driver1_b": {
                "in1": cfg.read("driver1", "bin1"),
                "in2": cfg.read("driver1", "bin2"),
                "pwm_pin": cfg.read("driver1", "pwmb"),
            },
            "driver2_a": {
                "in1": cfg.read("driver2", "ain1"),
                "in2": cfg.read("driver2", "ain2"),
                "pwm_pin": cfg.read("driver2", "pwma"),
            },
            "driver2_b": {
                "in1": cfg.read("driver2", "bin1"),
                "in2": cfg.read("driver2", "bin2"),
                "pwm_pin": cfg.read("driver2", "pwmb"),
            },
        }

        # Mappa posizione fisica → (canale, inversione)
        self._wheel: dict[str, tuple[dict, int]] = {}
        for drv_key, ch in self._channels.items():
            position  = cfg.read("motor_mapping",  drv_key)
            inversion = cfg.read("motor_inversion", drv_key)
            self._wheel[position] = (ch, inversion)

    def _load_params(self):
        """Carica velocità, tempi e parametri servo dal config."""
        cfg = self._cfg

        # ── Velocità ──────────────────────────────────────────────────────────
        self.base_speed           = cfg.read("speeds", "base_speed")
        self.max_speed            = cfg.read("speeds", "max_speed")
        self.min_speed            = cfg.read("speeds", "min_speed")
        self.turn_speed           = cfg.read("speeds", "turn_speed")
        self.min_turn_speed       = cfg.read("speeds", "min_turn_speed")
        self.curve_speed          = cfg.read("speeds", "curve_speed")
        self.search_turn_speed    = cfg.read("speeds", "search_turn_speed")
        self.right_angle_turn_speed = cfg.read("speeds", "right_angle_turn_speed")

        # ── Tempi ─────────────────────────────────────────────────────────────
        self.right_angle_turn_time = cfg.read("times", "right_angle_turn_time")
        self.search_time           = cfg.read("times", "search_time")
        self.servo_settle_time     = cfg.read("times", "servo_settle_time")
        self.maneuver_pause        = cfg.read("times", "maneuver_pause")

        # ── Servo ─────────────────────────────────────────────────────────────
        self._servo_pin    = cfg.read("servo", "pin")
        self._servo_freq   = cfg.read("servo", "freq")
        self._servo_min    = cfg.read("servo", "angle_min")
        self._servo_center = cfg.read("servo", "angle_center")
        self._servo_max    = cfg.read("servo", "angle_max")
        self._servo_angle  = self._servo_center

        # ── PWM ───────────────────────────────────────────────────────────────
        self._pwm_freq = cfg.read("pwm", "frequency")

    def _setup_gpio(self):
        """Inizializza tutti i pin GPIO e avvia i PWM."""
        for ch in self._channels.values():
            GPIO.setup(ch["in1"],     GPIO.OUT)
            GPIO.setup(ch["in2"],     GPIO.OUT)
            GPIO.setup(ch["pwm_pin"], GPIO.OUT)
            GPIO.output(ch["in1"],    GPIO.LOW)
            GPIO.output(ch["in2"],    GPIO.LOW)
            ch["pwm"] = GPIO.PWM(ch["pwm_pin"], self._pwm_freq)
            ch["pwm"].start(0)

        GPIO.setup(self._servo_pin, GPIO.OUT)
        self._servo_pwm = GPIO.PWM(self._servo_pin, self._servo_freq)
        self._servo_pwm.start(0)

    # ══════════════════════════════════════════════════════════════════════════
    #  Metodi interni motori
    # ══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _clamp(value: float, lo: float = -100.0, hi: float = 100.0) -> float:
        return max(lo, min(hi, value))

    def _set_wheel(self, position: str, speed: float):
        """
        Imposta la velocità di una singola ruota applicando l'inversione.
        speed: -100..+100
        """
        ch, inversion = self._wheel[position]
        speed = self._clamp(speed * inversion)

        if speed > 0:
            GPIO.output(ch["in1"], GPIO.HIGH)
            GPIO.output(ch["in2"], GPIO.LOW)
        elif speed < 0:
            GPIO.output(ch["in1"], GPIO.LOW)
            GPIO.output(ch["in2"], GPIO.HIGH)
        else:
            GPIO.output(ch["in1"], GPIO.LOW)
            GPIO.output(ch["in2"], GPIO.LOW)

        ch["pwm"].ChangeDutyCycle(abs(speed))

    # ══════════════════════════════════════════════════════════════════════════
    #  API pubblica — controllo diretto
    # ══════════════════════════════════════════════════════════════════════════

    def set_motors(self, fl: float, fr: float, rl: float, rr: float):
        """
        Imposta le 4 ruote individualmente.
        fl=front_left, fr=front_right, rl=rear_left, rr=rear_right
        Valori: -100..+100
        """
        self._set_wheel(self.FRONT_LEFT,  fl)
        self._set_wheel(self.FRONT_RIGHT, fr)
        self._set_wheel(self.REAR_LEFT,   rl)
        self._set_wheel(self.REAR_RIGHT,  rr)

    def set_speed(self, left: float, right: float):
        """
        Imposta la velocità del lato sinistro e destro (differential drive).
        Comodo per il PID: set_speed(base + correction, base - correction)
        """
        self.set_motors(left, right, left, right)

    # ══════════════════════════════════════════════════════════════════════════
    #  API pubblica — movimenti base (usano velocità dal config)
    # ══════════════════════════════════════════════════════════════════════════

    def forward(self, speed: float = None, duration: float = None):
        """
        Avanti.
        speed:    velocità (default: base_speed dal config)
        duration: se specificato, si muove per N secondi poi si ferma
        """
        speed = self._clamp(speed if speed is not None else self.base_speed, 0, 100)
        self.set_motors(speed, speed, speed, speed)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def backward(self, speed: float = None, duration: float = None):
        """
        Indietro.
        speed:    velocità (default: base_speed dal config)
        duration: se specificato, si muove per N secondi poi si ferma
        """
        speed = self._clamp(speed if speed is not None else self.base_speed, 0, 100)
        self.set_motors(-speed, -speed, -speed, -speed)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def stop(self):
        """Ferma tutti i motori (coast — ruote libere)."""
        self.set_motors(0, 0, 0, 0)

    def brake(self, duration: float = None):
        """
        Freno attivo: blocca le ruote elettricamente.
        duration: se specificato, frena per N secondi poi rilascia
        """
        for position in self._wheel:
            ch, _ = self._wheel[position]
            GPIO.output(ch["in1"], GPIO.HIGH)
            GPIO.output(ch["in2"], GPIO.HIGH)
            ch["pwm"].ChangeDutyCycle(100)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def turn_left(self, speed: float = None, duration: float = None):
        """
        Rotazione sul posto a sinistra (ruote destre avanti, sinistre indietro).
        speed:    velocità (default: turn_speed dal config)
        duration: se specificato, gira per N secondi poi si ferma
                  (default: right_angle_turn_time dal config)
        """
        speed = self._clamp(speed if speed is not None else self.turn_speed, 0, 100)
        self.set_motors(-speed, speed, -speed, speed)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def turn_right(self, speed: float = None, duration: float = None):
        """
        Rotazione sul posto a destra (ruote sinistre avanti, destre indietro).
        speed:    velocità (default: turn_speed dal config)
        duration: se specificato, gira per N secondi poi si ferma
        """
        speed = self._clamp(speed if speed is not None else self.turn_speed, 0, 100)
        self.set_motors(speed, -speed, speed, -speed)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    # ══════════════════════════════════════════════════════════════════════════
    #  API pubblica — manovre avanzate
    # ══════════════════════════════════════════════════════════════════════════

    def curve(self, speed: float = None, steering: float = 0):
        """
        Movimento con curvatura — usato principalmente dal PID.

        speed:    velocità base (default: base_speed). Può essere negativa (indietro).
        steering: valore di sterzo (-100..+100)
                  negativo → curva a sinistra
                  positivo → curva a destra

        La ruota esterna accelera, quella interna rallenta/inverte.
        La velocità di ogni lato viene tenuta entro [min_turn_speed, max_speed].
        """
        speed    = speed if speed is not None else self.base_speed
        speed    = self._clamp(speed)
        steering = self._clamp(steering)

        left_speed  = self._clamp(speed + steering, self.min_turn_speed, self.max_speed)
        right_speed = self._clamp(speed - steering, self.min_turn_speed, self.max_speed)

        self.set_speed(left_speed, right_speed)

    def right_angle_left(self, speed: float = None, duration: float = None):
        """
        Manovra angolo retto a sinistra (90°).
        Usa right_angle_turn_speed e right_angle_turn_time dal config se non specificati.
        """
        speed    = speed    if speed    is not None else self.right_angle_turn_speed
        duration = duration if duration is not None else self.right_angle_turn_time
        self.turn_left(speed, duration)

    def right_angle_right(self, speed: float = None, duration: float = None):
        """
        Manovra angolo retto a destra (90°).
        Usa right_angle_turn_speed e right_angle_turn_time dal config se non specificati.
        """
        speed    = speed    if speed    is not None else self.right_angle_turn_speed
        duration = duration if duration is not None else self.right_angle_turn_time
        self.turn_right(speed, duration)

    def search_left(self, duration: float = None):
        """
        Rotazione di ricerca linea verso sinistra.
        duration: (default: search_time dal config)
        """
        duration = duration if duration is not None else self.search_time
        self.turn_left(self.search_turn_speed, duration)

    def search_right(self, duration: float = None):
        """
        Rotazione di ricerca linea verso destra.
        duration: (default: search_time dal config)
        """
        duration = duration if duration is not None else self.search_time
        self.turn_right(self.search_turn_speed, duration)

    def pause(self, duration: float = None):
        """
        Pausa ferma tra manovre.
        duration: (default: maneuver_pause dal config)
        """
        self.stop()
        time.sleep(duration if duration is not None else self.maneuver_pause)

    # ══════════════════════════════════════════════════════════════════════════
    #  API pubblica — servo
    # ══════════════════════════════════════════════════════════════════════════

    def set_servo(self, angle: float, settle_time: float = None):
        """
        Porta il servo all'angolo specificato.
        angle:       0..180 gradi (0=sx, 90=centro, 180=dx)
        settle_time: attesa dopo il movimento (default: servo_settle_time dal config)
        """
        settle_time = settle_time if settle_time is not None else self.servo_settle_time
        angle = self._clamp(angle, self._servo_min, self._servo_max)
        duty  = 2.0 + (angle / 18.0)
        self._servo_pwm.ChangeDutyCycle(duty)
        self._servo_angle = angle
        if settle_time > 0:
            time.sleep(settle_time)
            self._servo_pwm.ChangeDutyCycle(0)  # evita ronzio

    def servo_center(self, settle_time: float = None):
        """Porta il servo al centro."""
        self.set_servo(self._servo_center, settle_time)

    def servo_left(self, degrees_from_center: float = 45, settle_time: float = None):
        """Porta il servo a sinistra di N gradi dal centro."""
        self.set_servo(self._servo_center - degrees_from_center, settle_time)

    def servo_right(self, degrees_from_center: float = 45, settle_time: float = None):
        """Porta il servo a destra di N gradi dal centro."""
        self.set_servo(self._servo_center + degrees_from_center, settle_time)

    @property
    def servo_angle(self) -> float:
        """Angolo attuale del servo (gradi)."""
        return self._servo_angle

    # ══════════════════════════════════════════════════════════════════════════
    #  Informazioni / debug
    # ══════════════════════════════════════════════════════════════════════════

    def print_config(self):
        """Stampa un riepilogo della configurazione attiva."""
        print("╔══════════════════════════════════════════╗")
        print("║         ENGINES — configurazione         ║")
        print("╠══════════════════════════════════════════╣")
        print("║  MAPPATURA RUOTE                         ║")
        for pos, (ch, inv) in self._wheel.items():
            print(f"║   {pos:<16} inversione: {inv:+d}       ║")
        print("╠══════════════════════════════════════════╣")
        print("║  VELOCITÀ                                ║")
        print(f"║   base_speed          = {self.base_speed:<5}            ║")
        print(f"║   max_speed           = {self.max_speed:<5}            ║")
        print(f"║   min_speed           = {self.min_speed:<5}            ║")
        print(f"║   turn_speed          = {self.turn_speed:<5}            ║")
        print(f"║   min_turn_speed      = {self.min_turn_speed:<5}            ║")
        print(f"║   right_angle_speed   = {self.right_angle_turn_speed:<5}            ║")
        print(f"║   search_turn_speed   = {self.search_turn_speed:<5}            ║")
        print("╠══════════════════════════════════════════╣")
        print("║  TEMPI                                   ║")
        print(f"║   right_angle_time    = {self.right_angle_turn_time:<5}s           ║")
        print(f"║   search_time         = {self.search_time:<5}s           ║")
        print(f"║   servo_settle_time   = {self.servo_settle_time:<5}s           ║")
        print(f"║   maneuver_pause      = {self.maneuver_pause:<5}s           ║")
        print("╚══════════════════════════════════════════╝")

    # ══════════════════════════════════════════════════════════════════════════
    #  Cleanup
    # ══════════════════════════════════════════════════════════════════════════

    def cleanup(self):
        """Ferma tutto e libera il GPIO. Da chiamare sempre alla fine."""
        self.stop()
        self._servo_pwm.stop()
        for ch in self._channels.values():
            ch["pwm"].stop()
        GPIO.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        return False
