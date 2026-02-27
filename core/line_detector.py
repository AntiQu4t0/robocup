"""
core/line_detector.py
---------------------
Acquisisce frame dalla Pi Camera AI e calcola l'errore della linea nera
rispetto al centro del frame. Produce anche un frame di debug annotato
per lo stream web.

Linea: nera su sfondo bianco.
Errore: pixel dal centro (negativo=linea a sinistra, positivo=linea a destra).

Architettura latenza-minima:
- Il loop di controllo gira alla massima velocità (nessun sleep).
- Il frame di debug viene encodato in JPEG solo se c'è un client web
  connesso (_debug_requested), riducendo il carico CPU quando non serve.
- La rotazione fisica è gestita direttamente da Picamera2 (transform)
  invece di OpenCV, eliminando una copia in memoria per frame.
"""

import cv2
import numpy as np
import threading
import time
from picamera2 import Picamera2
from libcamera import Transform
from CFGReader import CFGReader
from utils.logger import get_logger

log = get_logger("LineDetector")


class LineDetector:
    """
    Acquisisce frame dalla Pi Camera AI e rileva la linea nera.

    Metodi principali:
        start() / stop()          — avvia/ferma la camera
        get_error()               — errore corrente in pixel
        get_debug_frame()         — JPEG annotato per il web server
        is_line_lost()            — True se la linea non è visibile
        is_right_angle()          — True se rilevato angolo retto
        request_debug_frame()     — segnala che c'è un client web attivo
    """

    def __init__(self, config_file: str = "settings.cfg"):
        cfg = CFGReader(config_file)
        if not cfg:
            raise FileNotFoundError(f"File '{config_file}' non trovato!")

        # Camera
        self._width    = cfg.read("camera", "width")
        self._height   = cfg.read("camera", "height")
        self._fps      = cfg.read("camera", "fps")
        self._rotation = cfg.read("camera", "rotation")

        # Vision
        self._threshold      = cfg.read("vision", "threshold_value")
        self._blur_k         = cfg.read("vision", "blur_kernel")
        self._roi_top        = cfg.read("vision", "roi_top_ratio")
        self._roi_bottom     = cfg.read("vision", "roi_bottom_ratio")
        self._side_margin    = cfg.read("vision", "side_margin_ratio")
        self._min_brightness = cfg.read("vision", "track_min_brightness")

        # Line detection
        self._continuity_power = cfg.read("line", "continuity_weight_power")
        self._max_memory       = cfg.read("line", "max_frames_memory")
        self._ra_threshold     = cfg.read("line", "right_angle_error_threshold")
        self._ra_enabled       = cfg.read("line", "right_angle_enabled")

        # Stato interno
        self._error        = 0.0
        self._line_lost    = True
        self._right_angle  = False
        self._debug_frame  = None
        self._prev_centers = []
        self._lock         = threading.Lock()
        self._running      = False
        self._thread       = None
        self._camera       = None

        # Flag: encode JPEG solo se qualcuno lo sta guardando
        # Viene aggiornato dal generatore MJPEG ogni volta che serve un frame
        self._debug_requested  = False
        self._debug_request_ts = 0.0   # timestamp dell'ultima richiesta
        self._DEBUG_TIMEOUT    = 3.0   # secondi: se nessuno chiede, smette di encodare

        # Centro orizzontale del frame
        self._cx = self._width // 2

    # ── Camera lifecycle ───────────────────────────────────────────────────────

    def start(self):
        """Avvia la camera e il thread di acquisizione."""
        self._camera = Picamera2()

        # Gestisci la rotazione direttamente in hardware tramite Transform
        # Molto più veloce della rotazione OpenCV (nessuna copia aggiuntiva)
        transform = Transform()
        if self._rotation == 180:
            transform = Transform(hflip=True, vflip=True)
        elif self._rotation == 90:
            transform = Transform(rotation=90)
        elif self._rotation == 270:
            transform = Transform(rotation=270)

        config = self._camera.create_preview_configuration(
            main={"size": (self._width, self._height), "format": "RGB888"},
            transform=transform,
            controls={"FrameRate": self._fps},
            buffer_count=2,   # solo 2 buffer: meno latenza, meno memoria
        )
        self._camera.configure(config)
        self._camera.start()
        time.sleep(0.3)  # warm-up camera

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="LineDetector")
        self._thread.start()
        log.info(f"Camera avviata {self._width}x{self._height} @{self._fps}fps rot={self._rotation}°")

    def stop(self):
        """Ferma il thread e la camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._camera:
            self._camera.stop()
            self._camera.close()
        log.info("Camera fermata")

    # ── Loop acquisizione ─────────────────────────────────────────────────────

    def _loop(self):
        """
        Loop principale: gira alla massima velocità senza sleep.
        capture_array() con wait=True blocca fino al prossimo frame disponibile,
        garantendo la latenza minima senza busy-wait.
        """
        while self._running:
            try:
                frame = self._camera.capture_array()
                self._process(frame)
            except Exception as e:
                log.warning(f"Errore acquisizione frame: {e}")
                time.sleep(0.05)

    # ── Elaborazione frame ────────────────────────────────────────────────────

    def _process(self, frame: np.ndarray):
        h, w = frame.shape[:2]

        # ROI verticale
        # La camera è montata capovolta (rotation=180, gestita da Picamera2).
        # roi_top_ratio=0.55 → prendi dal 55% in giù = zona vicina al suolo.
        roi_top    = int(h * self._roi_top)
        roi_bottom = int(h * self._roi_bottom)
        roi = frame[roi_top:roi_bottom, :]

        # Margini laterali
        margin = int(w * self._side_margin)

        # Grayscale diretta sulla ROI (evita conversione sull'intero frame)
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)

        # Blur solo se blur_k > 1 (a bassa risoluzione spesso non serve)
        if self._blur_k > 1:
            k = self._blur_k | 1  # forza dispari
            gray = cv2.GaussianBlur(gray, (k, k), 0)

        _, binary = cv2.threshold(gray, self._threshold, 255, cv2.THRESH_BINARY_INV)

        # Maschera margini laterali
        if margin > 0:
            binary[:, :margin]   = 0
            binary[:, w-margin:] = 0

        # Trova contorni
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error       = 0.0
        line_lost   = True
        right_angle = False
        cx_line     = self._cx
        cy_line     = roi_top + (roi_bottom - roi_top) // 2

        if contours:
            valid = [c for c in contours if cv2.contourArea(c) > 50]
            if valid:
                best = self._best_contour(valid)
                M = cv2.moments(best)
                if M["m00"] > 0:
                    cx_line = int(M["m10"] / M["m00"])
                    cy_line = int(M["m01"] / M["m00"]) + roi_top
                    # Con rotation=180 l'asse X è specchiato:
                    # linea a destra nel frame → fisicamente a sinistra → errore negativo
                    raw_error = float(cx_line - self._cx)
                    error     = -raw_error if self._rotation == 180 else raw_error
                    line_lost = False

                    self._prev_centers.append(cx_line)
                    if len(self._prev_centers) > self._max_memory:
                        self._prev_centers.pop(0)

                    if self._ra_enabled and len(self._prev_centers) >= 3:
                        recent = [abs(c - self._cx) for c in self._prev_centers[-3:]]
                        if all(e > self._ra_threshold for e in recent):
                            right_angle = True

        if line_lost:
            self._prev_centers.clear()

        # ── Debug frame: encode JPEG solo se richiesto ────────────────────
        jpeg_bytes = None
        now = time.monotonic()
        debug_active = (now - self._debug_request_ts) < self._DEBUG_TIMEOUT

        if debug_active:
            debug = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Linee di riferimento
            cv2.line(debug, (margin, roi_top),   (margin, roi_bottom),   (255, 100, 0), 1)
            cv2.line(debug, (w-margin, roi_top), (w-margin, roi_bottom), (255, 100, 0), 1)
            cv2.line(debug, (0, roi_top),        (w, roi_top),           (0, 200, 255), 1)
            cv2.line(debug, (self._cx, 0),       (self._cx, h),          (0, 255, 0),   1)

            if not line_lost:
                cv2.drawContours(debug, [best], -1, (0, 0, 255), 1, offset=(0, roi_top))
                cv2.circle(debug, (cx_line, cy_line), 3, (0, 0, 255), -1)
                cv2.line(debug, (self._cx, cy_line), (cx_line, cy_line), (0, 255, 255), 1)

            mode_color = (0, 0, 255) if line_lost else (0, 255, 0)
            mode_text  = "LOST" if line_lost else f"e={error:+.0f}"
            if right_angle:
                mode_text  = "ANGLE"
                mode_color = (0, 165, 255)
            cv2.putText(debug, mode_text, (2, 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, mode_color, 1)

            # JPEG qualità 60: dimensione ridotta → meno dati in rete
            _, jpeg = cv2.imencode(".jpg", debug, [cv2.IMWRITE_JPEG_QUALITY, 60])
            jpeg_bytes = jpeg.tobytes()

        with self._lock:
            self._error       = error
            self._line_lost   = line_lost
            self._right_angle = right_angle
            if jpeg_bytes is not None:
                self._debug_frame = jpeg_bytes

    def _best_contour(self, contours: list):
        """
        Sceglie il contorno più rilevante usando:
        - area del contorno
        - vicinanza al centro precedente (continuità)
        """
        if not self._prev_centers:
            # Primo frame: prendi il più grande
            return max(contours, key=cv2.contourArea)

        last_cx = self._prev_centers[-1]
        best = None
        best_score = -1

        for c in contours:
            area = cv2.contourArea(c)
            if area < 100:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            dist = abs(cx - last_cx) + 1  # +1 evita div/0
            # Score: area pesata dalla vicinanza al centro precedente
            score = area / (dist ** self._continuity_power)
            if score > best_score:
                best_score = score
                best = c

        return best if best is not None else max(contours, key=cv2.contourArea)

    # ── API pubblica ──────────────────────────────────────────────────────────

    def get_error(self) -> float:
        """Errore corrente in pixel (negativo=linea a sx, positivo=linea a dx)."""
        with self._lock:
            return self._error

    def request_debug_frame(self):
        """
        Segnala che c'è un client web che vuole il frame di debug.
        Va chiamato dal generatore MJPEG ad ogni iterazione.
        Se non viene chiamato per DEBUG_TIMEOUT secondi, l'encode JPEG
        viene disabilitato automaticamente per risparmiare CPU.
        """
        self._debug_request_ts = time.monotonic()

    def get_debug_frame(self) -> bytes | None:
        """Frame JPEG annotato per lo stream web. None se non ancora disponibile."""
        with self._lock:
            return self._debug_frame

    def is_line_lost(self) -> bool:
        """True se la linea non è visibile nel frame corrente."""
        with self._lock:
            return self._line_lost

    def is_right_angle(self) -> bool:
        """True se è stato rilevato un angolo retto."""
        with self._lock:
            return self._right_angle

    def update_settings(self, **kwargs):
        """
        Aggiorna i parametri di visione a runtime (dalla web interface).
        Chiavi valide: threshold, blur_k, roi_top, roi_bottom,
                       side_margin, continuity_power
        """
        with self._lock:
            if "threshold"         in kwargs: self._threshold      = kwargs["threshold"]
            if "blur_k"            in kwargs: self._blur_k         = kwargs["blur_k"]
            if "roi_top"           in kwargs: self._roi_top        = kwargs["roi_top"]
            if "roi_bottom"        in kwargs: self._roi_bottom     = kwargs["roi_bottom"]
            if "side_margin"       in kwargs: self._side_margin    = kwargs["side_margin"]
            if "continuity_power"  in kwargs: self._continuity_power = kwargs["continuity_power"]
        log.info(f"Vision settings aggiornati: {kwargs}")
