"""
web/server.py
-------------
Web server Flask per il debug del robot in tempo reale.

Routes:
  GET  /              → dashboard HTML
  GET  /stream        → MJPEG live stream del frame elaborato
  GET  /status        → JSON con stato robot
  GET  /settings      → JSON con tutti i settings correnti
  POST /settings/pid    → aggiorna parametri PID
  POST /settings/vision → aggiorna parametri visione
  POST /settings/speeds → aggiorna velocità motori
  POST /settings/servo_camera → muove il servo camera
  POST /settings/save   → salva tutti i settings correnti su file
  POST /control/start → avvia il robot
  POST /control/stop  → ferma il robot
"""

import time
import threading
from flask import Flask, Response, jsonify, request, render_template
from CFGReader import CFGReader
from utils.logger import get_logger
from utils.cfg_writer import save_settings

log = get_logger("WebServer")


class WebServer:
    """
    Server Flask che espone la dashboard di debug.
    Il RobotController viene passato dall'esterno (dependency injection).
    """

    def __init__(self, controller, config_file: str = "settings.cfg"):
        cfg = CFGReader(config_file)
        if not cfg:
            raise FileNotFoundError(f"'{config_file}' non trovato!")

        self._host        = cfg.read("web", "host")
        self._port        = cfg.read("web", "port")
        self._debug       = cfg.read("web", "debug")
        self._stream_fps  = cfg.read("web", "stream_fps")
        self._controller  = controller
        self._config_file = config_file

        self._app = Flask(__name__, template_folder="templates")
        self._register_routes()

    # ── Routes ────────────────────────────────────────────────────────────────

    def _register_routes(self):
        app = self._app

        @app.route("/")
        def index():
            return render_template("index.html")

        # ── Stream MJPEG ───────────────────────────────────────────────────
        @app.route("/stream")
        def stream():
            return Response(
                self._mjpeg_generator(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        # ── Stato robot ────────────────────────────────────────────────────
        @app.route("/status")
        def status():
            return jsonify(self._controller.state.snapshot())

        # ── Settings GET ───────────────────────────────────────────────────
        @app.route("/settings")
        def settings():
            eng = self._controller.engines
            pid = self._controller.pid
            det = self._controller.detector
            return jsonify({
                "pid": {
                    "kp":           pid.kp,
                    "ki":           pid.ki,
                    "kd":           pid.kd,
                    "dead_zone":    pid.dead_zone,
                    "max_integral": pid.max_integral,
                },
                "speeds": {
                    "base_speed":  eng.base_speed,
                    "max_speed":   eng.max_speed,
                    "min_speed":   eng.min_speed,
                    "turn_speed":  eng.turn_speed,
                },
                "vision": {
                    "threshold":        det._threshold,
                    "blur_k":           det._blur_k,
                    "roi_top":          det._roi_top,
                    "roi_bottom":       det._roi_bottom,
                    "side_margin":      det._side_margin,
                    "continuity_power": det._continuity_power,
                },
                "camera": {
                    "servo_camera_angle": eng.servo_angle,
                },
            })

        # ── Settings POST PID ──────────────────────────────────────────────
        @app.route("/settings/pid", methods=["POST"])
        def settings_pid():
            data = request.get_json(force=True) or {}
            parsed = {}
            for key in ("kp", "ki", "kd", "dead_zone", "max_integral"):
                if key in data:
                    try:
                        parsed[key] = float(data[key])
                    except (ValueError, TypeError):
                        return jsonify({"error": f"Valore non valido per {key}"}), 400
            self._controller.update_pid(**parsed)
            log.info(f"PID aggiornato via web: {parsed}")
            return jsonify({"ok": True, "updated": parsed})

        # ── Settings POST Vision ───────────────────────────────────────────
        @app.route("/settings/vision", methods=["POST"])
        def settings_vision():
            data = request.get_json(force=True) or {}
            parsed = {}
            float_keys = ("roi_top", "roi_bottom", "side_margin", "continuity_power")
            int_keys   = ("threshold", "blur_k")
            for key in float_keys:
                if key in data:
                    try:
                        parsed[key] = float(data[key])
                    except (ValueError, TypeError):
                        return jsonify({"error": f"Valore non valido per {key}"}), 400
            for key in int_keys:
                if key in data:
                    try:
                        parsed[key] = int(data[key])
                    except (ValueError, TypeError):
                        return jsonify({"error": f"Valore non valido per {key}"}), 400
            self._controller.update_vision(**parsed)
            log.info(f"Vision aggiornato via web: {parsed}")
            return jsonify({"ok": True, "updated": parsed})

        # ── Settings POST Speeds ───────────────────────────────────────────
        @app.route("/settings/speeds", methods=["POST"])
        def settings_speeds():
            data = request.get_json(force=True) or {}
            parsed = {}
            for key in ("base_speed", "max_speed", "min_speed", "turn_speed"):
                if key in data:
                    try:
                        parsed[key] = float(data[key])
                    except (ValueError, TypeError):
                        return jsonify({"error": f"Valore non valido per {key}"}), 400
            self._controller.update_speeds(**parsed)
            log.info(f"Speeds aggiornato via web: {parsed}")
            return jsonify({"ok": True, "updated": parsed})

        # ── Servo camera ───────────────────────────────────────────────────
        @app.route("/settings/servo_camera", methods=["POST"])
        def settings_servo_camera():
            data = request.get_json(force=True) or {}
            if "angle" not in data:
                return jsonify({"error": "Campo 'angle' mancante"}), 400
            try:
                angle = float(data["angle"])
            except (ValueError, TypeError):
                return jsonify({"error": "Valore angle non valido"}), 400
            # Muovi il servo senza settle_time (movimento live dallo slider)
            self._controller.engines.set_servo(angle, settle_time=0)
            log.info(f"Servo camera → {angle}°")
            return jsonify({"ok": True, "angle": angle})

        # ── Salva settings su file ─────────────────────────────────────────
        @app.route("/settings/save", methods=["POST"])
        def settings_save():
            eng = self._controller.engines
            pid = self._controller.pid
            det = self._controller.detector
            try:
                save_settings(self._config_file, {
                    "pid": {
                        "kp":           pid.kp,
                        "ki":           pid.ki,
                        "kd":           pid.kd,
                        "dead_zone":    pid.dead_zone,
                        "max_integral": pid.max_integral,
                    },
                    "speeds": {
                        "base_speed":  eng.base_speed,
                        "max_speed":   eng.max_speed,
                        "min_speed":   eng.min_speed,
                        "turn_speed":  eng.turn_speed,
                    },
                    "vision": {
                        "threshold_value":  det._threshold,
                        "blur_kernel":      det._blur_k,
                        "roi_top_ratio":    det._roi_top,
                        "roi_bottom_ratio": det._roi_bottom,
                        "side_margin_ratio":det._side_margin,
                        "continuity_weight_power": det._continuity_power,
                    },
                    "camera": {
                        "servo_camera_angle": eng.servo_angle,
                    },
                })
                log.info("Settings salvati su file")
                return jsonify({"ok": True})
            except Exception as e:
                log.error(f"Errore salvataggio settings: {e}")
                return jsonify({"error": str(e)}), 500

        # ── Controllo robot ────────────────────────────────────────────────
        @app.route("/control/start", methods=["POST"])
        def control_start():
            self._controller.start()
            return jsonify({"ok": True, "mode": "following"})

        @app.route("/control/stop", methods=["POST"])
        def control_stop():
            self._controller.stop()
            return jsonify({"ok": True, "mode": "stopped"})

    # ── MJPEG generator ────────────────────────────────────────────────────────

    def _mjpeg_generator(self):
        """
        Genera il flusso MJPEG dal frame di debug del LineDetector.

        - Chiama request_debug_frame() ad ogni iterazione per segnalare
          che c'è un client attivo (abilita l'encode JPEG nel detector).
        - Invia solo frame nuovi (confronta puntatore oggetto), evitando
          di ritrasmettere lo stesso JPEG più volte.
        - Usa un intervallo minimo per non saturare la rete.
        """
        interval   = 1.0 / self._stream_fps
        det        = self._controller.detector
        last_frame = None

        while True:
            det.request_debug_frame()   # abilita encode JPEG nel detector
            frame = det.get_debug_frame()

            if frame is not None and frame is not last_frame:
                last_frame = frame
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    frame +
                    b"\r\n"
                )

            time.sleep(interval)

    # ── Avvio server ───────────────────────────────────────────────────────────

    def run(self, threaded: bool = True):
        """
        Avvia Flask.
        threaded=True → gira in background (usato da main.py)
        threaded=False → blocca (utile per debug standalone)
        """
        kwargs = dict(host=self._host, port=self._port,
                      debug=False, use_reloader=False)
        if threaded:
            t = threading.Thread(
                target=lambda: self._app.run(**kwargs),
                daemon=True
            )
            t.start()
            log.info(f"Web server su http://{self._host}:{self._port}")
        else:
            log.info(f"Web server su http://{self._host}:{self._port}")
            self._app.run(**kwargs)
