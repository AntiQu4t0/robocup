"""
main.py
-------
Entry point del robot RoboCup Jr Rescue Line.

Avvia:
  1. RobotController  → camera + PID + motori (thread separato)
  2. WebServer        → dashboard di debug su http://<ip>:5000

Uso:
    python3 main.py              # avvia tutto e parte subito
    python3 main.py --no-robot   # solo web server (debug senza hardware)
    python3 main.py --no-web     # solo robot, senza web server

Ctrl+C per fermare tutto.
"""

import sys
import time
import signal
import argparse
from core.robot_controller import RobotController
from web.server import WebServer
from utils.logger import get_logger

log = get_logger("Main")


def parse_args():
    p = argparse.ArgumentParser(description="RoboCup Jr Rescue Line — Robot Controller")
    p.add_argument("--no-robot", action="store_true", help="Non avvia il controller robot (solo web)")
    p.add_argument("--no-web",   action="store_true", help="Non avvia il web server (solo robot)")
    p.add_argument("--settings", default="settings.cfg", help="File settings (default: settings.cfg)")
    p.add_argument("--motors",   default="motors.cfg",   help="File motori (default: motors.cfg)")
    return p.parse_args()


def main():
    args = parse_args()

    log.info("=" * 50)
    log.info("  RoboCup Jr Rescue Line — Avvio")
    log.info("=" * 50)

    controller = None
    web        = None

    try:
        # ── Inizializza controller ─────────────────────────────────────────
        if not args.no_robot:
            log.info("Inizializzazione RobotController...")
            controller = RobotController(
                settings_file=args.settings,
                motors_file=args.motors,
            )
        else:
            log.info("--no-robot: controller robot disabilitato")

        # ── Inizializza web server ─────────────────────────────────────────
        if not args.no_web:
            if controller is None:
                log.error("--no-robot richiede anche --no-web oppure un controller mock.")
                sys.exit(1)
            log.info("Inizializzazione WebServer...")
            web = WebServer(controller, config_file=args.settings)
            web.run(threaded=True)
        else:
            log.info("--no-web: web server disabilitato")

        # ── Avvia il robot ─────────────────────────────────────────────────
        if controller is not None:
            log.info("Avvio loop robot...")
            controller.start()

        log.info("Sistema attivo. Premi Ctrl+C per fermare.")

        # ── Gestione SIGTERM (es. systemd) ─────────────────────────────────
        def _sigterm(sig, frame):
            log.info("SIGTERM ricevuto — arresto in corso...")
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, _sigterm)

        # ── Loop principale (mantiene il processo vivo) ────────────────────
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        log.info("\nCtrl+C — arresto in corso...")

    except FileNotFoundError as e:
        log.error(f"File di configurazione mancante: {e}")
        sys.exit(1)

    except Exception as e:
        log.error(f"Errore imprevisto: {e}", exc_info=True)

    finally:
        log.info("Pulizia risorse...")
        if controller is not None:
            try:
                controller.cleanup()
            except Exception as e:
                log.warning(f"Errore durante cleanup: {e}")
        log.info("Robot spento. Arrivederci!")


if __name__ == "__main__":
    main()
