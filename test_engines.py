"""
test_engines.py
---------------
Script interattivo per testare la classe Engines sul robot fisico.
Esegui con il robot SOLLEVATO da terra per i test motori,
oppure a terra per i test di movimento completo.

Uso:
    python3 test_engines.py
"""

import time
from Engines import Engines


def separator(title: str = ""):
    print()
    if title:
        print(f"  ── {title} {'─' * (42 - len(title))}")
    else:
        print(f"  {'─' * 48}")


def wait(msg: str = "Premi INVIO per continuare..."):
    input(f"\n  [{msg}]")


def test_config(eng: Engines):
    """Mostra la configurazione caricata."""
    separator("CONFIGURAZIONE")
    eng.print_config()
    wait()


def test_singoli(eng: Engines):
    """Testa ogni ruota individualmente per verificare inversioni."""
    separator("TEST RUOTE SINGOLE")
    print("  Il robot deve essere SOLLEVATO da terra.")
    print("  Ogni ruota girerà avanti poi indietro per 1 secondo.")
    wait("Premi INVIO per iniziare")

    wheels = [
        ("Anteriore Sinistra (front_left)",  ( 50,  0,  0,  0)),
        ("Anteriore Destra  (front_right)",  (  0, 50,  0,  0)),
        ("Posteriore Sinistra (rear_left)",  (  0,  0, 50,  0)),
        ("Posteriore Destra  (rear_right)",  (  0,  0,  0, 50)),
    ]

    for name, (fl, fr, rl, rr) in wheels:
        print(f"\n  → {name}")
        print("    AVANTI...")
        eng.set_motors(fl, fr, rl, rr)
        time.sleep(1)
        eng.stop()
        time.sleep(0.3)

        print("    INDIETRO...")
        eng.set_motors(-fl, -fr, -rl, -rr)
        time.sleep(1)
        eng.stop()
        time.sleep(0.3)

        risposta = input("    La ruota girava nel verso giusto? (s/n): ").strip().lower()
        if risposta != "s":
            print("    ⚠  Segna l'inversione su motors.cfg per questa ruota!")

    print("\n  ✓ Test ruote singole completato.")
    wait()


def test_movimenti(eng: Engines):
    """Testa i movimenti base: avanti, indietro, rotazioni."""
    separator("TEST MOVIMENTI BASE")
    print("  Metti il robot a TERRA per questo test.")
    wait("Premi INVIO per iniziare")

    moves = [
        ("Avanti (base_speed, 1.5s)",   lambda: eng.forward(duration=1.5)),
        ("Indietro (base_speed, 1.5s)",  lambda: eng.backward(duration=1.5)),
        ("Svolta sinistra sul posto",     lambda: eng.turn_left(duration=eng.right_angle_turn_time)),
        ("Svolta destra sul posto",       lambda: eng.turn_right(duration=eng.right_angle_turn_time)),
        ("Angolo retto sinistra",         lambda: eng.right_angle_left()),
        ("Angolo retto destra",           lambda: eng.right_angle_right()),
        ("Ricerca linea sinistra",        lambda: eng.search_left(duration=1.0)),
        ("Ricerca linea destra",          lambda: eng.search_right(duration=1.0)),
    ]

    for name, action in moves:
        wait(f"INVIO → {name}")
        action()
        eng.pause()
        print(f"    ✓ {name} eseguito.")

    print("\n  ✓ Test movimenti base completato.")
    wait()


def test_curve(eng: Engines):
    """Testa la funzione curve() usata dal PID."""
    separator("TEST CURVE (PID-style)")
    print("  Metti il robot a TERRA per questo test.")
    wait("Premi INVIO per iniziare")

    curves = [
        ("Rettilineo  (steering=0)",        0),
        ("Curva lieve sx (steering=-15)",  -15),
        ("Curva media sx (steering=-35)",  -35),
        ("Curva forte sx (steering=-60)",  -60),
        ("Curva lieve dx (steering=+15)",   15),
        ("Curva media dx (steering=+35)",   35),
        ("Curva forte dx (steering=+60)",   60),
    ]

    for name, steering in curves:
        wait(f"INVIO → {name}")
        eng.curve(speed=eng.base_speed, steering=steering)
        time.sleep(1.5)
        eng.stop()
        eng.pause()
        print(f"    ✓ {name} eseguito.")

    print("\n  ✓ Test curve completato.")
    wait()


def test_velocita(eng: Engines):
    """Testa diverse velocità in avanti."""
    separator("TEST VELOCITÀ")
    print("  Metti il robot a TERRA per questo test.")
    wait("Premi INVIO per iniziare")

    for speed in [eng.min_speed, eng.base_speed, eng.max_speed]:
        wait(f"INVIO → Avanti a speed={speed} per 2s")
        eng.forward(speed=speed, duration=2.0)
        eng.pause()
        print(f"    ✓ speed={speed} eseguita.")

    print("\n  ✓ Test velocità completato.")
    wait()


def test_freno(eng: Engines):
    """Testa stop() vs brake()."""
    separator("TEST STOP vs BRAKE")
    wait("INVIO → Avanti 1s poi STOP (coast)")
    eng.forward(duration=1.0)
    eng.stop()
    time.sleep(1.0)

    wait("INVIO → Avanti 1s poi BRAKE (frenata attiva)")
    eng.forward(duration=1.0)
    eng.brake(duration=0.5)
    time.sleep(0.5)

    print("  ✓ Differenza coast vs brake verificata.")
    wait()


def test_servo(eng: Engines):
    """Testa il servo in varie posizioni."""
    separator("TEST SERVO")
    wait("Premi INVIO per iniziare")

    servos = [
        ("Centro (90°)",        lambda: eng.servo_center()),
        ("Sinistra piena (0°)", lambda: eng.set_servo(0)),
        ("Centro (90°)",        lambda: eng.servo_center()),
        ("Destra piena (180°)", lambda: eng.set_servo(180)),
        ("Centro (90°)",        lambda: eng.servo_center()),
        ("Sinistra 30° dal centro", lambda: eng.servo_left(30)),
        ("Destra 30° dal centro",   lambda: eng.servo_right(30)),
        ("Centro (90°)",        lambda: eng.servo_center()),
    ]

    for name, action in servos:
        print(f"  → {name}")
        action()
        print(f"    ✓ angolo attuale: {eng.servo_angle}°")

    print("\n  ✓ Test servo completato.")
    wait()


def menu(eng: Engines):
    tests = {
        "1": ("Configurazione caricata",        lambda: test_config(eng)),
        "2": ("Ruote singole (robot sollevato)", lambda: test_singoli(eng)),
        "3": ("Movimenti base",                  lambda: test_movimenti(eng)),
        "4": ("Curve / PID steering",            lambda: test_curve(eng)),
        "5": ("Velocità (min / base / max)",     lambda: test_velocita(eng)),
        "6": ("Stop vs Brake",                   lambda: test_freno(eng)),
        "7": ("Servo",                           lambda: test_servo(eng)),
        "0": ("Esci",                            None),
    }

    while True:
        print()
        print("╔══════════════════════════════════════════════╗")
        print("║      TEST ENGINES — RoboCup Jr Rescue        ║")
        print("╠══════════════════════════════════════════════╣")
        for key, (name, _) in tests.items():
            print(f"║   {key})  {name:<40}║")
        print("╚══════════════════════════════════════════════╝")

        scelta = input("  Scelta: ").strip()
        if scelta not in tests:
            print("  Scelta non valida.")
            continue
        name, action = tests[scelta]
        if action is None:
            print("  Arrivederci!")
            break
        try:
            action()
        except KeyboardInterrupt:
            print("\n  Test interrotto (Ctrl+C).")
            eng.stop()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n  Inizializzazione Engines...")
    try:
        with Engines() as eng:
            menu(eng)
    except FileNotFoundError as e:
        print(f"\n  ERRORE: {e}")
    except KeyboardInterrupt:
        print("\n  Interrotto.")
