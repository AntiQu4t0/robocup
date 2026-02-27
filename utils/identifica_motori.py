"""
identifica_motori.py
--------------------
Script interattivo per identificare la posizione fisica di ogni motore sul robot.
Esegui questo script sul Raspberry Pi con il robot sollevato da terra.

Per ogni motore il programma:
  1. Lo fa girare AVANTI per 1.5 secondi
  2. Lo ferma
  3. Lo fa girare INDIETRO per 1.5 secondi
  4. Lo ferma
  5. Chiede all'utente di annotare quale ruota ha girato e in che verso

Al termine stampa un riepilogo da usare per compilare motors.cfg
"""

import RPi.GPIO as GPIO
import time
from CFGReader import CFGReader

# ── Carica configurazione ──────────────────────────────────────────────────────
cfg = CFGReader("motors.cfg")
if not cfg:
    raise FileNotFoundError("File motors.cfg non trovato!")

GPIO.setmode(GPIO.BCM)

# ── Pin driver 1 ───────────────────────────────────────────────────────────────
AIN1_1 = cfg.read("driver1", "ain1")
AIN2_1 = cfg.read("driver1", "ain2")
PWMA_1 = cfg.read("driver1", "pwma")

BIN1_1 = cfg.read("driver1", "bin1")
BIN2_1 = cfg.read("driver1", "bin2")
PWMB_1 = cfg.read("driver1", "pwmb")

# ── Pin driver 2 ───────────────────────────────────────────────────────────────
AIN1_2 = cfg.read("driver2", "ain1")
AIN2_2 = cfg.read("driver2", "ain2")
PWMA_2 = cfg.read("driver2", "pwma")

BIN1_2 = cfg.read("driver2", "bin1")
BIN2_2 = cfg.read("driver2", "bin2")
PWMB_2 = cfg.read("driver2", "pwmb")

PWM_FREQ = cfg.read("pwm", "frequency")

# ── Setup GPIO ─────────────────────────────────────────────────────────────────
all_pins = [
    AIN1_1, AIN2_1, PWMA_1,
    BIN1_1, BIN2_1, PWMB_1,
    AIN1_2, AIN2_2, PWMA_2,
    BIN1_2, BIN2_2, PWMB_2,
]
for pin in all_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

pwmA1 = GPIO.PWM(PWMA_1, PWM_FREQ)
pwmB1 = GPIO.PWM(PWMB_1, PWM_FREQ)
pwmA2 = GPIO.PWM(PWMA_2, PWM_FREQ)
pwmB2 = GPIO.PWM(PWMB_2, PWM_FREQ)

for p in [pwmA1, pwmB1, pwmA2, pwmB2]:
    p.start(0)

# ── Funzioni motore ────────────────────────────────────────────────────────────
TEST_DUTY  = 60   # % duty cycle durante il test (abbassa se il robot si muove troppo)
TEST_TIME  = 1.5  # secondi per ogni direzione

def _run(in1, in2, pwm, forward: bool):
    GPIO.output(in1, GPIO.HIGH if forward else GPIO.LOW)
    GPIO.output(in2, GPIO.LOW  if forward else GPIO.HIGH)
    pwm.ChangeDutyCycle(TEST_DUTY)

def _stop(in1, in2, pwm):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def test_motor(label: str, in1: int, in2: int, pwm) -> dict:
    """Testa un motore avanti/indietro e chiede all'utente di identificarlo."""
    print(f"\n{'='*55}")
    print(f"  MOTORE: {label}")
    print(f"{'='*55}")
    input("  Premi INVIO per far girare il motore AVANTI...")

    _run(in1, in2, pwm, forward=True)
    time.sleep(TEST_TIME)
    _stop(in1, in2, pwm)
    time.sleep(0.3)

    input("  Premi INVIO per far girare il motore INDIETRO...")
    _run(in1, in2, pwm, forward=False)
    time.sleep(TEST_TIME)
    _stop(in1, in2, pwm)
    time.sleep(0.3)

    print()
    print("  Quale ruota hai visto girare?")
    print("    1) Anteriore Destra  (front_right)")
    print("    2) Anteriore Sinistra(front_left)")
    print("    3) Posteriore Destra (rear_right)")
    print("    4) Posteriore Sinistra(rear_left)")
    scelta_pos = {
        "1": "front_right",
        "2": "front_left",
        "3": "rear_right",
        "4": "rear_left",
    }
    while True:
        pos = input("  Scelta (1-4): ").strip()
        if pos in scelta_pos:
            position = scelta_pos[pos]
            break
        print("  Inserisci un numero tra 1 e 4.")

    print()
    print("  Quando il programma ha detto AVANTI, la ruota girava:")
    print("    1) In avanti  → nessuna inversione (inversion = 1)")
    print("    2) All'indietro → motore invertito  (inversion = -1)")
    while True:
        inv = input("  Scelta (1/2): ").strip()
        if inv == "1":
            inversion = 1
            break
        elif inv == "2":
            inversion = -1
            break
        print("  Inserisci 1 o 2.")

    print(f"\n  ✓ {label} → posizione: {position}, inversion: {inversion}")
    return {"label": label, "position": position, "inversion": inversion}

# ── Elenco motori da testare ───────────────────────────────────────────────────
motors = [
    ("Driver1 - Canale A", AIN1_1, AIN2_1, pwmA1, "driver1_a"),
    ("Driver1 - Canale B", BIN1_1, BIN2_1, pwmB1, "driver1_b"),
    ("Driver2 - Canale A", AIN1_2, AIN2_2, pwmA2, "driver2_a"),
    ("Driver2 - Canale B", BIN1_2, BIN2_2, pwmB2, "driver2_b"),
]

# ── Main ───────────────────────────────────────────────────────────────────────
results = []

print("\n╔═══════════════════════════════════════════════════════╗")
print("║        IDENTIFICAZIONE MOTORI - RoboCup Jr Rescue    ║")
print("╚═══════════════════════════════════════════════════════╝")
print()
print("  Solleva il robot da terra prima di procedere!")
print("  Il robot NON deve toccare il suolo durante il test.")
print()
input("  Premi INVIO quando sei pronto...")

try:
    for label, in1, in2, pwm, cfg_key in motors:
        result = test_motor(label, in1, in2, pwm)
        result["cfg_key"] = cfg_key
        results.append(result)

    # ── Riepilogo finale ───────────────────────────────────────────────────────
    print("\n")
    print("╔═══════════════════════════════════════════════════════╗")
    print("║                    RIEPILOGO FINALE                  ║")
    print("╚═══════════════════════════════════════════════════════╝")
    print()
    print("  Copia questi valori in motors.cfg nella sezione")
    print("  [motor_mapping] e [motor_inversion]:\n")
    print("  [motor_mapping]")
    for r in results:
        print(f"  {r['cfg_key']} = {r['position']}")
    print()
    print("  [motor_inversion]")
    for r in results:
        print(f"  {r['cfg_key']} = {r['inversion']}")
    print()

    # Salva automaticamente su file
    save = input("  Vuoi aggiornare automaticamente motors.cfg? (s/n): ").strip().lower()
    if save == "s":
        _update_cfg(results)
        print("\n  ✓ motors.cfg aggiornato!")
    else:
        print("\n  Aggiorna motors.cfg manualmente con i valori sopra.")

except KeyboardInterrupt:
    print("\n\nInterrotto dall'utente.")

finally:
    for p in [pwmA1, pwmB1, pwmA2, pwmB2]:
        p.stop()
    GPIO.cleanup()
    print("GPIO liberato. Arrivederci!")


def _update_cfg(results: list):
    """Riscrive le sezioni motor_mapping e motor_inversion in motors.cfg."""
    with open("motors.cfg", "r", encoding="utf-8") as f:
        lines = f.readlines()

    mapping  = {r["cfg_key"]: r["position"]  for r in results}
    inversion = {r["cfg_key"]: r["inversion"] for r in results}

    new_lines = []
    section = None
    for line in lines:
        stripped = line.strip().lower()
        if stripped.startswith("[") and "]" in stripped:
            section = stripped[1:stripped.find("]")]
            new_lines.append(line)
            continue

        if section == "motor_mapping":
            if "=" in line:
                key = line.split("=")[0].strip().lower()
                if key in mapping:
                    new_lines.append(f"{key} = {mapping[key]}\n")
                    continue
        elif section == "motor_inversion":
            if "=" in line:
                key = line.split("=")[0].strip().lower()
                if key in inversion:
                    new_lines.append(f"{key} = {inversion[key]}\n")
                    continue

        new_lines.append(line)

    with open("motors.cfg", "w", encoding="utf-8") as f:
        f.writelines(new_lines)
