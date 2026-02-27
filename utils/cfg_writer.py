"""
utils/cfg_writer.py
-------------------
Funzione per aggiornare i valori in un file .cfg stile INI
preservando commenti, ordine e struttura originale.
"""


def save_settings(file_path: str, updates: dict):
    """
    Aggiorna i valori nel file .cfg preservando commenti e struttura.

    updates: dict annidato { sezione: { chiave: valore } }
    Esempio:
        save_settings("settings.cfg", {
            "pid": {"kp": 0.8, "ki": 0.005},
            "camera": {"servo_camera_angle": 95},
        })
    """
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # Normalizza le chiavi degli updates in lowercase
    normalized = {
        sec.lower(): {k.lower(): v for k, v in vals.items()}
        for sec, vals in updates.items()
    }

    new_lines = []
    current_section = None

    for line in lines:
        stripped = line.strip()

        # Sezione
        if stripped.startswith("[") and "]" in stripped:
            current_section = stripped[1:stripped.find("]")].strip().lower()
            new_lines.append(line)
            continue

        # Riga vuota o commento → lascia invariata
        if not stripped or stripped[0] in ("#", ";", "/"):
            new_lines.append(line)
            continue

        # Riga key = value
        if "=" in stripped and current_section in normalized:
            key_part = stripped.split("=")[0].strip().lower()
            if key_part in normalized[current_section]:
                val = normalized[current_section][key_part]
                # Formattazione: mantieni indentazione originale
                indent = line[: len(line) - len(line.lstrip())]
                # Numero: togli zeri trailing inutili ma mantieni precisione
                if isinstance(val, float):
                    formatted = f"{val:.6g}"
                else:
                    formatted = str(val)
                new_lines.append(f"{indent}{key_part} = {formatted}\n")
                continue

        new_lines.append(line)

    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_lines)
