# CFGReader.py sesso
import os

class CFGReader:
    def __init__(self, file_name: str):
        self.data = {}
        self.loaded = False
        
        if not os.path.exists(file_name):
            return

        with open(file_name, 'r', encoding='utf-8') as file:
            self.loaded = True
            current_section = None
            for line in file:
                line = self._remove_inline_comments(line).strip()
                if not line:
                    continue

                # Section header
                if line.startswith('[') and ']' in line:
                    current_section = line[1:line.find(']')].strip().lower()
                    if current_section not in self.data:
                        self.data[current_section] = {}
                    continue

                # key=value
                if '=' in line and current_section:
                    key, value = line.split('=', 1)
                    self.data[current_section][key.strip().lower()] = value.strip()

    def _remove_inline_comments(self, s: str) -> str:
        for marker in [';', '#', '//']:
            pos = s.find(marker)
            if pos != -1:
                if marker == '//' and not (pos == 0 or s[pos-1].isspace()):
                    continue
                s = s[:pos]
        return s

    def read(self, section: str, key: str):
        val = self.data[section.lower()][key.lower()]
        val_l = val.lower()

        # Booleans
        if val_l in ("true", "1", "on", "yes"): return True
        if val_l in ("false", "0", "off", "no"): return False

        # Numbers
        try:
            if '.' in val:
                return float(val)
            return int(val)
        except ValueError:
            return val  # fallback string

    def __bool__(self):
        return self.loaded