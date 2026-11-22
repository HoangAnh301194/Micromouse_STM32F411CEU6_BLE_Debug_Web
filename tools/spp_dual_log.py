"""Status/data classification and automatic per-connection log files."""
from datetime import datetime
from pathlib import Path
import re

DATA_PREFIXES = ("MPU,", "IR,", "ENC,", "ENC_CSV,", "CSV,", "DATA,", "MW,", "WI,", "WS,")
STATUS_PREFIXES = ("STATUS,", "Mode ", "Selected mode ", "SPP CONNECTED", "W,HELLO",
                   "W,MENU", "W,WAIT", "W,FATAL", "W,START", "W,END",
                   "W,REJECT", "CALIB ", "CALIB...", "READY CAL", "TEST ",
                   "TEST,", "STOP ", "START ", "MAPCFG", "MAPROW", "MAPEND")

def classify_spp_line(line):
    line = line.strip()
    if line.startswith(STATUS_PREFIXES):
        return "status"
    if line.startswith(DATA_PREFIXES):
        return "data"
    if line.startswith("W,"):
        fields = line.split(",")
        if len(fields) == 10:
            try:
                [int(v) for v in fields[1:]]
                return "data"
            except ValueError:
                pass
    return "status"

class DualSessionLog:
    def __init__(self, root):
        self.root = Path(root)
        self.files = {}
        self.paths = {}

    def open_session(self, port):
        self.close()
        root = self.root / "connections"
        root.mkdir(parents=True, exist_ok=True)
        now = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe = re.sub(r"[^A-Za-z0-9_-]", "_", port)[:40] or "COM"
        for i in range(100):
            stem = f"{now}_{safe}_{i:02d}"
            paths = {kind: root / f"{stem}_{kind}.log" for kind in ("data", "status")}
            try:
                one = paths["status"].open("x", encoding="utf-8", buffering=1)
            except FileExistsError:
                continue
            try:
                two = paths["data"].open("x", encoding="utf-8", buffering=1)
            except OSError:
                one.close()
                paths["status"].unlink(missing_ok=True)
                raise
            self.files = {"status": one, "data": two}
            self.paths = paths
            return
        raise FileExistsError("Cannot create unique dual SPP log files")

    def write(self, line, category=None):
        stream = category or classify_spp_line(line)
        if stream in self.files:
            self.files[stream].write(
                f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n")

    def close(self):
        for file in self.files.values():
            file.close()
        self.files = {}
