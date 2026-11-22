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


CSV_COLUMNS = (
    "host_time", "type", "ms", "valid", "age_us", "yaw_cd", "gz_cd",
    "L0", "L45", "L90", "R0", "R45", "R90",
    "left", "right", "pps_l", "pps_r",
    "dt_ms", "delta_l", "delta_r", "window_l", "window_r", "scan", "raw",
)

def data_to_csv_row(line, host_time=None):
    """Normalize telemetry to fixed columns; preserve unrecognized data in raw."""
    row = {col: "" for col in CSV_COLUMNS}
    row["host_time"] = host_time or datetime.now().isoformat(timespec="milliseconds")
    parts = line.strip().split(",")
    row["type"] = parts[0] if parts else ""
    row["raw"] = line.strip()
    for part in parts[1:]:
        key, sep, value = part.partition("=")
        if sep and key in row and key not in ("type", "raw", "host_time"):
            row[key] = value
    if row["type"] == "ENC_CSV" and len(parts) >= 6:
        for key, value in zip(("ms", "left", "right", "pps_l", "pps_r"), parts[1:6]):
            if "=" not in value:
                row[key] = value
    return row

def csv_row_text(row):
    import csv
    import io
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="")
    writer.writerow([row.get(key, "") for key in CSV_COLUMNS])
    return output.getvalue()

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
            paths = {"status": root / f"{stem}_status.log",
                     "data": root / f"{stem}_data.csv"}
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
            self.files["data"].write(",".join(CSV_COLUMNS) + "\n")
            return
        raise FileExistsError("Cannot create unique dual SPP log files")

    def write(self, line, category=None):
        stream = category or classify_spp_line(line)
        if stream == "data" and "data" in self.files:
            self.files["data"].write(csv_row_text(data_to_csv_row(line)) + "\n")
        elif stream == "status" and "status" in self.files:
            self.files["status"].write(
                f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n")

    def close(self):
        for file in self.files.values():
            file.close()
        self.files = {}
