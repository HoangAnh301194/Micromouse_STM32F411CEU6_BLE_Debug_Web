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


# Each System Test has its own CSV schema. ENC is exactly two counts;
# sensor metadata (valid) is retained only for IR/MPU diagnostics.
TEST_CSV_COLUMNS = {
    "ENC": ("ENC_L", "ENC_R"),
    "IR": ("valid", "L0", "L45", "L90", "R0", "R45", "R90"),
    "MPU": ("valid", "yaw_cd", "gz_cd"),
}
TEST_FILE_SUFFIX = {"ENC": "encoder", "IR": "ir", "MPU": "mpu"}
_DECIMAL = re.compile(r"[+-]?\d+\Z")


def sensor_test_csv(line):
    """Return (sensor, header, row) for a complete System Test measurement.

    On-wire encoder framing is 'ENC,<left>,<right>'. The leading tag is
    protocol routing, not a measured column. Historical key=value frames
    can be replayed without bringing other sensors into their CSV files.
    """
    parts = line.strip().split(",")
    if not parts or parts[0] not in TEST_CSV_COLUMNS:
        return None
    sensor = parts[0]
    fields = {}
    for item in parts[1:]:
        key, sep, value = item.partition("=")
        if sep and key:
            fields[key] = value

    if sensor == "ENC":
        if (len(parts) == 3 and
                all(_DECIMAL.fullmatch(value) for value in parts[1:])):
            values = tuple(parts[1:])
        elif all(_DECIMAL.fullmatch(fields.get(key, "")) for key in ("left", "right")):
            values = (fields["left"], fields["right"])
        else:
            return None
    elif sensor == "IR":
        keys = ("valid", "L0", "L45", "L90", "R0", "R45", "R90")
        if not all(_DECIMAL.fullmatch(fields.get(key, "")) for key in keys):
            return None
        values = tuple(fields[key] for key in keys)
    else:
        valid = fields.get("valid")
        if valid not in ("0", "1"):
            return None
        if valid == "0":
            values = ("0", "", "")
        else:
            if not all(_DECIMAL.fullmatch(fields.get(k, "")) for k in ("yaw_cd", "gz_cd")):
                return None
            values = ("1", fields["yaw_cd"], fields["gz_cd"])
    return sensor, TEST_CSV_COLUMNS[sensor], values


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
    if (row["type"] == "ENC" and len(parts) == 3 and
            all(_DECIMAL.fullmatch(value) for value in parts[1:])):
        row["left"], row["right"] = parts[1:]
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
        self._session_stem = None

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
            self._session_stem = root / stem
            self.files["data"].write(",".join(CSV_COLUMNS) + "\n")
            return
        raise FileExistsError("Cannot create unique dual SPP log files")

    def write(self, line, category=None):
        stream = category or classify_spp_line(line)
        if stream == "data" and "data" in self.files:
            sensor_record = sensor_test_csv(line)
            if sensor_record is not None:
                sensor, columns, values = sensor_record
                suffix = TEST_FILE_SUFFIX[sensor]
                if suffix not in self.files:
                    path = Path(str(self._session_stem) + "_" + suffix + ".csv")
                    handle = path.open("x", encoding="utf-8", buffering=1)
                    self.files[suffix] = handle
                    self.paths[suffix] = path
                    handle.write(",".join(columns) + "\n")
                self.files[suffix].write(",".join(values) + "\n")
            else:
                # Legacy mixed telemetry remains compatible with old data.csv.
                self.files["data"].write(csv_row_text(data_to_csv_row(line)) + "\n")
        elif stream == "status" and "status" in self.files:
            self.files["status"].write(
                f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n")

    def close(self):
        for file in self.files.values():
            file.close()
        self.files = {}
        self._session_stem = None
