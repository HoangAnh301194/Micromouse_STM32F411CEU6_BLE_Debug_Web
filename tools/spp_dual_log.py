"""SPP line classification and auto-saved, per-test continuous CSV only."""
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
    "ENC": ("timestamp", "ENC_L", "ENC_R"),
    "IR": ("valid", "L0", "L45", "L90", "R0", "R45", "R90"),
    "MPU": ("valid", "yaw_cd", "gz_cd"),
}
TEST_FILE_SUFFIX = {"ENC": "encoder", "IR": "ir", "MPU": "mpu"}
_DECIMAL = re.compile(r"[+-]?\d+\Z")


def sensor_test_csv(line, host_time=None):
    """Return (sensor, header, row) for one complete sensor-only CSV record.

    Encoder frames are either 'ENC,<left>,<right>' or legacy
    'ENC,ms=...,left=...,right=...,delta_l=...'. Both produce the same
    minimal timestamp,ENC_L,ENC_R table. The timestamp is host receive
    time in ISO 8601 local time; it is not MCU millis or a synchronization
    guarantee. No speed/IR/MPU fields leak into an encoder CSV row.
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
    if sensor == "ENC":
        values = (host_time or datetime.now().isoformat(timespec="milliseconds"),) + values
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
    """Automatic DATA-ONLY logger with one directory per System Test run.

    Kept under the historical class name for dashboard compatibility. COM
    connects, SPP handshake replies, mode events and status diagnostics NEVER
    create files. The first valid sample of an activated test creates:

        logs/spp_runs/Mode 7 - Encoder Test - YYYYMMDD_HHMMSS_ffffff_COM16/
            continuous_data.csv

    A STOP/START pause preserves one CSV. Selecting any test again creates a
    distinct run. The right-hand status terminal remains visible but is not
    saved. Older flat logs are not modified.
    """

    SENSOR_TEST_NAMES = {"IR": "IR Test", "ENC": "Encoder Test", "MPU": "MPU Test"}
    TEST_NAME_TO_SENSOR = {name: key for key, name in SENSOR_TEST_NAMES.items()}
    _SELECT_RE = re.compile(r"^Selected test [13]: (IR Test|MPU Test)$")
    _ENC_LOG_SELECT = "Selected encoder option 1: Log Receive"
    _MODE_RE = re.compile(r"^Selected mode ([1-7]):")

    def __init__(self, root):
        self.root = Path(root)
        self.files = {}
        self.paths = {}
        self.runs = []
        self.port = ""
        self.active_sensor = None
        self.active_folder = None
        self._active_file = None
        self._has_menu_context = False
        self._streaming = True

    def open_session(self, port):
        """Reset on connect WITHOUT creating any empty file/folder."""
        self.close()
        self.port = re.sub(r"[^A-Za-z0-9_-]", "_", port)[:40] or "COM"
        self.paths = {}
        self.runs = []

    def _close_active(self):
        if self._active_file:
            self._active_file.close()
        self._active_file = None
        self.files = {}
        self.active_folder = None

    def _deactivate(self):
        self._close_active()
        self.active_sensor = None
        self._streaming = False

    def _observe_status(self, line):
        """Status is used ONLY as in-memory routing metadata, never logged."""
        if line == self._ENC_LOG_SELECT:
            # The Encoder Test parent itself never enables measurements.
            self._has_menu_context = True
            self._close_active()
            self.active_sensor = "ENC"
            self._streaming = True
            return

        if (line.startswith("Encoder Test") or
                line.startswith("Selected encoder option ") or
                line == "Selected test 2: Encoder Test"):
            self._has_menu_context = True
            self._deactivate()
            return

        selected = self._SELECT_RE.fullmatch(line)
        if selected:
            self._has_menu_context = True
            self._close_active()
            self.active_sensor = self.TEST_NAME_TO_SENSOR[selected.group(1)]
            self._streaming = True
            return

        if line.startswith("System Test ") or line.startswith("System Test:"):
            # Browsing to another submenu item or Back invalidates the stream.
            self._has_menu_context = True
            self._deactivate()
            return

        mode = self._MODE_RE.match(line)
        if mode:
            self._has_menu_context = True
            self._deactivate()
            return

        if line.startswith("TEST STREAM STOPPED: "):
            if line.partition(": ")[2] == self.SENSOR_TEST_NAMES.get(self.active_sensor):
                self._streaming = False
        elif line.startswith("TEST STREAM STARTED: "):
            if line.partition(": ")[2] == self.SENSOR_TEST_NAMES.get(self.active_sensor):
                self._streaming = True

    def _open_run(self, sensor, headers, host_time):
        """Create a directory only on the FIRST valid sample of a test run."""
        try:
            first_sample = datetime.fromisoformat(host_time) if host_time else datetime.now()
        except ValueError:
            first_sample = datetime.now()
        stamp = first_sample.strftime("%Y%m%d_%H%M%S_%f")
        test = self.SENSOR_TEST_NAMES[sensor]
        if sensor == "ENC":
            test += " - Log Receive"
        label = f"Mode 7 - {test} - {stamp}_{self.port or 'COM'}"
        for attempt in range(100):
            name = label if attempt == 0 else f"{label}_{attempt:02d}"
            folder = self.root / name
            try:
                folder.mkdir(parents=True, exist_ok=False)
                break
            except FileExistsError:
                continue
        else:
            raise FileExistsError("Cannot allocate unique test log directory")
        path = folder / "continuous_data.csv"
        handle = None
        try:
            handle = path.open("x", encoding="utf-8", newline="", buffering=1)
            handle.write(",".join(headers) + "\n")
        except OSError:
            # Roll back a partial first-write without obscuring the failure.
            if handle is not None:
                handle.close()
            path.unlink(missing_ok=True)
            folder.rmdir()
            raise
        self._active_file = handle
        self.active_folder = folder
        self.files = {sensor: handle}
        self.paths[sensor] = path
        self.runs.append(path)

    def write(self, line, category=None, host_time=None):
        """Only valid continuous sensor measurements are persisted."""
        stream = category or classify_spp_line(line)
        if stream == "status":
            self._observe_status(line.strip())
            return None
        if stream != "data":
            return None

        record = sensor_test_csv(line, host_time=host_time)
        if record is None:
            # Reject non-System-Test telemetry instead of producing mixed CSV.
            return None
        sensor, headers, values = record
        if self.active_sensor is None:
            if self._has_menu_context or sensor == "ENC":
                # Encoder log is STRICTLY opt-in via Log Receive even after
                # reconnect; never infer it from an unsolicited ENC frame.
                return None
            # Preserve historical IR/MPU reconnect recovery if no menu
            # events have been received on this COM session.
            self.active_sensor = sensor
            self._streaming = True
        if sensor != self.active_sensor or not self._streaming:
            return None

        if self._active_file is None:
            self._open_run(sensor, headers, host_time)
        self._active_file.write(",".join(values) + "\n")
        return self.paths[sensor]

    def close(self):
        self._close_active()
        self.active_sensor = None
        self._has_menu_context = False
        self._streaming = True
        self.port = ""
