"""Automatic per-connection and per-run Classic SPP log files."""
from __future__ import annotations
from datetime import datetime
from pathlib import Path
import re

_SAFE = re.compile(r"[^A-Za-z0-9_-]+")
START = ("W,START,", "RUN,START", "RUN_START")
END = ("W,END,", "RUN,END", "RUN_END")

class SessionLog:
    """Writes a complete connection trace and individual marked robot runs."""
    def __init__(self, root: Path):
        self.root = Path(root)
        self.connection = None
        self.run = None
        self.connection_path = None
        self.run_path = None
        self.port = ""

    def _file(self, category: str):
        folder = self.root / category
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        port = _SAFE.sub("_", self.port)[:40]
        for i in range(100):
            name = folder / f"{stamp}_{port}_{i:02d}.log"
            try:
                return name.open("x", encoding="utf-8", buffering=1), name
            except FileExistsError:
                continue
        raise FileExistsError("Unable to create unique log")

    @staticmethod
    def _write(file, line):
        if file:
            file.write(f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n")
            file.flush()

    def open_session(self, port: str):
        self.close()
        self.port = port
        self.connection, self.connection_path = self._file("connections")
        self._write(self.connection, "# CONNECTED")

    def add_line(self, line: str):
        if not self.connection:
            return
        if line.startswith(START):
            if self.run:
                self._write(self.run, "# INTERRUPTED BY NEXT START")
                self.run.close()
                self.run = None
            self.run, self.run_path = self._file("runs")
        self._write(self.connection, line)
        self._write(self.run, line)
        if line.startswith(END) and self.run:
            self.run.close()
            self.run = None

    def close(self):
        if self.run:
            self._write(self.run, "# DISCONNECTED")
            self.run.close()
            self.run = None
        if self.connection:
            self._write(self.connection, "# DISCONNECTED")
            self.connection.close()
            self.connection = None
