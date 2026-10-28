"""Safe host-side decoders for JDY-33 Classic SPP (same USART2 BLE frames).

No serial, GUI, firmware, or Bluetooth dependency: can replay saved logs offline.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import re
import zlib


WALL_N, WALL_E, WALL_S, WALL_W = 1, 2, 4, 8
NUMERIC = re.compile(r"([A-Za-z_][A-Za-z_0-9]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))(?=,|\s|$)")


def kv_fields(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in line.split(",")[1:]:
        key, sep, value = item.partition("=")
        if sep:
            result[key.strip()] = value.strip()
    return result


def numeric_fields(line: str) -> dict[str, float]:
    """Extract only explicitly named numeric values; never guess units."""
    return {key: float(value) for key, value in NUMERIC.findall(line)}


class LineFramer:
    """Byte fragmentation/reassembly, including CRLF; bounded input memory."""

    def __init__(self, max_pending: int = 8192):
        self.pending = bytearray()
        self.max_pending = max_pending
        self.discarded = 0

    def feed(self, data: bytes) -> list[str]:
        self.pending.extend(data)
        lines: list[str] = []
        while True:
            cut = self.pending.find(b"\n")
            if cut < 0:
                break
            record = bytes(self.pending[:cut]).rstrip(b"\r")
            del self.pending[:cut + 1]
            if record:
                lines.append(record.decode("utf-8", "replace").strip())
        if len(self.pending) > self.max_pending:
            self.pending.clear()
            self.discarded += 1
        return lines


@dataclass
class MazeState:
    """Web console's map convention: y=0 is bottom, NESW bits 1/2/4/8."""

    n: int = 16
    walls: list[list[int]] = field(default_factory=list)
    visited: set[tuple[int, int]] = field(default_factory=set)
    x: int = 0
    y: int = 0
    direction: int = 0
    source: str = "WAITING"
    status: str = "Waiting for map"
    complete: bool = False
    pending_map: dict | None = None
    mz_chunks: dict[int, str] = field(default_factory=dict)

    def __post_init__(self):
        if not self.walls:
            self.walls = self._boundary(self.n)

    @staticmethod
    def _boundary(n: int) -> list[list[int]]:
        return [[(WALL_N if y == n - 1 else 0)
                 | (WALL_S if y == 0 else 0)
                 | (WALL_E if x == n - 1 else 0)
                 | (WALL_W if x == 0 else 0)
                 for x in range(n)] for y in range(n)]

    def _position(self, x: int, y: int, d: int) -> None:
        if not (0 <= x < self.n and 0 <= y < self.n and 0 <= d < 4):
            raise ValueError("Robot position outside maze")
        self.x, self.y, self.direction = x, y, d

    def _neighbors(self, x: int, y: int, value: int) -> None:
        for dx, dy, bit, opposite in (
            (0, 1, WALL_N, WALL_S), (1, 0, WALL_E, WALL_W),
            (0, -1, WALL_S, WALL_N), (-1, 0, WALL_W, WALL_E),
        ):
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.n and 0 <= ny < self.n:
                self.walls[ny][nx] = (
                    self.walls[ny][nx] | opposite if value & bit
                    else self.walls[ny][nx] & ~opposite
                )

    def _map_error(self, message: str) -> str:
        self.pending_map = None
        self.status = "MAP FAIL: " + message
        return self.status

    def apply(self, line: str) -> str | None:
        """Return status changes; never publish a partial/CRC-invalid MAP1."""
        if line.startswith("MAPCFG,"):
            v = kv_fields(line)
            try:
                n = int(v["w"])
                if (v.get("fmt") != "1" or not 2 <= n <= 16
                    or int(v["h"]) != n or int(v["len"]) != 8 + n * n
                    or not re.fullmatch(r"[0-9a-fA-F]{8}", v["crc"])):
                    raise ValueError("bad header")
                self.pending_map = {"n": n, "crc": int(v["crc"], 16),
                                    "src": v.get("src", "UNKNOWN"), "rows": {}}
                self.status = "MAP receiving 0/" + str(n)
                return self.status
            except (KeyError, ValueError):
                return self._map_error("invalid MAPCFG")

        if line.startswith("MAPROW,"):
            if self.pending_map is None:
                return None
            v, p = kv_fields(line), self.pending_map
            try:
                y = int(v["y"])
                row = v["hex"].upper()
                if (not 0 <= y < p["n"] or len(row) != p["n"]
                    or not re.fullmatch(r"[0-9A-F]+", row)
                    or (y in p["rows"] and p["rows"][y] != row)):
                    raise ValueError("row")
                p["rows"][y] = row
                self.status = "MAP receiving %d/%d" % (len(p["rows"]), p["n"])
                return self.status
            except (KeyError, ValueError):
                return self._map_error("invalid MAPROW")

        if line.startswith("MAPEND,"):
            if self.pending_map is None:
                return None
            p, v = self.pending_map, kv_fields(line)
            try:
                n = p["n"]
                if (int(v["rows"]) != n or int(v["crc"], 16) != p["crc"]
                    or len(p["rows"]) != n):
                    raise ValueError("header/count")
                raw = b"MAP1" + bytes([n, n, 0, 0]) + bytes(
                    int(digit, 16) for y in range(n) for digit in p["rows"][y]
                )
                if zlib.crc32(raw) & 0xFFFFFFFF != p["crc"]:
                    raise ValueError("CRC32")
                # Commit only after complete validation.
                self.n = n
                self.walls = [[int(v, 16) for v in p["rows"][y]]
                              for y in range(n)]
                self.visited.clear()  # wall nibble != visited state
                self.x, self.y, self.direction = 0, 0, 0
                self.source, self.complete = p["src"], True
                self.status = "MAP PASS %dx%d CRC=%08X %s" % (
                    n, n, p["crc"], p["src"])
                self.pending_map = None
                return self.status
            except (ValueError, KeyError):
                return self._map_error("incomplete or CRC mismatch")

        if line.startswith("CELL:"):
            try:
                parts = line[5:].split(",")
                x, y, d = int(parts[0]), int(parts[1]), int(parts[3])
                w = int(parts[2], 16)
                if not 0 <= w <= 15:
                    raise ValueError("wall nibble")
                self._position(x, y, d)
                self.walls[y][x] = w
                self._neighbors(x, y, w)
                self.visited.add((x, y))
                self.source, self.complete = "CELL", True
                self.status = "CELL (%d,%d) %s" % (x, y, "NESW"[d])
                return self.status
            except (ValueError, IndexError):
                return "CELL ignored: invalid fields"

        if line.startswith("MAZE:"):
            parts = line.split(":")
            try:
                pos = [int(x) for x in parts[1].split(",")]
                wh = parts[2]
                if len(wh) != 256 or not re.fullmatch("[0-9a-fA-F]{256}", wh):
                    raise ValueError("walls")
                self.n = 16
                self._position(*pos[:3])
                self.walls = [[int(wh[y * 16 + x], 16) for x in range(16)]
                              for y in range(16)]
                self.visited = {(x, y) for y in range(16) for x in range(16)
                                if self.walls[y][x] != 0}
                self.source, self.complete = "MAZE", True
                self.status = "MAZE 16x16"
                return self.status
            except (ValueError, IndexError, TypeError):
                return "MAZE ignored: invalid fields"

        if re.match(r"^MZ[0-7]:", line):
            idx = int(line[2])
            payload = line[4:].strip()
            if idx == 0:
                self.mz_chunks.clear()
                try:
                    pos, walls = payload.split(":", 1)
                    xyz = [int(x) for x in pos.split(",")]
                    self.n = 16
                    self._position(*xyz[:3])
                    payload = walls
                except (ValueError, IndexError, TypeError):
                    return "MZ0 ignored: invalid header"
            self.mz_chunks[idx] = payload
            if all(i in self.mz_chunks for i in range(4)):
                wh = "".join(self.mz_chunks[i] for i in range(4))
                if len(wh) != 256 or not re.fullmatch("[0-9a-fA-F]{256}", wh):
                    self.mz_chunks.clear()
                    return "MZ ignored: invalid walls"
                self.walls = [[int(wh[y * 16 + x], 16) for x in range(16)]
                              for y in range(16)]
                self.visited = {(x, y) for y in range(16) for x in range(16)
                                if self.walls[y][x] != 0}
                self.source, self.complete = "MZ", True
                self.status = "MZ walls 16x16"
                self.mz_chunks.clear()
                return self.status
        return None
