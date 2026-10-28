"""Conservative Windows Bluetooth COM classification from pyserial hardware IDs.

This is a heuristic from the BTHENUM instance-id address segment, not an
authoritative Windows Bluetooth connection-status API. COM Port Direction and
Name in Windows Bluetooth settings remain the authoritative user check.
"""
from __future__ import annotations

import re


_ADDRESS = re.compile(r"&0&([0-9A-F]{12})_[0-9A-F]+$", flags=re.IGNORECASE)


def classify_port(hwid: str) -> tuple[str, str | None]:
    """Return (incoming/outgoing/other/unknown, remote BD_ADDR if nonzero)."""
    if "BTHENUM" not in hwid.upper():
        return "other", None
    found = _ADDRESS.search(hwid)
    if found is None:
        return "unknown", None
    addr = found.group(1).upper()
    return ("incoming", None) if addr == "000000000000" else ("outgoing", addr)
