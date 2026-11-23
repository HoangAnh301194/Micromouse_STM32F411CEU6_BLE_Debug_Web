"""Safe, deterministic SPP firmware handshake retry policy.

Opening a Windows outgoing COM does not itself establish that the STM32 can
receive bytes or that the flashed firmware understands menu commands.
The *only* accepted confirmation is the exact line 'SPP CONNECTED'.

This module has no Bluetooth, motor, or Tkinter dependencies.
"""

from __future__ import annotations


class SPPHandshake:
    """Host sends only '?' while waiting for the menu firmware acknowledgement."""

    fast_interval_s = 0.8
    slow_interval_s = 3.0
    fast_attempts = 6

    def __init__(self):
        self.attempts = 0
        self.last_probe_at: float | None = None
        self.confirmed = False

    def should_probe(self, now: float) -> bool:
        if self.confirmed:
            return False
        if self.last_probe_at is None:
            return True
        interval = (self.fast_interval_s if self.attempts < self.fast_attempts
                    else self.slow_interval_s)
        return now - self.last_probe_at >= interval

    def record_probe(self, now: float) -> None:
        """Call only after serial.write(b'?') succeeded."""
        self.last_probe_at = now
        self.attempts += 1

    def observe_line(self, line: str) -> bool:
        if line.strip() == "SPP CONNECTED":
            self.confirmed = True
            return True
        return False
