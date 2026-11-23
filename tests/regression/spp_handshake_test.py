"""Deterministic no-radio SPP retry/ack tests for giomuadongbacv2."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from classic_spp_handshake import SPPHandshake


def main():
    state = SPPHandshake()
    assert state.should_probe(0.0)
    state.record_probe(0.0)
    assert state.attempts == 1
    assert not state.should_probe(0.79)
    assert state.should_probe(0.8)
    for attempt in range(2, 7):
        now = float(attempt)
        assert state.should_probe(now)
        state.record_probe(now)
    assert state.attempts == state.fast_attempts
    assert not state.should_probe(8.99)
    assert state.should_probe(9.0)  # 3.0s backoff after sixth probe
    assert not state.observe_line("MPU,ms=1,valid=1")
    assert not state.observe_line("SPP CONNECT")
    assert not state.observe_line("SPP CONNECTED EXTRA")
    assert state.observe_line("SPP CONNECTED\r\n")
    assert state.confirmed
    assert not state.should_probe(300.0)

    new_session = SPPHandshake()
    assert not new_session.confirmed
    assert new_session.should_probe(0.0)
    assert new_session.observe_line("  SPP CONNECTED  ")
    assert not new_session.should_probe(10.0)
    print("spp_handshake_test: PASS")


if __name__ == "__main__":
    main()
