#!/usr/bin/env python3
"""Synthetic parser-only regression; NEVER interpret as physical IR evidence."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import json


spec = importlib.util.spec_from_file_location("ir_full",
    Path(__file__).resolve().parents[2] / "tools" / "ir_full_qualification.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def make_trace(n=256, direction="rising", missing=-1, drops=0):
    start = 1000
    values = []
    for seq in range(n):
        lo, hi = (80, 600) if direction == "rising" else (600, 80)
        raw = lo if seq < n // 2 else hi
        slow = lo if seq < n // 2 + 7 else hi
        fast = lo if seq < n // 2 + 3 else hi
        hybrid = lo if seq < n // 2 + 4 else hi
        def line(x):
            return "|".join(str(x) for _ in range(6))
        line_data = (f"IBD,id=1,seq={seq},us={start+seq*826},scan={100+seq},"
                     f"d={line(raw)},med={line(fast)},e10={line(slow)},"
                     f"e25={line(fast)},hy={line(hybrid)}")
        values.append(line_data)
    if missing >= 0:
        del values[missing]
    return (["IB,BEGIN,id=1,count=256,first_scan=100,last_scan=355,locked=1"]
            + values + [f"IB,END,id=1,count=256,uart_drop_delta={drops},"
                        "dma_err=0,dma_to=0,ovr=0,fail=0,locked=1"])

def test_valid():
    rows, quality = module.parse_burst(make_trace())
    assert quality["valid"], quality
    assert len(rows) == 256 * 6
    edge = module.transition(rows, "L0", "rising")
    assert edge["status"] == "COARSE_RELATIVE_EVENT_ACCEPTED", edge
    assert edge["relative_lag_us"]["ema10"] == 7*826, edge
    assert edge["relative_lag_us"]["ema25"] == 3*826, edge
    assert quality["absolute_wall_event_latency_supported"] is False

def test_falling():
    rows, quality = module.parse_burst(make_trace(direction="falling"))
    assert quality["valid"]
    edge = module.transition(rows, "R0", "falling")
    assert edge["status"] == "COARSE_RELATIVE_EVENT_ACCEPTED", edge

def test_missing():
    _, quality = module.parse_burst(make_trace(missing=5))
    assert not quality["valid"], quality

def test_dropped():
    _, quality = module.parse_burst(make_trace(drops=1))
    assert not quality["valid"], quality

def test_bad_adc():
    seq = make_trace()
    seq[1] = seq[1].replace("d=80|80|80|80|80|80",
                            "d=80|80|5000|80|80|80")
    _, q = module.parse_burst(seq)
    assert not q["valid"], q

def test_no_edge():
    rows, _ = module.parse_burst(make_trace())
    edge = module.transition(rows, "L0", "falling")
    assert edge["status"] == "INCONCLUSIVE"

def test_static_wrapper_preserves_setup_and_distances():
    # Regression for 2026-10-27 physical capture: the static collector rejects
    # --output when the parent run directory already contains setup.json.
    with TemporaryDirectory() as td:
        dest = Path(td) / "run"
        dest.mkdir()
        original_setup = {"distance_left_mm": 60.0, "distance_right_mm": 60.0}
        (dest / "setup.json").write_text(json.dumps(original_setup), encoding="utf-8")
        args = SimpleNamespace(
            port="COM6", seconds=30.0, case="static", surface="white_matte",
            light="room", trial=1, notes="two side walls",
            distance=None, distance_left=60.0, distance_right=60.0,
        )

        def fake_run(cmd, cwd=None, check=False):
            pos = cmd.index("--output")
            data = Path(cmd[pos+1])
            assert data == dest / "data"
            assert data.parent.is_dir()
            assert not data.exists(), "Static collector requires nonexistent output"
            assert "--distance-mm" not in cmd, "Distinct L/R distances must not be collapsed"
            data.mkdir()
            (data / "ir_filter_trace.csv").write_text("test", encoding="utf-8")
            (data / "summary.json").write_text(
                json.dumps({"valid_capture": True, "channels": {},
                            "metadata": {"trial": 1, "distance_mm": None}}),
                encoding="utf-8",
            )
            return SimpleNamespace(returncode=0)

        with patch.object(module.subprocess, "run", side_effect=fake_run):
            module.capture_static(args, dest)

        original = json.loads((dest / "setup.json").read_text(encoding="utf-8"))
        assert original == original_setup
        summary = json.loads((dest / "summary.json").read_text(encoding="utf-8"))
        assert summary["valid_capture"]
        assert summary["metadata"]["distance_left_mm"] == 60.0
        assert summary["metadata"]["distance_right_mm"] == 60.0
        assert summary["trace_directory"] == "data"
        assert (dest / "data" / "ir_filter_trace.csv").read_text(encoding="utf-8") == "test"
        assert (dest / "data" / "summary.json").is_file()

        # A repeat in the same run folder must fail closed, not overwrite data.
        try:
            with patch.object(module.subprocess, "run", side_effect=fake_run):
                module.capture_static(args, dest)
        except ValueError as exc:
            assert "already exists" in str(exc)
        else:
            raise AssertionError("Capture must protect previously collected raw data")


if __name__ == "__main__":
    for fn in (test_valid, test_falling, test_missing, test_dropped,
               test_bad_adc, test_no_edge,
               test_static_wrapper_preserves_setup_and_distances):
        fn()
    print("PASS: IR burst parser regression (synthetic, no board measurement)")
