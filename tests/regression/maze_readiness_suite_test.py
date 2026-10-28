#!/usr/bin/env python3
"""Synthetic regression of the offline maze-readiness tools.
No BLE radio, serial port, firmware flash or physical motor test.
"""
from __future__ import annotations
import csv
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import json

ROOT = Path(__file__).resolve().parents[2]
P = ROOT / "tools" / "maze_readiness_suite.py"
spec = importlib.util.spec_from_file_location("maze_readiness_suite", P)
assert spec and spec.loader
suite = importlib.util.module_from_spec(spec)
spec.loader.exec_module(suite)


def labeled(path: Path, mode: str):
    columns = ("ms",) + suite.CHANNELS + suite.LABELS
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        frames = (20 if mode == "train" else 36)
        for n in range(frames):
            wall = 1 if (n>=10 if mode=="train" else 12<=n<24) else 0
            row = {"ms": n * 10}
            for i,c in enumerate(suite.CHANNELS):
                no_wall = 100 + 60*i
                near_wall = 1050 + 60*i
                value = near_wall if wall else no_wall
                if mode == "test" and n == 3:
                    value = 3500  # A single transient must NOT count as wall.
                row[c] = value
                row["wall_"+c] = wall
            writer.writerow(row)


def ble(path: Path, done=True):
    reason = "DONE" if done else "IR_STALE"
    text = f"""[WEB] W,START,mm=720,diam=34,cpr=1430,pwmcap=800,error=REL_NORM,l=460,r=664
[WEB] W,END,{reason},ms=150,d10={7201 if done else 1200},avgen1000=15,peakn1000=35,vpk=700,pkpwm=580,locked=1,trace=3
[WEB] W,50,200,461,668,10,400,500,300,300
[WEB] W,100,500,470,655,-20,580,850,600,620
[WEB] W,POST,dist10={7210 if done else 1400},drift10={9 if done else 200},locked=1
[WEB] W,150,{7201 if done else 1200},462,666,15,680,700,560,550
[WEB] W,TRACE_END
"""
    path.write_text(text, encoding="utf-8")


def main():
    with TemporaryDirectory(prefix="maze-readiness-synthetic-") as d:
        root = Path(d)
        train, test = root/"train.csv", root/"test.csv"
        labeled(train,"train")
        labeled(test,"test")
        assert train.resolve()!=test.resolve()
        wall = suite.analyze_wall_events(train,test,root/"wall",confirm_frames=3)
        assert wall["train_frames"]==20 and wall["test_frames"]==36
        assert len(wall["results"])==6
        for row in wall["results"]:
            assert row["f1"] is not None and row["f1"]>.75, row
            assert row["false_positive"]<=2, row
            assert row["gt_transitions"]==2, row
            assert row["missed_transitions"]==0, row
            assert row["p95_detection_delay_ms"]==20.0, row
        assert not wall["maze_wall_detection_qualified"]
        assert (root/"wall"/"sensor_confusion.csv").is_file()

        try:
            suite.analyze_wall_events(train,train,root/"no_leak")
        except ValueError as e:
            assert "different files" in str(e)
        else:
            raise AssertionError("TRAIN=TEST leakage must be rejected")

        # Label domain is strict; reject invalid ADC/ground-truth data.
        bad = root/"bad.csv"
        source = test.read_text(encoding="utf8")
        bad.write_text(source.replace(",1050,", ",5000,",1), encoding="utf8")
        try:
            suite.read_labeled_csv(bad)
        except ValueError as exc:
            assert "ADC" in str(exc)
        else:
            raise AssertionError("Out-of-range ADC should be rejected")

        good_log,bad_log=root/"run_1.txt",root/"run_2.txt"
        ble(good_log, True)
        ble(bad_log, False)
        motion=suite.analyze_motion([good_log,bad_log],root/"motion")
        assert motion["runs"]==2 and motion["complete_logs"]==2
        assert motion["done_runs"]==1 and motion["review_no_flags"]==1, motion
        assert motion["results"][0]["mae_norm"] is not None
        assert motion["results"][0]["stop_mm"]==720.1
        assert motion["results"][1]["drift_mm"]==20.0
        assert motion["results"][1]["reason"]=="IR_STALE"
        assert "NOT_720MM_DONE" in motion["results"][1]["review_flags"]
        assert (root/"motion"/"motion_summary.json").is_file()

        incomplete=root/"missing.txt"
        incomplete.write_text(good_log.read_text().replace("W,TRACE_END","W,TRUNCATED"),encoding="utf8")
        broken=suite.analyze_motion([incomplete],root/"incomplete")
        assert broken["complete_logs"]==0
        assert "MISSING_OR_INVALID_BLE_LOG" in broken["results"][0]["review_flags"]

        # Combined runner is the future one-command workflow. Preflight
        # subprocesses mocked to prevent hardware or unexpected dependencies.
        fake={"software_checks_ok":True,"mode":"offline_software_only"}
        with patch.object(suite,"preflight",return_value=fake):
            total=suite.analyze_all([good_log],train,test,root/"all")
        assert total["software_checks_ok"] and total["ble_logs_complete"]
        assert not total["physical_acceptance"]
        assert (root/"all"/"all_summary.json").is_file()
        assert (root/"all"/"wall_events"/"wall_detection_report.json").is_file()

        # Entry-point failures are explicit and do not create fake PASS.
        assert suite.main(["wall-events","--train",str(train),
                            "--test",str(train),"--out",str(root/"reject")])==2
        assert suite.main(["motion","--logs",str(good_log),
                            "--out",str(root/"cli")])==0
    print("MAZE_READINESS_SUITE_SYNTHETIC_PASS")


if __name__=="__main__":
    main()
