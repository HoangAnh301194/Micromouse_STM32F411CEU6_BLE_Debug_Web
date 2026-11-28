"""Time-aligned wheel telemetry; absent signals remain absent."""
from collections import deque
from datetime import datetime
from spp_dual_log import sensor_test_csv

class WheelPlotData:
    def __init__(self):
        self.rows = deque(maxlen=2400)
        self.clear()

    def clear(self):
        self.rows.clear()
        self.origin = None
        self.kind = None
        self.count_origin = None

    def add(self, line, host_time):
        record = sensor_test_csv(line, host_time=host_time)
        if not record or record[0] not in ("ENC", "ONEBOX", "PIVOT"):
            return False
        kind, cols, values = record
        d = dict(zip(cols, values))
        try:
            raw_t = (int(d["t_ms"])/1000 if kind=="ONEBOX" else
                     int(d["mcu_ms"])/1000 if kind=="PIVOT" else datetime.fromisoformat(d["timestamp"]).timestamp())
            counts = [int(d["ENC_"+s]) for s in ("L","R")]
        except (ValueError, TypeError, KeyError):
            return False
        if self.kind != kind or (self.rows and raw_t < self.origin+self.rows[-1]["t"]-0.000001):
            self.clear()
        if self.origin is None:
            self.origin = 0 if kind=="ONEBOX" else raw_t
            self.count_origin = counts
            self.kind = kind
        row = {"t": raw_t-self.origin}
        for i,s in enumerate(("L","R")):
            row["count_"+s] = counts[i]-self.count_origin[i]
            for key,col in (("pwm","PWM_"+s),("ref","REF_"+s+"_PPS"),("speed","SPEED_"+s+"_PPS")):
                if d.get(col) not in (None,""):
                    row[key+"_"+s] = int(d[col])
            if "ref_"+s in row and "speed_"+s in row:
                row["error_"+s] = row["ref_"+s]-row["speed_"+s]
        self.rows.append(row)
        return True

    def add_live(self, fields, now):
        try:
            counts=[int(fields[k]) for k in ("left","right")]
            values={k: int(fields[k]) for k in ("speed_l","speed_r","pwm_l","pwm_r")}
        except (KeyError, TypeError, ValueError):
            return False
        if self.kind!="LIVE":
            self.clear()
            self.kind="LIVE"
            self.origin=now
            self.count_origin=counts
        row={"t":now-self.origin}
        for i,side in enumerate(("L","R")):
            row["count_"+side]=counts[i]-self.count_origin[i]
            row["speed_"+side]=values["speed_"+side.lower()]
            row["pwm_"+side]=values["pwm_"+side.lower()]
        self.rows.append(row)
        return True
