"""Passive MPU yaw display; positive yaw is CCW from the upward zero axis."""
import math
from spp_dual_log import sensor_test_csv


def heading_point(deg, radius=1.0):
    a = math.radians(deg % 360.0)
    return -radius * math.sin(a), -radius * math.cos(a)


class AngleSample:
    def __init__(self):
        self.clear()

    def clear(self):
        self.yaw = None
        self.valid = False
        self.age = None

    def consume(self, line):
        record = sensor_test_csv(line)
        if not record or record[0] not in ("MPU","PIVOT","HEADING"):
            return False
        row = dict(zip(record[1], record[2]))
        self.age = int(row["age_us"]) if row["age_us"] else None
        self.valid = row["valid"] == "1" and (self.age is None or 0 <= self.age <= 3000)
        self.yaw = float(row["angle_deg"] if record[0] in ("PIVOT","HEADING") else row["yaw_deg"]) if self.valid else None
        return True


def draw_angle(canvas, sample, state):
    c = canvas
    c.delete("all")
    w, h = c.winfo_width(), c.winfo_height()
    if w < 140 or h < 120:
        return
    cx, cy = w / 2, (h + 5) / 2
    r = max(25, min(w / 2 - 40, h / 2 - 44))
    def point(deg, radius):
        x, y = heading_point(deg, radius)
        return cx+x, cy+y
    for frac in (.25, .5, .75, 1):
        rr = r * frac
        c.create_oval(cx-rr, cy-rr, cx+rr, cy+rr, outline="#21516b" if frac < 1 else "#43b5df", width=1 if frac < 1 else 2)
    for a in range(0, 360, 15):
        x,y = point(a,r)
        xi,yi = point(a,r-(8 if a%45==0 else 4))
        c.create_line(x,y,xi,yi,fill="#6dacc4")
        if a%45==0:
            c.create_line(cx,cy,x,y,fill="#24475b",dash=(3,4))
            lx,ly=point(a,r+18)
            label = a if a <= 180 else a-360
            c.create_text(lx,ly,text=f"{label}°",fill="#b4d5e2",font=("TkDefaultFont",9))
    x,y=point(0,r)
    c.create_line(cx,cy,x,y,fill="#92a7b2",dash=(4,4))
    if sample.valid and sample.yaw is not None:
        a=sample.yaw
        x,y=point(a,r*.95)
        color="#51ddff" if state=="LIVE" else "#8ba5b0"
        c.create_line(cx,cy,x,y,fill=color,width=3,arrow="last")
        # Top-view mouse: front tip, two wheels, body; rotate all with yaw.
        size=min(18,r*.23)
        fx,fy=heading_point(a)
        rx,ry=heading_point(a-90)
        def poly(coords):
            return [v for u,t in coords for v in (cx+size*(u*rx+t*fx),cy+size*(u*ry+t*fy))]
        c.create_polygon(*poly([(-.65,-.75),(.65,-.75),(.65,.5),(0,1),(-.65,.5)]),fill="#153c4c",outline=color,width=2)
        for side in (-1,1):
            c.create_polygon(*poly([(side*.72-.12,-.48),(side*.72+.12,-.48),(side*.72+.12,.40),(side*.72-.12,.40)]),fill="#8aa9b8")
        label=f"YAW  {a:+.2f}°"
    else:
        color="#d4a65b"
        label="YAW  —"
    c.create_text(cx,14,text=label,fill=color,font=("TkDefaultFont",15,"bold"))
    age=f"  |  age {sample.age} µs" if sample.age is not None else ""
    c.create_text(cx,h-13,text=state+age,fill="#b4d5e2",font=("TkDefaultFont",9))
