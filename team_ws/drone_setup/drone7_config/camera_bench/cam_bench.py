#!/usr/bin/env python3
"""Bench-measure OAK-D topics: rate, drops, L/R sync, size, depth validity.

usage: cam_bench.py SECONDS [LABEL]
Subscribes to every /oak/**/camera_info (cheap) for rate/sync, grabs ONE frame of each
image topic for size/encoding, and subscribes continuously to the depth image (if any)
for valid-pixel fraction.
"""
import sys, time, json, statistics as st
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else "run"
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
BPP = {"mono8": 1, "8UC1": 1, "bgr8": 3, "rgb8": 3, "16UC1": 2, "mono16": 2, "32FC1": 4}


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Bench(Node):
    def __init__(self):
        super().__init__("cam_bench")
        self.ci = {}      # topic -> list of (recv_time, stamp, w, h)
        self.img = {}     # image topic -> (w, h, encoding)
        self.depth = []   # (valid_frac, center_median_m)
        self.subs = []
        self.depth_topic = None

    def discover(self):
        names = dict(self.get_topic_names_and_types())
        for t, types in names.items():
            if not t.startswith("/oak"):
                continue
            if "sensor_msgs/msg/CameraInfo" in types and t not in self.ci:
                self.ci[t] = []
                self.subs.append(self.create_subscription(
                    CameraInfo, t, lambda m, t=t: self.ci[t].append(
                        (time.monotonic(), stamp(m), m.width, m.height)), QOS))
            if "sensor_msgs/msg/Image" in types and t not in self.img and t.endswith("image_raw"):
                self.img[t] = None
                is_depth = "stereo" in t or "depth" in t
                if is_depth and self.depth_topic is None:
                    self.depth_topic = t
                    self.subs.append(self.create_subscription(Image, t, self.on_depth, QOS))
                else:
                    holder = {}
                    def once(m, t=t, holder=holder):
                        if self.img[t] is None:
                            self.img[t] = (m.width, m.height, m.encoding)
                    holder["s"] = self.create_subscription(Image, t, once, QOS)
                    self.subs.append(holder["s"])

    def on_depth(self, m):
        t = self.depth_topic
        if self.img[t] is None:
            self.img[t] = (m.width, m.height, m.encoding)
        if m.encoding not in ("16UC1", "mono16"):
            return
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        h, w = d.shape
        c = d[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
        cv = c[c > 0]
        self.depth.append((float((d > 0).mean()), float(np.median(cv)) / 1000.0 if cv.size else None))


def pct(a, p):
    a = sorted(a)
    return a[min(len(a) - 1, int(p / 100 * len(a)))] if a else None


def main():
    rclpy.init()
    n = Bench()
    t0 = time.monotonic()
    # discovery window (driver may still be starting)
    while time.monotonic() - t0 < 6:
        n.discover()
        rclpy.spin_once(n, timeout_sec=0.1)
    for v in n.ci.values():
        v.clear()
    n.depth.clear()
    t1 = time.monotonic()
    while time.monotonic() - t1 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    out = {"label": LABEL, "seconds": DUR, "topics": {}}
    total_bw = 0.0
    for t, v in sorted(n.ci.items()):
        if len(v) < 2:
            out["topics"][t] = {"msgs": len(v)}
            continue
        ss = [s for _, s, _, _ in v]
        dt = [(b - a) * 1000 for a, b in zip(ss, ss[1:])]
        nominal = st.median(dt)
        gaps = sum(1 for x in dt if x > 1.5 * nominal)
        lost = sum(round(x / nominal) - 1 for x in dt if x > 1.5 * nominal)
        img_t = t.replace("camera_info", "image_raw")
        info = n.img.get(img_t)
        rate = (len(v) - 1) / (ss[-1] - ss[0])
        r = {"rate_hz": round(rate, 2), "w": v[0][2], "h": v[0][3],
             "dt_p50_ms": round(nominal, 2), "dt_p95_ms": round(pct(dt, 95), 2),
             "dt_max_ms": round(max(dt), 2), "gaps": gaps, "frames_lost": lost,
             "lost_pct": round(100 * lost / (len(v) + lost), 2)}
        if info:
            w, h, enc = info
            r["encoding"] = enc
            bw = w * h * BPP.get(enc, 0) * rate / 1e6
            r["MBps"] = round(bw, 1)
            total_bw += bw
        out["topics"][t] = r
    # L/R sync between any left*/right* camera_info pairs
    for lt in [t for t in n.ci if "/left" in t]:
        rt = lt.replace("/left", "/right")
        if rt in n.ci and n.ci[lt] and n.ci[rt]:
            rs = {round(s, 3) for _, s, _, _ in n.ci[rt]}
            ls = [s for _, s, _, _ in n.ci[lt]]
            rsl = sorted(s for _, s, _, _ in n.ci[rt])
            import bisect
            d = []
            for s in ls:
                i = bisect.bisect_left(rsl, s)
                cands = [rsl[j] for j in (i - 1, i) if 0 <= j < len(rsl)]
                d.append(min(abs(c - s) for c in cands) * 1000)
            out[f"sync {lt.split('/')[2]}"] = {"p50_ms": round(pct(d, 50), 3), "p95_ms": round(pct(d, 95), 3),
                                               "pairs_within_1ms_pct": round(100 * sum(x < 1 for x in d) / len(d), 1)}
    if n.depth:
        vf = [a for a, _ in n.depth]
        cm = [b for _, b in n.depth if b is not None]
        out["depth"] = {"topic": n.depth_topic, "frames": len(n.depth),
                        "valid_frac_p50": round(pct(vf, 50), 3), "valid_frac_p10": round(pct(vf, 10), 3),
                        "center_median_m_p50": round(pct(cm, 50), 3) if cm else None}
    out["est_link_MBps"] = round(total_bw, 1)
    print("RESULT " + json.dumps(out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
