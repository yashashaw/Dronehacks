"""Simple steering policies on top of a depth algorithm.

    --algo hazard/depthsuite/policies.py:CenterThenRight --arg open_m=2.0
    --algo hazard/depthsuite/policies.py:CenterThenRight --arg turn_deg=25 --arg order=right,left

CenterThenRight (the team's rule), every check is the drone-shaped 3D tube from
common.tube_blocked (0.6 m wide x 0.25 m tall x open_m long):
    if the straight-ahead tube is clear       -> go straight   (steer 0)
    elif the tube turned turn_deg right is clear -> turn right (steer -turn_deg)
    elif the tube turned turn_deg left is clear  -> turn left  (steer +turn_deg)
    else                                      -> STOP          (steer 0)
A tube with too little depth to judge counts as blocked. Depth comes from StereoSGBM at
320x200 on the CPU. Python 3.6 / OpenCV 4.1 compatible (Jetson).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from baselines import StereoBM, StereoSGBM  # noqa: E402
from common import TUBE_H_M, TUBE_W_M, tube_blocked  # noqa: E402


class CenterThenRight:
    def __init__(self, open_m=2.0, turn_deg=25.0, order="right,left", width_m=TUBE_W_M, height_m=TUBE_H_M,
                 depth="sgbm"):
        self.open_m, self.turn = float(open_m), float(turn_deg)
        self.w, self.h = float(width_m), float(height_m)
        self.order = [o.strip() for o in str(order).split(",") if o.strip()]
        self.bearing = {"center": 0.0, "right": -self.turn, "left": self.turn}
        self.depth = StereoSGBM() if depth == "sgbm" else StereoBM()

    def predict(self, s):
        d = self.depth.predict(s)["depth"]
        info = {}
        for name in ["center"] + self.order:
            blocked, clr, _ = tube_blocked(d, range_m=self.open_m, width_m=self.w, height_m=self.h,
                                           bearing_deg=self.bearing[name])
            info[name] = clr
            if not blocked:
                return {"depth": d, "depth_kind": "metric", "steer_deg": self.bearing[name],
                        "action": "straight" if name == "center" else "turn_" + name,
                        "clearance_m": info, "blocked": name != "center"}
        return {"depth": d, "depth_kind": "metric", "steer_deg": 0.0, "action": "stop",
                "clearance_m": info, "blocked": True}
