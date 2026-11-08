"""Reference algorithms for the depth decision suite (all CPU, no GPU).

    --algo hazard/depthsuite/baselines.py:StraightAhead     floor: always steer 0
    --algo hazard/depthsuite/baselines.py:StereoBM          OpenCV block matching, 320x200
    --algo hazard/depthsuite/baselines.py:StereoSGBM        semi-global matching, 320x200
    --algo hazard/depthsuite/baselines.py:FlowNearness      mono: optical-flow magnitude from
                                                            prev_left (no model, no GPU)
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Sample, depth_to_left_view, rectify_pair  # noqa: E402


class StraightAhead:
    def predict(self, s: Sample) -> dict:
        return {"steer_deg": 0.0, "blocked": False}  # never sees a hazard: the floor


class _Stereo:
    scale = 0.3125  # 1024x640 -> 320x200

    def __init__(self, num_disp: int = 32, block: int = 11):
        self.num_disp, self.block = int(num_disp), int(block)
        self.matcher = self._make()

    def predict(self, s: Sample) -> dict:
        l, r, HL = rectify_pair(s.left, s.right, s.rectify, self.scale)
        disp = self.matcher.compute(l, r).astype(np.float32) / 16.0
        disp[disp <= 0.5] = np.nan
        depth = (s.rectify["fx_px"] * self.scale) * s.rectify["baseline_m"] / disp
        return {"depth": depth_to_left_view(depth, HL), "depth_kind": "metric"}


class StereoBM(_Stereo):
    def _make(self):
        m = cv2.StereoBM_create(numDisparities=self.num_disp, blockSize=self.block)
        m.setTextureThreshold(8)
        m.setUniquenessRatio(10)
        return m


class StereoSGBM(_Stereo):
    def _make(self):
        b = self.block if self.block % 2 else self.block + 1
        b = min(b, 7)
        return cv2.StereoSGBM_create(minDisparity=0, numDisparities=self.num_disp, blockSize=b,
                                     P1=8 * b * b, P2=32 * b * b, uniquenessRatio=10,
                                     speckleWindowSize=50, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM)


class FlowNearness:
    """Monocular, model-free: when moving forward, near things flow faster. Uses dense
    Farneback flow at 256x160 against prev_left, after removing the median flow (crude
    rotation compensation). Returns flow magnitude as relative inverse depth."""

    def predict(self, s: Sample) -> dict:
        a = cv2.resize(s.prev_left, (256, 160), interpolation=cv2.INTER_AREA)
        b = cv2.resize(s.left, (256, 160), interpolation=cv2.INTER_AREA)
        f = cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 3, 15, 3, 5, 1.1, 0)
        f -= np.median(f.reshape(-1, 2), axis=0)
        mag = np.linalg.norm(f, axis=2) + 1e-3
        return {"depth": mag, "depth_kind": "inverse"}
