"""The contract your model implements to be scored by run_suite.py.

    from hazard.testsuite.interface import Frame, TurnPredictor

    class MyModel(TurnPredictor):
        def reset(self): ...                      # called once before each stream
        def predict(self, f: Frame) -> dict:      # called once per frame, in time order
            return {"yaw_0.5s": ..., "yaw_1s": ..., "yaw_2s": ...}

Prediction keys: "yaw_<h>s" = predicted heading change (degrees, + = LEFT) between now
and h seconds from now, for h in Frame.horizons. Omit a key to skip that horizon.
Extra keys are ignored by scoring but kept in predictions.csv (e.g. "free_dir_deg").

Rules that keep the score honest
  * Only use what Frame gives you. Labels, the manifest and future frames are off limits.
  * Frames arrive in time order at ~10 fps, possibly with gaps (dt_s > 0.1). Keep any
    history you need inside the object.
  * predict() is timed. Do your work in it, not in a background thread.
"""
import numpy as np


class Frame:
    """idx: dataset index (10 fps grid). t_s: seconds since recording start.
    dt_s: seconds since the previous frame in this stream (0 for the first).
    left/right: uint8 gray 640x1024, OAK left mono + timestamp-matched right mono.
    flagged: True if the filter rejected it (hand on lens, ...). In flight you would not
    know this; it is given so you can study it.
    camera: fx, fy, cx, cy, baseline_m (approximate). horizons: label horizons in s."""

    def __init__(self, idx, t_s, dt_s, left, right, flagged=False, camera=None,
                 horizons=(0.5, 1.0, 2.0)):
        self.idx, self.t_s, self.dt_s = idx, t_s, dt_s
        self.left, self.right, self.flagged = left, right, flagged
        self.camera = camera or {}
        self.horizons = tuple(horizons)


class TurnPredictor:
    """Base class. Subclass it, or provide any object with reset() and predict()."""

    def reset(self) -> None:
        pass

    def predict(self, f: Frame) -> dict:
        raise NotImplementedError
