"""pytest hazard/tests  -- fast checks of the labeler maths and the scoring harness."""
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "hazard"))
from vision import camera_matrix, rotation_between  # noqa: E402

DS = ROOT / "data" / "walk" / "hazard_10fps"


def _textured(seed=0):
    rng = np.random.default_rng(seed)
    img = cv2.resize(rng.integers(0, 255, (40, 64), dtype=np.uint8), (512, 320), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(img, (3, 3), 0)


def _rotate(g, yaw_left_deg, pitch_up_deg, K):
    a, b = math.radians(-yaw_left_deg), math.radians(pitch_up_deg)
    Ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    H = K @ (Ry @ Rx).T @ np.linalg.inv(K)
    return cv2.warpPerspective(g, H, (g.shape[1], g.shape[0]))


@pytest.mark.parametrize("yaw,pitch", [(3, 0), (-3, 0), (0, 2), (0, -2), (2, -1)])
def test_rotation_sign_and_size(yaw, pitch):
    K = camera_matrix()
    g = _textured()
    y, p, n = rotation_between(g, _rotate(g, yaw, pitch, K), K)
    assert n > 50
    assert abs(y - yaw) < 0.3 and abs(p - pitch) < 0.3


def test_blank_frame_is_nan():
    g = np.full((320, 512), 128, np.uint8)
    y, p, n = rotation_between(g, g)
    assert math.isnan(y) and math.isnan(p)


SET = ROOT / "data" / "walk" / "depth_set_400"


@pytest.mark.skipif(not (SET / "samples.csv").exists(), reason="run build_set.py first")
def test_hazard_scoring_with_labels(tmp_path):
    import json
    import shutil
    n = 12
    mini = tmp_path / "set"
    for d in ("left", "right", "prev_left"):
        (mini / d).mkdir(parents=True)
        for i in range(n):
            shutil.copyfile(SET / d / "{:03d}.jpg".format(i), mini / d / "{:03d}.jpg".format(i))
    shutil.copyfile(SET / "rectify.json", mini / "rectify.json")
    lines = (SET / "samples.csv").read_text().splitlines()[:n + 1]
    (mini / "samples.csv").write_text("\n".join(lines) + "\n")
    truth = [1, 0, 1, -1, 0, 1, 0, 0, 1, 1, 0, 1]   # -1 = unsure, must be ignored
    (mini / "hazard_labels.csv").write_text("id,blocked\n" + "".join("{},{}\n".format(i, b) for i, b in enumerate(truth)))
    for algo, expect_recall in (("StraightAhead", 0.0), ("StereoSGBM", None)):
        out = tmp_path / algo
        p = subprocess.run([sys.executable, "-I", str(ROOT / "hazard/depthsuite/run_depth_suite.py"),
                            "--algo", str(ROOT / "hazard/depthsuite/baselines.py") + ":" + algo,
                            "--set", str(mini), "--out", str(out)], capture_output=True, text=True, cwd=ROOT)
        assert p.returncode == 0, p.stderr
        hz = json.loads((out / "report.json").read_text())["hazard"]
        assert hz["scored"] == 11 and hz["hazards"] == 6 and hz["clears"] == 5
        assert hz["caught"] + hz["missed_hazards"] == 6
        if expect_recall is not None:
            assert hz["hazard_recall"] == expect_recall and hz["missed_hazards"] == 6
        names = sorted(f.name for f in (out / "images").glob("*.jpg"))
        assert len(names) == n and all(any(t in x for t in ("CAUGHT", "MISSED_HAZARD", "FALSE_STOP", "CLEAR_OK", "_OK", "_MISS"))
                                       for x in names)


@pytest.mark.skipif(not (DS / "manifest.csv").exists(), reason="run prepare_walk.py + filter_walk.py first")
def test_oracle_scores_perfectly(tmp_path):
    out = subprocess.run([sys.executable, "-I", str(ROOT / "hazard/testsuite/run_suite.py"),
                          "--model", str(ROOT / "hazard/testsuite/baselines.py") + ":OraclePredictor",
                          "--model-arg", f"dataset={DS}", "--dataset", str(DS), "--split", "all",
                          "--out", str(tmp_path)], capture_output=True, text=True, cwd=ROOT)
    assert out.returncode == 0, out.stderr
    import json
    rep = json.loads((tmp_path / "report.json").read_text())
    for v in rep["horizons"].values():
        assert v["scored"] > 0 and v["mae_deg"] < 0.01 and v["direction_acc"] == 1.0
