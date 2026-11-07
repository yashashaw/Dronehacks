# Stereo depth probe (drone 7, 2026-11-06)

`stereo_probe.py` grabs one hardware-synced `/oak/{left,right}/image_raw` pair inside the
container, rectifies it with the published `camera_info`, and runs OpenCV SGBM.

| Check | Result |
|---|---|
| L/R stamp difference (synced) | 0.02 ms (good) |
| Baseline from `camera_info` P[0,3]/fx | 0.0754 m (matches TF: 0.037 + 0.038) |
| Row alignment, raw images | dy median -28 px |
| Row alignment after rectifying with `camera_info` | dy median -7 px, p10/p90 -8.6/+3.8 |
| SGBM valid pixels | 19.7 %, depth speckled and incoherent (`depth_probe.png`) |

Conclusion: the driver publishes `camera_info` with K from calibration but D = 0 and no
stereo rotation, so it is not a valid rectification (left/right differ in fx 627/637 and
cy 326/304). Host-side SGBM on the published images does not give usable depth. Dense depth
needs either the device's real calibration (rectification + distortion) or the camera's
on-device StereoDepth node, which costs USB bandwidth on a link the repo measured to be near
its ceiling.
