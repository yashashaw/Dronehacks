# Camera images + camera depth streaming together (no SLAM running), drone 7

Driver only (`depthai_ros_driver_v3`, `i_pipeline_type: depth`), no cuVSLAM, no converters, 15 s rate test + 10 s recording.

| Stream | Rate |
|---|---|
| /oak/left/image_raw (1024x640 bgr8) | 25.3 Hz |
| /oak/right/image_raw | 24.5 Hz |
| /oak/stereo/image_raw (depth, 320x200 16UC1 mm, frame oak_rgb_camera_optical_frame) | 28.1 Hz |
| /oak/rgb/* | not published in this pipeline config |

Depth over 84 synced frames (scene: person and room): valid pixels 32-42 % (mean 39 %), nearest valid reading 0.31-0.33 m,
median 0.46-0.54 m. Recorded: left image with depth colormap and per-frame stats (`img_depth_sample.png`, `img_depth_stats.csv`;
the mp4 is on the laptop under `loc_stack/runs/img_depth/`).
Neither EKF2 nor the installed cuVSLAM 3.2 node has a depth-image input.
