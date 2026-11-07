source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh
T=/root/team_ws/loc_stack; OUT=/root/team_ws/walk_runs/depth_test; mkdir -p $OUT; : > $OUT/log.txt
YSRC=$(ros2 pkg prefix --share oakd_vslam_bringup)/config/oakd_vslam.yaml
sed "s/i_pipeline_type: stereo/i_pipeline_type: depth/" $YSRC > $T/oakd_vslam_depth.yaml
python3 - <<'PY'
p='/root/team_ws/loc_stack/oakd_vslam_depth.yaml'; s=open(p).read()
add="    stereo:\n      i_publish_topic: true\n      i_depth_preset: DEFAULT\n      i_subpixel: false\n      i_extended_disparity: false\n      i_lr_check: true\n"
i=s.index("    pipeline_gen:"); s=s[:i]+add+s[i:]; open(p,'w').write(s)
PY
grep -n "pipeline_type\|stereo:" $T/oakd_vslam_depth.yaml >> $OUT/log.txt
sed "s#os.path.join(bringup_share, 'config', 'oakd_vslam.yaml')#'$T/oakd_vslam_depth.yaml'#" $T/oakd_vslam_imu.launch.py > $T/oakd_vslam_depth.launch.py
grep -n "oakd_vslam_depth.yaml" $T/oakd_vslam_depth.launch.py >> $OUT/log.txt
tmux respawn-pane -k -t skybeta:slam "bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh; exec ros2 launch $T/oakd_vslam_depth.launch.py camera_x:=0.1053 camera_y:=0 camera_z:=0.0304 camera_roll:=0 camera_pitch:=0 camera_yaw:=0 enable_localization_n_mapping:=False enable_slam_visualization:=False enable_landmarks_view:=False'"
for i in $(seq 1 45); do n=$(timeout 4 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | grep -c average); [ "$n" -gt 0 ] && break; sleep 2; done; sleep 6
echo "== topics now (depth/stereo/rect)" >> $OUT/log.txt; ros2 topic list | grep -iE "stereo|depth|disp|rect|points" >> $OUT/log.txt
for t in /oak/stereo/image_raw /oak/left/image_raw /oak/right/image_raw /visual_slam/tracking/odometry; do echo "-- $t" >> $OUT/log.txt; timeout 9 ros2 topic hz $t 2>&1 | grep -E "average|does not" | tail -1 >> $OUT/log.txt; done
python3 - >> $OUT/log.txt 2>&1 <<'PY'
import numpy as np, rclpy, time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node('dprobe'); got = []
n.create_subscription(Image, '/oak/stereo/image_raw', lambda m: got.append(m), qos_profile_sensor_data)
t = time.time()
while not got and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.2)
if not got: print('NO depth message received on /oak/stereo/image_raw')
else:
    m = got[0]; print('depth msg: %dx%d encoding=%s frame=%s' % (m.width, m.height, m.encoding, m.header.frame_id))
    dt = {'16UC1': np.uint16, '32FC1': np.float32}.get(m.encoding)
    if dt is not None:
        d = np.frombuffer(m.data, dt).reshape(m.height, -1)[:, :m.width].astype(np.float64); v = d > 0
        sc = 1000.0 if m.encoding == '16UC1' else 1.0
        print('valid pixels: %.1f%%  depth m  min %.2f  p5 %.2f  median %.2f  p95 %.2f  max %.2f' % (100 * v.mean(), *(np.percentile(d[v], [0, 5, 50, 95, 100]) / sc)))
PY
echo END >> $OUT/log.txt
