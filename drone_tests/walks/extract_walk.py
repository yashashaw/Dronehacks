import sys, csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr","cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {"/visual_slam/tracking/odometry","/visual_slam/status","/fmu/out/vehicle_local_position","/fmu/out/vehicle_attitude","/fmu/out/vehicle_status_v1","/fmu/in/vehicle_visual_odometry","/oak/left/camera_info"}
r.set_filter(rosbag2_py.StorageFilter(topics=sorted(want & set(types))))
cls = {t: get_message(types[t]) for t in want if t in types}
w = {k: csv.writer(open(f"{out}/{k}.csv","w")) for k in ["vio","vio_status","lpos","att","status","ev_in","cam"]}
w["vio"].writerow(["t","stamp","x","y","z","qx","qy","qz","qw"])
w["vio_status"].writerow(["t","vo_state"])
w["lpos"].writerow(["t","px4_ts","x","y","z","vx","vy","vz","heading","xy_valid","z_valid","dist_bottom","dist_bottom_valid","xy_reset","z_reset","dead_reckoning"])
w["att"].writerow(["t","q0","q1","q2","q3"])
w["status"].writerow(["t","arming_state","nav_state"])
w["ev_in"].writerow(["t","x","y","z","reset_counter","quality"])
w["cam"].writerow(["t","stamp"])
n = 0; k = 0
while r.has_next():
    try:
        topic, data, t = r.read_next()
    except Exception as e:
        print("stopped at corruption:", e); break
    m = deserialize_message(data, cls[topic]); ts = t * 1e-9; n += 1
    if topic == "/visual_slam/tracking/odometry":
        p = m.pose.pose.position; q = m.pose.pose.orientation
        w["vio"].writerow([f"{ts:.4f}", f"{m.header.stamp.sec + m.header.stamp.nanosec*1e-9:.4f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    elif topic == "/visual_slam/status":
        w["vio_status"].writerow([f"{ts:.4f}", m.vo_state])
    elif topic == "/fmu/out/vehicle_local_position":
        w["lpos"].writerow([f"{ts:.4f}", m.timestamp, m.x, m.y, m.z, m.vx, m.vy, m.vz, m.heading, int(m.xy_valid), int(m.z_valid), m.dist_bottom, int(m.dist_bottom_valid), m.xy_reset_counter, m.z_reset_counter, int(m.dead_reckoning)])
    elif topic == "/fmu/out/vehicle_attitude":
        k += 1
        if k % 4 == 0: w["att"].writerow([f"{ts:.4f}", *m.q])
    elif topic == "/fmu/out/vehicle_status_v1":
        w["status"].writerow([f"{ts:.4f}", m.arming_state, m.nav_state])
    elif topic == "/fmu/in/vehicle_visual_odometry":
        w["ev_in"].writerow([f"{ts:.4f}", *m.position, m.reset_counter, m.quality])
    elif topic == "/oak/left/camera_info":
        w["cam"].writerow([f"{ts:.4f}", f"{m.header.stamp.sec + m.header.stamp.nanosec*1e-9:.4f}"])
print("messages read:", n)
print("DONE")
