import time, numpy as np, cv2, rclpy, message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node('epi2'); info = {}; out = {}
for k, t in (('L', '/oak/left/camera_info'), ('R', '/oak/right/camera_info')):
    n.create_subscription(CameraInfo, t, (lambda k: lambda m: info.setdefault(k, m))(k), 10)
def gray(m):
    a = np.frombuffer(m.data, np.uint8)
    return a.reshape(m.height, m.width, 3)[:, :, 0].copy() if m.encoding == 'bgr8' else a.reshape(m.height, m.step)[:, :m.width].copy()
def cb(l, r):
    if 'L' not in out: out['L'], out['R'], out['dt'] = gray(l), gray(r), abs((l.header.stamp.sec - r.header.stamp.sec) + (l.header.stamp.nanosec - r.header.stamp.nanosec) * 1e-9)
sl = message_filters.Subscriber(n, Image, '/oak/left/image_raw', qos_profile=qos_profile_sensor_data)
sr = message_filters.Subscriber(n, Image, '/oak/right/image_raw', qos_profile=qos_profile_sensor_data)
message_filters.ApproximateTimeSynchronizer([sl, sr], 10, 0.004).registerCallback(cb)
t = time.time()
while ('L' not in out or len(info) < 2) and time.time() - t < 15: rclpy.spin_once(n, timeout_sec=0.1)
print('synced pair dt = %.2f ms' % (out['dt'] * 1000))
KL, KR = np.array(info['L'].k).reshape(3, 3), np.array(info['R'].k).reshape(3, 3)
B = abs(info['L'].p[3]) / info['L'].p[0]
print('fxL %.1f fxR %.1f cyL %.1f cyR %.1f baseline %.4f m' % (KL[0, 0], KR[0, 0], KL[1, 2], KR[1, 2], B))
f = (KL[0, 0] + KR[0, 0]) / 2; W, H = 1024, 640
P = np.array([[f, 0, W / 2], [0, f, (KL[1, 2] + KR[1, 2]) / 2], [0, 0, 1.0]])
mapL = cv2.initUndistortRectifyMap(KL, None, np.eye(3), P, (W, H), cv2.CV_32FC1)
mapR = cv2.initUndistortRectifyMap(KR, None, np.eye(3), P, (W, H), cv2.CV_32FC1)
L = cv2.remap(out['L'], *mapL, cv2.INTER_LINEAR); R = cv2.remap(out['R'], *mapR, cv2.INTER_LINEAR)
cv2.imwrite('/tmp/rect_L.png', L); cv2.imwrite('/tmp/rect_R.png', R)
orb = cv2.ORB_create(3000)
def stats(a, b, tag):
    ka, da = orb.detectAndCompute(a, None); kb, db = orb.detectAndCompute(b, None)
    ms = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(da, db, k=2)
    g = [m for m, s in ms if m.distance < 0.7 * s.distance]
    dy = np.array([kb[m.trainIdx].pt[1] - ka[m.queryIdx].pt[1] for m in g]); dx = np.array([ka[m.queryIdx].pt[0] - kb[m.trainIdx].pt[0] for m in g])
    ok = (np.abs(dy) < 60) & (dx > 0) & (dx < 200)
    print('%s: %d good matches (%d plausible)  dy median %+.2f  p10/p90 %+.2f/%+.2f   disparity median %.1f px' % (tag, len(g), ok.sum(), np.median(dy[ok]), *np.percentile(dy[ok], [10, 90]), np.median(dx[ok])))
    return dy[ok], dx[ok]
stats(out['L'], out['R'], 'raw      ')
dy, dx = stats(L, R, 'rectified')
sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=128, blockSize=7, P1=8 * 49, P2=32 * 49, uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
d = sg.compute(L, R).astype(np.float32) / 16.0
v = d > 1
print('SGBM valid px: %.1f%%   depth (m) p5/median/p95: %s' % (100 * v.mean(), np.round(np.percentile(f * B / d[v], [5, 50, 95]), 2)))
cm = cv2.applyColorMap(np.clip(d * 2, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO); cm[~v] = 0
cv2.imwrite('/tmp/rect_disp.png', np.hstack([cv2.cvtColor(L, cv2.COLOR_GRAY2BGR), cm]))
