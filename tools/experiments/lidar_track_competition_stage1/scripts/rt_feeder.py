#!/usr/bin/env python3
"""15 Hz real-rate feeder for the real Main node (Competition-allowed channels only).
usage: rt_feeder.py FRAMES_COMP_CSV OUT_CSV [--rate 15] [--seconds 40] [--start 0]
Publishes, paced by absolute CLOCK_MONOTONIC deadlines (no drift, lateness recorded):
  /velodyne_points  sensor_msgs/PointCloud2 (x,y,z,intensity float32, frame velodyne) at --rate
  /imu              sensor_msgs/Imu at 50 Hz (orientation from the scene's IMU-yaw channel, roll=pitch=0)
  /Ego_topic        morai_msgs/EgoVehicleStatus at 50 Hz with ONLY velocity.x set (= vel_x channel);
                    main.launch remaps /Competition_topic to /Ego_topic. No GPS is published.
Header stamps = rospy wall time at publish (like the bridge receive stamp). The scene is replayed at the
target rate regardless of its recorded rate (~9-10 Hz): this is a throughput/latency test, not an accuracy test.
OUT_CSV: seq, pub_mono_ns, stamp_ns, deadline_mono_ns, late_ms, n_points, scene_idx
"""
import csv, math, sys, time
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from morai_msgs.msg import EgoVehicleStatus

args = sys.argv[1:]
frames_csv, out_csv = args[0], args[1]
rate = 15.0; seconds = 40.0; start = 0
for i, a in enumerate(args):
    if a == "--rate": rate = float(args[i + 1])
    if a == "--seconds": seconds = float(args[i + 1])
    if a == "--start": start = int(args[i + 1])
rows = list(csv.DictReader(open(frames_csv)))[start:]
n_scans = min(len(rows), int(seconds * rate))
rows = rows[:n_scans]
clouds = []
for r in rows:
    a = np.load(r["npy"]).astype(np.float32)[:, :4]
    clouds.append((a.tobytes(), a.shape[0]))
yaw = np.unwrap(np.array([float(r["imu_yaw_rad"]) for r in rows]))
vx = np.array([float(r["vel_x_mps"]) for r in rows])
fields = [PointField("x", 0, 7, 1), PointField("y", 4, 7, 1), PointField("z", 8, 7, 1), PointField("intensity", 12, 7, 1)]

rospy.init_node("rt_feeder", anonymous=True, disable_signals=True)
pub_l = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=1, tcp_nodelay=True)
pub_i = rospy.Publisher("/imu", Imu, queue_size=5, tcp_nodelay=True)
pub_c = rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=5, tcp_nodelay=True)
time.sleep(3.0)   # let the node connect

period_l = 1.0 / rate; period_s = 1.0 / 50.0
t0 = time.monotonic_ns() + 500_000_000
end = t0 + int(n_scans * period_l * 1e9)
next_l = t0; next_s = t0 - int(0.2e9)   # IMU/CAN start 200 ms before the first scan
seq = 0
out = open(out_csv, "w", newline="")
w = csv.writer(out)
w.writerow(["seq", "pub_mono_ns", "stamp_ns", "deadline_mono_ns", "late_ms", "n_points", "scene_idx"])
while seq < n_scans:
    nxt = min(next_l, next_s)
    now = time.monotonic_ns()
    if nxt > now:
        time.sleep((nxt - now) / 1e9)
    now = time.monotonic_ns()
    if next_s <= now and next_s <= next_l:
        # fractional scan index of this sample time
        f = max(0.0, min(n_scans - 1.0, (next_s - t0) / 1e9 * rate))
        k = int(f); u = f - k; k2 = min(k + 1, n_scans - 1)
        y = yaw[k] * (1 - u) + yaw[k2] * u
        v = vx[k] * (1 - u) + vx[k2] * u
        st = rospy.Time.now()
        m = Imu(); m.header.stamp = st; m.header.frame_id = "imu"
        m.orientation.z = math.sin(0.5 * y); m.orientation.w = math.cos(0.5 * y)
        pub_i.publish(m)
        c = EgoVehicleStatus(); c.header.stamp = st; c.velocity.x = v
        pub_c.publish(c)
        next_s += int(period_s * 1e9)
        continue
    data, n = clouds[seq]
    pc = PointCloud2()
    st = rospy.Time.now()
    pc.header.stamp = st; pc.header.frame_id = "velodyne"; pc.header.seq = seq
    pc.height = 1; pc.width = n; pc.fields = fields; pc.is_bigendian = False
    pc.point_step = 16; pc.row_step = 16 * n; pc.is_dense = False; pc.data = data
    pm = time.monotonic_ns()
    pub_l.publish(pc)
    w.writerow([seq, pm, st.to_nsec(), next_l, (pm - next_l) / 1e6, n, int(rows[seq]["idx"])])
    seq += 1
    next_l += int(period_l * 1e9)
out.close()
time.sleep(1.5)
print("FEEDER_DONE scans=%d rate=%.3f" % (n_scans, rate))
