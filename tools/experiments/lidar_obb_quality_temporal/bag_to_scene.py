#!/usr/bin/env python3
"""Static GT bag -> harness scene input (frames.csv + npy), read-only on the bag.

Why: every /imu message of this bag has a zero orientation quaternion (|q| = 0), which the production
SensorPoseHistory rejects (IMU not ready), so the bag cannot be replayed with its own IMU: production
would publish nothing for any of its 291 scans. The scene form feeds the same LiDAR scans, stamped with
their header stamps, with the ego pose of /Ego_topic (latest at or before the scan; heading deg -> rad,
roll = pitch = 0, speed = |velocity|) exactly like the Lidar_Data scenes are fed. GT tables come from
extract_bag_gt.py (same idx / stamp_ns).
usage: bag_to_scene.py BAG OUT_SCENE_DIR
"""
import sys

sys.dont_write_bytecode = True

import bisect, csv, hashlib, json, math, os, sys
import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main(bag_path, out):
    os.makedirs(os.path.join(out, "lidar"), exist_ok=True)
    ego, scans, imu_norms = [], [], []
    with rosbag.Bag(bag_path) as bag:
        for topic, m, t in bag.read_messages(topics=["/Ego_topic", "/velodyne_points", "/imu"]):
            if topic == "/Ego_topic":
                ego.append((m.header.stamp.to_nsec(), m))
            elif topic == "/imu":
                q = m.orientation
                imu_norms.append(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))
            else:
                pts = np.array(list(point_cloud2.read_points(m, field_names=("x", "y", "z", "intensity"), skip_nans=False)),
                               dtype=np.float32).reshape(-1, 4)
                scans.append((m.header.stamp.to_nsec(), pts))
    ego.sort(key=lambda a: a[0]); ek = [e[0] for e in ego]
    rows = []
    for idx, (ns, pts) in enumerate(scans):
        npy = os.path.join(out, "lidar", "bag_%04d.npy" % idx)
        np.save(npy, pts)
        e = ego[bisect.bisect_right(ek, ns) - 1][1]
        v = math.sqrt(e.velocity.x ** 2 + e.velocity.y ** 2 + e.velocity.z ** 2)
        rows.append([idx, "bag_%d" % idx, "%.6f" % (ns * 1e-9), ns, "", npy, "%.6f" % e.position.x, "%.6f" % e.position.y,
                     "%.6f" % e.position.z, "%.9f" % math.radians(e.heading), "%.6f" % v, 0, 0])
    for k in range(1, len(rows)):
        rows[k][4] = "%.6f" % ((rows[k][3] - rows[k - 1][3]) * 1e-9)
    fpath = os.path.join(out, "frames.csv")
    with open(fpath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "stem", "t_lidar", "stamp_ns", "dt_prev_s", "npy", "ego_x", "ego_y", "ego_z", "ego_yaw",
                    "ego_speed_mps", "n_gt", "n_ignore"])
        w.writerows(rows)
    rep = {"bag": bag_path, "bag_sha256": sha(bag_path), "n_scans": len(scans),
           "imu_messages": len(imu_norms), "imu_zero_quaternion": int(sum(1 for n in imu_norms if n < 1e-6)),
           "imu_quaternion_norm_max": max(imu_norms) if imu_norms else None,
           "frames_csv_sha256": sha(fpath), "ego_speed_max": max(float(r[10]) for r in rows)}
    json.dump(rep, open(os.path.join(out, "STATIC_BAG_SCENE.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
