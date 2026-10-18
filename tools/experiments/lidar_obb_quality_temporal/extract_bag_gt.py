#!/usr/bin/env python3
"""Static GT bag -> frames.csv / gt.csv in the same layout as prepare_inputs.py (read-only on the bag).

Per /velodyne_points message (bag order = harness --bag order, idx 0..N-1, stamp = header stamp):
  latest /Ego_topic and /Object_topic with header stamp <= scan stamp
  body box = R(-ego_heading) * (object.position - ego.position), yaw = object.heading - ego.heading
  class: npc_list -> 0 (vehicle), pedestrian_list -> 1 (pedestrian); size = (x: length, y: width, z: height)
MORAI heading is degrees (ENU, CCW). Velocity GT = 0 (every object of this bag is static; asserted).
usage: extract_bag_gt.py BAG OUT_DIR
"""
import sys

sys.dont_write_bytecode = True

import bisect, csv, hashlib, json, math, os, sys
import rosbag


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main(bag_path, out):
    os.makedirs(out, exist_ok=True)
    ego, obj, scans = [], [], []
    with rosbag.Bag(bag_path) as bag:
        for topic, m, t in bag.read_messages(topics=["/Ego_topic", "/Object_topic", "/velodyne_points"]):
            ns = m.header.stamp.to_nsec()
            if topic == "/Ego_topic":
                ego.append((ns, m))
            elif topic == "/Object_topic":
                obj.append((ns, m))
            else:
                scans.append(ns)
    ego.sort(key=lambda a: a[0]); obj.sort(key=lambda a: a[0])
    ek, ok = [e[0] for e in ego], [o[0] for o in obj]
    fpath, gpath = os.path.join(out, "frames.csv"), os.path.join(out, "gt.csv")
    n_gt = 0
    with open(fpath, "w", newline="") as ff, open(gpath, "w", newline="") as gf:
        fw, gw = csv.writer(ff), csv.writer(gf)
        fw.writerow(["idx", "stem", "t_lidar", "stamp_ns", "dt_prev_s", "npy", "ego_x", "ego_y", "ego_z", "ego_yaw",
                     "ego_speed_mps", "n_gt", "n_ignore"])
        gw.writerow(["idx", "stamp_ns", "kind", "track_id", "class_id", "x", "y", "z", "w", "l", "h", "yaw", "vx", "vy",
                     "gx", "gy", "yaw_global"])
        prev = None
        for idx, ns in enumerate(scans):
            ie, io = bisect.bisect_right(ek, ns) - 1, bisect.bisect_right(ok, ns) - 1
            if ie < 0 or io < 0:
                e = None
            else:
                e, o = ego[ie][1], obj[io][1]
            boxes = []
            if e is not None:
                yaw_e = math.radians(e.heading)
                c, s = math.cos(yaw_e), math.sin(yaw_e)
                for lst, cls in (("npc_list", 0), ("pedestrian_list", 1)):
                    for b in getattr(o, lst):
                        dx, dy = b.position.x - e.position.x, b.position.y - e.position.y
                        x, y = c * dx + s * dy, -s * dx + c * dy
                        # Every object of this bag is static (GT_BAG_ANALYSIS speed_max <= 1.7e-4); the MORAI
                        # velocity unit/axis convention is therefore irrelevant and velocity GT is exactly 0.
                        if math.hypot(b.velocity.x, b.velocity.y) > 0.01:
                            raise SystemExit("object %d is not static" % b.unique_id)
                        vx = vy = 0.0
                        yaw = math.atan2(math.sin(math.radians(b.heading) - yaw_e), math.cos(math.radians(b.heading) - yaw_e))
                        boxes.append([idx, ns, "gt", b.unique_id, cls, "%.4f" % x, "%.4f" % y,
                                      "%.4f" % (b.position.z - e.position.z), "%.4f" % b.size.y, "%.4f" % b.size.x,
                                      "%.4f" % b.size.z, "%.6f" % yaw, "%.4f" % vx, "%.4f" % vy,
                                      "%.4f" % b.position.x, "%.4f" % b.position.y, "%.6f" % math.radians(b.heading)])
            for r in boxes:
                gw.writerow(r)
            n_gt += len(boxes)
            fw.writerow([idx, "bag_%d" % idx, "%.6f" % (ns * 1e-9), ns, "" if prev is None else "%.6f" % ((ns - prev) * 1e-9),
                         "", "%.6f" % (e.position.x if e else float("nan")), "%.6f" % (e.position.y if e else float("nan")),
                         "%.6f" % (e.position.z if e else float("nan")), "%.9f" % (math.radians(e.heading) if e else 0.0),
                         "0.0", len(boxes), 0])
            prev = ns
    man = {"bag": bag_path, "bag_sha256": sha_file(bag_path), "n_scans": len(scans), "n_gt_boxes": n_gt,
           "frames_csv_sha256": sha_file(fpath), "gt_csv_sha256": sha_file(gpath),
           "velocity_note": "velocity GT = 0: every object of this bag is static (asserted |v| <= 0.01 per message)"}
    json.dump(man, open(os.path.join(out, "STATIC_BAG_INPUT.json"), "w"), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
