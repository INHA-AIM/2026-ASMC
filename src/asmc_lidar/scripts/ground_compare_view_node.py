#!/usr/bin/env python3
"""Read-only three-method LiDAR ground comparison on one frozen GT frame.

Display offsets deliberately create three *review columns*, not real vehicle/map
geometry. Never subscribe to these topics for planning or localization.
"""
import json
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


METHODS = (
    ("jaeho", "jaeho_reference", 28.0, "Jaeho reference (RANSAC)"),
    ("patchwork", "pp_h180", 0.0, "Patchwork++"),
    ("linefit", "lf_90x60_gap2", -28.0, "VLP16 linefit"),
)
ACTOR_CLASSES = {86, 125, 135, 145, 155, 165, 175, 185, 118, 164}
COLOR_GROUND = 0x697887
COLOR_NONGROUND = 0xF59E0B
COLOR_DELETED_ACTOR = 0xFF42D0
COLOR_LEFT_ROAD = 0x25D6F2
FIELDS = [PointField("x", 0, PointField.FLOAT32, 1),
          PointField("y", 4, PointField.FLOAT32, 1),
          PointField("z", 8, PointField.FLOAT32, 1),
          PointField("rgb", 12, PointField.FLOAT32, 1)]


def find_row(path, wanted):
    if not path.is_file():
        raise FileNotFoundError(str(path))
    # Check the small identifier before parsing a potentially large mask row.
    with path.open() as stream:
        for line in stream:
            if wanted in line:
                row = json.loads(line)
                if row.get("input_id") == wanted:
                    return row
    raise ValueError("frame {!r} not found in {}".format(wanted, path))


def rotation_matrix(pose):
    roll, pitch, yaw = [np.deg2rad(float(pose[key]))
                        for key in ("roll_deg", "pitch_deg", "yaw_deg")]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]], dtype=np.float32)


def read_frame(root, frame_id, results, patchwork_results, patchwork_key):
    source = find_row(root / "sync/snapshots.jsonl", frame_id)
    current = find_row(results, frame_id)
    patch = find_row(patchwork_results, frame_id) if patchwork_results else current
    if source["input_id"] != current["input_id"] or source["input_id"] != patch["input_id"]:
        raise ValueError("frame identity mismatch")
    for result in (current, patch):
        if result["stamp_ns"] != source["scan_effective_stamp_ns"]:
            raise ValueError("observation stamp mismatch")
    cloud = np.fromfile(root / "sync" / source["cloud_path"], dtype="<f4")
    if cloud.size % 4:
        raise ValueError("invalid XYZI length")
    cloud = cloud.reshape(-1, 4)
    with np.load(root / "sync/labels" / (frame_id + ".npz")) as labels:
        semantic = labels["semantic"].copy()
    keys = {"jaeho": (current, "jaeho_reference"),
            "patchwork": (patch, patchwork_key),
            "linefit": (current, "lf_90x60_gap2")}
    masks = {}
    for name, (result, key) in keys.items():
        if key not in result["methods"]:
            raise ValueError("{} missing method {}".format(name, key))
        mask = np.asarray(result["methods"][key]["mask"], dtype=np.uint8)
        if len(mask) != len(cloud) or not np.isin(mask, [0, 1, 2]).all():
            raise ValueError("{} point/mask mismatch".format(name))
        masks[name] = mask
    if len(semantic) != len(cloud):
        raise ValueError("GT semantic length mismatch")
    reference_roi = masks["jaeho"] != 0
    if any(not np.array_equal(mask != 0, reference_roi) for mask in masks.values()):
        raise ValueError("ROI differs between methods")
    return cloud, semantic, masks, source


def cloud_message(xyz, semantic, mask, offset_y, stamp):
    selected = mask != 0
    points = xyz[selected].copy()
    points[:, 1] += offset_y
    chosen = mask[selected]
    labels = semantic[selected]
    colors = np.full(len(points), COLOR_NONGROUND, dtype=np.uint32)
    colors[chosen == 1] = COLOR_GROUND
    colors[(chosen == 1) & np.isin(labels, tuple(ACTOR_CLASSES))] = COLOR_DELETED_ACTOR
    colors[(chosen == 2) & (labels == 127)] = COLOR_LEFT_ROAD
    rgb = colors.view(np.float32)
    header = Header(stamp=stamp, frame_id="ground_comparison")
    samples = list(zip(points[:, 0], points[:, 1], points[:, 2], rgb))
    return point_cloud2.create_cloud(header, FIELDS, samples)


def marker_array(frame_id, offsets, stamp):
    markers = MarkerArray()
    for i, (name, label, y) in enumerate(offsets):
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id="ground_comparison")
        marker.ns = "ground_methods"
        marker.id = i
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 12.0
        marker.pose.position.y = y
        marker.pose.position.z = 4.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.5
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = "{} | {} | {}".format(label, name, frame_id)
        markers.markers.append(marker)
    return markers


def main():
    rospy.init_node("ground_compare_view")
    root = Path(rospy.get_param("~data_root", "/data/lidar_assessment/20261004_gt_dual_ground"))
    frame_id = rospy.get_param("~frame_id", "sync_001211")
    results = Path(rospy.get_param("~results", str(root / "reports/sync_method_sweep_v3_with_jaeho.jsonl")))
    patchwork_results_value = rospy.get_param("~patchwork_results", "")
    patchwork_results = Path(patchwork_results_value) if patchwork_results_value else None
    patchwork_key = rospy.get_param("~patchwork_key", "pp_h180")
    pose = rospy.get_param("~sensor_pose_base_link")
    cloud, semantic, masks, source = read_frame(root, frame_id, results,
                                                patchwork_results, patchwork_key)
    position = np.array([float(pose["x_m"]), float(pose["y_m"]),
                         float(pose["z_m"])], dtype=np.float32)
    xyz = cloud[:, :3] @ rotation_matrix(pose).T + position
    stamp = rospy.Time.now()
    names = [(name, patchwork_key if name == "patchwork" else label, offset)
             for name, key, offset, label in METHODS]
    pubs = []
    for name, _, offset in names:
        topic = "/lidar_candidate/ground_compare/{}".format(name)
        pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
        pub.publish(cloud_message(xyz, semantic, masks[name], offset, stamp))
        mask = masks[name]
        actor = np.isin(semantic, tuple(ACTOR_CLASSES))
        rospy.loginfo("%s frame=%s ground=%d nonground=%d deleted_actor_points=%d asphalt_left=%d",
                      name, frame_id, int(np.sum(mask == 1)), int(np.sum(mask == 2)),
                      int(np.sum((mask == 1) & actor)),
                      int(np.sum((mask == 2) & (semantic == 127))))
        # Keep latched publishers alive until RViz subscribes.
        pubs.append(pub)
    label_pub = rospy.Publisher("/lidar_candidate/ground_compare/labels", MarkerArray,
                                queue_size=1, latch=True)
    label_pub.publish(marker_array(frame_id, names, stamp))
    rospy.logwarn("review-only display uses artificial lateral offsets; frame=ground_comparison is NOT base_link/map")
    rospy.loginfo("source stamp_ns=%s; ground=gray, nonground=orange, deleted GT actor=magenta, retained GT asphalt=cyan",
                  source["scan_effective_stamp_ns"])
    rospy.spin()


if __name__ == "__main__":
    main()
