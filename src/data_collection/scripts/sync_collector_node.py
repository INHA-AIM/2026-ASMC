#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auxiliary ROS snapshot collector (profile-based), not synchronized Capture GT.

latest_timer latches newest messages and snapshots at collect_hz. This does not
provide MORAI external ticks or guarantee camera/GT timestamp pairing. Use the
Capture workflow for supervised RGB/GT and raw rosbag for deployment timing.

Usage:
  export ASMC_DATA=...
  rosrun data_collection sync_collector_node.py _profile:=perception_core
  # or
  roslaunch data_collection sync_collector.launch profile:=dry_run
"""

from __future__ import print_function

import os
import struct
import sys

import rospy
import rospkg

try:
    import numpy as np
except ImportError:
    np = None

from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from morai_msgs.msg import (
    CollisionData,
    EgoVehicleStatus,
    GetTrafficLightStatus,
    GPSMessage,
    ObjectStatusList,
)

from data_collection.profile_loader import load_profile
from data_collection.writer import DatasetWriter, default_asmc_data_root


def _stamp_ns(header):
    if header is None:
        return 0
    return int(header.stamp.secs) * 10**9 + int(header.stamp.nsecs)


def _vec3(v):
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def ego_to_dict(msg):
    d = {
        "unique_id": int(msg.unique_id),
        "acceleration": _vec3(msg.acceleration),
        "position": _vec3(msg.position),
        "velocity": _vec3(msg.velocity),
        "heading": float(msg.heading),
        "accel": float(msg.accel),
        "brake": float(msg.brake),
    }
    # beta_drive
    if hasattr(msg, "wheel_angle"):
        d["wheel_angle"] = float(msg.wheel_angle)
    # 26.R1 legacy (should not appear with beta_drive build)
    if hasattr(msg, "front_steer_angle"):
        d["front_steer_angle"] = float(msg.front_steer_angle)
    if hasattr(msg, "lateral_offset"):
        d["lateral_offset"] = float(msg.lateral_offset)
    return d


def objects_to_dict(msg):
    def one(o):
        return {
            "unique_id": int(o.unique_id),
            "type": int(getattr(o, "type", 0)),
            "name": getattr(o, "name", ""),
            "position": _vec3(o.position),
            "velocity": _vec3(o.velocity),
            "acceleration": _vec3(o.acceleration) if hasattr(o, "acceleration") else None,
            "heading": float(getattr(o, "heading", 0.0)),
            "size": _vec3(o.size) if hasattr(o, "size") else None,
        }

    return {
        "num_of_npcs": int(getattr(msg, "num_of_npcs", len(msg.npc_list))),
        "num_of_pedestrian": int(getattr(msg, "num_of_pedestrian", len(msg.pedestrian_list))),
        "num_of_obstacle": int(getattr(msg, "num_of_obstacle", len(msg.obstacle_list))),
        "npc_list": [one(o) for o in msg.npc_list],
        "pedestrian_list": [one(o) for o in msg.pedestrian_list],
        "obstacle_list": [one(o) for o in msg.obstacle_list],
    }


def tl_to_dict(msg):
    return {
        "trafficLightIndex": str(msg.trafficLightIndex),
        "trafficLightType": int(msg.trafficLightType),
        "trafficLightStatus": int(msg.trafficLightStatus),
    }


def collision_to_dict(msg):
    # CollisionData fields vary; keep a shallow dump of known attrs
    out = {}
    for name in ("global_offset_x", "global_offset_y", "global_offset_z"):
        if hasattr(msg, name):
            out[name] = float(getattr(msg, name))
    if hasattr(msg, "collision_object") and msg.collision_object:
        out["objects"] = []
        for o in msg.collision_object:
            item = {"unique_id": int(getattr(o, "unique_id", -1))}
            if hasattr(o, "position"):
                item["position"] = _vec3(o.position)
            out["objects"].append(item)
    return out


def gps_to_dict(msg):
    return {
        "latitude": float(msg.latitude),
        "longitude": float(msg.longitude),
        "altitude": float(getattr(msg, "altitude", 0.0)),
        "eastOffset": float(getattr(msg, "eastOffset", 0.0)),
        "northOffset": float(getattr(msg, "northOffset", 0.0)),
    }


def imu_to_dict(msg):
    return {
        "orientation": {
            "x": float(msg.orientation.x),
            "y": float(msg.orientation.y),
            "z": float(msg.orientation.z),
            "w": float(msg.orientation.w),
        },
        "angular_velocity": _vec3(msg.angular_velocity),
        "linear_acceleration": _vec3(msg.linear_acceleration),
    }


def pointcloud_to_xyzi_bytes(msg):
    """Pack x,y,z,intensity as float32 little-endian. Returns (bytes, n)."""
    if np is None:
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        buf = bytearray()
        for p in pts:
            x, y, z = p[0], p[1], p[2]
            inten = p[3] if len(p) > 3 else 0.0
            buf.extend(struct.pack("<ffff", float(x), float(y), float(z), float(inten)))
        return bytes(buf), len(pts)
    arr = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
    if not arr:
        return b"", 0
    a = np.asarray(arr, dtype=np.float32)
    if a.ndim == 1:
        a = a.reshape(1, -1)
    if a.shape[1] < 4:
        pad = np.zeros((a.shape[0], 4 - a.shape[1]), dtype=np.float32)
        a = np.concatenate([a, pad], axis=1)
    a = a[:, :4].astype(np.float32, copy=False)
    return a.tobytes(order="C"), int(a.shape[0])


class SyncCollector(object):
    def __init__(self):
        rp = rospkg.RosPack()
        pkg = rp.get_path("data_collection")
        profile_name = rospy.get_param("~profile", "dry_run")
        profile_path = rospy.get_param(
            "~profile_file",
            os.path.join(pkg, "config", "profiles", "{}.yaml".format(profile_name)),
        )
        if not os.path.isfile(profile_path):
            rospy.logfatal("profile not found: %s", profile_path)
            sys.exit(1)

        self.profile = load_profile(profile_path)
        max_frames = int(rospy.get_param("~max_frames", 0))
        if max_frames > 0:
            self.profile["max_frames"] = max_frames
        collect_hz = float(rospy.get_param("~collect_hz", 0))
        if collect_hz > 0:
            self.profile["collect_hz"] = collect_hz
        run_id = rospy.get_param("~run_id", "") or None
        data_root = rospy.get_param("~data_root", "") or default_asmc_data_root()
        asmc = os.environ.get("ASMC") or os.path.dirname(os.path.dirname(pkg))

        self.writer = DatasetWriter(
            self.profile, run_id=run_id, data_root=data_root, asmc_cwd=asmc
        )
        rospy.loginfo(
            "collecting → %s (profile=%s hz=%.1f)",
            self.writer.root,
            self.profile["name"],
            self.profile["collect_hz"],
        )

        self._latest = {}
        self._subs = []
        ch = self.profile["channels"]
        topics = self.profile["topics"]

        def sub(key, msg_type, store_key=None):
            if not ch.get(key):
                return
            sk = store_key or key
            topic = topics[key]
            self._subs.append(
                rospy.Subscriber(
                    topic,
                    msg_type,
                    lambda m, k=sk: self._cb(k, m),
                    queue_size=1,
                )
            )
            rospy.loginfo("subscribe %s → %s", topic, sk)

        sub("ego", EgoVehicleStatus)
        sub("competition_ego", EgoVehicleStatus)
        sub("objects", ObjectStatusList)
        sub("collision", CollisionData)
        sub("tl_status", GetTrafficLightStatus)
        sub("lidar", PointCloud2)
        sub("gps", GPSMessage)
        sub("imu", Imu)
        for cam in ("cam_front", "cam_left", "cam_right", "cam_extra"):
            sub(cam, CompressedImage)

        self._stopping = False
        hz = max(0.1, float(self.profile["collect_hz"]))
        self._timer = rospy.Timer(rospy.Duration(1.0 / hz), self._on_tick)
        rospy.on_shutdown(self._shutdown)

    def _cb(self, key, msg):
        self._latest[key] = msg

    def _on_tick(self, _evt):
        if self._stopping:
            return
        max_frames = int(self.profile.get("max_frames") or 0)
        if max_frames > 0 and self.writer.frame_idx >= max_frames:
            rospy.loginfo("max_frames=%d reached — shutting down", max_frames)
            rospy.signal_shutdown("max_frames")
            return

        ch = self.profile["channels"]
        missing = []
        payload = {"missing": missing, "cams": {}}

        ego = self._latest.get("ego")
        if ch.get("ego"):
            if ego is None:
                if self.profile.get("require_ego", True):
                    self.writer._skipped["no_ego"] += 1
                    return
                missing.append("ego")
            else:
                payload["ego"] = ego_to_dict(ego)
                payload["sim_time"] = float(ego.header.stamp.to_sec()) if ego.header.stamp else rospy.get_time()
                payload["stamp_ns"] = _stamp_ns(ego.header)

        if ch.get("competition_ego"):
            cego = self._latest.get("competition_ego")
            if cego is None:
                missing.append("competition_ego")
            else:
                payload["competition_ego"] = ego_to_dict(cego)

        if payload.get("sim_time") is None:
            payload["sim_time"] = rospy.get_time()
            payload["stamp_ns"] = int(payload["sim_time"] * 1e9)

        for cam in ("cam_front", "cam_left", "cam_right", "cam_extra"):
            if not ch.get(cam):
                continue
            msg = self._latest.get(cam)
            if msg is None:
                missing.append(cam)
            else:
                payload["cams"][cam] = bytes(msg.data)

        if ch.get("lidar"):
            msg = self._latest.get("lidar")
            if msg is None:
                missing.append("lidar")
            else:
                blob, n = pointcloud_to_xyzi_bytes(msg)
                payload["lidar_xyzi"] = blob
                payload["lidar_n"] = n

        if ch.get("objects"):
            msg = self._latest.get("objects")
            if msg is None:
                missing.append("objects")
            else:
                payload["objects"] = objects_to_dict(msg)

        if ch.get("tl_status"):
            msg = self._latest.get("tl_status")
            if msg is None:
                missing.append("tl_status")
            else:
                payload["tl_status"] = tl_to_dict(msg)

        if ch.get("collision"):
            msg = self._latest.get("collision")
            if msg is None:
                missing.append("collision")
            else:
                payload["collision"] = collision_to_dict(msg)

        if ch.get("gps"):
            msg = self._latest.get("gps")
            if msg is None:
                missing.append("gps")
            else:
                payload["gps"] = gps_to_dict(msg)

        if ch.get("imu"):
            msg = self._latest.get("imu")
            if msg is None:
                missing.append("imu")
            else:
                payload["imu"] = imu_to_dict(msg)

        idx = self.writer.write_frame(payload)
        if idx % 20 == 0:
            rospy.loginfo(
                "frame %d missing=%s root=%s",
                idx,
                missing or "-",
                self.writer.root,
            )

    def _shutdown(self):
        self._stopping = True
        self.writer.close()
        rospy.loginfo(
            "closed run %s frames=%d skipped=%s",
            self.writer.run_id,
            self.writer.frame_idx,
            self.writer._skipped,
        )


def main():
    rospy.init_node("sync_collector")
    SyncCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
