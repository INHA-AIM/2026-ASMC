#!/usr/bin/env python3
"""Log nearest operational-candidate map context at 1 Hz and on demand."""

from __future__ import annotations

from collections import deque
import csv
import json
import math
from pathlib import Path
import threading
import time

from morai_msgs.msg import EgoVehicleStatus, GPSMessage
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String

from asmc_hd_map.live_pose import projected_gps_to_base_link, quaternion_yaw_rad
from asmc_hd_map.review_context import LaneReviewIndex


class MapReviewLoggerNode:
    def __init__(self) -> None:
        map_path = Path(rospy.get_param("~map_path")).resolve()
        with map_path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if bool(document.get("build", {}).get("operational")):
            raise ValueError("review logger expects a non-operational candidate map")
        self.index = LaneReviewIndex(document)
        self.rate_hz = float(rospy.get_param("~log_rate_hz", 1.0))
        if self.rate_hz <= 0.0:
            raise ValueError("log_rate_hz must be positive")
        self.minimum_period = 1.0 / self.rate_hz
        self.last_log = 0.0
        self.latest_context = None
        self.latest_lock = threading.Lock()
        self.output = Path(rospy.get_param(
            "~anchor_output", "/tmp/asmc_hd_map_review_anchors.csv",
        )).resolve()
        self.publisher = rospy.Publisher(
            "/asmc_hd_map/review/context", String, queue_size=1,
        )
        self.capture = rospy.Subscriber(
            "/asmc_hd_map/review/capture_anchor", Empty,
            self._capture_anchor, queue_size=2,
        )
        self.pose_source = str(rospy.get_param("~pose_source", "gps_imu")).lower()
        self.subscribers = []
        if self.pose_source == "gps_imu":
            from pyproj import Transformer
            origin = document.get("frame", {}).get("local_origin_in_global", [])
            if len(origin) != 3:
                raise ValueError("candidate map has no three-element origin")
            self.map_origin_xy = np.asarray(origin[:2], dtype=np.float64)
            self.lever_arm_xy = np.asarray([
                float(rospy.get_param("~gps_lever_arm_x_m", 0.35)),
                float(rospy.get_param("~gps_lever_arm_y_m", 0.0)),
            ])
            self.maximum_gap = float(rospy.get_param("~maximum_gps_imu_gap_ms", 100.0)) / 1000.0
            self.transformer = Transformer.from_crs(
                "EPSG:4326", f"EPSG:{int(rospy.get_param('~utm_epsg', 32652))}", always_xy=True,
            )
            self.imu = deque(maxlen=200)
            self.imu_lock = threading.Lock()
            self.subscribers.extend([
                rospy.Subscriber(str(rospy.get_param("~imu_topic", "/imu")), Imu, self._on_imu, queue_size=10),
                rospy.Subscriber(str(rospy.get_param("~gps_topic", "/gps")), GPSMessage, self._on_gps, queue_size=2),
            ])
        elif self.pose_source == "ego_status":
            self.subscribers.append(rospy.Subscriber(
                str(rospy.get_param("~ego_topic", "/Ego_topic")),
                EgoVehicleStatus, self._on_ego, queue_size=2,
            ))
        else:
            raise ValueError("pose_source must be gps_imu or ego_status")
        rospy.logwarn(
            "HD-map review logger is read-only and candidate-only; capture topic=%s output=%s",
            "/asmc_hd_map/review/capture_anchor", self.output,
        )

    @staticmethod
    def _compact(context: dict) -> str:
        left = context.get("left_boundary") or {}
        right = context.get("right_boundary") or {}
        stop = context.get("next_stopline") or {}
        return (
            "HDMAP_REVIEW "
            f"x={context['map_x_m']:.3f} y={context['map_y_m']:.3f} "
            f"heading={context['heading_deg']:.2f} lane={context['lane_id']} "
            f"lane_s={context['lane_station_m']:.2f} route_s={context.get('route_s_m')} "
            f"lat={context['lateral_offset_m']:.2f} width={context.get('lane_width_m')} "
            f"quality={context.get('geometry_quality')} roles={','.join(context.get('roles', []))} "
            f"left={left.get('color')}/{left.get('pattern')}/{left.get('crossing_rule')} "
            f"right={right.get('color')}/{right.get('pattern')}/{right.get('crossing_rule')} "
            f"next_stop={stop.get('node_id')}:{stop.get('route_distance_m')}"
        )

    def _process_pose(self, x_m: float, y_m: float, heading_deg: float, stamp) -> None:
        now = time.monotonic()
        if now - self.last_log < self.minimum_period:
            return
        context = self.index.query(x_m, y_m, heading_deg)
        if context is None:
            rospy.logwarn_throttle(2.0, "HDMAP_REVIEW no lane within 20m")
            return
        context["stamp_sec"] = stamp.to_sec()
        context["candidate_only"] = True
        payload = json.dumps(context, ensure_ascii=False, sort_keys=True)
        self.publisher.publish(String(data=payload))
        rospy.loginfo(self._compact(context))
        with self.latest_lock:
            self.latest_context = context
        self.last_log = now

    def _capture_anchor(self, _message) -> None:
        with self.latest_lock:
            context = None if self.latest_context is None else dict(self.latest_context)
        if context is None:
            rospy.logwarn("capture_anchor ignored: no pose context yet")
            return
        self.output.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "stamp_sec", "map_x_m", "map_y_m", "heading_deg", "lane_id",
            "lane_station_m", "route_s_m", "lateral_offset_m", "lane_width_m",
            "geometry_quality", "roles", "left_boundary", "right_boundary", "next_stopline",
        ]
        exists = self.output.is_file() and self.output.stat().st_size > 0
        with self.output.open("a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
            if not exists:
                writer.writeheader()
            row = {key: context.get(key) for key in fields}
            for key in ("roles", "left_boundary", "right_boundary", "next_stopline"):
                row[key] = json.dumps(row[key], ensure_ascii=False, sort_keys=True)
            writer.writerow(row)
        rospy.logwarn("HDMAP_REVIEW anchor captured: %s", self.output)

    def _on_ego(self, message) -> None:
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        self._process_pose(message.position.x, message.position.y, message.heading, stamp)

    def _on_imu(self, message) -> None:
        try:
            q = message.orientation
            yaw = quaternion_yaw_rad(q.x, q.y, q.z, q.w)
        except ValueError:
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.imu_lock:
            self.imu.append((stamp.to_sec(), yaw))

    def _on_gps(self, message) -> None:
        if int(message.status) <= 0:
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.imu_lock:
            samples = tuple(self.imu)
        if not samples:
            return
        imu_stamp, yaw = min(samples, key=lambda value: abs(value[0] - stamp.to_sec()))
        if abs(imu_stamp - stamp.to_sec()) > self.maximum_gap:
            rospy.logwarn_throttle(2.0, "HDMAP_REVIEW GPS/IMU gap too large")
            return
        easting, northing = self.transformer.transform(message.longitude, message.latitude)
        base_xy = projected_gps_to_base_link(
            (easting, northing), self.map_origin_xy, yaw, self.lever_arm_xy,
        )
        self._process_pose(float(base_xy[0]), float(base_xy[1]), math.degrees(yaw), stamp)


def main() -> None:
    rospy.init_node("map_review_logger")
    MapReviewLoggerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
