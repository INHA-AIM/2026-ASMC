#!/usr/bin/env python3
"""Publish planner-facing map-frame Lane and OccupancyGrid crops around Ego."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading
import time

from asmc_msgs.msg import HdMapMeta, LaneArray, LaneAssociation, MissionZoneArray
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from asmc_hd_map.msg import HdMapContext, HdMapRoute
from asmc_hd_map.context_publisher import ContextPublisher

from asmc_hd_map.ego_footprint import (
    VehicleFootprint,
    footprint_corners,
    footprint_drivable_summary,
)

from asmc_hd_map.lane_association import (
    LaneAssociator,
    LaneAssociationResult,
    LaneGeometry,
)
from asmc_hd_map.live_pose import (
    projected_gps_to_base_link,
    quaternion_yaw_rad,
    valid_wgs84_position,
)
from asmc_hd_map.local_crop import (
    crop_occupancy_grid,
    crop_ego_occupancy_grid,
    EgoWindow,
    select_ego_bounds,
    geometry_bounds,
    select_intersecting_bounds,
)


class LocalMapCropNode:
    """Derive non-latched local topics without changing global geometry or IDs."""

    def __init__(self) -> None:
        self.extent_m = float(rospy.get_param("~extent_m", 120.0))
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        if not math.isfinite(self.extent_m) or self.extent_m <= 0.0:
            raise ValueError("extent_m must be positive and finite")
        if not math.isfinite(self.publish_hz) or self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive and finite")
        self.crop_mode = str(rospy.get_param("~crop_mode", "ego"))
        if self.crop_mode not in {"ego", "map_square"}:
            raise ValueError("crop_mode must be ego or map_square")
        self.window = EgoWindow(**{
            key: float(rospy.get_param("~" + key, value))
            for key, value in dict(front_m=90., rear_m=30., left_m=30., right_m=30.).items()
        })
        self.minimum_period = 1.0 / self.publish_hz
        self.pose_stale_timeout_sec = float(
            rospy.get_param("~pose_stale_timeout_sec", 0.5)
        )
        if not math.isfinite(self.pose_stale_timeout_sec) or self.pose_stale_timeout_sec <= 0.0:
            raise ValueError("pose_stale_timeout_sec must be positive and finite")
        self.pose_source = str(rospy.get_param("~pose_source", "gps_imu")).strip().lower()
        if self.pose_source not in {"gps_imu", "ego_status", "pose_stamped"}:
            raise ValueError("pose_source must be gps_imu, ego_status, or pose_stamped")

        self.publish_crops = bool(rospy.get_param("~publish_crops", True))
        self.context = ContextPublisher(
            float(rospy.get_param("~boundary_ahead_m", 200.)),
            float(rospy.get_param("~boundary_behind_m", 30.)),
        )
        self.pose_status = HdMapContext.POSE_WAITING
        self.lock = threading.RLock()
        self.meta = None
        self.global_lanes = None
        self.lane_bounds = []
        self.global_grid_message = None
        self.global_occupancy = None
        self.latest_gps_stamp = None
        self.latest_pose = None
        self.latest_pose_received_monotonic = 0.0
        self.lane_associator = None
        self.latest_association = LaneAssociationResult(valid=False, state=0)
        self.last_publish = 0.0
        self.first_publish_logged = False
        # Debug-only ego footprint on the HD map. base_link is the same rear-axle
        # reference used by current-lane association. The map->base_link TF is
        # off by default so it never competes with a runtime localization TF;
        # the debug launch enables it with RViz so the view can follow base_link.
        self.publish_ego_markers = bool(rospy.get_param("~publish_ego_markers", False))
        self.publish_ego_tf = bool(rospy.get_param("~publish_ego_tf", False))
        self.ego_frame_id = str(rospy.get_param("~ego_frame_id", "base_link"))
        self.footprint = VehicleFootprint(
            rear_overhang_m=float(rospy.get_param("~vehicle_rear_overhang_m", 0.79)),
            wheelbase_m=float(rospy.get_param("~vehicle_wheelbase_m", 3.0)),
            front_overhang_m=float(rospy.get_param("~vehicle_front_overhang_m", 0.845)),
            width_m=float(rospy.get_param("~vehicle_width_m", 1.892)),
        )
        self.ego_markers_pub = (
            rospy.Publisher("/hd_map/debug/ego_markers", MarkerArray, queue_size=1)
            if self.publish_ego_markers else None
        )
        self.tf_broadcaster = None
        if self.publish_ego_tf:
            import tf2_ros

            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        if self.pose_source == "gps_imu":
            try:
                from pyproj import Transformer
            except ImportError as error:
                raise RuntimeError("gps_imu pose source requires pyproj") from error
            map_path = Path(str(rospy.get_param("~map_path", ""))).expanduser()
            if not map_path.is_file():
                raise ValueError("gps_imu pose source requires an existing map_path")
            with map_path.open("r", encoding="utf-8") as stream:
                map_document = json.load(stream)
            frame = map_document.get("frame") or {}
            origin = frame.get("local_origin_in_global") or []
            if len(origin) != 3 or not np.isfinite(np.asarray(origin, dtype=np.float64)).all():
                raise ValueError("map_path requires a finite three-element global origin")
            self.map_origin_xy = np.asarray(origin[:2], dtype=np.float64)
            self.gps_lever_arm_xy = np.asarray([
                float(rospy.get_param("~gps_lever_arm_x_m", 0.35)),
                float(rospy.get_param("~gps_lever_arm_y_m", 0.0)),
            ])
            self.maximum_imu_gap_sec = float(
                rospy.get_param("~maximum_gps_imu_gap_ms", 100.0)
            ) / 1000.0
            self.gps_offset_tolerance_m = float(
                rospy.get_param("~gps_offset_tolerance_m", 1.0)
            )
            if self.maximum_imu_gap_sec <= 0.0 or self.gps_offset_tolerance_m < 0.0:
                raise ValueError(
                    "maximum_gps_imu_gap_ms must be positive and gps_offset_tolerance_m non-negative"
                )
            utm_epsg = int(rospy.get_param("~utm_epsg", 32652))
            self.utm_transformer = Transformer.from_crs(
                "EPSG:4326", f"EPSG:{utm_epsg}", always_xy=True
            )
            self.imu_samples = deque(maxlen=200)

        self.lanes_pub = rospy.Publisher(
            "/hd_map/local/lanes", LaneArray, queue_size=1, latch=False
        ) if self.publish_crops else None
        self.grid_pub = rospy.Publisher(
            "/hd_map/local/drivable_grid", numpy_msg(OccupancyGrid), queue_size=1, latch=False
        ) if self.publish_crops else None
        self.association_pub = rospy.Publisher(
            "/hd_map/local/current_lane", LaneAssociation, queue_size=1, latch=False
        )
        self.subscribers = [
            rospy.Subscriber("/hd_map/global/route", HdMapRoute, self._on_route, queue_size=1),
            rospy.Subscriber("/hd_map/global/mission_zones", MissionZoneArray, self._on_missions, queue_size=1),
            rospy.Subscriber("/hd_map/meta", HdMapMeta, self._on_meta, queue_size=1),
            rospy.Subscriber(
                "/hd_map/global/lanes", LaneArray, self._on_global_lanes, queue_size=1
            ),
        ]
        if self.publish_crops or self.publish_ego_markers:
            self.subscribers.append(rospy.Subscriber(
                "/hd_map/global/drivable_grid",
                OccupancyGrid,
                self._on_global_grid,
                queue_size=1, buff_size=32 * 1024 * 1024,
            ))
        if self.pose_source == "gps_imu":
            self.subscribers.extend([
                rospy.Subscriber(
                    str(rospy.get_param("~imu_topic", "/imu")),
                    Imu,
                    self._on_imu,
                    queue_size=10,
                ),
                rospy.Subscriber(
                    str(rospy.get_param("~gps_topic", "/gps")),
                    GPSMessage,
                    self._on_gps,
                    queue_size=2,
                ),
            ])
        elif self.pose_source == "ego_status":
            self.subscribers.append(rospy.Subscriber(
                str(rospy.get_param("~ego_topic", "/Ego_topic")),
                EgoVehicleStatus,
                self._on_ego,
                queue_size=1,
            ))
        else:
            self.subscribers.append(rospy.Subscriber(
                str(rospy.get_param("~pose_topic", "/localization/pose")),
                PoseStamped,
                self._on_pose_stamped,
                queue_size=1,
            ))
        self.publish_timer = rospy.Timer(
            rospy.Duration.from_sec(self.minimum_period), self._on_publish_timer
        )
        rospy.loginfo(
            "HD-map context ready pose_source=%s optional_crop_mode=%s window=%s health=%.1fHz "
            "geometry_frame=map%s",
            self.pose_source,
            self.crop_mode,
            self.window if self.crop_mode == "ego" else self.extent_m,
            self.publish_hz,
            " origin=(%.3f,%.3f)" % tuple(self.map_origin_xy)
            if self.pose_source == "gps_imu" else "",
        )

    def _on_meta(self, message: HdMapMeta) -> None:
        with self.lock:
            self.meta = message
            self._configure_context(meta=message)

    def _configure_context(self, **layers):
        try:
            self.context.configure(**layers)
        except ValueError as error:
            rospy.logerr("HD context rejected map snapshot: %s", error)

    def _on_route(self, message):
        with self.lock:
            self._configure_context(route=message)

    def _on_missions(self, message):
        with self.lock:
            self._configure_context(missions=message)

    def _invalidate_pose(self, status):
        with self.lock:
            self.pose_status = status
            self.context.invalidate()
            if self.lane_associator is not None:
                self.lane_associator.reset()
        self._publish_association()
        self._publish_context()

    def _publish_context(self):
        with self.lock:
            status = self.pose_status
            if self.latest_pose is not None and status == HdMapContext.POSE_VALID and self._pose_is_stale(self.latest_pose[3]):
                status = self.pose_status = HdMapContext.POSE_STALE
            lane_ready = self.meta is not None and self.global_lanes is not None and self._identity_matches(self.meta, self.global_lanes)
            self.context.publish(self.latest_pose, status, self.latest_association, lane_ready)

    @staticmethod
    def _lane_message_bounds(lane):
        polygon = [[point.x, point.y] for point in lane.polygon]
        centerline = [[point.x, point.y] for point in lane.centerline]
        return geometry_bounds((polygon, centerline))

    def _on_global_lanes(self, message: LaneArray) -> None:
        if message.header.frame_id != "map" or not message.complete:
            rospy.logerr("local crop rejected incomplete or non-map global LaneArray")
            return
        bounds = [self._lane_message_bounds(lane) for lane in message.lanes]
        try:
            associator = LaneAssociator(
                [
                    LaneGeometry.from_coordinates(
                        lane_id=lane.id,
                        link_id=lane.link_id,
                        lane_number=lane.lane_number,
                        lane_number_scope_id=lane.lane_number_scope_id,
                        polygon_xy=[(point.x, point.y) for point in lane.polygon],
                        centerline_xy=[(point.x, point.y) for point in lane.centerline],
                        left_adjacent_lane_id=lane.left_adjacent_lane_id,
                        right_adjacent_lane_id=lane.right_adjacent_lane_id,
                        predecessor_lane_ids=tuple(lane.predecessor_lane_ids),
                        successor_lane_ids=tuple(lane.successor_lane_ids),
                        bidirectional=(lane.direction == lane.DIRECTION_BIDIRECTIONAL),
                        route_assigned=(
                            lane.route_station_start_m >= 0.0
                            and lane.route_station_end_m >= 0.0
                        ),
                        intersection=(lane.lane_type == lane.TYPE_INTERSECTION),
                    )
                    for lane in message.lanes
                ],
                switch_penetration_m=float(
                    rospy.get_param("~lane_switch_penetration_m", 0.30)
                ),
                switch_confirmation_samples=int(
                    rospy.get_param("~lane_switch_confirmation_samples", 2)
                ),
                boundary_band_m=float(
                    rospy.get_param("~lane_boundary_band_m", 0.15)
                ),
                nearby_distance_m=float(
                    rospy.get_param("~lane_nearby_distance_m", 2.0)
                ),
                intersection_switch_margin_m=float(
                    rospy.get_param("~lane_intersection_switch_margin_m", 0.75)
                ),
            )
        except ValueError as error:
            rospy.logerr("local crop rejected invalid lane association geometry: %s", error)
            return
        with self.lock:
            self.global_lanes = message
            self.lane_bounds = bounds
            self.lane_associator = associator
            if self.latest_pose is not None:
                x_m, y_m, yaw_rad, _ = self.latest_pose
                self.latest_association = associator.update(x_m, y_m, yaw_rad)

    def _on_global_grid(self, message: OccupancyGrid) -> None:
        if message.header.frame_id != "map":
            rospy.logerr("local crop rejected non-map global OccupancyGrid")
            return
        orientation = message.info.origin.orientation
        if (
            abs(orientation.x) > 1e-9
            or abs(orientation.y) > 1e-9
            or abs(orientation.z) > 1e-9
            or abs(abs(orientation.w) - 1.0) > 1e-9
        ):
            rospy.logerr("local crop requires an axis-aligned global OccupancyGrid")
            return
        expected = int(message.info.width) * int(message.info.height)
        if len(message.data) != expected:
            rospy.logerr("local crop rejected malformed global OccupancyGrid payload")
            return
        occupancy = np.asarray(message.data, dtype=np.int8).reshape(
            int(message.info.height), int(message.info.width)
        )
        with self.lock:
            self.global_grid_message = message
            self.global_occupancy = occupancy

    def _on_ego(self, message: EgoVehicleStatus) -> None:
        x_m = float(message.position.x)
        y_m = float(message.position.y)
        if abs(x_m) <= 1e-6 and abs(y_m) <= 1e-6:
            rospy.logerr_throttle(
                2.0,
                "local HD-map crop rejected /Ego_topic.position=(0,0): use the default "
                "gps_imu source or provide a normalized map-frame pose",
            )
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        self._set_pose(x_m, y_m, math.radians(float(message.heading)), stamp)

    def _on_imu(self, message: Imu) -> None:
        try:
            orientation = message.orientation
            yaw = quaternion_yaw_rad(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "local HD-map crop ignored invalid IMU: %s", error)
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.lock:
            self.imu_samples.append((stamp.to_sec(), yaw))

    def _on_gps(self, message: GPSMessage) -> None:
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        age = (rospy.Time.now() - stamp).to_sec()
        with self.lock:
            if (abs(age) > self.pose_stale_timeout_sec
                    or (self.latest_pose is not None and stamp <= self.latest_pose[3])
                    or (self.latest_gps_stamp is not None and stamp <= self.latest_gps_stamp)):
                return  # Ignore packets older than either recovery or outage.
            self.latest_gps_stamp = stamp
        if int(message.status) <= 0:
            self._invalidate_pose(HdMapContext.POSE_GPS_UNAVAILABLE)
            rospy.logwarn_throttle(
                2.0, "local HD-map crop ignored invalid GPS status=%s", message.status
            )
            return
        if not valid_wgs84_position(message.latitude, message.longitude):
            self._invalidate_pose(HdMapContext.POSE_GPS_UNAVAILABLE)
            rospy.logwarn_throttle(
                2.0,
                "local HD-map crop ignored invalid GPS coordinate latitude=%s longitude=%s",
                message.latitude,
                message.longitude,
            )
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.lock:
            samples = tuple(self.imu_samples)
        if not samples:
            self._invalidate_pose(HdMapContext.POSE_INVALID)
            rospy.logwarn_throttle(2.0, "local HD-map crop is waiting for IMU orientation")
            return
        imu_stamp, yaw = min(samples, key=lambda value: abs(value[0] - stamp.to_sec()))
        gap_sec = abs(imu_stamp - stamp.to_sec())
        if gap_sec > self.maximum_imu_gap_sec:
            self._invalidate_pose(HdMapContext.POSE_INVALID)
            rospy.logwarn_throttle(
                2.0,
                "local HD-map crop GPS/IMU gap %.1fms exceeds %.1fms",
                gap_sec * 1000.0,
                self.maximum_imu_gap_sec * 1000.0,
            )
            return
        raw_offset = np.asarray(
            [float(message.eastOffset), float(message.northOffset)], dtype=np.float64
        )
        offset_delta = float(np.linalg.norm(raw_offset - self.map_origin_xy))
        if not math.isfinite(offset_delta) or offset_delta > self.gps_offset_tolerance_m:
            self._invalidate_pose(HdMapContext.POSE_INVALID)
            rospy.logerr_throttle(
                2.0,
                "local HD-map crop rejected GPS: sensor offset differs from map origin by "
                "%.3fm (sensor=(%.3f,%.3f), map=(%.3f,%.3f))",
                offset_delta,
                raw_offset[0],
                raw_offset[1],
                self.map_origin_xy[0],
                self.map_origin_xy[1],
            )
            return
        try:
            easting, northing = self.utm_transformer.transform(
                float(message.longitude), float(message.latitude)
            )
            base_xy = projected_gps_to_base_link(
                (easting, northing), self.map_origin_xy, yaw, self.gps_lever_arm_xy
            )
        except (ValueError, OverflowError) as error:
            self._invalidate_pose(HdMapContext.POSE_INVALID)
            rospy.logwarn_throttle(2.0, "local HD-map crop GPS conversion failed: %s", error)
            return
        self._set_pose(float(base_xy[0]), float(base_xy[1]), yaw, stamp)

    def _on_pose_stamped(self, message: PoseStamped) -> None:
        if message.header.frame_id != "map":
            rospy.logwarn_throttle(
                2.0, "local HD-map crop ignored PoseStamped outside the map frame"
            )
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        orientation = message.pose.orientation
        try:
            yaw = quaternion_yaw_rad(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        except ValueError as error:
            rospy.logwarn_throttle(
                2.0, "local HD-map crop ignored invalid PoseStamped: %s", error
            )
            return
        self._set_pose(
            float(message.pose.position.x), float(message.pose.position.y), yaw, stamp
        )

    def _set_pose(self, x_m: float, y_m: float, yaw_rad: float, stamp) -> None:
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_rad)):
            rospy.logwarn_throttle(2.0, "local HD-map crop ignored non-finite pose")
            return
        source_age = (rospy.Time.now() - stamp).to_sec()
        if source_age > self.pose_stale_timeout_sec or source_age < -self.pose_stale_timeout_sec:
            rospy.logwarn_throttle(2.0, "local HD-map ignored old/future pose stamp")
            return
        with self.lock:
            if self.latest_pose is not None and stamp <= self.latest_pose[3]:
                return
            recovering = self.latest_pose is None or self._pose_is_stale(self.latest_pose[3])
            if recovering:
                self.context.invalidate()
                if self.lane_associator is not None:
                    self.lane_associator.reset()
            self.pose_status = HdMapContext.POSE_VALID
            self.latest_pose = (x_m, y_m, yaw_rad, stamp)
            self.latest_pose_received_monotonic = time.monotonic()
            if self.lane_associator is not None:
                self.latest_association = self.lane_associator.update(
                    x_m, y_m, yaw_rad
                )
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = self.ego_frame_id
            transform.transform.translation.x = x_m
            transform.transform.translation.y = y_m
            transform.transform.rotation.z = math.sin(0.5 * yaw_rad)
            transform.transform.rotation.w = math.cos(0.5 * yaw_rad)
            self.tf_broadcaster.sendTransform(transform)
        self._publish_association()
        self._publish_context()

    def _on_publish_timer(self, _event) -> None:
        self._publish_context()
        self._publish_association()
        self._try_publish(force=True)
        self._publish_ego_markers()

    def _publish_ego_markers(self) -> None:
        if self.ego_markers_pub is None or self.ego_markers_pub.get_num_connections() == 0:
            return
        with self.lock:
            if self.latest_pose is None:
                return
            x_m, y_m, yaw_rad, _stamp = self.latest_pose
            age_sec = time.monotonic() - self.latest_pose_received_monotonic
            stale = self._pose_is_stale(_stamp)
            association = self.latest_association
            grid = self.global_grid_message
            occupancy = self.global_occupancy
        summary = None
        if grid is not None and occupancy is not None:
            summary = footprint_drivable_summary(
                x_m, y_m, yaw_rad, occupancy, float(grid.info.resolution),
                (grid.info.origin.position.x, grid.info.origin.position.y), self.footprint,
            )
        self.ego_markers_pub.publish(
            self._ego_markers(x_m, y_m, yaw_rad, stale, age_sec, association, summary)
        )

    def _ego_markers(self, x_m, y_m, yaw_rad, stale, age_sec, association, summary) -> MarkerArray:
        def marker(namespace: str, marker_type: int) -> Marker:
            value = Marker()
            value.header.frame_id = "map"
            value.ns = namespace
            value.type = marker_type
            value.action = Marker.ADD
            value.pose.orientation.w = 1.0
            return value

        def color(r: float, g: float, b: float, a: float) -> ColorRGBA:
            return ColorRGBA(r=r, g=g, b=b, a=a)

        def point(x: float, y: float, z: float) -> Point:
            return Point(x=x, y=y, z=z)

        outside = float(summary["outside_fraction"]) if summary else 0.0
        if stale:
            fill_color = color(0.55, 0.55, 0.55, 0.40)
        elif outside <= 0.0:
            fill_color = color(0.15, 0.9, 0.3, 0.45)
        elif outside < 0.10:
            fill_color = color(1.0, 0.85, 0.1, 0.55)
        else:
            fill_color = color(1.0, 0.2, 0.15, 0.6)
        z = 0.8
        offset = self.footprint.center_offset_m
        body = marker("ego_footprint_fill", Marker.CUBE)
        body.pose.position = point(
            x_m + math.cos(yaw_rad) * offset, y_m + math.sin(yaw_rad) * offset, z
        )
        body.pose.orientation.z = math.sin(0.5 * yaw_rad)
        body.pose.orientation.w = math.cos(0.5 * yaw_rad)
        body.scale.x, body.scale.y, body.scale.z = self.footprint.length_m, self.footprint.width_m, 0.05
        body.color = fill_color

        corners = footprint_corners(x_m, y_m, yaw_rad, self.footprint)
        outline = marker("ego_footprint_outline", Marker.LINE_STRIP)
        outline.scale.x = 0.08
        outline.color = color(1.0, 1.0, 1.0, 0.95)
        outline.points = [point(cx, cy, z + 0.03) for cx, cy in corners + corners[:1]]

        corner_marks = marker("ego_footprint_corners", Marker.SPHERE_LIST)
        corner_marks.scale.x = corner_marks.scale.y = corner_marks.scale.z = 0.35
        corner_marks.points = [point(cx, cy, z + 0.05) for cx, cy in corners]
        flags = summary["corner_drivable"] if summary else [True] * 4
        corner_marks.colors = [
            color(0.15, 1.0, 0.3, 1.0) if ok else color(1.0, 0.1, 0.1, 1.0) for ok in flags
        ]

        heading = marker("ego_heading", Marker.ARROW)
        heading.scale.x, heading.scale.y, heading.scale.z = 0.18, 0.45, 0.6
        heading.color = color(1.0, 1.0, 1.0, 0.95)
        nose = self.footprint.wheelbase_m + self.footprint.front_overhang_m + 1.5
        heading.points = [
            point(x_m, y_m, z + 0.06),
            point(x_m + math.cos(yaw_rad) * nose, y_m + math.sin(yaw_rad) * nose, z + 0.06),
        ]

        base_link = marker("ego_base_link", Marker.SPHERE)
        base_link.pose.position = point(x_m, y_m, z + 0.08)
        base_link.scale.x = base_link.scale.y = base_link.scale.z = 0.4
        base_link.color = color(0.2, 0.6, 1.0, 1.0)

        state_names = {
            LaneAssociation.STATE_INSIDE: "INSIDE",
            LaneAssociation.STATE_BOUNDARY: "BOUNDARY",
            LaneAssociation.STATE_NEARBY: "NEARBY",
            LaneAssociation.STATE_STALE: "STALE",
        }
        lane_text = (
            "lane %s %s lat %+.2f m" % (
                association.link_id[-4:] if association.link_id else "-",
                "STALE (last)" if stale else state_names.get(association.state, "UNKNOWN"),
                association.signed_lateral_offset_m,
            )
            if association.valid else "lane -"
        )
        text = marker("ego_status_text", Marker.TEXT_VIEW_FACING)
        text.pose.position = point(x_m, y_m, z + 3.0)
        text.scale.z = 0.9
        text.color = color(1.0, 1.0, 1.0, 1.0)
        text.text = "\n".join([
            "GPS STALE %.1f s (last pose)" % age_sec if stale else "base_link %.1f, %.1f  yaw %.0f°" % (
                x_m, y_m, math.degrees(yaw_rad)
            ),
            lane_text,
            "outside drivable %.0f%%" % (100.0 * outside) if summary else "drivable grid pending",
        ])
        output = MarkerArray()
        output.markers = [body, outline, corner_marks, heading, base_link, text]
        return output

    @staticmethod
    def _identity_matches(meta: HdMapMeta, lanes: LaneArray) -> bool:
        return (
            meta.complete
            and meta.operational
            and lanes.build_id == meta.build_id
            and lanes.map_id == meta.map_id
            and lanes.geometry_sha256 == meta.geometry_sha256
        )

    def _publish_association(self):
        # Lane output follows every distinct pose, independent of raster work.
        with self.lock:
            if self.meta is None or self.global_lanes is None or self.latest_pose is None:
                return
            lanes = self.global_lanes
            if not self._identity_matches(self.meta, lanes):
                return
            x_m, y_m, _yaw, stamp = self.latest_pose
            association = self.latest_association
            pose_stale = self._pose_is_stale(stamp)
        association_output = LaneAssociation()
        association_output.header.stamp = stamp
        association_output.header.frame_id = "map"
        association_output.build_id = lanes.build_id
        association_output.map_id = lanes.map_id
        association_output.geometry_sha256 = lanes.geometry_sha256
        association_output.valid = bool(association.valid and not pose_stale)
        association_output.state = (
            LaneAssociation.STATE_STALE if pose_stale else association.state
        )
        association_output.reference_type = (
            LaneAssociation.REFERENCE_BASE_LINK_REAR_AXLE
        )
        association_output.reference_point.x = x_m
        association_output.reference_point.y = y_m
        association_output.reference_point.z = 0.0
        association_output.lane_id = association.lane_id
        association_output.link_id = association.link_id
        association_output.lane_number = association.lane_number
        association_output.lane_number_scope_id = association.lane_number_scope_id
        association_output.centerline_distance_m = association.centerline_distance_m
        association_output.signed_lateral_offset_m = association.signed_lateral_offset_m
        association_output.polygon_clearance_m = association.polygon_clearance_m
        association_output.heading_error_rad = association.heading_error_rad
        association_output.confidence = 0.0 if pose_stale else association.confidence
        association_output.previous_lane_id = association.previous_lane_id
        association_output.transition_pending = association.transition_pending
        association_output.transition_candidate_lane_id = (
            association.transition_candidate_lane_id
        )
        association_output.transition_candidate_samples = min(
            255, association.transition_candidate_samples
        )
        self.association_pub.publish(association_output)

    def _pose_is_stale(self, stamp):
        return (
            self.pose_status != HdMapContext.POSE_VALID
            or time.monotonic() - self.latest_pose_received_monotonic > self.pose_stale_timeout_sec
            or (rospy.Time.now() - stamp).to_sec() > self.pose_stale_timeout_sec
        )

    def _try_publish(self, force: bool) -> None:
        if not self.publish_crops:
            return
        now = time.monotonic()
        with self.lock:
            if not force and now - self.last_publish < self.minimum_period:
                return
            if (
                self.meta is None
                or self.global_lanes is None
                or self.global_grid_message is None
                or self.global_occupancy is None
                or self.latest_pose is None
            ):
                return
            meta = self.meta
            lanes = self.global_lanes
            grid = self.global_grid_message
            occupancy = self.global_occupancy
            lane_bounds = list(self.lane_bounds)
            x_m, y_m, yaw_rad, stamp = self.latest_pose
            pose_stale = self._pose_is_stale(stamp)
            if not self._identity_matches(meta, lanes):
                rospy.logerr_throttle(2.0, "local HD-map crop detected global build identity drift")
                return

        if pose_stale:
            return  # Do not refresh a raster using an expired pose.
        need_lanes = self.lanes_pub.get_num_connections() > 0
        need_grid = self.grid_pub.get_num_connections() > 0
        if not need_lanes and not need_grid:
            return
        if self.crop_mode == "ego":
            selected_indices = select_ego_bounds(lane_bounds, (x_m, y_m), yaw_rad, self.window) if need_lanes else []
            cropped = crop_ego_occupancy_grid(
                occupancy, float(grid.info.resolution),
                (grid.info.origin.position.x, grid.info.origin.position.y),
                (x_m, y_m), yaw_rad, self.window,
            ) if need_grid else None
        else:
            selected_indices = select_intersecting_bounds(lane_bounds, (x_m, y_m), self.extent_m) if need_lanes else []
            cropped = crop_occupancy_grid(
                occupancy, float(grid.info.resolution),
                (grid.info.origin.position.x, grid.info.origin.position.y),
                (x_m, y_m), self.extent_m,
            ) if need_grid else None
        lane_output = LaneArray()
        lane_output.header.stamp = stamp
        lane_output.header.frame_id = "map"
        lane_output.build_id = lanes.build_id
        lane_output.map_id = lanes.map_id
        lane_output.geometry_sha256 = lanes.geometry_sha256
        lane_output.complete = False
        lane_output.lanes = [lanes.lanes[index] for index in selected_indices]

        grid_output = None
        if cropped is not None:
            grid_output = numpy_msg(OccupancyGrid)()
            grid_output.header.stamp = stamp
            grid_output.header.frame_id = "map"
            grid_output.info.map_load_time = grid.info.map_load_time
            grid_output.info.resolution = cropped.resolution
            grid_output.info.width = cropped.width
            grid_output.info.height = cropped.height
            grid_output.info.origin.position.x = cropped.origin_x
            grid_output.info.origin.position.y = cropped.origin_y
            grid_output.info.origin.position.z = grid.info.origin.position.z
            grid_yaw = yaw_rad if self.crop_mode == "ego" else 0.0
            grid_output.info.origin.orientation.z = math.sin(.5 * grid_yaw)
            grid_output.info.origin.orientation.w = math.cos(.5 * grid_yaw)
            grid_output.data = cropped.data.reshape(-1)

        self.last_publish = now
        if need_lanes:
            self.lanes_pub.publish(lane_output)
        if grid_output is not None:
            self.grid_pub.publish(grid_output)
        if not self.first_publish_logged and grid_output is not None:
            rospy.loginfo(
                "local HD-map crop published x=%.2f y=%.2f lanes=%d grid=%dx%d@%.2fm "
                "build=%s",
                x_m,
                y_m,
                len(lane_output.lanes),
                grid_output.info.width,
                grid_output.info.height,
                grid_output.info.resolution,
                lane_output.build_id,
            )
            self.first_publish_logged = True


def main() -> None:
    rospy.init_node("local_map_crop")
    try:
        LocalMapCropNode()
    except (RuntimeError, TypeError, ValueError) as error:
        rospy.logfatal("local HD-map crop refused invalid configuration: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
