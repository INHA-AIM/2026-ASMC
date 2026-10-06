"""Small ROS context stream over immutable, selectable global map layers."""
import math

import rospy
from geometry_msgs.msg import Point
from asmc_msgs.msg import MissionZone
from asmc_hd_map.msg import HdMapContext, MissionBoundary
from asmc_hd_map.route_reference import RouteReference
from asmc_hd_map.semantic_query import SemanticMapIndex


class ContextPublisher:
    def __init__(self, ahead_m=200., behind_m=30.):
        if not all(math.isfinite(v) and v >= 0 for v in (ahead_m, behind_m)):
            raise ValueError("context boundary distances must be finite and nonnegative")
        self.ahead_m, self.behind_m = ahead_m, behind_m
        self.pub = rospy.Publisher('/hd_map/context', HdMapContext, queue_size=1)
        self.meta = self.route = self.missions = self.reference = self.index = None
        self.hint = None
        self.last_valid = None
        self.snapshot_identity = None

    def configure(self, meta=None, route=None, missions=None):
        if meta is not None:
            self.meta = meta
        if route is not None:
            self.route = route
        if missions is not None:
            self.missions = missions
        self.reference = self.index = None
        if any(v is None for v in (self.meta, self.route, self.missions)):
            return
        m, r = self.meta, self.route
        sources = dict(zip(m.source_ids, m.source_sha256))
        identity = (m.build_id, m.map_id, m.geometry_sha256, tuple(m.source_sha256),
                    r.route_id, r.route_sha256, self.missions.source_sha256)
        if identity != self.snapshot_identity:
            self.hint = self.last_valid = None
        if (r.header.frame_id != 'map' or r.build_id != m.build_id
                or r.map_id != m.map_id or r.geometry_sha256 != m.geometry_sha256
                or r.route_sha256 != sources.get("official_global_path")
                or self.missions.source_sha256 != sources.get("competition_metadata")
                or self.missions.route_id != r.route_id):
            raise ValueError('mixed official route identity or frame')
        reference = RouteReference([(p.x, p.y, p.z) for p in r.points], r.stations_m)
        if abs(reference.length_m - r.length_m) > 1e-6:
            raise ValueError('route length differs from official stations')
        index = SemanticMapIndex(m, {'missions': self.missions})
        for zone in self.missions.zones:
            if not 0 <= zone.route_station_start_m <= zone.route_station_end_m <= r.length_m + 1e-6:
                raise ValueError('mission stations outside official route')
        self.snapshot_identity = identity
        self.reference, self.index = reference, index

    def invalidate(self):
        self.hint = None  # Recovery must reacquire, rather than continue old progress.

    def publish(self, pose, status, association, lane_identity_valid):
        out = HdMapContext()
        out.header.frame_id = 'map'
        out.header.stamp = rospy.Time.now()
        out.pose_status = status
        out.reference_type = out.REFERENCE_BASE_LINK_REAR_AXLE
        out.route_station_m = out.route_distance_m = -1.
        out.last_valid_route_station_m = -1.
        if self.meta is not None:
            out.build_id, out.map_id = self.meta.build_id, self.meta.map_id
        if self.reference is not None:
            out.route_id, out.route_sha256 = self.route.route_id, self.route.route_sha256
        out.pose_valid = status == out.POSE_VALID
        if pose is not None:
            x, y, yaw, stamp = pose
            out.pose_stamp = stamp
            if out.pose_valid:
                out.ego_pose.position.x, out.ego_pose.position.y = x, y
                out.ego_pose.orientation.z = math.sin(.5 * yaw)
                out.ego_pose.orientation.w = math.cos(.5 * yaw)
                out.lane_valid = lane_identity_valid and association.valid
                if out.lane_valid:
                    out.lane_id, out.link_id = association.lane_id, association.link_id
                    out.lane_number = association.lane_number
                if self.reference is not None:
                    projection = self.reference.project((x, y), hint_m=self.hint)
                    if projection is not None:
                        self.hint = projection.station_m
                        out.route_valid = True
                        out.route_station_m, out.route_distance_m = projection.station_m, projection.distance_m
                        active = self.index.missions_at_station(projection.station_m)
                        out.active_mission_ids = [z.id for z in active]
                        limit = self.index.speed_at_station(projection.station_m)
                        if limit is not None:
                            out.has_route_speed_limit = True
                            out.route_speed_limit_mps = limit['effective_limit_kph'] / 3.6
                        for boundary in self.index.mission_boundaries(
                                projection.station_m, self.ahead_m, self.behind_m,
                                self.reference.length_m if self.route.closed_loop else None):
                            b = MissionBoundary()
                            b.mission_id, b.mission_type = boundary['mission'].id, boundary['mission'].mission_type
                            b.boundary_type = {'enter': b.ENTER, 'exit': b.EXIT, 'point': b.POINT}[boundary['boundary_type']]
                            b.link_id = boundary['link_id']
                            b.route_station_m, b.distance_m = boundary['route_station_m'], boundary['distance_m']
                            b.point = Point(*self.reference.point_at(min(b.route_station_m, self.reference.length_m)))
                            out.boundaries.append(b)
                        out.gps_shadow_zone_ids = [z.id for z in active if z.mission_type == MissionZone.TYPE_GPS_SHADOW]
                        self.last_valid = (stamp, projection.station_m, list(out.active_mission_ids), list(out.gps_shadow_zone_ids))
        if not out.pose_valid:
            self.invalidate()
        if self.last_valid is not None:
            out.last_valid_context_available = True
            out.last_valid_pose_stamp, out.last_valid_route_station_m, out.last_valid_mission_ids, shadow = self.last_valid
            if not out.pose_valid:
                out.gps_shadow_zone_ids = shadow
                out.gps_shadow_context_from_last_valid_pose = True
        self.pub.publish(out)
        return out
