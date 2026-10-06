#!/usr/bin/env python3
"""Read-only example: select global layers, index once, query official-route context."""
from asmc_msgs.msg import (
    CompetitionCheckpointArray, DrivableAreaArray, HdMapMeta, LaneArray,
    LaneMarkingArray, MissionZone, MissionZoneArray,
)
import rospy
from asmc_hd_map.semantic_query import SemanticMapIndex
from asmc_hd_map.msg import HdMapContext


class HdMapConsumerExample:
    def __init__(self):
        self.messages = {}
        self.index = None
        self.meta = None
        self.topics = {
            "markings": ("/hd_map/global/lane_markings", LaneMarkingArray),
            "lanes": ("/hd_map/global/lanes", LaneArray),
            "areas": ("/hd_map/global/drivable_areas", DrivableAreaArray),
            "checkpoints": ("/hd_map/global/checkpoints", CompetitionCheckpointArray),
            "missions": ("/hd_map/global/mission_zones", MissionZoneArray),
        }
        self.layers = rospy.get_param("~layers", ["lanes", "missions", "markings"])
        if not isinstance(self.layers, list) or not self.layers or any(k not in self.topics for k in self.layers):
            raise ValueError("layers must be a nonempty list of supported semantic layers")
        self.subscribers = [rospy.Subscriber("/hd_map/meta", HdMapMeta, self._meta, queue_size=1)]
        for key in self.layers:
            topic, message_type = self.topics[key]
            self.subscribers.append(rospy.Subscriber(
                topic, message_type, self._layer, callback_args=key, queue_size=1,
                buff_size=16 * 1024 * 1024,
            ))
        self.subscribers.append(rospy.Subscriber(
            "/hd_map/context", HdMapContext, self._context, queue_size=1,
        ))

    def _meta(self, message):
        self.meta = message
        self._rebuild()

    def _layer(self, message, key):
        self.messages[key] = message
        self._rebuild()

    def _rebuild(self):
        self.index = None
        if self.meta is None or any(k not in self.messages for k in self.layers):
            return
        try:
            self.index = SemanticMapIndex(self.meta, {k: self.messages[k] for k in self.layers})
        except ValueError as error:
            rospy.logwarn("HD-map snapshot pending: %s", error)
            return
        rospy.loginfo("HD-map selectable cache ready build=%s layers=%s", self.index.build_id, self.layers)

    def _context(self, message):
        index = self.index
        if index is None or message.build_id != index.build_id:
            return
        if not message.pose_valid or not message.route_valid:
            rospy.loginfo_throttle(2., "HD pose unavailable status=%d last_shadow_ids=%s",
                                  message.pose_status, message.gps_shadow_zone_ids)
            return
        missions = index.missions_at_station(message.route_station_m)
        signals = [b for b in message.boundaries if b.boundary_type == b.POINT
                   and b.mission_type == MissionZone.TYPE_TRAFFIC_LIGHT and b.distance_m >= 0]
        rospy.loginfo_throttle(2., "lane=%s official_s=%.1f speed_mps=%s missions=%s signals=%d",
                              message.lane_id, message.route_station_m,
                              message.route_speed_limit_mps if message.has_route_speed_limit else None,
                              [m.id for m in missions], len(signals))
        # Local geometry is an optional query over the full cached vectors:
        # index.spatial((x, y), yaw, layers=["lanes"]). No raster subscription.
        # Reproject message.boundaries[].point onto the planner's own reference
        # if its smoothed/resampled station namespace differs from official_s.


if __name__ == "__main__":
    rospy.init_node("hd_map_consumer_example")
    HdMapConsumerExample()
    rospy.spin()
