#!/usr/bin/env python3
"""Read-only example: select global layers, index once, query current link."""
from asmc_msgs.msg import (
    CompetitionCheckpointArray, DrivableAreaArray, HdMapMeta, LaneArray,
    LaneAssociation, LaneMarkingArray, MissionZoneArray,
)
import rospy
from asmc_hd_map.semantic_query import SemanticMapIndex


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
            "/hd_map/local/current_lane", LaneAssociation, self._association, queue_size=1,
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

    def _association(self, message):
        index = self.index
        if index is None or not message.valid or message.build_id != index.build_id:
            return
        speed = index.speed_limit(message.link_id)
        missions = index.active_missions(message.link_id)
        rospy.loginfo_throttle(2., "Current lane=%s effective_limit_kph=%s missions=%s",
                              message.lane_id, speed["effective_limit_kph"] if speed else None,
                              [m.id for m in missions])
        # A planner can also call index.spatial((x, y), yaw, layers=["lanes"]),
        # and index.signals_ahead(route_station_m, braking_horizon_m).
        # Estimate route station from the matched lane/route, not nearest Euclidean light.


if __name__ == "__main__":
    rospy.init_node("hd_map_consumer_example")
    HdMapConsumerExample()
    rospy.spin()
