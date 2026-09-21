#!/usr/bin/env python3
"""Minimal read-only example for planning/control HD-map consumers."""

from __future__ import annotations

from asmc_msgs.msg import (
    CompetitionCheckpointArray,
    DrivableAreaArray,
    HdMapMeta,
    LaneArray,
    LaneAssociation,
    LaneMarkingArray,
    MissionZoneArray,
)
from nav_msgs.msg import OccupancyGrid
import rospy


class HdMapConsumerExample:
    def __init__(self) -> None:
        self.messages = {}
        self.local_messages = {}
        self.reported_build_id = ""
        self.local_reported = False
        self.last_lane_id = None
        topics = {
            "meta": ("/hd_map/meta", HdMapMeta),
            "markings": ("/hd_map/global/lane_markings", LaneMarkingArray),
            "lanes": ("/hd_map/global/lanes", LaneArray),
            "areas": ("/hd_map/global/drivable_areas", DrivableAreaArray),
            "grid": ("/hd_map/global/drivable_grid", OccupancyGrid),
            "checkpoints": ("/hd_map/global/checkpoints", CompetitionCheckpointArray),
            "missions": ("/hd_map/global/mission_zones", MissionZoneArray),
        }
        self.subscribers = [
            rospy.Subscriber(
                str(rospy.get_param("~" + key + "_topic", topic)),
                message_type,
                self._callback,
                callback_args=key,
                queue_size=1,
            )
            for key, (topic, message_type) in topics.items()
        ]
        self.subscribers.extend([
            rospy.Subscriber(
                "/hd_map/local/lanes", LaneArray, self._local_callback,
                callback_args="lanes", queue_size=1,
            ),
            rospy.Subscriber(
                "/hd_map/local/drivable_grid", OccupancyGrid, self._local_callback,
                callback_args="grid", queue_size=1,
            ),
            rospy.Subscriber(
                "/hd_map/local/current_lane", LaneAssociation,
                self._association_callback, queue_size=1,
            ),
        ])

    def _callback(self, message, key: str) -> None:
        self.messages[key] = message
        if len(self.messages) != 7:
            return
        meta = self.messages["meta"]
        identified = [
            self.messages[key].build_id
            for key in ("markings", "lanes", "areas", "checkpoints", "missions")
        ]
        if not meta.complete or not meta.operational:
            rospy.logerr_throttle(5.0, "HD map is not a complete operational product")
            return
        if any(value != meta.build_id for value in identified):
            rospy.logerr_throttle(5.0, "HD-map topic build IDs do not match")
            return
        global_lane_ids = [value.id for value in self.messages["lanes"].lanes]
        if len(global_lane_ids) != len(set(global_lane_ids)):
            rospy.logerr_throttle(5.0, "global Lane IDs are not unique")
            return
        if meta.build_id == self.reported_build_id:
            return
        grid = self.messages["grid"]
        rospy.loginfo(
            "HD map ready build=%s markings=%d lanes=%d areas=%d checkpoints=%d "
            "missions=%d grid=%dx%d@%.2fm",
            meta.build_id,
            len(self.messages["markings"].markings),
            len(self.messages["lanes"].lanes),
            len(self.messages["areas"].areas),
            len(self.messages["checkpoints"].checkpoints),
            len(self.messages["missions"].zones),
            grid.info.width,
            grid.info.height,
            grid.info.resolution,
        )
        rospy.loginfo(
            "Planner authority is the complete latched global map: index Lane and "
            "DrivableArea geometry once by map position, match current link/route station "
            "against global MissionZone values, and reject provisional zones as mandatory "
            "rules unless planner policy explicitly opts in."
        )
        rospy.loginfo(
            "Union every VALIDATED DrivableArea regardless of source_type; LANE_CLIPPED/HYBRID "
            "and source_dataset are provenance, not traversal cost. The derived OccupancyGrid "
            "already rasterizes that union uniformly as value 0."
        )
        rospy.loginfo(
            "Local lanes/grid are an optional map-frame cache for bounded lookup and "
            "visualization, not a planning horizon; markings, checkpoints, and missions "
            "remain global static metadata."
        )
        self.reported_build_id = meta.build_id
        self._report_local_if_ready()

    def _local_callback(self, message, key: str) -> None:
        self.local_messages[key] = message
        self._report_local_if_ready()

    def _report_local_if_ready(self) -> None:
        if self.local_reported or self.reported_build_id == "":
            return
        if set(self.local_messages) != {"lanes", "grid"}:
            return
        lanes = self.local_messages["lanes"]
        grid = self.local_messages["grid"]
        if lanes.complete:
            rospy.logerr_throttle(5.0, "local LaneArray must be marked complete=false")
            return
        if lanes.build_id != self.reported_build_id:
            rospy.logerr_throttle(5.0, "local/global HD-map build IDs do not match")
            return
        if lanes.header.frame_id != "map" or grid.header.frame_id != "map":
            rospy.logerr_throttle(5.0, "local HD-map geometry must remain in the map frame")
            return
        global_lane_ids = {value.id for value in self.messages["lanes"].lanes}
        local_lane_ids = [value.id for value in lanes.lanes]
        if len(local_lane_ids) != len(set(local_lane_ids)):
            rospy.logerr_throttle(5.0, "local Lane IDs are not unique")
            return
        if not set(local_lane_ids).issubset(global_lane_ids):
            rospy.logerr_throttle(5.0, "local Lane IDs are not a subset of the global map")
            return
        rospy.loginfo(
            "Local HD map ready lanes=%d grid=%dx%d@%.2fm; IDs and geometry stay in "
            "the global map namespace, so consumers must not apply a second base_link transform.",
            len(lanes.lanes), grid.info.width, grid.info.height, grid.info.resolution,
        )
        self.local_reported = True

    def _association_callback(self, message: LaneAssociation) -> None:
        if message.state == LaneAssociation.STATE_STALE:
            rospy.logwarn_throttle(2.0, "current-lane association is stale")
            return
        lane_key = (message.valid, message.lane_id, message.state)
        if lane_key == self.last_lane_id:
            return
        self.last_lane_id = lane_key
        if not message.valid:
            rospy.logwarn(
                "No traversable current lane at rear-axle reference (%.2f, %.2f)",
                message.reference_point.x, message.reference_point.y,
            )
            return
        rospy.loginfo(
            "Current lane link=%s lane=%d scope=%s state=%d lateral=%+.2fm "
            "clearance=%+.2fm confidence=%.2f",
            message.link_id, message.lane_number, message.lane_number_scope_id,
            message.state, message.signed_lateral_offset_m,
            message.polygon_clearance_m, message.confidence,
        )


def main() -> None:
    rospy.init_node("hd_map_consumer_example")
    HdMapConsumerExample()
    rospy.spin()


if __name__ == "__main__":
    main()
