#!/usr/bin/env python3
"""Adapt MORAI simulator objects to the ASMC perception contract."""

import math

import rospy
from asmc_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Point
from morai_msgs.msg import ObjectStatus, ObjectStatusList


class MoraiObjectsAdapter:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param("~output_topic", "/perception/objects")
        self._publisher = rospy.Publisher(
            output_topic, DetectedObjectArray, queue_size=1
        )
        self._subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self._callback, queue_size=1
        )
        rospy.loginfo(
            "MORAI objects adapter: %s -> %s", input_topic, output_topic
        )

    @staticmethod
    def _positive_or(value, fallback):
        return value if math.isfinite(value) and value > 0.0 else fallback

    @classmethod
    def _convert_object(cls, source: ObjectStatus) -> DetectedObject:
        target = DetectedObject()
        target.id = source.unique_id
        target.type = max(0, min(255, source.type))
        target.center.x = source.position.x
        target.center.y = source.position.y
        target.center.z = source.position.z
        target.yaw = math.radians(source.heading)
        # MORAI reports ObjectStatus velocity in km/h in the object's local
        # frame. The ASMC contract expects map-frame components in m/s.
        local_vx = source.velocity.x / 3.6
        local_vy = source.velocity.y / 3.6
        cosine = math.cos(target.yaw)
        sine = math.sin(target.yaw)
        target.vx = cosine * local_vx - sine * local_vy
        target.vy = sine * local_vx + cosine * local_vy
        target.yaw_rate = 0.0
        target.length = cls._positive_or(source.size.x, 4.5)
        target.width = cls._positive_or(source.size.y, 1.8)
        target.height = cls._positive_or(source.size.z, 1.5)

        half_length = 0.5 * target.length
        half_width = 0.5 * target.width
        # Consistent winding for the lower face, followed by the upper face.
        local_corners = (
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
            (half_length, -half_width),
        )
        lower_z = source.position.z
        upper_z = lower_z + target.height
        corners = []
        for z in (lower_z, upper_z):
            for local_x, local_y in local_corners:
                corners.append(
                    Point(
                        x=target.center.x + cosine * local_x - sine * local_y,
                        y=target.center.y + sine * local_x + cosine * local_y,
                        z=z,
                    )
                )
        target.obb_corners = corners
        return target

    def _callback(self, message: ObjectStatusList):
        converted = DetectedObjectArray()
        converted.header.stamp = rospy.Time.now()
        converted.header.frame_id = "map"

        seen_ids = set()
        for source in (
            list(message.npc_list)
            + list(message.pedestrian_list)
            + list(message.obstacle_list)
        ):
            if source.unique_id in seen_ids:
                rospy.logwarn_throttle(
                    2.0,
                    "Ignoring duplicate MORAI object id=%d",
                    source.unique_id,
                )
                continue
            seen_ids.add(source.unique_id)
            converted.objects.append(self._convert_object(source))

        self._publisher.publish(converted)


if __name__ == "__main__":
    rospy.init_node("morai_objects_adapter")
    MoraiObjectsAdapter()
    rospy.spin()
