#!/usr/bin/env python3
"""Forward typed LiDAR tracks to the standalone Frenet planner over UDP."""

import math
import socket
import struct

import rospy
from Msg.msg import TrackedObjectArray


MAGIC = b"LIDOBJ01"
VERSION = 1
HEADER = struct.Struct("<8sHHQ")
OBJECT = struct.Struct("<iHH7f")
MAX_OBJECTS = 128


def classification_type(name):
    value = name.strip().lower()
    if value in ("person", "pedestrian"):
        return 0
    if value in ("car", "truck", "bus", "vehicle"):
        return 1
    return 2


class TrackedObjectUdpBridge:
    def __init__(self):
        self.destination_ip = rospy.get_param("~destination_ip", "127.0.0.1")
        self.destination_port = int(
            rospy.get_param("~destination_port", 7506))
        self.expected_frame = rospy.get_param("~expected_frame", "velodyne")
        self.max_missed_frames = int(
            rospy.get_param("~max_missed_frames", 2))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.publisher_address = (
            self.destination_ip, self.destination_port)
        self.sent_frames = 0
        self.sent_objects = 0

        rospy.Subscriber(
            "/perception/tracked_objects",
            TrackedObjectArray,
            self.on_objects,
            queue_size=1,
            buff_size=1 << 20,
        )
        rospy.Timer(rospy.Duration(5.0), self.report)
        rospy.loginfo(
            "tracked-object UDP bridge -> %s:%d",
            self.destination_ip,
            self.destination_port,
        )

    def on_objects(self, message):
        if message.header.frame_id != self.expected_frame:
            rospy.logerr_throttle(
                5.0,
                "tracked objects frame '%s' != expected '%s'; frame dropped",
                message.header.frame_id,
                self.expected_frame,
            )
            return

        encoded = []
        for source in message.objects:
            if len(encoded) >= MAX_OBJECTS:
                break
            if source.missed_frames > self.max_missed_frames:
                continue

            q = source.pose.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)
            values = (
                source.pose.position.x,
                source.pose.position.y,
                yaw,
                source.velocity.linear.x,
                source.velocity.linear.y,
                source.size.x,
                source.size.y,
            )
            if not all(math.isfinite(value) for value in values):
                continue
            if source.size.x <= 0.0 or source.size.y <= 0.0:
                continue

            encoded.append(OBJECT.pack(
                int(source.id),
                classification_type(source.classification),
                min(int(source.missed_frames), 0xFFFF),
                *values
            ))

        stamp_ns = message.header.stamp.to_nsec()
        packet = HEADER.pack(
            MAGIC, VERSION, len(encoded), stamp_ns) + b"".join(encoded)
        self.socket.sendto(packet, self.publisher_address)
        self.sent_frames += 1
        self.sent_objects += len(encoded)

    def report(self, _event):
        rospy.loginfo(
            "tracked-object UDP bridge: frames=%d objects=%d",
            self.sent_frames,
            self.sent_objects,
        )


if __name__ == "__main__":
    rospy.init_node("tracked_object_udp_bridge")
    TrackedObjectUdpBridge()
    rospy.spin()
