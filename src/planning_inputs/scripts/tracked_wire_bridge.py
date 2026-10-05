#!/usr/bin/env python3
"""Read main's legacy LiDAR wire format without importing its nested Msg package.

The frozen Msg/TrackedObjectArray and asmc_msgs/TrackedObjectArray have identical
ROS1 MD5 and serialized fields. This bridge checks both type and MD5 before
deserializing. A development Msg type with extra motion fields is rejected.
"""
import rospy
from asmc_msgs.msg import TrackedObjectArray


def compatible(header, expected_md5):
    return (header.get('type') in ('Msg/TrackedObjectArray',
                                   'asmc_msgs/TrackedObjectArray')
            and header.get('md5sum') == expected_md5)


class TrackedWireBridge:
    def __init__(self):
        input_topic = rospy.get_param('~input_topic', '/last_lidar')
        output_topic = rospy.get_param('~output_topic',
                                       '/planning/input/tracked_objects')
        if input_topic == output_topic:
            raise ValueError('planning wire bridge input/output must differ')
        self.pub = rospy.Publisher(output_topic, TrackedObjectArray, queue_size=2)
        self.sub = rospy.Subscriber(input_topic, rospy.AnyMsg, self.callback,
                                    queue_size=2)

    def callback(self, wire):
        if not compatible(wire._connection_header, TrackedObjectArray._md5sum):
            rospy.logerr_throttle(1.0, 'Planning rejected incompatible LiDAR type/MD5; rebuild the main sensor workspace')
            return
        message = TrackedObjectArray()
        try:
            message.deserialize(wire._buff)
        except Exception as error:
            rospy.logerr_throttle(1.0, 'Planning LiDAR decode failed: %s', error)
            return
        self.pub.publish(message)


if __name__ == '__main__':
    rospy.init_node('planning_tracked_wire_bridge')
    TrackedWireBridge()
    rospy.spin()
