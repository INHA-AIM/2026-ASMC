#!/usr/bin/env python3
"""Optional legacy /YOLO/BBox -> typed camera boxes bridge.

This file imports Msg only when launched. asmc_lidar itself builds and runs
without the nested lidar+sensorfusion workspace. No image or object is
restamped, and an invalid stamp is not relabelled as an empty observation.
"""

import rospy
from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray


def main():
    try:
        from Msg.msg import Yolo_bbox_arr
    except ImportError as exc:
        raise RuntimeError(
            "legacy bridge requires the separate lidar+sensorfusion/Msg overlay; "
            "default asmc_lidar does not require it"
        ) from exc

    rospy.init_node("asmc_lidar_legacy_yolo_bridge")
    publisher = rospy.Publisher("/perception/camera/objects", CameraObjectBoxArray, queue_size=4)
    width = rospy.get_param("~image_width", 1280)
    height = rospy.get_param("~image_height", 720)

    def relay(message):
        output = CameraObjectBoxArray()
        output.header = message.header
        output.image_width = width
        output.image_height = height
        output.valid = not message.header.stamp.is_zero() and message.header.frame_id == "Camera-1"
        for old in message.yolo_bbox_arr:
            box = CameraObjectBox()
            box.xmin, box.ymin = old.xMin, old.yMin
            box.xmax, box.ymax = old.xMax, old.yMax
            box.class_name = old.classes
            box.confidence_valid = False  # Original Msg type discards YOLO confidence.
            output.boxes.append(box)
        publisher.publish(output)

    rospy.Subscriber("/YOLO/BBox", Yolo_bbox_arr, relay, queue_size=4)
    rospy.loginfo("optional legacy YOLO box relay ready; %dx%d Camera-1", width, height)
    rospy.spin()


if __name__ == "__main__":
    main()
