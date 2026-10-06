#!/usr/bin/env python3
"""Adapt front-camera candidates to the unchanged Main YOLO wire format."""
import copy
import math

import rospy
from asmc_msgs.msg import CameraObjectBoxArray
from Msg.msg import Yolo_bbox, Yolo_bbox_arr


EXPECTED_MD5 = "310c671afb3693e7909db6453d115b79"
ALLOWED_NAMES = ("bus", "truck", "person", "car")


def check_message_build(actual=None):
    actual = CameraObjectBoxArray._md5sum if actual is None else actual
    if actual != EXPECTED_MD5:
        raise RuntimeError("stale outer asmc_msgs: CameraObjectBoxArray MD5=%s expected=%s; rebuild and source outer workspace"
                           % (actual, EXPECTED_MD5))


def convert(message, allowed_names=ALLOWED_NAMES, conf_thres=0.7):
    if (not message.valid or message.header.frame_id != "camera_front"
            or message.header.stamp.is_zero() or not message.image_width or not message.image_height):
        return None
    allowed = {str(name).strip().lower() for name in allowed_names}
    output = Yolo_bbox_arr(header=copy.deepcopy(message.header))
    for box in message.boxes:
        coordinates = (box.xmin, box.ymin, box.xmax, box.ymax)
        if (not box.confidence_valid or not math.isfinite(box.confidence) or box.confidence < conf_thres
                or box.source_class_name.lower() not in allowed or not all(math.isfinite(v) for v in coordinates)):
            continue
        x1, y1, x2, y2 = map(int, coordinates)
        output.yolo_bbox_arr.append(Yolo_bbox(xMin=x1, yMin=y1, xMax=x2, yMax=y2,
                                             classes=box.source_class_name))
    output.objc = len(output.yolo_bbox_arr)
    return output


def main():
    rospy.init_node("camera_objects_yolo_bbox_adapter")
    try:
        check_message_build()
    except RuntimeError as exc:
        rospy.logfatal(str(exc))
        raise SystemExit(1)
    allowed = rospy.get_param("~allowed_names", list(ALLOWED_NAMES))
    threshold = float(rospy.get_param("~conf_thres", 0.7))
    publisher = rospy.Publisher("/YOLO/BBox", Yolo_bbox_arr, queue_size=5)

    def callback(message):
        result = convert(message, allowed, threshold)
        if result is not None:
            publisher.publish(result)
        else:
            rospy.logwarn_throttle(5, "camera object adapter withheld invalid/non-front input")

    rospy.Subscriber("/perception/camera/objects", CameraObjectBoxArray, callback, queue_size=5)
    rospy.loginfo("camera objects adapter ready: input MD5=%s", EXPECTED_MD5)
    rospy.spin()


if __name__ == "__main__":
    main()
