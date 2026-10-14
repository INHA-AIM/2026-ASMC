#!/usr/bin/env python3
"""Adapt front-camera candidates to the unchanged Main YOLO wire format."""
import copy
import math

import rospy
from asmc_msgs.msg import CameraObjectBoxArray
from Msg.msg import Yolo_bbox, Yolo_bbox_arr


EXPECTED_MD5 = "310c671afb3693e7909db6453d115b79"
ALLOWED_NAMES = ("person", "dog", "bicycle", "car", "motorcycle", "bus", "truck")


def validate_threshold(value):
    if isinstance(value, bool):
        raise ValueError('adapter confidence must be finite and in [0,1]')
    try:
        threshold = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('adapter confidence must be finite and in [0,1]') from exc
    if not math.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError('adapter confidence must be finite and in [0,1]')
    return threshold


def check_message_build(actual=None):
    actual = CameraObjectBoxArray._md5sum if actual is None else actual
    if actual != EXPECTED_MD5:
        raise RuntimeError("stale outer asmc_msgs: CameraObjectBoxArray MD5=%s expected=%s; rebuild and source outer workspace"
                           % (actual, EXPECTED_MD5))


def convert(message, allowed_names=ALLOWED_NAMES, conf_thres=0.7):
    conf_thres = validate_threshold(conf_thres)
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
    profile = rospy.get_param("~class_profile", "mission16_v1")
    if profile not in ('legacy', 'mission16_v1'):
        raise ValueError('unsupported camera class profile')
    if profile == 'mission16_v1':
        from asmc_camera.detection.mission import CLASSES
        defaults = list(CLASSES)
    else:
        defaults = list(ALLOWED_NAMES)
    allowed = rospy.get_param("~allowed_names", defaults)
    threshold = validate_threshold(rospy.get_param("~conf_thres", 0.0))
    # Cache subscription avoids a master RPC on every camera callback. This is
    # an additional consumer cutoff; it cannot recover boxes cut by the producer.
    rospy.set_param("~conf_thres", threshold)
    last_threshold = threshold
    publisher = rospy.Publisher("/YOLO/BBox", Yolo_bbox_arr, queue_size=5)

    def callback(message):
        nonlocal last_threshold
        try:
            current = validate_threshold(rospy.get_param_cached("~conf_thres", threshold))
        except (ValueError, rospy.ROSException) as exc:
            rospy.logerr_throttle(5, "camera object adapter withheld input: invalid consumer cutoff: %s", exc)
            return
        if current != last_threshold:
            rospy.loginfo("camera object adapter consumer cutoff changed: %.3f -> %.3f", last_threshold, current)
            last_threshold = current
        result = convert(message, allowed, current)
        if result is not None:
            publisher.publish(result)
        else:
            rospy.logwarn_throttle(5, "camera object adapter withheld invalid/non-front input")

    rospy.Subscriber("/perception/camera/objects", CameraObjectBoxArray, callback, queue_size=5)
    rospy.loginfo("camera objects adapter ready: input MD5=%s", EXPECTED_MD5)
    rospy.spin()


if __name__ == "__main__":
    main()
