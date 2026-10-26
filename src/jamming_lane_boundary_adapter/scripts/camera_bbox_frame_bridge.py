#!/usr/bin/env python3
"""Join canonical camera detections with the exact original front image.

Former Main/Yolo_bbox_arr conversion is folded into this existing input node;
no inference, legacy wire publisher or extra relay process is needed.
"""
from collections import OrderedDict
import copy
import math
import threading
import time

import rospy
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.mission import CLASSES
from lane_boundary_adapter.msg import CameraBBox, CameraBBoxFrame

# 전방 공용 검출을 같은 시각의 원본 영상과 연결한다.
IMAGE_TOPIC = '/image_jpeg/compressed'  # 원본 전방 영상
BBOX_TOPIC = '/perception/camera/objects'
OUTPUT_TOPIC = '/planning/camera_bbox_lane/frame'  # 판정 입력
CACHE_SIZE = 90  # 30Hz 기준 최대 약 3초. timestamp가 맞는 영상만 사용.
WAIT_SEC = 0.12  # BBOX가 원본 영상보다 먼저 도착할 때 최대 대기


class FrameBridge:
    def __init__(self):
        self.images, self.pending = OrderedDict(), OrderedDict()
        self.lock = threading.Lock()
        self.cache_size = int(rospy.get_param('~cache_size', CACHE_SIZE))
        self.wait_sec = float(rospy.get_param('~wait_sec', WAIT_SEC))
        # Preserve the old jamming CPU adapter's additional cutoff. Producer
        # default0.40 and LiDAR consumer0.0 are separate policies.
        self.confidence = float(rospy.get_param('~conf_thres', .7))
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('invalid conf_thres')
        if self.cache_size < 1 or not 0 < self.wait_sec <= 1:
            raise ValueError('invalid cache_size/wait_sec')
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', OUTPUT_TOPIC), CameraBBoxFrame, queue_size=1)
        self.image_sub = rospy.Subscriber(rospy.get_param('~image_topic', IMAGE_TOPIC), CompressedImage, self.image, queue_size=2, buff_size=2**24)
        self.bbox_sub = rospy.Subscriber(rospy.get_param('~bbox_topic', BBOX_TOPIC), CameraObjectBoxArray, self.bbox, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(.03), self.expire)

    def emit(self, boxes, image=None, reason=''):
        frame = CameraBBoxFrame(header=boxes.header, valid=not reason, reason=reason)
        if not boxes.valid:
            frame.valid, frame.reason = False, 'invalid_camera_detections'
        if image is not None:
            frame.image = image
            if image.header.frame_id != boxes.header.frame_id:
                frame.valid, frame.reason = False, 'image_frame_mismatch'
        frame.boxes = [CameraBBox(b.xmin, b.ymin, b.xmax, b.ymax, b.source_class_name) for b in boxes.boxes]
        # Per-box non-finite/occluded geometry stays with the existing C++
        # classifier, which reports UNKNOWN for that box rather than free space.
        self.pub.publish(frame)

    def image(self, msg):
        key = msg.header.stamp.to_nsec()
        if key == 0:
            return
        with self.lock:
            self.images[key] = msg
            if key in self.pending:
                boxes, _ = self.pending.pop(key)
                self.emit(boxes, msg)
            while len(self.images) > self.cache_size:
                self.images.popitem(last=False)

    def bbox(self, msg):
        if msg.header.frame_id != 'camera_front':
            return  # same stamp from left/right must not replace front evidence
        msg = copy.deepcopy(msg)
        if not msg.image_width or not msg.image_height:
            msg.valid = False
        try:
            threshold = float(rospy.get_param_cached('~conf_thres', self.confidence))
            if not math.isfinite(threshold) or not 0 <= threshold <= 1:
                raise ValueError('invalid cutoff')
        except (ValueError, TypeError, rospy.ROSException):
            rospy.logerr_throttle(5., 'camera frame withheld: invalid consumer cutoff')
            return
        # Invalid evidence must not be converted to valid empty/free space.
        # Keep clipped/non-finite geometry for the C++ UNKNOWN classifier.
        for b in msg.boxes:
            if (not b.confidence_valid or not math.isfinite(b.confidence) or
                    not 0 <= b.confidence <= 1 or not b.source_class_name):
                msg.valid = False
        msg.boxes = [b for b in msg.boxes if b.source_class_name.lower() in CLASSES and
                     b.confidence_valid and math.isfinite(b.confidence) and b.confidence >= threshold]
        key = msg.header.stamp.to_nsec()
        with self.lock:
            if key == 0:
                self.emit(msg, reason='zero_bbox_stamp')
            elif key in self.images:
                self.emit(msg, self.images[key])
            else:
                self.pending[key] = (msg, time.monotonic())
                while len(self.pending) > self.cache_size:
                    _, (old, _) = self.pending.popitem(last=False)
                    self.emit(old, reason='image_cache_overflow')

    def expire(self, _):
        with self.lock:
            for key, (boxes, received) in list(self.pending.items()):
                if time.monotonic() - received > self.wait_sec:
                    self.pending.pop(key)
                    self.emit(boxes, reason='matching_image_missing')


if __name__ == '__main__':
    rospy.init_node('camera_bbox_frame_bridge')
    node = FrameBridge()
    rospy.spin()
