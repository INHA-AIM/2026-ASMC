#!/usr/bin/env python3
"""Join camera-owned detections to the exact original image, without Main."""
from collections import OrderedDict
import threading
import time
import math

import rospy
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import CameraObjectBoxArray
from lane_boundary_adapter.msg import CameraBBox, CameraBBoxFrame

# 전방 카메라가 발행하는 원본 좌표와 시각을 그대로 사용한다.
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
        self.confidence_threshold = float(rospy.get_param('~confidence_threshold', .7))
        if not math.isfinite(self.confidence_threshold) or not 0 <= self.confidence_threshold <= 1:
            raise ValueError('invalid confidence_threshold')
        if self.cache_size < 1 or not 0 < self.wait_sec <= 1:
            raise ValueError('invalid cache_size/wait_sec')
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', OUTPUT_TOPIC), CameraBBoxFrame, queue_size=1)
        self.image_sub = rospy.Subscriber(rospy.get_param('~image_topic', IMAGE_TOPIC), CompressedImage, self.image, queue_size=2, buff_size=2**24)
        self.source_type = rospy.get_param('~source_type', 'camera_objects')
        if self.source_type == 'camera_objects':
            self.bbox_sub = rospy.Subscriber(rospy.get_param('~bbox_topic', BBOX_TOPIC), CameraObjectBoxArray, self.camera_boxes, queue_size=2)
        elif self.source_type == 'legacy_yolo':
            from Msg.msg import Yolo_bbox_arr
            self.bbox_sub = rospy.Subscriber(rospy.get_param('~bbox_topic', '/YOLO/BBox'), Yolo_bbox_arr, self.bbox, queue_size=2)
        else:
            raise ValueError('unsupported camera bbox source_type')
        self.timer = rospy.Timer(rospy.Duration(.03), self.expire)

    def emit(self, boxes, image=None, reason=''):
        frame = CameraBBoxFrame(header=boxes.header, valid=not reason, reason=reason)
        if isinstance(boxes, CameraBBoxFrame):
            frame.valid = frame.valid and boxes.valid
            frame.reason = reason or boxes.reason
            frame.boxes = boxes.boxes
        else:
            if boxes.objc != len(boxes.yolo_bbox_arr):
                frame.valid, frame.reason = False, 'bbox_count_mismatch'
            frame.boxes = [CameraBBox(b.xMin, b.yMin, b.xMax, b.yMax, b.classes) for b in boxes.yolo_bbox_arr]
        if image is not None:
            frame.image = image
            if image.header.frame_id != boxes.header.frame_id:
                frame.valid, frame.reason = False, 'image_frame_mismatch'
        self.pub.publish(frame)

    def camera_boxes(self, msg):
        # Left/right observations share this topic; only front pixels match image_topic.
        if msg.header.frame_id != 'camera_front':
            return
        self.bbox(camera_frame(msg, self.confidence_threshold))

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


def camera_frame(msg, confidence_threshold=.7):
    """Pure conversion: keep source time, raw labels and valid empty detections."""
    frame = CameraBBoxFrame(header=msg.header, valid=msg.valid)
    if not msg.valid or not msg.image_width or not msg.image_height:
        frame.valid = False
        frame.reason = 'invalid_camera_frame'
        return frame
    for b in msg.boxes:
        coords = (b.xmin, b.ymin, b.xmax, b.ymax)
        if (not b.confidence_valid or not math.isfinite(b.confidence) or
                not 0 <= b.confidence <= 1 or not all(math.isfinite(v) for v in coords) or
                not 0 <= b.xmin < b.xmax <= msg.image_width or
                not 0 <= b.ymin < b.ymax <= msg.image_height or not b.source_class_name):
            frame.valid, frame.reason = False, 'invalid_camera_box'
            frame.boxes = []
            return frame
        if b.confidence < confidence_threshold:
            continue  # Preserve the existing jamming consumer cutoff.
        frame.boxes.append(CameraBBox(b.xmin, b.ymin, b.xmax, b.ymax, b.source_class_name))
    return frame


if __name__ == '__main__':
    rospy.init_node('camera_bbox_frame_bridge')
    node = FrameBridge()
    rospy.spin()
