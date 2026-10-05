#!/usr/bin/env python3
"""기존 YOLO/Msg는 수정하지 않고, 원본 영상과 BBOX를 stamp로 정확히 묶는다."""
from collections import OrderedDict
import threading
import time

import rospy
from sensor_msgs.msg import CompressedImage
from Msg.msg import Yolo_bbox_arr
from lane_boundary_adapter.msg import CameraBBox, CameraBBoxFrame

# 모든 설정 기본값. YOLO 토픽은 주행 전용 remap으로 기존 fusion과 분리한다.
IMAGE_TOPIC = '/image_jpeg/compressed'  # 원본 전방 영상
BBOX_TOPIC = '/planning/camera_bbox_lane/yolo'  # 기존 YOLO 출력
OUTPUT_TOPIC = '/planning/camera_bbox_lane/frame'  # 판정 입력
CACHE_SIZE = 90  # 30Hz 기준 최대 약 3초. timestamp가 맞는 영상만 사용.
WAIT_SEC = 0.12  # BBOX가 원본 영상보다 먼저 도착할 때 최대 대기


class FrameBridge:
    def __init__(self):
        self.images, self.pending = OrderedDict(), OrderedDict()
        self.lock = threading.Lock()
        self.cache_size = int(rospy.get_param('~cache_size', CACHE_SIZE))
        self.wait_sec = float(rospy.get_param('~wait_sec', WAIT_SEC))
        if self.cache_size < 1 or not 0 < self.wait_sec <= 1:
            raise ValueError('invalid cache_size/wait_sec')
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', OUTPUT_TOPIC), CameraBBoxFrame, queue_size=1)
        self.image_sub = rospy.Subscriber(rospy.get_param('~image_topic', IMAGE_TOPIC), CompressedImage, self.image, queue_size=2, buff_size=2**24)
        self.bbox_sub = rospy.Subscriber(rospy.get_param('~bbox_topic', BBOX_TOPIC), Yolo_bbox_arr, self.bbox, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(.03), self.expire)

    def emit(self, boxes, image=None, reason=''):
        frame = CameraBBoxFrame(header=boxes.header, valid=not reason, reason=reason)
        if boxes.objc != len(boxes.yolo_bbox_arr):
            frame.valid, frame.reason = False, 'bbox_count_mismatch'
        if image is not None:
            frame.image = image
            if image.header.frame_id != boxes.header.frame_id:
                frame.valid, frame.reason = False, 'image_frame_mismatch'
        frame.boxes = [CameraBBox(b.xMin, b.yMin, b.xMax, b.yMax, b.classes) for b in boxes.yolo_bbox_arr]
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
