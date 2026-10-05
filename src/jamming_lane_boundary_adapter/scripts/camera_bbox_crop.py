#!/usr/bin/env python3
"""주행 전용 YOLO에서 보닛 제외. 왼쪽 위 원점/픽셀 크기/stamp는 원본 유지."""
import math
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

# 이 파일의 인자는 ROS private parameter로 변경 가능하다.
INPUT_TOPIC = '/image_jpeg/compressed'  # 원본 영상: 차선 인식과 BBOX 동기화는 계속 이 영상을 사용.
OUTPUT_TOPIC = '/planning/camera_bbox_lane/detector_input/compressed'  # YOLO 전용 잘린 영상.
HOOD_CROP_BOTTOM_RATIO = 0.21  # 아래에서 제외할 비율. 현재 720px 영상에서 y>=568 제외.
JPEG_QUALITY = 95  # 재압축 품질. 좌표를 바꾸는 resize는 하지 않는다.


class CameraBBoxCrop:
    def __init__(self):
        self.ratio = float(rospy.get_param('~hood_crop_bottom_ratio', HOOD_CROP_BOTTOM_RATIO))
        self.quality = int(rospy.get_param('~jpeg_quality', JPEG_QUALITY))
        if not math.isfinite(self.ratio) or not 0 <= self.ratio < .5:
            raise ValueError('hood_crop_bottom_ratio must be in [0, 0.5)')
        if not 1 <= self.quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', OUTPUT_TOPIC), CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~input_topic', INPUT_TOPIC), CompressedImage,
                                    self.image, queue_size=1, buff_size=2**24)

    def image(self, msg):
        if self.ratio == 0:
            self.pub.publish(msg)
            return
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('cannot decode source image')
            keep_height = int(frame.shape[0] * (1.0 - self.ratio))
            cropped = frame[:keep_height, :]
            ok, encoded = cv2.imencode('.jpg', cropped, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                raise ValueError('cannot encode cropped image')
            self.pub.publish(CompressedImage(header=msg.header, format='jpeg', data=encoded.tobytes()))
        except (ValueError, cv2.error) as exc:
            # 실패 시 빈 검출을 만들지 않고 발행을 멈춰 downstream timeout이 적용되게 한다.
            rospy.logwarn_throttle(2.0, 'YOLO hood crop failed: %s' % exc)


if __name__ == '__main__':
    rospy.init_node('camera_bbox_crop')
    node = CameraBBoxCrop()
    rospy.spin()
