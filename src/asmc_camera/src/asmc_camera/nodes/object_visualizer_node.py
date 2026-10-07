#!/usr/bin/env python3
"""YOLO overlay on the exact source image, independent of LiDAR/GPS."""
import cv2
import message_filters
import numpy as np
import rospy
from asmc_msgs.msg import CameraObjectBoxArray
from sensor_msgs.msg import CompressedImage

from asmc_camera.detection.visualization import render_object_overlay


class ObjectVisualizerNode:
    def __init__(self):
        self.quality = int(rospy.get_param('~jpeg_quality', 80))
        if not 1 <= self.quality <= 100:
            raise ValueError('jpeg_quality must be in [1,100]')
        self.publisher = rospy.Publisher(
            '/perception/camera/objects/debug/image/compressed', CompressedImage, queue_size=1)
        self.image = message_filters.Subscriber(
            rospy.get_param('~image_topic', '/image_jpeg/compressed'), CompressedImage,
            queue_size=5, buff_size=16 << 20)
        self.objects = message_filters.Subscriber(
            '/perception/camera/objects', CameraObjectBoxArray, queue_size=5)
        self.sync = message_filters.TimeSynchronizer(
            (self.image,self.objects), int(rospy.get_param('~sync_queue_size',30)))
        self.sync.registerCallback(self.render)

    def render(self, source, detection):
        if not self.publisher.get_num_connections():
            return
        # Never put an old box on a newer image or a different camera.
        if source.header.stamp != detection.header.stamp or source.header.frame_id != detection.header.frame_id:
            rospy.logwarn_throttle(2., 'object overlay rejected mismatched source header')
            return
        image = cv2.imdecode(np.frombuffer(source.data,np.uint8),cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(2., 'object overlay cannot decode JPEG')
            return
        if detection.valid and (detection.image_height,detection.image_width) != image.shape[:2]:
            rospy.logwarn_throttle(2., 'object overlay rejected mismatched image dimensions')
            return
        output = render_object_overlay(image,detection)
        ok, encoded = cv2.imencode('.jpg',output,[cv2.IMWRITE_JPEG_QUALITY,self.quality])
        if ok:
            self.publisher.publish(CompressedImage(header=source.header,format='jpeg',data=encoded.tobytes()))


def main():
    rospy.init_node('camera_object_visualizer')
    ObjectVisualizerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
