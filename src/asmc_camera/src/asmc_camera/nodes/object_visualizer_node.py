#!/usr/bin/env python3
"""YOLO overlay on the exact source image, independent of LiDAR/GPS."""
import cv2
import message_filters
import numpy as np
import rospy
from asmc_msgs.msg import CameraObjectBoxArray
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import DiagnosticArray

from asmc_camera.detection.visualization import render_object_overlay
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ego_hood_filter import EgoHoodFilter


class ObjectVisualizerNode:
    def __init__(self):
        self.hood_mask = HoodMask.load(bool(rospy.get_param('~hood_mask',False)),
                                      rospy.get_param('~hood_mask_config',''))
        self.ego_hood_filter = EgoHoodFilter.load(bool(rospy.get_param('~ego_hood_filter',False)),
                                                  rospy.get_param('~ego_hood_filter_config',''))
        if self.hood_mask.enabled and self.ego_hood_filter.enabled:
            raise ValueError('hood input mask and ego hood post-filter are mutually exclusive')
        self.debug_raw = bool(rospy.get_param('~debug_raw',False))
        self.producer_filter_profile = None
        self.producer_profile = None
        self.diagnostics = rospy.Subscriber('/perception/camera/objects/diagnostics',DiagnosticArray,
                                           self._diagnostic,queue_size=1)
        self.quality = int(rospy.get_param('~jpeg_quality', 80))
        if not 1 <= self.quality <= 100:
            raise ValueError('jpeg_quality must be in [1,100]')
        self.publisher = rospy.Publisher(
            '/perception/camera/objects/debug/image/compressed', CompressedImage, queue_size=1)
        self.image = message_filters.Subscriber(
            rospy.get_param('~image_topic', '/image_jpeg/compressed'), CompressedImage,
            queue_size=5, buff_size=16 << 20)
        # Several cameras may share a stamp. Reject other views before the
        # synchronizer consumes the front image, rather than after pairing it.
        self.expected_frame_id=rospy.get_param('~frame_id','camera_front')
        self.objects = message_filters.SimpleFilter()
        self.object_subscriber=rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,
            lambda m:self._accept_view(self.objects,m),queue_size=5)
        inputs = [self.image, self.objects]
        if self.debug_raw:
            self.raw = message_filters.SimpleFilter()
            self.raw_subscriber=rospy.Subscriber('/perception/camera/objects/debug/raw',CameraObjectBoxArray,
                lambda m:self._accept_view(self.raw,m),queue_size=5)
            inputs.append(self.raw)
        self.sync = message_filters.TimeSynchronizer(inputs, int(rospy.get_param('~sync_queue_size',30)))
        self.sync.registerCallback(self.render)

    def _accept_view(self, target, message):
        if message.header.frame_id==self.expected_frame_id:
            target.signalMessage(message)

    def _diagnostic(self, message):
        for status in message.status:
            if status.name == 'camera_objects/inference':
                values = {v.key:v.value for v in status.values}
                if 'hood_mask_enabled' in values and 'hood_mask_config_sha256' in values:
                    self.producer_profile = (values['hood_mask_enabled'],values['hood_mask_config_sha256'])
                if 'ego_hood_filter_enabled' in values and 'ego_hood_filter_config_sha256' in values:
                    self.producer_filter_profile = (values['ego_hood_filter_enabled'],values['ego_hood_filter_config_sha256'],values.get('debug_raw_enabled'))

    def render(self, source, detection, raw=None):
        if not self.publisher.get_num_connections():
            return
        expected = (str(self.hood_mask.enabled),self.hood_mask.config_sha256)
        if ((self.producer_profile is None and self.hood_mask.enabled) or
            (self.producer_profile is not None and self.producer_profile != expected)):
            rospy.logwarn_throttle(2.,'object overlay waiting for matching producer hood mask configuration')
            return
        expected_filter = (str(self.ego_hood_filter.enabled),self.ego_hood_filter.config_sha256,str(self.debug_raw))
        if ((self.producer_filter_profile is None and (self.ego_hood_filter.enabled or self.debug_raw)) or
            (self.producer_filter_profile is not None and self.producer_filter_profile != expected_filter)):
            rospy.logwarn_throttle(2.,'object overlay waiting for matching producer ego hood filter configuration')
            return
        if raw is not None:
            if (raw.header.stamp != detection.header.stamp or raw.header.frame_id != detection.header.frame_id or
                    (raw.valid and (raw.image_width,raw.image_height) != (detection.image_width,detection.image_height))):
                rospy.logwarn_throttle(2.,'object overlay rejected mismatched raw detection header/dimensions')
                return
            if detection.valid:
                from collections import Counter
                def key(b):return (b.xmin,b.ymin,b.xmax,b.ymax,b.class_name,b.source_class_name,b.confidence,b.confidence_valid)
                if not raw.valid or Counter(map(key,detection.boxes))-Counter(map(key,raw.boxes)):
                    rospy.logwarn_throttle(2.,'object overlay rejected filtered boxes absent from raw result')
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
        output = render_object_overlay(image,detection,self.hood_mask,raw,self.ego_hood_filter)
        ok, encoded = cv2.imencode('.jpg',output,[cv2.IMWRITE_JPEG_QUALITY,self.quality])
        if ok:
            self.publisher.publish(CompressedImage(header=source.header,format='jpeg',data=encoded.tobytes()))


def main():
    rospy.init_node('camera_object_visualizer')
    ObjectVisualizerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
