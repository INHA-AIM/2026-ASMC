#!/usr/bin/env python3
"""Draw the typed traffic-light result on its exact source frame (debug only).

The node is started by visualization.launch only when the traffic-light
producer is enabled. It renders nothing until someone subscribes.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rospy
from asmc_msgs.msg import TrafficLightDetection
from sensor_msgs.msg import CompressedImage
from asmc_camera.runtime.demand_inputs import DemandSubscriber, DemandInputs

from asmc_camera.traffic_light_visualization import (
    StampedCache,
    render_traffic_light_overlay,
)


class TrafficLightVisualizerNode:
    def __init__(self) -> None:
        image_topic = str(
            rospy.get_param("~image_topic", "/cam_extra/image_jpeg/compressed")
        )
        detection_topic = str(
            rospy.get_param("~detection_topic", "/perception/traffic_light/detection")
        )
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 80))
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.images = StampedCache(int(rospy.get_param("~image_cache_size", 30)))
        self._lock = threading.Lock()
        self.rendered = 0
        self.unmatched = 0
        self.decode_errors = 0
        self._last_log_wall = time.monotonic()

        self.publisher = rospy.Publisher(
            "/perception/traffic_light/debug/image/compressed",
            CompressedImage,
            queue_size=1,
        )
        self.image_subscriber = DemandSubscriber(
            image_topic,
            CompressedImage,
            lambda:self.publisher.get_num_connections() > 0,
            self._image_callback,
            queue_size=5,
            buff_size=int(rospy.get_param("~subscriber_buffer_bytes", 16 << 20)),
        )
        self.detection_subscriber = DemandSubscriber(
            detection_topic,
            TrafficLightDetection,
            lambda:self.publisher.get_num_connections() > 0,
            self._detection_callback,
            queue_size=5,
        )
        self.demand_inputs = DemandInputs([self.image_subscriber,self.detection_subscriber])
        rospy.loginfo(
            "traffic-light visualizer image=%s detection=%s",
            image_topic,
            detection_topic,
        )

    def _image_callback(self, message: CompressedImage) -> None:
        with self._lock:
            self.images.put(int(message.header.stamp.to_nsec()), message)

    def _detection_callback(self, detection: TrafficLightDetection) -> None:
        if self.publisher.get_num_connections() == 0:
            return
        with self._lock:
            source = self.images.get(int(detection.header.stamp.to_nsec()))
            matched = source is not None
            if source is None:
                source = self.images.latest()
        if source is None:
            return
        image = cv2.imdecode(np.frombuffer(source.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.decode_errors += 1
            rospy.logwarn_throttle(2.0, "traffic-light visualizer cannot decode JPEG")
            return
        rendered = render_traffic_light_overlay(
            image,
            state=detection.state,
            label=detection.label,
            confidence=detection.confidence,
            valid=detection.valid,
            stale=detection.stale,
            latency_ms=detection.inference_latency_ms,
            candidate_labels=list(detection.candidate_labels),
            candidate_confidences=list(detection.candidate_confidences),
            source_matched=matched,
        )
        ok, encoded = cv2.imencode(
            ".jpg", rendered, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return
        output = CompressedImage()
        output.header = detection.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self.publisher.publish(output)
        self.rendered += 1
        if not matched:
            self.unmatched += 1
        now = time.monotonic()
        if now - self._last_log_wall >= 5.0:
            rospy.loginfo(
                "traffic-light visualizer rendered=%d unmatched=%d decode_errors=%d",
                self.rendered,
                self.unmatched,
                self.decode_errors,
            )
            self._last_log_wall = now


def main() -> None:
    rospy.init_node("camera_traffic_light_visualizer")
    TrafficLightVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
