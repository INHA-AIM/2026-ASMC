#!/usr/bin/env python3
"""Subscriber-driven view of the actual shadow lane family output."""
import threading
import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import LaneBoundaryFamily
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.family_view import family_segments
from asmc_camera.bev.visualization import draw_metric_grid, draw_stateful_lane_family
from asmc_camera.runtime.demand_inputs import DemandInputs, DemandSubscriber
from asmc_camera.runtime.ros_control import LaneEpochGate


class FamilyViewNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.sample = None
        self.gate = LaneEpochGate(on_standby=self._clear)
        self.grid = BevGridSpec.from_mapping({k: rospy.get_param('~'+k) for k in
            ('frame_id','x_min_m','x_max_m','y_min_m','y_max_m','resolution_m')})
        self.publisher = rospy.Publisher('/perception/lane/debug/shared_family_bev/compressed',
                                         CompressedImage, queue_size=1)
        self.input = DemandSubscriber('/perception/lane/boundary_family', LaneBoundaryFamily,
                                     self._requested, self._receive, queue_size=1)
        self.inputs = DemandInputs([self.input])
        self.timer = rospy.Timer(rospy.Duration(.1), self._publish)

    def _clear(self):
        with self.lock:
            self.sample = None

    def _requested(self):
        return self.publisher.get_num_connections() > 0

    def _receive(self, message):
        with self.gate.lock:
            if not self.gate.allows(message.header.stamp):
                return
            try:
                segments = family_segments(message)
            except ValueError as error:
                self._clear()
                rospy.logwarn_throttle(2., 'family view rejected: %s', error)
                return
            with self.lock:
                self.sample = (message.header, segments, message.status_reason,
                               self.gate.context(), time.monotonic())

    def _publish(self, _event):
        if not self._requested():
            self._clear()
            return
        # The gate precedes the sample lock, as in the receive callback.
        with self.gate.lock, self.lock:
            sample = self.sample
            canvas = np.zeros((*self.grid.shape,3), np.uint8)
            draw_metric_grid(canvas, self.grid)
            current = bool(sample and sample[3] == self.gate.context() and
                           self.gate.allows(sample[0].stamp) and time.monotonic()-sample[4] <= .35 and
                           -.05 <= (rospy.Time.now()-sample[0].stamp).to_sec() <= .35)
            reason = sample[2] if current else 'WAITING / STALE'
            draw_stateful_lane_family(canvas, sample[1] if current else [], self.grid, reason)
            success, jpeg = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY,80])
            if success:
                output = CompressedImage(format='jpeg',data=jpeg.tobytes())
                if sample:
                    output.header = sample[0]
                self.publisher.publish(output)


def main():
    rospy.init_node('camera_lane_view')
    FamilyViewNode()
    rospy.spin()
