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
        self.latest_event = threading.Event()
        self.last_display_stamp = None
        self.gate = LaneEpochGate(on_standby=self._clear)
        self.grid = BevGridSpec.from_mapping({k: rospy.get_param('~'+k) for k in
            ('frame_id','x_min_m','x_max_m','y_min_m','y_max_m','resolution_m')})
        self.background = np.zeros((*self.grid.shape, 3), np.uint8)
        draw_metric_grid(self.background, self.grid)
        cv2.putText(self.background, 'LEFT  ^ FORWARD  RIGHT', (2, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, .27, (180, 180, 180), 1)
        self.background.setflags(write=False)
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
                self.latest_event.set()

    def _publish(self, _event):
        if not self._requested():
            self._clear()
            return
        # A batched 15Hz producer can deliver two frames together then pause.
        # If this tick would repeat the last source, wait briefly for the next
        # real source instead. ROS Timer retains its original 10Hz deadlines.
        with self.lock:
            repeat = bool(self.sample and self.sample[0].stamp.to_nsec() == self.last_display_stamp)
            self.latest_event.clear()
        if repeat:
            self.latest_event.wait(.04)
        # Snapshot only. Rendering/encoding must not block a newer family or
        # status callback. Recheck the epoch before publishing the finished JPEG.
        with self.gate.lock, self.lock:
            sample = self.sample
            context = self.gate.context()
            current = self._current(sample, context)
        if current and sample[0].stamp.to_nsec() == self.last_display_stamp:
            return
        canvas = self.background.copy()
        draw_stateful_lane_family(canvas, sample[1] if current else [], self.grid,
                                  sample[2] if current else 'WAITING / STALE')
        success, jpeg = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return
        output = CompressedImage(format='jpeg', data=jpeg.tobytes())
        if sample:
            output.header = sample[0]
        with self.gate.lock:
            if context != self.gate.context() or not self._requested():
                return
            if current and not self._current(sample, context):
                return
            self.publisher.publish(output)
            self.last_display_stamp = sample[0].stamp.to_nsec() if current else None

    def _current(self, sample, context):
        return bool(sample and sample[3] == context and
                    self.gate.allows(sample[0].stamp) and
                    time.monotonic()-sample[4] <= .35 and
                    -.05 <= (rospy.Time.now()-sample[0].stamp).to_sec() <= .35)



def main():
    rospy.init_node('camera_lane_view')
    FamilyViewNode()
    rospy.spin()
