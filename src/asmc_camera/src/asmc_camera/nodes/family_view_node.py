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
from asmc_camera.runtime.latest_work import LatestWork


class FamilyViewNode:
    def __init__(self, arrival_driven=True):
        self.lock = threading.RLock()
        self.sample = None
        self.arrival_driven = arrival_driven
        self.work = LatestWork()
        self.last_display_key = None
        self.render_errors = 0
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
        self.worker = None
        if self.arrival_driven:
            self.worker = threading.Thread(target=self._worker, name='family-view', daemon=True)
            self.worker.start()
        # Only expiry/invalid-state refresh uses a timer. Fresh results wake
        # the worker immediately, with one replaceable pending notification.
        self.timer = rospy.Timer(rospy.Duration(.1),
                                 self._heartbeat if self.arrival_driven else self._publish)
        rospy.on_shutdown(self.close)

    def _clear(self, wake=True):
        with self.lock:
            self.sample = None
        self.work.clear()
        if wake and self.arrival_driven:
            self.work.offer(True)

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
        if self.arrival_driven:
            self.work.offer(True)

    def _publish(self, _event):
        if not self._requested():
            self._clear(wake=False)
            return
        # Snapshot only. Rendering/encoding must not block a newer family or
        # status callback. Recheck the epoch before publishing the finished JPEG.
        with self.gate.lock, self.lock:
            sample = self.sample
            context = self.gate.context()
            current = self._current(sample, context)
        reason = sample[2] if current else 'WAITING / STALE'
        key = (context, sample[0].stamp.to_nsec() if sample else None,
               current, bool(sample and sample[1]) if current else False, reason)
        if key == self.last_display_key:
            return
        canvas = self.background.copy()
        draw_stateful_lane_family(canvas, sample[1] if current else [], self.grid,
                                  reason)
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
            self.last_display_key = key

    def _heartbeat(self, _event):
        if not self._requested():
            self._clear(wake=False)
            return
        with self.gate.lock, self.lock:
            current = self._current(self.sample, self.gate.context())
        if not current:
            self.work.offer(True)

    def _worker(self):
        while self.work.pop() is not None and not rospy.is_shutdown():
            try:
                self._publish(None)
            except Exception as error:
                self.render_errors += 1
                rospy.logerr_throttle(2., 'family view worker failed: %s', error)

    def close(self):
        self.timer.shutdown()
        self.work.close()
        self.inputs.close()
        if self.worker is not None and threading.current_thread() is not self.worker:
            self.worker.join(timeout=1.)

    def _current(self, sample, context):
        return bool(sample and sample[3] == context and
                    self.gate.allows(sample[0].stamp) and
                    time.monotonic()-sample[4] <= .35 and
                    -.05 <= (rospy.Time.now()-sample[0].stamp).to_sec() <= .35)



def main():
    rospy.init_node('camera_lane_view')
    FamilyViewNode()
    rospy.spin()
