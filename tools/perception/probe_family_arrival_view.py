#!/usr/bin/env python3
"""Experimental arrival-driven family renderer; remap its output for comparison.

Use an isolated debug output. Does not change operational default, refit lanes,
modify source stamps, or publish any perception/control results.
"""
import threading
import rospy
from asmc_camera.nodes.family_view_node import FamilyViewNode
from asmc_camera.runtime.latest_work import LatestWork


class ArrivalView(FamilyViewNode):
    def __init__(self):
        self.work = LatestWork()
        super().__init__()
        self.timer.shutdown()
        self.worker = threading.Thread(target=self._worker, name='family-arrival-render', daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(.1), self._stale_tick)
        rospy.on_shutdown(self._close)

    def _receive(self, message):
        super()._receive(message)
        self.work.offer(True)

    def _stale_tick(self, _event):
        with self.gate.lock, self.lock:
            current = self._current(self.sample, self.gate.context())
        if not current: self.work.offer(True)

    def _worker(self):
        while self.work.pop() is not None and not rospy.is_shutdown():
            self._publish(None)

    def _close(self):
        self.timer.shutdown(); self.work.close(); self.worker.join(timeout=1.)
        self.inputs.close()


if __name__ == '__main__':
    rospy.init_node('family_arrival_trial')
    ArrivalView()
    rospy.spin()
