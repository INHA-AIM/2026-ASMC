import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from asmc_camera.runtime.latest_work import LatestWork
from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode


class LatestLaneWorkTest(unittest.TestCase):
    def test_one_pending_fit_and_callback_does_not_wait_for_current_fit(self):
        node = LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.fit_work, node.fit_stop = LatestWork(), threading.Event()
        entered, release, finished, offered = [threading.Event() for _ in range(4)]
        values = []
        def process(value):
            values.append(value)
            if value == 'first':
                entered.set(); release.wait(2.)
            else:
                finished.set()
        node._process = process
        node.fit_worker = threading.Thread(target=node._fit_worker, daemon=True)
        node.fit_worker.start()
        try:
            node.fit_work.offer('first')
            self.assertTrue(entered.wait(1.))
            def offer():
                node.fit_work.offer('old-pending')
                node.fit_work.offer('newest')
                offered.set()
            callback = threading.Thread(target=offer, daemon=True); callback.start()
            self.assertTrue(offered.wait(1.))
            self.assertEqual(node.fit_work.replaced, 1)
            release.set(); self.assertTrue(finished.wait(1.))
            self.assertEqual(values, ['first', 'newest'])
        finally:
            release.set(); node._shutdown()
        self.assertFalse(node.fit_worker.is_alive())
        self.assertFalse(node.fit_work.offer('after-close'))

    def test_close_discards_pending_and_wakes_idle_worker(self):
        work = LatestWork(); work.offer('pending'); work.close()
        self.assertIsNone(work.pop())
        work = LatestWork(); result = []
        thread = threading.Thread(target=lambda: result.append(work.pop()), daemon=True)
        thread.start(); work.close(); thread.join(1.)
        self.assertFalse(thread.is_alive()); self.assertEqual(result, [None])

    def test_old_generation_cannot_enter_fit_after_source_cutoff_reset(self):
        node = LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.processing_lock = threading.Lock(); node.fit_stop = threading.Event()
        node.epoch_gate = SimpleNamespace(lock=threading.RLock(), enabled=True, generation=2, cutoff=0)
        node.boundary_family_publisher = Mock()
        node._process({'front': SimpleNamespace(generation=1)})
        node.boundary_family_publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
