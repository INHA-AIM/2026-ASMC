import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from collections import deque
import numpy as np

from asmc_camera.runtime.latest_work import LatestWork
from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode
from asmc_camera.nodes.lane_vectorizer_node import _ViewFrame
from std_msgs.msg import Header
import rospy


class LatestLaneWorkTest(unittest.TestCase):
    def test_newest_complete_triplet_wins_and_consumed_frames_are_not_reused(self):
        node=LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.views=['front','left','right']; node.max_skew_ns=4
        node.queue_drops={v:0 for v in node.views};node.sync_rejected=0
        node.buffers={v:deque(SimpleNamespace(stamp_ns=n) for n in values) for v,values in
                      [('front',[10,20,30]),('left',[11,21,31]),('right',[12,22])]}
        result=node._take_synchronized_locked()
        self.assertEqual([result[v].stamp_ns for v in node.views],[20,21,22])
        self.assertEqual([f.stamp_ns for f in node.buffers['front']],[30])
        self.assertIsNone(node._take_synchronized_locked())
        node.buffers['right'].append(SimpleNamespace(stamp_ns=32))
        result=node._take_synchronized_locked()
        self.assertEqual([result[v].stamp_ns for v in node.views],[30,31,32])

    def test_mode_change_does_not_wait_for_calculation_and_discards_its_result(self):
        node=LaneVectorizerNode.__new__(LaneVectorizerNode)
        current=[(True,2,0,2)]
        node.epoch_gate=SimpleNamespace(lock=threading.RLock(),enabled=True,
            context=lambda:current[0],allows=lambda _:True)
        node.processing_lock=threading.Lock();node.fit_stop=threading.Event()
        node.frame_id='base_link';node.expected_boundary_count=3
        node.boundary_family_publisher=Mock();node._boundary_family_message=Mock(return_value=SimpleNamespace(valid=False))
        node._trace_supply=Mock();entered,release=threading.Event(),threading.Event()
        def calculate(*_):
            entered.set();release.wait(2.)
            return (),(),(),(),None,{'reason':'test'}
        node._compute_family_frame=calculate
        frame=_ViewFrame('front',10,Header(stamp=rospy.Time(0,10)),None,None,0.,current[0])
        worker=threading.Thread(target=lambda:node._process({'front':frame}),daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            acquired=node.epoch_gate.lock.acquire(timeout=.1)
            self.assertTrue(acquired)
            if acquired:
                current[0]=(True,3,10,3);node.epoch_gate.lock.release()
        finally:
            release.set();worker.join(2.)
        self.assertFalse(worker.is_alive())
        node.boundary_family_publisher.publish.assert_not_called()

    def test_grid_receipt_does_not_wait_for_the_fitting_epoch_lock(self):
        node=LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.epoch_gate=SimpleNamespace(lock=threading.Lock(),context=lambda: ('new',),
                                        enabled=True,mode=2,cutoff=0)
        node.inputs={'front':0}; node.pair_errors={'front':0}; node.queue_drops={'front':0}
        node.projectors={'front':SimpleNamespace(model_hw=(2,2))}
        node._mono8=lambda *args: np.zeros((2,2),np.uint8)
        node.buffer_lock=threading.Lock(); node.buffers={'front':deque()}
        node.buffer_generation=None; node.queue_size=2; node.fit_work=LatestWork()
        node._take_synchronized_locked=lambda: None
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:10)))
        done=threading.Event(); failures=[]
        def receive():
            try: node._view_callback('front',message,message)
            except Exception as error: failures.append(error)
            finally: done.set()
        with node.epoch_gate.lock:
            worker=threading.Thread(target=receive,daemon=True); worker.start()
            self.assertTrue(done.wait(.5))
        worker.join(1.)
        self.assertFalse(failures)
        self.assertEqual(len(node.buffers['front']),1)

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
        node.epoch_gate = SimpleNamespace(lock=threading.RLock(), enabled=True, generation=2, cutoff=0, context=lambda: 2)
        node.boundary_family_publisher = Mock()
        node._process({'front': SimpleNamespace(generation=1)})
        node.boundary_family_publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
