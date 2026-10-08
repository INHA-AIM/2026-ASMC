import threading
import unittest
from asmc_camera.runtime.state import InferenceState, STANDBY, PREPARE, ACTIVE


class Clock:
    now = 1.
    def __call__(self):
        return self.now


class StateTest(unittest.TestCase):
    def test_per_view_rate_is_independent_and_transition_discards_all_views(self):
        clock=Clock()
        state=InferenceState(['front','left','right'],managed=True,clock=clock)
        state.request(ACTIVE,20.)
        for view in state.views:
            token=state.begin({view:1},per_view=True)
            self.assertIsNotNone(token)
            state.complete(token,{view:1})
        self.assertIsNone(state.begin({'front':2},per_view=True))
        clock.now+=.05
        old=state.begin({'front':2},per_view=True)
        self.assertIsNotNone(old)
        state.observe('front',2)
        state.request(PREPARE)
        self.assertFalse(state.complete(old,{'front':2}))
        self.assertIsNone(state.begin({'front':2},per_view=True))
        self.assertIsNotNone(state.begin({'left':3},per_view=True))
        self.assertIsNone(state.begin({},per_view=True))

    def setUp(self):
        self.clock = Clock()
        self.state = InferenceState(['front'], managed=True, clock=self.clock)

    def frame(self, stamp, mode=PREPARE):
        self.state.observe('front', stamp)
        token = self.state.begin({'front': stamp})
        self.assertIsNotNone(token)
        self.state.complete(token, {'front': stamp})
        self.clock.now += .101

    def test_standby_no_compute_and_three_new_frames_prepare(self):
        self.assertIsNone(self.state.begin({'front': 1}))
        self.state.request(PREPARE)
        for n in range(1, 4):
            self.frame(n)
            self.assertEqual(n == 3, self.state.snapshot()['prepared'])
        self.clock.now += .36
        self.assertFalse(self.state.snapshot()['prepared'])

    def test_transition_rejects_pending_and_inflight_results(self):
        self.state.request(ACTIVE)
        self.state.observe('front', 10)
        token = self.state.begin({'front': 10})
        self.state.request(STANDBY)
        self.assertFalse(self.state.complete(token, {'front': 10}))
        with self.state.output_guard(token) as allowed:
            self.assertFalse(allowed)
        self.state.request(ACTIVE)
        self.assertIsNone(self.state.begin({'front': 10}))
        self.assertIsNotNone(self.state.begin({'front': 11}))

    def test_rate_rejects_early_work_and_tolerates_arrival_jitter(self):
        self.state.request(ACTIVE, 10)
        for i in range(20):
            self.clock.now = 1 + i * .05 + (i % 2) * .0005
            self.state.observe('front', i + 1)
            token = self.state.begin({'front': i + 1})
            if token is not None:
                self.state.complete(token, {'front': i + 1})
        self.assertEqual(10, self.state.successful_batches)

    def test_multiview_readiness_requires_all_views(self):
        self.state = InferenceState(['front', 'left'], managed=True, clock=self.clock)
        self.state.request(PREPARE)
        for i in range(3):
            self.frame(i + 1)
        self.assertFalse(self.state.snapshot()['prepared'])

    def test_models_do_not_share_modes_or_counters(self):
        other = InferenceState(['camera4'], managed=True, clock=self.clock)
        self.state.request(ACTIVE, 20)
        self.assertEqual(STANDBY, other.mode)
        self.assertEqual(0, other.generation)

    def test_invalid_requests_and_idempotent_policy_requests(self):
        for mode, hz in [(9, 10), (ACTIVE, -1), (ACTIVE, float('nan')), (ACTIVE, 121)]:
            self.assertFalse(self.state.request(mode, hz)[0])
        _, generation, _ = self.state.request(PREPARE, 10)
        self.assertEqual(generation, self.state.request(PREPARE, 0)[1])

    def test_errors_and_source_epoch_reset_prepared_history(self):
        self.state.request(PREPARE)
        for i in range(3):
            self.frame(10 + i)
        self.assertTrue(self.state.snapshot()['prepared'])
        token = self.state.begin({'front': 13})
        self.state.complete(token, {'front': 13}, False, 'decode_error')
        self.assertFalse(self.state.snapshot()['prepared'])
        self.state.observe('front', 1)
        self.assertFalse(self.state.snapshot()['prepared'])
        self.assertFalse(self.state.complete(token, {'front': 13}))

    def test_transition_waits_for_publication_guard(self):
        self.state.request(ACTIVE)
        token = self.state.begin({'front': 1})
        entered, finished = threading.Event(), threading.Event()
        def transition():
            entered.set()
            self.state.request(STANDBY)
            finished.set()
        with self.state.output_guard(token) as allowed:
            self.assertTrue(allowed)
            worker = threading.Thread(target=transition)
            worker.start()
            self.assertTrue(entered.wait(1))
            self.assertFalse(finished.wait(.02))
        worker.join(1)
        self.assertTrue(finished.is_set())


if __name__ == '__main__':
    unittest.main()
