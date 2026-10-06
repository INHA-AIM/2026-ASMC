import unittest
from types import SimpleNamespace

from asmc_camera.runtime.cadence import CoherentFrames, downstream_gate
from asmc_camera.bev.performance_monitor import source_time_gate_with_reset


class CadenceTest(unittest.TestCase):
    def test_managed_phase_jitter_keeps_fresh_results_without_duplicates(self):
        stamps = [1000000000, 1099999999, 1200000001, 1299999999]
        last, deadline = 0, 0
        for stamp in stamps:
            admitted, deadline, reset = downstream_gate(stamp, last, deadline, 100000000, 10)
            self.assertTrue(admitted)
            self.assertFalse(reset)
            last = stamp
        self.assertFalse(downstream_gate(last, last, deadline, 100000000, 10)[0])
        self.assertFalse(downstream_gate(last-1, last, deadline, 100000000, 10)[0])
        self.assertTrue(downstream_gate(100, last, deadline, 100000000, 10)[2])

    def test_legacy_or_faster_producer_retains_existing_ceiling(self):
        for hz in (0, 20):
            self.assertEqual(downstream_gate(1099999999, 1000000000, 1100000000,
                100000000, hz), source_time_gate_with_reset(1099999999, 1000000000,
                1100000000, 100000000))

    def test_partial_views_stay_pending_until_coherent_or_new_generation(self):
        def frame(view, stamp):
            return SimpleNamespace(view=view, stamp=SimpleNamespace(to_nsec=lambda: stamp))
        buffer = CoherentFrames(('front','left','right'), 35)
        self.assertIsNone(buffer.offer([frame('front',100)],1))
        self.assertIsNone(buffer.offer([frame('left',101)],1))
        self.assertEqual(len(buffer.offer([frame('right',102)],1)),3)
        self.assertIsNone(buffer.offer([frame('front',200)],1))
        self.assertNotIn('left', buffer.pending)
        self.assertIsNone(buffer.offer([frame('left',202)],2))
        self.assertEqual(set(buffer.pending), {'left'})
        buffer.clear()
        self.assertEqual(buffer.pending,{})
