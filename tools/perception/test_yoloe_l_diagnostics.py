import unittest

from analyze_yoloe_l_diagnostics import camera_admission


class AdmissionTest(unittest.TestCase):
    def test_uses_acceptance_window_and_preserves_missing_admissions(self):
        rows = [dict(event='input', monotonic=t, view='left') for t in [9., 10., 10.05, 30.]]
        rows += [dict(event='begin', monotonic=10.02, stamps={'left': 1}),
                 dict(event='rate_wait', monotonic=10.06, stamps={'left': 2}),
                 dict(event='published', monotonic=10.1, stamps={'left': 1}, timings={'inference_ms': 12}),
                 dict(event='trace_end', monotonic=31.)]
        result = camera_admission({'common_active_started': 10.}, rows)
        self.assertEqual(result['views']['left']['events']['input'], 2)
        self.assertEqual(result['views']['left']['events']['begin'], 1)
        self.assertEqual(result['views']['left']['events']['rate_wait'], 1)
        self.assertEqual(result['views']['front']['events']['published'], 0)
        self.assertTrue(result['trace_complete'])

    def test_missing_or_duplicated_end_is_not_complete(self):
        for rows in [[], [dict(event='trace_end', monotonic=1.)]*2]:
            self.assertFalse(camera_admission({'common_active_started': 0.}, rows)['trace_complete'])


if __name__ == '__main__':
    unittest.main()
