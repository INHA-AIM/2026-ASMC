import math
import unittest
from scen_runner.records.local_model_audit import audit, pairs


class ModelAuditTest(unittest.TestCase):
    def test_no_future_or_stale_history(self):
        rows, reasons = pairs([(1., .2), (1.3, .2)], [(1.01, 2.)], [(0.9, .3)], 0.)
        self.assertFalse(rows)
        self.assertEqual(reasons, {'no_causal_history': 1, 'history_stale': 1})

    def test_known_delay_on_held_out_samples(self):
        speed = [(i*.02, 3.) for i in range(1000)]
        command = [(i*.02, .2*math.sin(i*.13)) for i in range(1000)]
        imus = [(i*.02, .8*math.tan(command[i-5][1])) for i in range(5,1000)]
        result = audit(imus, speed, command)
        best = result['train_selected_candidate']
        self.assertAlmostEqual(best['delay_s'], .1)
        self.assertAlmostEqual(best['train_gain'], .8, places=2)
        self.assertLess(best['held_out']['p95_rad_s'], .02)
        self.assertFalse(result['changed_runtime'])

    def test_no_excitation_is_not_model_validation(self):
        result = audit([(1., 0.)], [(1., 2.)], [(.95, 0.)])
        self.assertIsNone(result['train_selected_candidate'])
        self.assertEqual(result['configured']['residual']['count'], 0)


if __name__ == '__main__':
    unittest.main()
