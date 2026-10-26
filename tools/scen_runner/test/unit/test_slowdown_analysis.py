import sys
import unittest
import json
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from analyze_slowdown_survey import Timeline, analyze, classify


class SlowdownEvidenceTest(unittest.TestCase):
    def test_preparation_failure_is_retained_without_a_driving_average(self):
        with tempfile.TemporaryDirectory() as directory:
            trial = Path(directory)
            for name in ('oracle.jsonl', 'ros.jsonl'):
                (trial / name).write_text('')
            (trial / 'result.json').write_text(json.dumps({'error': 'actor readiness deadline'}))
            report = analyze(trial, {})
            self.assertEqual(report['scope'], 'driving_not_started')
            self.assertTrue(report['no_driving_samples'])
            self.assertFalse(report['passed'])
            self.assertNotIn('mean_including_stops_kmh', report)
            self.assertEqual(json.loads((trial / 'slowdown-summary.json').read_text()), report)

    def test_no_future_or_stale_join_and_latched_mode(self):
        t = Timeline([dict(topic='features', monotonic=2., x=1),
                      dict(topic='mode', monotonic=1., mode=1)])
        self.assertEqual(t.at('features', 1.9), {})
        self.assertEqual(t.at('features', 2.4), {})
        self.assertEqual(t.at('features', 2.1)['x'], 1)
        self.assertEqual(t.at('mode', 100., None)['mode'], 1)

    def test_actual_red_wait_is_not_a_review_candidate(self):
        feature = dict(traffic_light_id='L', hard_rule_reason='traffic_light')
        args = [dict(speed_kmh=0, aspects={'L':1}), dict(maneuver_type=5), {}, feature, {}, {}, {'L':16}, 0]
        self.assertEqual(classify(*args)[0], 'signal_wait')
        args[0]['aspects']['L'] = 16
        self.assertEqual(classify(*args)[0], 'signal_perception_review')
        args[0]['aspects'] = {}
        self.assertEqual(classify(*args)[0], 'other_planned_stop')

    def test_recovering_acceleration_is_not_called_excessive_braking(self):
        args = [dict(speed_kmh=25), dict(maneuver_type=0,desired_speed_mps=16),
                dict(applied_speed_mps=8), {'ego_speed':7}, {}, {}, {}, 1.]
        self.assertEqual(classify(*args)[0], 'accelerating')
        args[-1] = -.5
        self.assertEqual(classify(*args)[0], 'keep_profile_review')

    def test_jamming_uses_its_own_diagnostics_not_old_normal_command(self):
        args = [dict(speed_kmh=2), dict(maneuver_type=5), {}, {}, {'mode':1},
                dict(statuses=[dict(message='projection_source_expired')]), {}, 0]
        self.assertEqual(classify(*args)[0], 'jamming_input_review')


if __name__ == '__main__':
    unittest.main()
