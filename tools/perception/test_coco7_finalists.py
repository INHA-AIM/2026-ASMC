import unittest
from copy import deepcopy
from analyze_coco7_finalists import FINALISTS, live_summary


class FinalistEvidenceTest(unittest.TestCase):
    def fixture(self):
        metric = dict(hz=14.9, input_to_output_wall_ms=dict(p95=80., p99=100.), max_gap_sec=.18)
        return [dict(job=dict(model=m, width=w, weather_requested=weather, repeat=r),
                     observation_window_completed=True, passed=True, failures=[],
                     measurements={'yolo_'+v: deepcopy(metric) for v in ('front', 'left', 'right')},
                     target_attainment={v: dict(met=True) for v in ('front', 'left', 'right')},
                     resources_startup_and_active=dict(gpu_memory_mib=dict(maximum=5500.)))
                for weather in ('sunny', 'foggy') for m, w in FINALISTS for r in (1, 2, 3)]

    def test_planning_failure_does_not_disappear_or_masquerade_as_model_failure(self):
        rows = self.fixture()
        rows[0].update(passed=False, failures=['planning trajectory deadline gate'])
        result = live_summary(rows)[0]
        self.assertFalse(result['whole_stack_passed'])
        self.assertTrue(result['camera_supply_passed'])
        self.assertEqual(1, len(result['failures']))

    def test_partial_repeat_or_aborted_window_cannot_pass(self):
        rows = self.fixture()
        self.assertFalse(live_summary(rows[1:])[0]['camera_supply_passed'])
        rows[0]['observation_window_completed'] = False
        self.assertFalse(live_summary(rows)[0]['complete'])
        with self.assertRaises(ValueError):
            live_summary(rows + [rows[1]])

    def test_yolo_failure_and_target_miss_stay_separate(self):
        rows = self.fixture()
        rows[0].update(passed=False, failures=['yolo front supply/latency gate'])
        rows[0]['target_attainment']['front']['met'] = False
        result = live_summary(rows)[0]
        self.assertFalse(result['camera_supply_passed'])
        self.assertFalse(result['target_15hz_passed'])
        self.assertEqual(180., result['maximum_gap_ms'])

    def test_unrecognized_failure_cannot_be_reported_as_camera_pass(self):
        rows = self.fixture()
        rows[0].update(passed=False, failures=['unexpected measurement exception'])
        self.assertFalse(live_summary(rows)[0]['camera_supply_passed'])

    def test_runtime_provenance_includes_canonical_lidar_source_and_launch(self):
        import tempfile
        from pathlib import Path
        from camera_experiment_metrics import provenance
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            engine = root/'fixture.engine'
            engine.write_bytes(b'fixture')
            sources = provenance(root, engine, 'a'*40)['source_sha256']
            self.assertIn('src/asmc_lidar/src/fusion/front_camera_fusion.cpp', sources)
            self.assertIn('src/asmc_lidar/launch/lidar.launch', sources)


if __name__ == '__main__': unittest.main()
