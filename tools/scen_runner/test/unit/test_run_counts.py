"""Configured total runs include the first drive and reject accidental unlimited values."""
import sys
import tempfile
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.run_counts import RunBudget,SECTIONS


def settings(scene=10):
    return dict(scene_runs=scene,section_runs={name:i+1 for i,name in enumerate(SECTIONS)})


class RunCountsTest(unittest.TestCase):
    def test_first_drive_consumes_one_and_one_means_no_respawn(self):
        budget=RunBudget(settings(1))
        self.assertEqual(budget.started,0);self.assertTrue(budget.can_start)
        budget.start_run()
        self.assertEqual(budget.started,1);self.assertFalse(budget.can_start)
        with self.assertRaises(RuntimeError):budget.start_run()

    def test_ten_means_initial_plus_nine_restarts(self):
        budget=RunBudget(settings(10))
        for i in range(10):
            self.assertTrue(budget.can_start);budget.start_run()
            self.assertEqual(budget.started,i+1)
        self.assertFalse(budget.can_start)
        self.assertEqual(budget.snapshot()['remaining'],0)

    def test_each_section_uses_its_own_limit_and_new_selection_starts_fresh(self):
        budget=RunBudget(settings())
        for name in SECTIONS:
            budget.reset('section',name)
            self.assertEqual(budget.limit,settings()['section_runs'][name])
            self.assertEqual(budget.started,0)
            budget.start_run()
        budget.reset('scene','static')
        self.assertEqual(budget.limit,10);self.assertEqual(budget.started,0)

    def test_invalid_values_and_missing_keys_do_not_fall_back_to_infinite(self):
        for value in (0,-1,1.5,True,'10',None):
            with self.subTest(value=value),self.assertRaises(ValueError):RunBudget(settings(value))
        bad=settings();bad['section_runs']['dynamic']=False
        with self.assertRaises(ValueError):RunBudget(bad)
        bad=settings();del bad['section_runs']['tunnel']
        with self.assertRaises(ValueError):RunBudget(bad)

    def test_yaml_load_is_independent_of_cwd_and_reports_malformed_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'counts.yaml'
            path.write_text('scene_runs: 3\nsection_runs: {static: 1, dynamic: 2, roundabout: 3, high_speed: 4, tunnel: 5}\n')
            budget=RunBudget.load('section','tunnel',path)
            self.assertEqual(budget.limit,5);self.assertEqual(budget.path,str(path))
            path.write_text('scene_runs: [broken\n')
            with self.assertRaisesRegex(ValueError,'counts.yaml'):RunBudget.load(path=path)


if __name__=='__main__':unittest.main()
