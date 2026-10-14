import unittest
from types import SimpleNamespace
from run_camera_lr_branch import scale_learning_rate
from finish_camera_epoch_boundary import completed

class BranchTests(unittest.TestCase):
    def test_scales_scheduler_and_groups_without_reset(self):
        fn=lambda e:1-e/40*.99
        class Schedule:
            base_lrs=[.001]*3
            lr_lambdas=[fn]*3
            _last_lr=[.00082675]*3
            def get_last_lr(self):return self._last_lr
        slots={'moment':'preserved'}
        t=SimpleNamespace(start_epoch=8,scheduler=Schedule(),optimizer=SimpleNamespace(param_groups=[dict(lr=.00082675,initial_lr=.001) for _ in range(3)],state=slots),args=SimpleNamespace(lr0=.001))
        r=scale_learning_rate(t,.0002)
        self.assertIs(t.optimizer.state,slots)
        for base,g in zip(t.scheduler.base_lrs,t.optimizer.param_groups):
            self.assertAlmostEqual(base*fn(8),.0002)
            self.assertAlmostEqual(base,g['initial_lr'])
            self.assertAlmostEqual(base*fn(9),.0001938279301745636)
        self.assertFalse(r['optimizer_moments_reset'])
        with self.assertRaises(ValueError):scale_learning_rate(t,0)

    def test_only_full_requested_checkpoint_qualifies(self):
        self.assertFalse(completed({'completed_epochs':13,'epoch_boundary_pause':True},14))
        self.assertFalse(completed({'completed_epochs':14},14))
        self.assertTrue(completed({'completed_epochs':14,'epoch_boundary_pause':True},14))
        with self.assertRaises(RuntimeError):
            completed({'completed_epochs':15,'epoch_boundary_pause':True},14)
        with self.assertRaises(RuntimeError):
            completed({'completed_epochs':14,'epoch_boundary_pause':True,'errors':['bad loss']},14)

if __name__=='__main__':unittest.main()
