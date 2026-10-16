import sys
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.validation_lease import ValidationLease,ValidationBusy
from scen_runner.runtime.validation_episode import EpisodeController


class LeaseTest(unittest.TestCase):
    def test_same_endpoint_is_exclusive_and_can_start_after_owner_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            first=ValidationLease('host',7789,directory)
            second=ValidationLease('host',7789,directory)
            try:
                first.acquire();first.acquire()
                self.assertTrue(first.held)
                with self.assertRaises(ValidationBusy):second.acquire()
                self.assertFalse(second.held)
                first.release();second.acquire()
                self.assertTrue(second.held)
            finally:
                first.release();second.release()

    def test_rejected_begin_and_close_make_no_ego_or_actor_changes(self):
        link=Mock();lease=Mock(held=False)
        lease.acquire.side_effect=ValidationBusy('another validator')
        engine=EpisodeController(link,Mock(),seed=42,lease=lease)
        with self.assertRaises(ValidationBusy):engine.begin(1)
        link.hold.assert_not_called();link.reset.assert_not_called()
        engine.stop();link.release.assert_not_called();link.destroy.assert_not_called()
        self.assertEqual(engine.phase,'stopped')

    def test_cleanup_failure_keeps_lease_until_cleanup_and_release_succeed(self):
        link=Mock();lease=Mock(held=True)
        engine=EpisodeController(link,Mock(),seed=42,lease=lease)
        adapter=S(name='high_speed',cleanup=Mock(side_effect=RuntimeError('failed')))
        engine.adapters=[adapter]
        with self.assertRaises(RuntimeError):engine.stop()
        lease.release.assert_not_called()
        adapter.cleanup.side_effect=None;engine.stop()
        lease.release.assert_called_once();link.release.assert_called_once()

    def test_process_exit_releases_lock_without_removing_metadata_file(self):
        with tempfile.TemporaryDirectory() as directory:
            code='''import sys,os
sys.path.insert(0,sys.argv[1])
from scen_runner.runtime.validation_lease import ValidationLease
lease=ValidationLease("host",7789,sys.argv[2]);lease.acquire()
os._exit(0)
'''
            subprocess.run([sys.executable,'-c',code,str(Path(__file__).resolve().parents[2]),directory],check=True)
            lease=ValidationLease('host',7789,directory)
            try:
                self.assertTrue(lease.path.exists());lease.acquire();self.assertTrue(lease.held)
            finally:lease.release()


if __name__=='__main__':unittest.main()
