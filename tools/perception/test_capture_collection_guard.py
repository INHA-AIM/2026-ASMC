import json
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from monitor_capture_memory import decision, check_render, GIB
import run_capture_route_collection as runner


class CollectionGuardTest(unittest.TestCase):
    def sample(self, private=8, commit=.4, available=4000):
        return dict(simulators=[dict(private_bytes=private*GIB)],
                    committed_bytes=commit*100*GIB,commit_limit_bytes=100*GIB,
                    available_mb=available)

    def test_memory_boundaries_and_missing_telemetry(self):
        self.assertEqual(decision(self.sample())[0],'continue')
        self.assertEqual(decision(self.sample(private=16))[0],'stop_between_laps')
        self.assertEqual(decision(self.sample(commit=.65))[0],'stop_between_laps')
        self.assertEqual(decision(self.sample(private=32))[0],'abort')
        self.assertEqual(decision(self.sample(commit=.85))[0],'abort')
        self.assertEqual(decision(self.sample(available=511))[0],'abort')
        self.assertEqual(decision(dict(simulators=[],commit_limit_bytes=1))[0],'abort')
        self.assertEqual(decision(dict(simulators=[{}],commit_limit_bytes=0))[0],'abort')

    def test_guard_interrupts_owned_child_and_cannot_pass(self):
        with tempfile.TemporaryDirectory() as folder:
            data=Path(folder);run=data/'capture_runs/test';run.mkdir(parents=True)
            (run/'validation.json').write_text(json.dumps({'controller':'pp'}))
            stop=data/'abort.json';stop.write_text('critical memory pressure')
            child=MagicMock(pid=12345,returncode=0);child.poll.return_value=None
            with patch.object(runner,'make_run_id',return_value='test'), \
                 patch.object(runner.subprocess,'Popen',return_value=child), \
                 patch('os.killpg') as kill, \
                 patch.object(runner,'motion_summary',return_value={}), \
                 patch.object(runner,'collection_trial_passed',return_value=True):
                result=runner.trial(data,'pp',30,None,'test',abort_file=stop)
            kill.assert_called_once_with(12345,signal.SIGINT)
            self.assertFalse(result['passed'])
            self.assertEqual(result['collection_guard_abort'],'critical memory pressure')

    def test_constant_render_is_not_a_valid_capture(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for sensor in ('CAMERA_1','CAMERA_2','CAMERA_3'):
                (root/sensor).mkdir()
                im=Image.new('RGB',(10,10),'black')
                if sensor!='CAMERA_2':im.putpixel((2,3),(10,20,30))
                im.save(root/sensor/'test_Intensity.png')
            self.assertEqual(check_render(root,{'custom_name':'test'})['constant_views'],['CAMERA_2'])


if __name__=='__main__':unittest.main()
