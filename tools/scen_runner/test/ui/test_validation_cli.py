"""Exercise the real CLI entry point without creating a GUI or contacting MORAI."""
from pathlib import Path
import subprocess
import os
import signal
import time
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import unittest
import tempfile
from unittest.mock import patch
from types import SimpleNamespace as S


class ValidationCliTest(unittest.TestCase):
    def test_real_sigint_closes_qt_application_when_remote_cleanup_is_unavailable(self):
        # Every link/engine RPC is mocked in the child. Only Qt and the OS
        # signal path are real; this test cannot spawn or control MORAI actors.
        code='''
import sys
from unittest.mock import Mock,patch
from PyQt5 import QtCore,QtWidgets
from scen_runner.ui import validation_gui as gui
class Window(QtWidgets.QWidget):
    def __init__(self,*args,**kwargs):
        super().__init__()
        self.runner=gui.ValidationRunner('scene','static',42,save_collisions=False)
        self.runner.link=self.runner.engine.link=Mock(connected=True)
        self.runner.engine.lease=Mock(held=True)
        self.runner.monitor.start=Mock();self.runner.monitor.wait=Mock()
        self.runner.engine.begin=Mock(side_effect=lambda *_:setattr(self.runner.engine,'phase','running'))
        self.runner.engine.step=Mock(return_value=None);self.runner.engine.read_states=Mock()
        self.runner.snapshot=Mock()
        self.runner.engine.stop=Mock(side_effect=ConnectionError('simulator closed'))
        self.runner.finished_clean.connect(self.done)
        self.runner.start()
        QtCore.QTimer.singleShot(200,lambda:print('READY',flush=True))
    def done(self):
        self.runner.wait(1000);print('CLOSED',flush=True);self.close()
with patch.object(gui,'ValidationWindow',Window),patch.object(gui,'require_morai_endpoint'):
    sys.argv=['validation_gui'];raise SystemExit(gui.main())
'''
        root=Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'child.log'
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONPATH=str(root),PYTHONDONTWRITEBYTECODE='1')
            with log.open('w') as stream:
                child=subprocess.Popen([sys.executable,'-c',code],stdout=stream,stderr=subprocess.STDOUT,env=env)
            try:
                deadline=time.monotonic()+10
                while 'READY' not in log.read_text() and child.poll() is None and time.monotonic()<deadline:
                    time.sleep(.05)
                self.assertIn('READY',log.read_text())
                child.send_signal(signal.SIGINT)
                self.assertEqual(child.wait(timeout=5),0,log.read_text())
                self.assertIn('CLOSED',log.read_text())
            finally:
                if child.poll() is None:child.kill();child.wait()

    def test_shell_entry_runs_from_outside_the_repository(self):
        script=Path(__file__).resolve().parents[2]/'run_validation.sh'
        with tempfile.TemporaryDirectory() as folder:
            result=subprocess.run(['bash',str(script),'--help'],cwd=folder,capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('--replay-browser',result.stdout)

    def test_help_reaches_argument_parser_and_exposes_run_count_option(self):
        script=Path(__file__).resolve().parents[2]/'run_validation.sh'
        result=subprocess.run(['bash',str(script),'--help'],capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('--run-counts',result.stdout)
        self.assertIn('--mode',result.stdout)


    def test_dll_cli_selects_all_existing_contact_fixes_and_retained_scripts(self):
        from scen_runner.morai.dll_patch import main,COMMANDS
        root=Path(__file__).resolve().parents[2]
        for script,_ in COMMANDS.values():self.assertTrue((root/'dll/scripts'/script).is_file(),script)
        with patch('scen_runner.morai.dll_patch.windows_path',side_effect=str), \
             patch('scen_runner.morai.dll_patch.subprocess.run',return_value=S(stdout=b'ok',returncode=0)) as run:
            self.assertEqual(main(['build-fix','--managed','fixture']),0)
        command=run.call_args.args[0]
        for flag in ('-FixSelf','-FixContacts','-LifecycleTrace','-SpawnTrace','-StaticContacts'):
            self.assertIn(flag,command)
        self.assertTrue(command[command.index('-File')+1].endswith('build_trace.ps1'))

    def test_dll_shell_help_does_not_require_windows_or_contact_the_simulator(self):
        script=Path(__file__).resolve().parents[2]/'run_runtime_trace.sh'
        result=subprocess.run(['bash',str(script),'--help'],capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('restore-path-cleanup-fix',result.stdout)
        self.assertNotIn('build-activation-trace',result.stdout)


if __name__=='__main__':unittest.main()
