"""Run shell entry points against fixture Docker; never touch live containers."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]


class StorageLauncherTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name);self.repo=self.base/'team repo';self.repo.mkdir()
        (self.repo/'docker/perception-train').mkdir(parents=True)
        (self.repo/'docker/ros-noetic').mkdir(parents=True)
        self.record=self.base/'calls.jsonl'
        for name in ('scripts/docker_ros_up.sh','scripts/docker_perception_up.sh',
                     'scripts/docker_training_job.sh','scripts/lib/storage.sh',
                     'src/data_collection/src/data_collection/storage_paths.py',
                     'src/data_collection/src/data_collection/__init__.py'):
            target=self.repo/name;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((ROOT/name).read_bytes())
        docker=self.base/'bin/docker';docker.parent.mkdir()
        docker.write_text('''#!/usr/bin/env python3
import json,os,sys
with open(os.environ['DOCKER_RECORD'],'a') as stream:stream.write(json.dumps(sys.argv[1:])+'\\n')
if sys.argv[1:2]==['inspect']:print('true')
elif 'config' in sys.argv:
 print('    network_mode: host\\n      TWINLITE_ROOT: /opt/baselines/TwinLiteNetPlus\\n        target: /opt/baselines/TwinLiteNetPlus')
elif 'python3' in sys.argv and 'default_artifact_root' in sys.argv[sys.argv.index('python3')+2]:
 print('/managed training/perception_eval/jobs/'+sys.argv[-1])
''');docker.chmod(0o755)
        self.env={'PATH':str(docker.parent)+os.pathsep+os.environ['PATH'],
                  'HOME':str(self.base),'DOCKER_RECORD':str(self.record),'PYTHONDONTWRITEBYTECODE':'1'}

    def run_script(self,name,*args,env=None):
        return subprocess.run(['bash',str(self.repo/'scripts'/name),*args],cwd=self.repo,env=dict(self.env,**(env or {})),text=True,capture_output=True)

    def calls(self):
        return [json.loads(x) for x in self.record.read_text().splitlines()] if self.record.exists() else []

    def test_config_is_readonly_and_preserves_sibling_data_default(self):
        for name in ('docker_ros_up.sh','docker_perception_up.sh'):
            result=self.run_script(name,'config')
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn(str(self.base/'data')+' -> /data',result.stdout)
            self.assertFalse((self.base/'data').exists())
        self.assertTrue(all(call[-1]=='config' for call in self.calls()))

    def test_explicit_data_path_with_spaces_is_preserved(self):
        custom=self.base/"data O'Connor"
        result=self.run_script('docker_perception_up.sh','config',env={'ASMC_DATA':str(custom)})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn(str(custom)+' -> /data',result.stdout)
        self.assertFalse(custom.exists())

    def test_relative_override_is_made_absolute_for_compose(self):
        result=self.run_script('docker_perception_up.sh','config',env={'ASMC_DATA':'../separate data'})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn(str(self.base/'separate data')+' -> /data',result.stdout)
        self.assertFalse((self.base/'separate data').exists())

    def test_profile_adds_overlay_without_overriding_callers_data(self):
        local=self.base/'.storage';local.mkdir();overlay=self.base/'overlay with spaces.yaml';overlay.write_text('services: {}')
        (local/'env.sh').write_text('export ASMC_DATA=/profile/default\nexport ASMC_TRAIN_STORAGE_COMPOSE='+repr(str(overlay))+'\n')
        custom=self.base/'explicit data'
        result=self.run_script('docker_perception_up.sh','config',env={'ASMC_DATA':str(custom)})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn(str(custom)+' -> /data',result.stdout)
        self.assertIn(str(overlay),self.calls()[-1]);self.assertFalse(custom.exists())

    def test_missing_explicit_profile_or_overlay_fails_before_docker(self):
        for variable in ('ASMC_STORAGE_ENV','ASMC_TRAIN_STORAGE_COMPOSE'):
            result=self.run_script('docker_perception_up.sh','config',env={variable:str(self.base/'missing')})
            self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.calls(),[])

    def test_failed_guard_never_creates_a_shadow_data_directory(self):
        guard=self.base/'guard.py';guard.write_text('raise SystemExit(7)\n')
        result=self.run_script('docker_perception_up.sh','up',env={'ASMC_STORAGE_GUARD':str(guard),'ASMC_STORAGE_WORKSPACE':str(self.base)})
        self.assertNotEqual(result.returncode,0);self.assertFalse((self.base/'data').exists())
        self.assertEqual(self.calls(),[])

    def test_portable_up_creates_default_data_only_when_requested(self):
        result=self.run_script('docker_perception_up.sh','up')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue((self.base/'data').is_dir())
        self.assertEqual(self.calls()[0][-3:],['up','-d','--force-recreate'])

    def test_local_policy_needs_no_private_guard_but_refuses_missing_volume(self):
        local=self.base/'.storage';local.mkdir()
        missing=self.base/'detached-volume'
        (local/'policy.json').write_text(json.dumps({'storage_root':str(missing)}))
        result=self.run_script('docker_perception_up.sh','up')
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(missing.exists());self.assertFalse((self.base/'data').exists())
        self.assertEqual(self.calls(),[])

    def test_down_does_not_require_attached_storage(self):
        guard=self.base/'guard.py';guard.write_text('raise SystemExit(7)\n')
        result=self.run_script('docker_ros_up.sh','down',env={'ASMC_STORAGE_GUARD':str(guard)})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertFalse((self.base/'data').exists());self.assertEqual(self.calls()[-1][-1],'down')

    def test_invalid_job_usage_does_not_contact_docker(self):
        for args in (('wrong','job'),('start','job'),('logs','job','bad')):
            self.assertEqual(self.run_script('docker_training_job.sh',*args).returncode,2)
        self.assertEqual(self.calls(),[])

    def test_job_paths_with_apostrophes_are_positional_not_interpolated(self):
        root="/training O'Connor/jobs"
        result=self.run_script('docker_training_job.sh','start','job','--','echo',"literal O'Connor ; $HOME",env={'ASMC_TRAIN_JOB_ROOT':root})
        self.assertEqual(result.returncode,0,result.stderr)
        calls=[x for x in self.calls() if 'bash' in x]
        self.assertEqual(len(calls),2)
        for call in calls:
            script=call[call.index('-lc')+1]
            self.assertNotIn(root,script)
            self.assertEqual(call[call.index('-lc')+3],root+'/job')
            self.assertIn('"$1',script)


if __name__=='__main__':unittest.main()
