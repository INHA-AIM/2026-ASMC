import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from data_collection import storage_paths as paths


class StoragePathsTests(unittest.TestCase):
    def test_unmanaged_clone_and_worktree_default_to_sibling_data(self):
        for worktree in (False, True):
            with self.subTest(worktree=worktree), tempfile.TemporaryDirectory() as tmp:
                repo=Path(tmp)/'team repo';repo.mkdir()
                if worktree:(repo/'.git').write_text('gitdir: example')
                else:(repo/'.git').mkdir()
                fake=repo/'src/data_collection/src/data_collection/storage_paths.py'
                with patch.dict(os.environ,{},clear=True),patch.object(paths,'__file__',str(fake)),patch.object(paths,'_in_container',return_value=False):
                    self.assertEqual(paths.default_data_root(),str(repo.parent/'data'))
                    self.assertEqual(paths.default_artifact_root(repo),repo/'artifacts')
                self.assertFalse((repo.parent/'data').exists())

    def test_container_default_and_legacy_project_override(self):
        with patch.dict(os.environ,{},clear=True),patch.object(paths,'_in_container',return_value=True),patch.object(paths.Path,'is_dir',return_value=True):
            self.assertEqual(paths.default_data_root(),'/data')
        with patch.dict(os.environ,{'AIM_PROJECT':'/team/workspace'},clear=True),patch.object(paths,'_local_policy',return_value=None),patch.object(paths,'_in_container',return_value=False):
            self.assertEqual(paths.default_data_root(),'/team/workspace/data')

    def test_explicit_missing_configuration_is_not_silently_unmanaged(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(os.environ,{'ASMC_STORAGE_WORKSPACE':tmp},clear=True):
                with self.assertRaises(RuntimeError):paths._local_policy()
            with patch.dict(os.environ,{'ASMC_DATA_LOCATIONS':str(Path(tmp)/'missing.json')},clear=True),patch.object(paths,'__file__',str(Path(tmp)/'fake.py')):
                with self.assertRaises(RuntimeError):paths._local_policy()

    def test_category_escape_rejected_without_policy(self):
        for category in ('/elsewhere','../elsewhere'):
            with self.subTest(category=category),patch.object(paths,'_local_policy',return_value=None):
                with self.assertRaises(RuntimeError):paths.validate_output('/team/data',category)

    def test_managed_free_space_uses_resolved_data_volume(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);archive=root/'archive';(archive/'data/datasets').mkdir(parents=True)
            mixed=root/'mixed';mixed.mkdir();(mixed/'datasets').symlink_to(archive/'data/datasets')
            point,fs,source=paths._mount_point(archive)
            policy={'storage_root':str(archive),'expected_mount':{'point':point,'type':fs,'source':source},'minimum_free_bytes':10}
            usage=type('Usage',(),{'free':9})()
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy),patch.object(paths,'_in_container',return_value=False),patch.object(paths.shutil,'disk_usage',return_value=usage) as probe:
                with self.assertRaises(RuntimeError):paths.validate_output(mixed,'datasets','new')
                self.assertEqual(probe.call_args[0][0],archive/'data/datasets')
            self.assertFalse((archive/'data/datasets/new').exists())

    def test_managed_readonly_container_output_and_symlink_escape_rejected(self):
        policy={'storage_root':'/managed','minimum_free_bytes':0}
        with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy),patch.object(paths,'_in_container',return_value=True),patch.object(paths,'_mount_point',return_value=('/data','bind','source')),patch.object(paths,'_writable',return_value=False):
            with self.assertRaises(RuntimeError):paths.validate_output('/data','datasets','new')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);archive=root/'archive';archive.mkdir();(archive/'artifacts').mkdir()
            outside=root/'outside';outside.mkdir();(archive/'artifacts/escape').symlink_to(outside)
            point,fs,source=paths._mount_point(archive)
            policy={'storage_root':str(archive),'expected_mount':{'point':point,'type':fs,'source':source},'minimum_free_bytes':0}
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy),patch.object(paths,'_in_container',return_value=False):
                with self.assertRaises(RuntimeError):paths.validate_bulk_output(archive/'artifacts/escape/new')

    def test_unmanaged_writers_preserve_team_layout_and_refuse_run_reuse(self):
        from data_collection.capture_mode import CaptureRunWriter
        from data_collection.writer import DatasetWriter
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=None):
            root=Path(tmp)/'data'
            capture=CaptureRunWriter(str(root),'capture-test',{});capture.close()
            writer=DatasetWriter({'channels':{}},run_id='dataset-test',data_root=str(root));writer.close()
            with self.assertRaises(FileExistsError):CaptureRunWriter(str(root),'capture-test',{})
            with self.assertRaises(RuntimeError):DatasetWriter({'channels':{}},run_id='dataset-test',data_root=str(root))
            self.assertTrue((root/'capture_runs/capture-test/meta.json').is_file())
            self.assertTrue((root/'datasets/dataset-test/meta.yaml').is_file())

    def test_physical_output_precedes_merged_map_root(self):
        with patch.dict(os.environ,{'ASMC_STORAGE_DATA':'/physical/data','ASMC_DATA':'/mixed/data'},clear=True):
            self.assertEqual(paths.default_data_root(),'/physical/data')

    def test_explicit_data_without_managed_policy(self):
        with patch.dict(os.environ,{'ASMC_DATA':'/custom/data'},clear=True):
            self.assertEqual(paths.default_data_root(),'/custom/data')

    def test_explicit_fixture_opt_out(self):
        with patch.dict(os.environ,{'ASMC_STORAGE_UNMANAGED_OUTPUT':'1'}):
            paths.validate_output('/not-created','datasets')

    def test_no_policy_preserves_other_team_pcs(self):
        with patch.object(paths,'_local_policy',return_value=None):
            paths.validate_output('/a/team/explicit/data','datasets')

    def test_missing_volume_never_creates_shadow(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'missing';policy={'storage_root':str(p)}
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy):
                with self.assertRaises(RuntimeError):paths.validate_output(str(p/'data'),'datasets')
            self.assertFalse(p.exists())

    def test_wrong_mount_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            policy={'storage_root':tmp,'expected_mount':{'point':'/different','type':'9p','source':'D:'}}
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy):
                with self.assertRaises(RuntimeError):paths.validate_output(str(Path(tmp)/'data'),'datasets')

    def test_c_output_rejected_in_managed_workspace(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive=Path(tmp)/'archive';archive.mkdir();point,fs,source=paths._mount_point(archive)
            policy={'storage_root':str(archive),'expected_mount':{'point':point,'type':fs,'source':source},'minimum_free_bytes':0}
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy):
                with self.assertRaises(RuntimeError):paths.validate_output(str(Path(tmp)/'unmanaged'),'datasets')

    def test_default_missing_root_is_not_historical_guess(self):
        with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value={'storage_root':'/configured'}),patch.object(paths.Path,'is_dir',return_value=False):
            self.assertEqual(paths.default_data_root(),'/configured/data')

    def test_run_id_traversal_rejected_even_in_fixture_mode(self):
        with patch.dict(os.environ,{'ASMC_STORAGE_UNMANAGED_OUTPUT':'1'}):
            with self.assertRaises(RuntimeError):paths.validate_output('/tmp','datasets','../escaped')

    def test_new_worktree_discovers_workspace_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=Path(tmp);(w/'.storage').mkdir()
            (w/'.storage/policy.json').write_text(json.dumps({'storage_root':'/managed'}))
            fake=w/'new-worktree/src/data_collection/storage_paths.py'
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'__file__',str(fake)):
                self.assertEqual(paths._local_policy()['storage_root'],'/managed')

    def test_stale_environment_does_not_create_old_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive=Path(tmp)/'archive';archive.mkdir();point,fs,source=paths._mount_point(archive)
            old=Path(tmp)/'old-workspace/data'
            policy={'storage_root':str(archive),'expected_mount':{'point':point,'type':fs,'source':source},'minimum_free_bytes':0}
            with patch.dict(os.environ,{'ASMC_DATA':str(old)},clear=True),patch.object(paths,'_local_policy',return_value=policy):
                with self.assertRaises(RuntimeError):paths.validate_output(paths.default_data_root(),'datasets')
            self.assertFalse(old.exists())

    def test_managed_training_rejects_c_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);point,fs,source=paths._mount_point(root)
            policy={'storage_root':str(root),'expected_mount':{'point':point,'type':fs,'source':source},'minimum_free_bytes':0}
            with patch.dict(os.environ,{},clear=True),patch.object(paths,'_local_policy',return_value=policy),patch.object(paths.Path,'is_file',return_value=False):
                self.assertEqual(paths.default_artifact_root('/unrelated'),root/'artifacts')
                with self.assertRaises(RuntimeError):paths.validate_bulk_output('/elsewhere/run')
                self.assertEqual(paths.validate_bulk_output(root/'artifacts/new-run'),str(root/'artifacts/new-run'))
            self.assertFalse((root/'artifacts').exists())


if __name__=='__main__':unittest.main()
