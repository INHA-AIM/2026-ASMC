"""Moved archives stay readable without weakening capture identity checks."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from data_collection.data_locations import capture_directory_windows, load_locations, resolve_sensor_root
from data_collection.capture_grpc import MoraiCaptureClient


class DataLocationsTest(unittest.TestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {}, clear=True)
        self.env.start()
        self.addCleanup(self.env.stop)

    def test_team_defaults_have_no_archive_or_aim_dependency(self):
        locations = load_locations()
        self.assertEqual(locations['capture_archive_wsl'], '')
        self.assertEqual(locations['aim_directory_wsl'], '')
        self.assertEqual(locations['aim_legacy_directory_wsl'], '')
        self.assertEqual(capture_directory_windows('Z:\\Simulator\\SensorData'),
                         'Z:\\Simulator\\SensorData')

    def test_private_config_and_environment_override_are_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); config = root/'locations.json'
            config.write_text(json.dumps(dict(schema_version=1,
                capture_directory_windows='Y:\\Sim\\SensorData',
                capture_archive_wsl='archive', aim_legacy_directory_wsl='old-alias',
                storage_contract={'storage_root': '/private'})))
            with patch.dict(os.environ, ASMC_DATA_LOCATIONS=str(config)):
                locations = load_locations()
                self.assertEqual(locations['capture_archive_wsl'], str(root/'archive'))
                self.assertEqual(locations['aim_legacy_directory_wsl'], str(root/'old-alias'))
                self.assertEqual(locations['storage_contract']['storage_root'], '/private')
                self.assertEqual(capture_directory_windows(), 'Y:\\Sim\\SensorData')
                with patch.dict(os.environ, ASMC_CAPTURE_DIR_WINDOWS='X:\\Other Sim\\SensorData',
                                ASMC_CAPTURE_ARCHIVE=str(root/'elsewhere'), ASMC_AIM_ROOT=str(root/'aim')):
                    self.assertEqual(capture_directory_windows(), 'X:\\Other Sim\\SensorData')
                    self.assertEqual(load_locations()['aim_directory_wsl'], str(root/'aim'))
                    self.assertEqual(load_locations()['capture_archive_wsl'], str(root/'elsewhere'))
            self.assertFalse((root/'archive').exists())
            self.assertFalse((root/'elsewhere').exists())

    def test_invalid_explicit_configuration_and_source_fail_without_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp)/'missing'
            with patch.dict(os.environ, ASMC_DATA_LOCATIONS=str(missing)):
                with self.assertRaisesRegex(ValueError, 'Cannot read'):
                    load_locations()
            with self.assertRaisesRegex(ValueError, 'unavailable'):
                resolve_sensor_root(missing, [])
            with patch.dict(os.environ, ASMC_SENSOR_ROOT=str(missing)):
                with self.assertRaisesRegex(ValueError, 'unavailable'):
                    resolve_sensor_root(None, [])
            self.assertFalse(missing.exists())

    def test_empty_optional_root_does_not_search_cwd_or_rewrite_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); record = dict(custom_name='run_000000', source_files=['historical/source'])
            with patch('data_collection.data_locations.load_locations', return_value=dict(
                    capture_directory_wsl='', capture_archive_wsl='')):
                with self.assertRaisesRegex(ValueError, 'not found'):
                    resolve_sensor_root(None, [record])
                self.assertEqual(record['source_files'], ['historical/source'])
            self.assertEqual(list(root.iterdir()), [])

    def test_alias_dedup_and_late_duplicate_never_mix_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); active, archive = root/'active', root/'archive'
            records = [dict(custom_name='run_{:06d}'.format(i)) for i in range(20)]
            for base, name in ((active, records[0]['custom_name']),
                               (archive, records[-1]['custom_name'])):
                path = base/'CAMERA_1'/(name+'_Intensity.png')
                path.parent.mkdir(parents=True); path.write_bytes(b'view')
            with patch('data_collection.data_locations.load_locations', return_value=dict(
                    capture_directory_wsl=str(active), capture_archive_wsl=str(archive))):
                with self.assertRaisesRegex(ValueError, 'ambiguous'):
                    resolve_sensor_root(None, records)
            alias = root/'alias'; alias.symlink_to(active, target_is_directory=True)
            with patch('data_collection.data_locations.load_locations', return_value=dict(
                    capture_directory_wsl=str(active), capture_archive_wsl=str(alias))):
                self.assertEqual(resolve_sensor_root(None, records), active)

    def test_empty_records_and_unsafe_names_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'without successful records'):
            resolve_sensor_root(None, [])
        for name in ('', '../run', 'a/b', 'a\\b'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                resolve_sensor_root(None, [dict(custom_name=name)])

    def test_grpc_preserves_unique_names_and_sends_explicit_default_directory(self):
        client = MoraiCaptureClient.__new__(MoraiCaptureClient)
        client._request_type = SimpleNamespace
        client._timeout = 2.
        client._sim_time_raw = lambda: None
        seen = []
        def save(request, timeout):
            seen.append(request)
            return SimpleNamespace(status=1)
        client._sensor = SimpleNamespace(SaveSensorData=save)
        result = client.capture('unique_run_000001')
        self.assertTrue(seen[0].is_custom_file_name)
        self.assertEqual(seen[0].custom_file_name, 'unique_run_000001')
        self.assertEqual(seen[0].file_dir, capture_directory_windows())
        self.assertTrue(seen[0].file_dir.endswith('SaveFile\\SensorData'))
        self.assertEqual(result['morai_file_dir'], seen[0].file_dir)

    def test_root_selection_rejects_ambiguity_and_preserves_explicit_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = root/'locations.json'
            active, archive = root/'active', root/'archive'
            config.write_text(json.dumps(dict(schema_version=1,
                capture_directory_wsl=str(active), capture_archive_wsl=str(archive))))
            record = dict(custom_name='same_run_000000')
            with patch.dict(os.environ, ASMC_DATA_LOCATIONS=str(config)):
                with self.assertRaisesRegex(ValueError, 'not found'):
                    resolve_sensor_root(None, [record])
                for base in (archive, active):
                    path = base/'CAMERA_1/same_run_000000_Intensity.png'
                    path.parent.mkdir(parents=True)
                    path.write_bytes(b'native image')
                    if base == archive:
                        self.assertEqual(resolve_sensor_root(None, [record]), archive)
                with self.assertRaisesRegex(ValueError, 'ambiguous'):
                    resolve_sensor_root(None, [record])
                self.assertEqual(resolve_sensor_root(archive, [record]), archive)

    def test_partial_front_view_can_be_found_in_archive(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); archive = root/'archive'
            path = archive/'CAMERA_2/run_000000_Intensity.png'
            path.parent.mkdir(parents=True);path.write_bytes(b'left view')
            config = root/'locations.json'
            config.write_text(json.dumps(dict(schema_version=1,
                capture_directory_wsl=str(root/'missing'), capture_archive_wsl=str(archive))))
            with patch.dict(os.environ, ASMC_DATA_LOCATIONS=str(config)):
                self.assertEqual(resolve_sensor_root(None, [dict(custom_name='run_000000')]), archive)


if __name__ == '__main__':
    unittest.main()
