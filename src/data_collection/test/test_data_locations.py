"""Moved archives stay readable without weakening capture identity checks."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from data_collection.data_locations import capture_directory_windows, resolve_sensor_root
from data_collection.capture_grpc import MoraiCaptureClient


class DataLocationsTest(unittest.TestCase):
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
