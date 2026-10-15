"""Local endpoints/paths must not alter scene settings or require a live MORAI."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner import config
from scen_runner.morai import dll_patch
from scen_runner.morai.morai_link import MoraiLink


class LocalConfigTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.local = self.root/'config/local.yaml'
        self.local.parent.mkdir()
        for module in (config, dll_patch):
            override = patch.object(module, 'LOCAL_CONFIG', self.local)
            override.start();self.addCleanup(override.stop)

    def write(self, data):
        self.local.write_text(yaml.safe_dump(data), encoding='utf-8')

    def test_missing_local_file_preserves_offline_settings_and_requires_explicit_host(self):
        actual = config.load_config()
        self.assertEqual(actual, config.load_yaml(config.DEFAULT_CONFIG))
        with self.assertRaisesRegex(ValueError, 'local.example.yaml.*morai.host'):
            config.require_morai_endpoint(actual['morai'])

    def test_example_has_no_personal_defaults_and_requires_host_before_connecting(self):
        self.local.write_text((config.TOOL_ROOT/'config/local.example.yaml').read_text())
        local = config.load_local_config()
        self.assertIsNone(local['morai']['host'])
        self.assertIsNone(local['dll']['managed_path'])
        with self.assertRaisesRegex(ValueError, 'morai.host'):
            config.require_morai_endpoint(config.load_config()['morai'])

    def test_only_endpoint_changes_shared_scenario_settings(self):
        self.write(dict(morai=dict(host='morai.example.test', port=17789),
                        dll=dict(managed_path='/mnt/c/Some Simulator/Simulator_Data/Managed')))
        expected = config.load_yaml(config.DEFAULT_CONFIG)
        expected['morai'].update(host='morai.example.test', port=17789)
        actual = config.load_config()
        self.assertEqual(actual, expected)
        self.assertEqual(config.require_morai_endpoint(actual['morai']), ('morai.example.test', 17789))
        self.assertNotIn('dll', actual)

    def test_host_only_retains_shared_default_port(self):
        self.write(dict(morai=dict(host='morai.example.test')))
        self.assertEqual(config.require_morai_endpoint(config.load_config()['morai']), ('morai.example.test', 7789))

    def test_explicit_shared_config_retains_its_objects_and_gets_local_endpoint(self):
        shared = self.root/'scenario.yaml'
        shared.write_text(yaml.safe_dump(dict(objects=[dict(speed=18)], morai=dict(host='old', port=7789))))
        self.write(dict(morai=dict(host='new.example.test')))
        self.assertEqual(config.load_config(shared),
                         dict(objects=[dict(speed=18)], morai=dict(host='new.example.test', port=7789)))

    def test_malformed_or_unrelated_local_settings_are_rejected(self):
        for data in ([], 0, dict(map={}), dict(morai=['host']), dict(morai=dict(poll_hz=100)),
                     dict(dll=dict(managed='typo'))):
            with self.subTest(data=data):
                self.write(data)
                with self.assertRaisesRegex(ValueError, 'local.yaml'):
                    config.load_config()
        self.local.write_text('morai: [')
        with self.assertRaisesRegex(ValueError, '개인 설정 YAML 오류'):
            config.load_config()

    def test_invalid_endpoint_fails_before_creating_a_simulator_client(self):
        for host, port in [(None, 7789), (' ', 7789), (123, 7789), ('morai', 0),
                           ('morai', 65536), ('morai', True), ('morai', '7789')]:
            with self.subTest(host=host, port=port):
                base = config.load_config();base['morai'].update(host=host, port=port)
                link = MoraiLink(base)
                with patch('proto.sim_adapter.SimAdapter') as client:
                    with self.assertRaisesRegex(ValueError, 'morai.host|morai.port'):
                        link.connect()
                    client.assert_not_called()

    def test_cli_without_host_stops_before_qt_or_simulator_setup(self):
        from scen_runner.ui import validation_gui as gui
        for args in (['--mode', 'scene'], ['--mode', 'section', '--section', 'dynamic'],
                     ['--replay-browser']):
            with self.subTest(args=args), patch.object(sys, 'argv', ['validation_gui', *args]), \
                 patch.object(gui.QtWidgets, 'QApplication') as qt, \
                 contextlib.redirect_stderr(io.StringIO()) as error:
                with self.assertRaises(SystemExit) as stopped:
                    gui.main()
                self.assertEqual(stopped.exception.code, 2)
                self.assertIn('morai.host', error.getvalue())
                self.assertIn(str(self.local), error.getvalue())
                qt.assert_not_called()

    def test_dll_uses_local_path_without_requiring_morai_host(self):
        managed = '/mnt/d/Simulator With Spaces/Simulator_Data/Managed'
        self.write(dict(dll=dict(managed_path=managed)))
        with patch.object(dll_patch, 'windows_path', side_effect=str), \
             patch.object(dll_patch.subprocess, 'run', return_value=S(stdout=b'ok', returncode=0)) as process:
            self.assertEqual(dll_patch.main(['build-fix']), 0)
        command = process.call_args.args[0]
        self.assertEqual(command[command.index('-ManagedPath')+1], managed)
        self.assertIn('-StaticContacts', command)

    def test_dll_explicit_path_overrides_even_broken_local_file(self):
        self.local.write_text('morai: [')
        managed = '/mnt/e/Other Simulator/Simulator_Data/Managed'
        with patch.object(dll_patch, 'windows_path', side_effect=str), \
             patch.object(dll_patch.subprocess, 'run', return_value=S(stdout=b'ok', returncode=0)) as process:
            self.assertEqual(dll_patch.main(['restore', '--managed', managed]), 0)
        command = process.call_args.args[0]
        self.assertEqual(command[command.index('-ManagedPath')+1], managed)
        self.assertIn('-Restore', command)

    def test_dll_missing_path_stops_before_windows_commands(self):
        for local in ({}, dict(dll=dict(managed_path=None)), dict(dll=dict(managed_path='  '))):
            with self.subTest(local=local):
                self.write(local)
                with patch.object(dll_patch, 'windows_path') as windows, \
                     patch.object(dll_patch.subprocess, 'run') as process, \
                     contextlib.redirect_stderr(io.StringIO()) as error:
                    with self.assertRaises(SystemExit) as stopped:
                        dll_patch.main(['install-fix'])
                    self.assertEqual(stopped.exception.code, 2)
                    self.assertIn('dll.managed_path', error.getvalue())
                    self.assertIn('--managed', error.getvalue())
                    windows.assert_not_called();process.assert_not_called()

    def test_collision_provenance_does_not_record_personal_config(self):
        from scen_runner.records import collision_archive as archive
        self.write(dict(morai=dict(host='private.example.test')))
        (self.local.parent/'shared.yaml').write_text('speed: 18\n')
        with patch.object(archive, 'TOOL_ROOT', self.root), \
             patch.object(archive, 'LOCAL_CONFIG', self.local), \
             patch.object(archive.subprocess, 'check_output', return_value=''):
            record = archive.provenance()
        self.assertIn(str(self.local.parent/'shared.yaml'), record['sha256'])
        self.assertNotIn(str(self.local), record['sha256'])
        self.assertNotIn('private.example.test', json.dumps(record))


if __name__ == '__main__':unittest.main()
