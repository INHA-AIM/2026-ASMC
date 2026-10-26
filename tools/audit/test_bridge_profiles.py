#!/usr/bin/env python3

import sys
import os
import json
import subprocess
import unittest
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools/audit"))
from validate_bridge_profiles import validate  # noqa: E402
from render_morai_network_profiles import load_contract, render_profile  # noqa: E402


class BridgeProfileContractTest(unittest.TestCase):
    def test_r1_controller_selection_matches_verified_export(self):
        contract = load_contract()
        source = json.loads((REPO_ROOT / contract['network_templates']['26r1']['template']).read_text())
        original = copy.deepcopy(source)
        for transport in ('local', 'lan2lan'):
            with self.subTest(transport=transport):
                rendered = render_profile(contract, '26r1', transport)
                control = next(e for ego in rendered['egoNetworkData'] for e in ego['listEgoNetworkInfo'] if e['Division_TYPE']==3)
                self.assertEqual(control['PUBSUB_TYPE'], 771)
                self.assertEqual(control['EgoCtrlConfig']['_PUBSUB_TYPE'], 771)
                self.assertEqual(control['EgoCtrlConfig']['egoCtrlMode'], 3)
                self.assertEqual(control['EgoCtrlConfig']['udpConfig']['receiveBufferSize'], 59)
        # The historical 786/531-byte source is provenance, not a new default.
        self.assertEqual(source, original)
        self.assertEqual(next(e for ego in source['egoNetworkData'] for e in ego['listEgoNetworkInfo'] if e['Division_TYPE']==3)['PUBSUB_TYPE'],786)

    def test_canonical_udp_boundary_exposes_sign_without_changing_team_default(self):
        launch = ET.parse(REPO_ROOT / 'src/asmc_bridge/launch/morai_udp_bridge.launch').getroot()
        params = {p.get('name'):p.get('value') for p in launch.find('node').findall('param')}
        self.assertEqual(params['invert_steer'], '$(arg invert_steer)')
        self.assertEqual(params['cmd_payload_bytes'], '23')
        self.assertEqual(next(a.get('default') for a in launch.findall('arg') if a.get('name')=='invert_steer'),'false')
        root = ET.parse(REPO_ROOT / 'src/asmc_bridge/launch/bridge.launch').getroot()
        self.assertEqual(next(a.get('default') for a in root.findall('arg') if a.get('name')=='invert_steer'),'false')
        network = next(i for i in root.findall('include') if i.get('file').endswith('/morai_udp_bridge.launch'))
        self.assertEqual(next(a.get('value') for a in network.findall('arg') if a.get('name')=='invert_steer'),'$(arg invert_steer)')

    def test_repository_contract_is_valid(self):
        result = validate(REPO_ROOT / "config/bridge/bridge_profiles_v1.json")
        self.assertEqual("p3g_bridge_local_closeout_v1", result["baseline_id"])
        self.assertEqual("morai_udp", result["runtime_protocol"])
        self.assertEqual("9093/9094 mode 2", result["ctrl"])
        self.assertEqual("9088/9099", result["ego_status"])
        self.assertEqual(
            "9797/9798 type 787; local optional, field gate pending",
            result["competition_status"],
        )
        self.assertFalse(result["competition_status_required_default"])
        self.assertEqual("azimuth", result["lidar_frame_mode"])
        self.assertTrue(result["v3_configured_cycle_accepted"])
        self.assertEqual(674, result["full_graph_tests"])
        self.assertEqual(["lan2lan", "local"], result["transport_profiles"])
        self.assertEqual(4, result["network_profiles"])
        self.assertEqual(4, result["sensor_transport_profiles"])
        self.assertEqual("competition", result["default_stream_profile"])
        self.assertEqual("asmc_bridge/bridge.launch", result["canonical_launch"])
        self.assertEqual(15, result["published_topic_count"])
        self.assertEqual(["asmc_bridge"], result["packages"])

    def test_10hz_regression_and_15hz_deployment_are_distinct(self):
        result = validate(REPO_ROOT / "config/bridge/bridge_profiles_v1.json")
        self.assertAlmostEqual(0.1, result["sensor_sets"]["v2"]["lidar_period_s"])
        self.assertAlmostEqual(1.0 / 15.0, result["sensor_sets"]["v3"]["lidar_period_s"])

    def test_gt_updates_both_lidars_and_preserves_sensor_identity(self):
        result = validate(REPO_ROOT / "config/bridge/bridge_profiles_v1.json")
        self.assertEqual([2, 2], result['sensor_sets']['GT']['lidar_frequency_indices'])
        for period in result['sensor_sets']['GT']['lidar_periods_s']:
            self.assertAlmostEqual(1.0 / 15.0, period)
        sensors = json.loads((REPO_ROOT / 'config/morai/sensors/ASMC_sensor_set_GT.json').read_text())
        self.assertEqual([5, 11], [row['m_SensorUniqueID'] for row in sensors['Lidar3DList']])
        self.assertEqual([9299, 9311], [row['lc']['udpConfig']['destinationPort'] for row in sensors['Lidar3DList']])


class BridgeCommandLineTest(unittest.TestCase):
    def parse(self, *args, extra_env=None):
        path = REPO_ROOT / 'scripts/udp_bridge.sh'
        # Exercise the actual parser before any ROS sourcing, port binding, or
        # model execution. No parser implementation is duplicated in the test.
        prefix = path.read_text().split('NETWORK_PRESET_26R1=', 1)[0]
        report = '\nprintf "%s\\n" "$TRANSPORT_PROFILE" "$STREAM_PROFILE" "$REQUIRE_COMPETITION_STATUS" "${LAUNCH_ARGS[@]}"\n'
        env = dict(os.environ, ASMC=str(REPO_ROOT))
        for key in ('ASMC_TRANSPORT', 'ASMC_STREAMS'):
            env.pop(key, None)
        env.update(extra_env or {})
        return subprocess.run(['bash', '-c', prefix + report, str(path), *args],
                              env=env, capture_output=True, text=True, timeout=5)

    def test_default_and_gt_keep_timestamp_and_overrides_explicit(self):
        default = self.parse()
        self.assertEqual(default.returncode, 0, default.stderr)
        self.assertEqual(default.stdout.splitlines()[:3], ['local', 'competition', 'false'])
        gt = self.parse('--gt', 'timestamp_mode:=source', 'cmd_port:=9093')
        self.assertEqual(gt.returncode, 0, gt.stderr)
        self.assertEqual(gt.stdout.splitlines(), ['local', 'development', 'false',
                         'timestamp_mode:=source', 'cmd_port:=9093'])
        steering = self.parse('invert_steer:=true')
        self.assertEqual(steering.returncode, 0, steering.stderr)
        self.assertEqual(steering.stdout.splitlines(),
                         ['local', 'competition', 'false', 'invert_steer:=true'])

    def test_transport_and_field_gate_remain_independent_of_gt(self):
        field = self.parse('--transport', 'lan2lan', '--require-competition-status')
        self.assertEqual(field.returncode, 0, field.stderr)
        self.assertEqual(field.stdout.splitlines()[:3], ['lan2lan', 'competition', 'true'])
        for args in (('--gt', '--transport', 'lan2lan'), ('--gt', '--require-competition-status')):
            result = self.parse(*args)
            self.assertEqual(result.returncode, 2, args)

    def test_obsolete_or_unknown_selectors_fail_before_ros_startup(self):
        for args in (('--streams', 'development'), ('--streams=competition',),
                     ('stream_profile:=development',), ('--gt=false',), ('--unknown',)):
            result = self.parse(*args)
            self.assertEqual(result.returncode, 2, args)
        self.assertEqual(self.parse(extra_env={'ASMC_STREAMS': 'development'}).returncode, 2)


if __name__ == "__main__":
    unittest.main()
