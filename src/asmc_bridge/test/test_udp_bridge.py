#!/usr/bin/env python3

import importlib.util
import os
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest import mock


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "morai_udp_bridge.py")
)
sys.path.insert(0, os.path.join(os.path.dirname(SCRIPT), '..', 'src'))
SPEC = importlib.util.spec_from_file_location("morai_udp_bridge_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
FIXTURE = os.path.join(
    os.path.dirname(__file__), "fixtures", "competition_vehicle_status_181.hex"
)


def fixture_packet():
    with open(FIXTURE, "r", encoding="ascii") as fixture:
        return bytes.fromhex(fixture.read())


class VehicleStatusPacketTest(unittest.TestCase):
    def test_competition_fixture_decodes(self):
        decoded = MODULE.decode_vehicle_status_packet(fixture_packet())
        self.assertEqual(181, decoded["packet_length"])
        self.assertEqual(152, decoded["payload_length"])
        self.assertEqual((123, 456789000), (decoded["sec"], decoded["nsec"]))
        self.assertEqual((2, 4), (decoded["ctrl_mode"], decoded["gear"]))
        self.assertAlmostEqual(36.0, decoded["signed_velocity_kph"])
        self.assertEqual(2025, decoded["map_data_id"])
        self.assertAlmostEqual(-131.6898, decoded["position"][0], places=4)
        self.assertAlmostEqual(-428.3310, decoded["position"][1], places=4)
        self.assertAlmostEqual(61.2981, decoded["heading_deg"], places=4)
        self.assertEqual("A2256W000148", decoded["link_id"])
        self.assertEqual((), decoded["extension"])

    def test_declared_payload_length_mismatch_is_rejected(self):
        packet = bytearray(fixture_packet())
        packet[len(MODULE.EGO_HDR):len(MODULE.EGO_HDR) + 4] = struct.pack(
            "<i", 151
        )
        with self.assertRaisesRegex(ValueError, "declared payload"):
            MODULE.decode_vehicle_status_packet(bytes(packet))

    def test_missing_tail_is_rejected(self):
        packet = fixture_packet()[:-2] + b"XX"
        with self.assertRaisesRegex(ValueError, "invalid tail"):
            MODULE.decode_vehicle_status_packet(packet)


class InputRequirementTest(unittest.TestCase):
    def test_competition_status_is_optional_by_default(self):
        self.assertFalse(MODULE.input_required("competition"))
        self.assertTrue(MODULE.input_required("ego"))
        self.assertTrue(MODULE.input_required("col"))

    def test_field_gate_can_require_competition_status(self):
        self.assertTrue(MODULE.input_required("competition", True))

    @staticmethod
    def detail_packet():
        return MODULE.OPTIONAL_DETAIL_EGO_HDR + struct.pack('<i', 105) + b'\0' * 12 + b'\0' * 105 + b'\r\n'

    def test_detail_envelope_is_optional_only_in_local_competition_channel(self):
        packet = self.detail_packet()
        self.assertEqual(len(packet), 138)
        self.assertTrue(MODULE.is_optional_competition_variant('competition', packet, 'local', False))
        for kind, transport, required in [('ego', 'local', False), ('competition', 'lan2lan', False), ('competition', 'local', True)]:
            with self.subTest(kind=kind, transport=transport, required=required):
                self.assertFalse(MODULE.is_optional_competition_variant(kind, packet, transport, required))
        with self.assertRaisesRegex(ValueError, 'header mismatch'):
            MODULE.decode_vehicle_status_packet(packet)

    def test_malformed_detail_and_unknown_headers_stay_errors(self):
        packet = self.detail_packet()
        header_size = len(MODULE.OPTIONAL_DETAIL_EGO_HDR)
        declared_wrong = packet[:header_size] + struct.pack('<i', 104) + packet[header_size + 4:]
        for malformed in (packet[:-1], packet + b'\0', packet[:-2] + b'XX', declared_wrong, b'?' + packet[1:]):
            with self.subTest(packet=malformed[:20]):
                self.assertFalse(MODULE.is_optional_competition_variant('competition', malformed, 'local', False))


class ReportingStatusTest(unittest.TestCase):
    def test_send_failure_is_visible_then_idle_recovery_preserves_history(self):
        telemetry = MODULE.BridgeTelemetry()
        telemetry.register('cmd', transmit=True)
        telemetry.increment('cmd', 'send_errors')
        owner = SimpleNamespace(telemetry=telemetry, _health={},
            _next_console=float('inf'), reporter=SimpleNamespace(failures=0),
            runtime_profile='morai_udp', transport_profile='local',
            stream_profile='competition', timestamp_mode='receive',
            pub_diagnostics=mock.Mock())
        with mock.patch.object(MODULE.rospy.Time, 'now', return_value=MODULE.rospy.Time(1)), \
                mock.patch.object(MODULE.rospy, 'logwarn') as warning:
            MODULE.publish_report(owner, 'morai_udp_bridge')
        status = owner.pub_diagnostics.publish.call_args[0][0].status[0]
        self.assertEqual(status.level, status.WARN)
        self.assertEqual(status.message, 'recent packet or send errors')
        self.assertEqual(warning.call_args[0][5], 1)
        with mock.patch.object(MODULE.rospy.Time, 'now', return_value=MODULE.rospy.Time(2)):
            MODULE.publish_report(owner, 'morai_udp_bridge')
        status = owner.pub_diagnostics.publish.call_args[0][0].status[0]
        self.assertEqual(status.level, status.OK)
        self.assertEqual({value.key: value.value for value in status.values}['send_errors'], '1')

    def test_optional_unsupported_is_visible_but_not_usable_or_warning(self):
        telemetry = MODULE.BridgeTelemetry()
        telemetry.register('competition', optional=True)
        telemetry.unsupported('competition', '#DetailEgoInfo$')
        owner = SimpleNamespace(telemetry=telemetry, _health={},
            _next_console=float('inf'), reporter=SimpleNamespace(failures=0),
            runtime_profile='morai_udp', transport_profile='local',
            stream_profile='competition', timestamp_mode='receive',
            last_competition_link_id='', pub_diagnostics=mock.Mock())
        with mock.patch.object(MODULE.rospy.Time, 'now', return_value=MODULE.rospy.Time(1)), \
                mock.patch.object(MODULE.rospy, 'logwarn') as warning:
            MODULE.publish_report(owner, 'morai_udp_bridge')
        warning.assert_not_called()
        status = owner.pub_diagnostics.publish.call_args[0][0].status[0]
        self.assertEqual(status.level, status.OK)
        values = {value.key: value.value for value in status.values}
        self.assertEqual(values['health'], 'UNSUPPORTED')
        self.assertEqual(values['messages'], '0')
        self.assertEqual(values['data_available'], 'false')
        self.assertEqual(values['unsupported_packets'], '1')
        self.assertEqual(values['header_errors'], '0')


class SourceTimestampTest(unittest.TestCase):
    def bridge(self):
        value = MODULE.MoraiUdpBridge.__new__(MODULE.MoraiUdpBridge)
        value.timestamp_mode = "source"
        value.telemetry = MODULE.BridgeTelemetry()
        value.telemetry.register('ego')
        value.telemetry.register('competition')
        value._last_source_ns = {}
        return value

    def test_valid_packet_stamp_is_preserved(self):
        bridge = self.bridge()
        stamp = bridge.packet_stamp(123, 456, "ego")
        self.assertEqual(stamp.secs, 123)
        self.assertEqual(stamp.nsecs, 456)
        self.assertEqual(bridge.telemetry.snapshot()[2]['ego']['timestamp_fallbacks'], 0)

    def test_regression_falls_back_and_is_counted(self):
        bridge = self.bridge()
        bridge.packet_stamp(123, 456, "ego")
        with mock.patch.object(
            MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time(999, 1)
        ):
            stamp = bridge.packet_stamp(122, 456, "ego")
        self.assertEqual(stamp, MODULE.rospy.Time(999, 1))
        row = bridge.telemetry.snapshot()[2]['ego']
        self.assertEqual(row['timestamp_fallbacks'], 1)
        self.assertEqual(row['timestamp_regressions'], 1)


if __name__ == "__main__":
    unittest.main()
