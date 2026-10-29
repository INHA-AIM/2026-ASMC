from pathlib import Path
import socket
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from standalone import ports_from_params,require_tcp_free
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scen_runner'))
from scen_runner.runtime.private_master import PrivateMaster

class StandaloneOwnershipTest(unittest.TestCase):
    def test_command_destination_is_not_mistaken_for_receive_port(self):
        p={'/morai_udp_bridge/enable_ego_status':True,'/morai_udp_bridge/ego_port':9099,
           '/morai_udp_bridge/cmd_ip':'127.0.0.1','/morai_udp_bridge/cmd_port':9093,
           '/morai_sensor_bridge/enable_camera':True,'/morai_sensor_bridge/cameras':[{'port':9291}],
           '/behavior_dashboard_server/port':8088,'/perception/lidar/lidar_monitor/port':8091}
        plan=ports_from_params(p,11311)
        self.assertEqual(plan['master'],11311)
        self.assertEqual(plan['udp'],[9099,9291]);self.assertNotIn(9093,plan['udp'])
        self.assertEqual(plan['http'],[8088,8091])
    def test_duplicate_web_or_udp_is_refused(self):
        with self.assertRaisesRegex(ValueError,'Duplicate master/web'):
            ports_from_params({'/behavior_dashboard_server/port':8088},8088)
        with self.assertRaisesRegex(ValueError,'Duplicate UDP'):
            ports_from_params({'/morai_sensor_bridge/enable_camera':True,
                               '/morai_sensor_bridge/cameras':[{'port':9291},{'port':9291}]},11413)
    def test_busy_master_is_not_reused_or_replaced(self):
        with socket.socket() as s:
            s.bind(('127.0.0.1',0));s.listen();port=s.getsockname()[1]
            with self.assertRaisesRegex(RuntimeError,'already owned'):require_tcp_free([port])
            with socket.create_connection(('127.0.0.1',port),timeout=.2):pass
    def test_disabled_stream_ports_are_not_reserved(self):
        self.assertEqual(ports_from_params({'/morai_udp_bridge/enable_object':False,
                                          '/morai_udp_bridge/object_port':7505},11413)['udp'],[])
    def test_recently_closed_listener_is_not_a_live_owner(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            server.bind(('127.0.0.1',0));server.listen();port=server.getsockname()[1]
            with socket.create_connection(('127.0.0.1',port),timeout=.2) as client:
                accepted,_=server.accept();accepted.close()
                self.assertEqual(client.recv(1),b'')
        require_tcp_free([port])
    def test_strict_master_refuses_busy_port_before_launching_anything(self):
        with socket.socket() as s:
            s.bind(('127.0.0.1',0));s.listen();port=s.getsockname()[1]
            owner=PrivateMaster('/unused',port,strict_port=True)
            with self.assertRaisesRegex(RuntimeError,'no existing master was reused'):
                owner.__enter__()
            self.assertIsNone(owner.child)
            with socket.create_connection(('127.0.0.1',port),timeout=.2):pass
if __name__=='__main__':unittest.main()
