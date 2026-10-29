import socket
import unittest
from scen_runner.runtime.private_master import available_port, require_udp_ports_free

class PrivateMasterPortsTest(unittest.TestCase):
    def test_busy_master_is_never_reused(self):
        with socket.socket() as busy:
            busy.bind(('127.0.0.1', 0));busy.listen()
            chosen=available_port(busy.getsockname()[1])
            self.assertNotEqual(chosen,busy.getsockname()[1])
            self.assertNotEqual(chosen,11311)

    def test_shared_master_is_never_selected(self):
        self.assertNotEqual(available_port(11311),11311)

    def test_busy_udp_is_refused_without_reuse(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as busy:
            busy.bind(('0.0.0.0',0))
            with self.assertRaisesRegex(RuntimeError,'already owned'):
                require_udp_ports_free([busy.getsockname()[1]])
