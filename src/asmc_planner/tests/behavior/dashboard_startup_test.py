#!/usr/bin/env python3
import http.server
from pathlib import Path
import socket
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from dashboard_startup import run_on_port


class PortOwnershipTest(unittest.TestCase):
    def test_busy_port_never_registers_duplicate_ros_node(self):
        with socket.socket() as occupied:
            occupied.bind(('127.0.0.1', 0))
            occupied.listen()
            start = Mock()
            self.assertEqual(2, run_on_port(occupied.getsockname()[1],
                             http.server.BaseHTTPRequestHandler, start, '127.0.0.1'))
            start.assert_not_called()
            # The existing owner remains reachable.
            with socket.create_connection(occupied.getsockname(), timeout=1):
                pass

    def test_reserves_before_ros_and_releases_after_failure(self):
        address = []
        def fail(server):
            address.append(server.server_address)
            with socket.socket() as duplicate:
                with self.assertRaises(OSError):
                    duplicate.bind(server.server_address)
            raise RuntimeError('ROS init failed')
        with self.assertRaisesRegex(RuntimeError, 'ROS init failed'):
            run_on_port(0, http.server.BaseHTTPRequestHandler, fail, '127.0.0.1')
        with socket.socket() as retry:
            retry.bind(address[0])

    def test_normal_exit_propagates_and_releases(self):
        self.assertEqual(17, run_on_port(0, http.server.BaseHTTPRequestHandler,
                                       lambda server: 17, '127.0.0.1'))


if __name__ == '__main__':
    unittest.main()
