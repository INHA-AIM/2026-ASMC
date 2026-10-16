"""Real ROS master, isolated port; no simulator or driving nodes are started."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.orchestration.ros_registry import proxy, prune_stale_planning_nodes
from scen_runner.orchestration.stack import check_conflicts

try:
    from rosmaster.master import Master
except ImportError:
    Master = None


NODE = r'''
import json, os, sys, xmlrpc.client
from xmlrpc.server import SimpleXMLRPCServer
server = SimpleXMLRPCServer(('127.0.0.1', 0), logRequests=False)
server.register_function(lambda caller: [1, '', os.getpid()], 'getPid')
server.register_function(lambda *args: [1, '', 0], 'publisherUpdate')
uri = 'http://127.0.0.1:%d/' % server.server_address[1]
master = xmlrpc.client.ServerProxy(sys.argv[1])
name = sys.argv[2]
master.registerPublisher(name, '/planning/trajectory', 'std_msgs/String', uri)
master.registerSubscriber(name, '/planning/objects', 'std_msgs/String', uri)
master.registerService(name, name+'/get_loggers', 'rosrpc://127.0.0.1:12345', uri)
print(json.dumps({'uri': uri, 'pid': os.getpid()}), flush=True)
server.serve_forever()
'''


@unittest.skipIf(Master is None, 'Run in the ROS container')
class RealMasterTest(unittest.TestCase):
    def test_killed_registered_node_can_restart_but_live_node_cannot_be_taken_over(self):
        master = Master(port=0)
        master.start()
        self.addCleanup(master.stop)
        name = '/scen_runner_registry_regression'
        nodes = [S(namespace='/', name=name[1:])]
        children = []
        def cleanup():
            for child in children:
                if child.poll() is None:
                    child.kill()
                child.wait(timeout=3)
                child.stdout.close()
        self.addCleanup(cleanup)
        def start():
            child = subprocess.Popen([sys.executable, '-c', NODE, master.uri, name,
                                      '__name:='+name[1:]], stdout=subprocess.PIPE, text=True)
            children.append(child)
            self.assertTrue(select.select([child.stdout], [], [], 5)[0], 'test node startup timed out')
            return child, json.loads(child.stdout.readline())
        with patch.dict(os.environ, ROS_MASTER_URI=master.uri), proxy(master.uri) as api:
            child, info = start()
            graph = lambda: api.getSystemState('/test')[2]
            # A live process must remain a hard conflict.
            self.assertEqual(prune_stale_planning_nodes(nodes, graph()), [])
            with self.assertRaisesRegex(RuntimeError, '중복'):
                check_conflicts('planning', nodes, graph())
            # A separate stale name must not be swept up by targeted cleanup.
            api.registerSubscriber('/unrelated_observer', '/planning/objects', 'std_msgs/String', info['uri'])
            child.terminate()
            child.wait(timeout=3)
            self.assertEqual(api.lookupNode('/test', name)[0], 1)
            self.assertEqual(prune_stale_planning_nodes(nodes, graph()), [name])
            check_conflicts('planning', nodes, graph())
            self.assertEqual(api.lookupNode('/test', name)[0], -1)
            self.assertEqual(api.lookupNode('/test', '/unrelated_observer')[0], 1)
            # A fresh instance now registers and responds on its own API.
            replacement, fresh = start()
            with proxy(fresh['uri']) as node:
                self.assertEqual(node.getPid('/test')[2], replacement.pid)
            self.assertEqual(prune_stale_planning_nodes(nodes, graph()), [])


if __name__ == '__main__':
    unittest.main()
