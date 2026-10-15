import copy
from contextlib import nullcontext
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.orchestration import ros_registry as registry
from scen_runner.orchestration.stack import check_conflicts


class Master:
    def __init__(self):
        self.uri = 'http://localhost:12345/'
        self.graph = [[['/planning/trajectory', ['/asmc_planner']]],
                      [['/planning/objects', ['/asmc_planner', '/observer']]],
                      [['/asmc_planner/get_loggers', ['/asmc_planner']]]]
        self.calls = []

    def lookupNode(self, caller, name):
        return [1, '', self.uri]

    def getSystemState(self, caller):
        return [1, '', copy.deepcopy(self.graph)]

    def lookupService(self, caller, service):
        return [1, '', 'rosrpc://localhost:23456']

    def remove(self, index, name, topic, api):
        self.calls.append((index, name, topic, api))
        for row in self.graph[index]:
            if row[0] == topic:
                row[1].remove(name)
        self.graph[index] = [row for row in self.graph[index] if row[1]]
        return [1, '', 1]

    def unregisterPublisher(self, name, topic, api):
        return self.remove(0, name, topic, api)

    def unregisterSubscriber(self, name, topic, api):
        return self.remove(1, name, topic, api)

    def unregisterService(self, name, topic, api):
        return self.remove(2, name, topic, api)


class RegistryTest(unittest.TestCase):
    def setUp(self):
        self.master = Master()
        self.nodes = [S(namespace='/', name='asmc_planner')]
        for target, value in (('proxy', lambda uri: nullcontext(self.master)),
                              ('local_process_exists', False), ('connection_refused', True)):
            if callable(value):
                p = patch.object(registry, target, side_effect=value)
            else:
                p = patch.object(registry, target, return_value=value)
            p.start()
            self.addCleanup(p.stop)

    def prune(self):
        return registry.prune_stale_planning_nodes(self.nodes, self.master.graph)

    def test_dead_registration_unblocks_restart_without_removing_observer(self):
        with self.assertRaisesRegex(RuntimeError, '중복'):
            check_conflicts('planning', self.nodes, self.master.graph)
        self.assertEqual(self.prune(), ['/asmc_planner'])
        check_conflicts('planning', self.nodes, self.master.graph)
        self.assertEqual(self.master.graph, [[], [['/planning/objects', ['/observer']]], []])
        self.assertEqual({c[1] for c in self.master.calls}, {'/asmc_planner'})
        self.assertEqual({c[3] for c in self.master.calls if c[0] != 2}, {self.master.uri})

    def test_live_or_unresponsive_node_is_still_blocked(self):
        registry.connection_refused.return_value = False
        self.assertEqual(self.prune(), [])
        self.assertEqual(self.master.calls, [])
        with self.assertRaisesRegex(RuntimeError, '중복'):
            check_conflicts('planning', self.nodes, self.master.graph)

    def test_process_still_initializing_is_not_removed(self):
        registry.local_process_exists.return_value = True
        self.assertEqual(self.prune(), [])
        self.assertEqual(self.master.calls, [])
        registry.connection_refused.assert_not_called()

    def test_replacement_at_new_uri_or_recovered_node_is_not_removed(self):
        for change in ('uri', 'recovered', 'process'):
            with self.subTest(change=change):
                self.master = Master()
                registry.local_process_exists.side_effect = None
                if change == 'uri':
                    def refused(uri):
                        self.master.uri = 'http://localhost:54321/'
                        return True
                    registry.connection_refused.side_effect = refused
                elif change == 'recovered':
                    registry.connection_refused.side_effect = [True, False]
                else:
                    registry.connection_refused.side_effect = None
                    registry.local_process_exists.side_effect = [False, True]
                self.assertEqual(self.prune(), [])
                self.assertEqual(self.master.calls, [])

    def test_unrelated_or_remote_registrations_are_not_removed(self):
        self.nodes = [S(namespace='/', name='different_planner')]
        self.assertEqual(self.prune(), [])
        self.nodes = [S(namespace='/', name='asmc_planner')]
        self.master.uri = 'http://remote-machine:12345/'
        self.assertEqual(self.prune(), [])
        self.assertEqual(self.master.calls, [])

    def test_unregistration_stops_if_another_instance_claims_the_name(self):
        original = self.master.unregisterService
        def replace(*args):
            result = original(*args)
            self.master.uri = 'http://localhost:54321/'
            return result
        self.master.unregisterService = replace
        self.prune()
        self.assertEqual(len(self.master.calls), 1)
        self.assertEqual(self.master.graph[0][0][1], ['/asmc_planner'])


class RefusalTest(unittest.TestCase):
    def test_only_connection_refused_establishes_dead_api(self):
        for error, expected in ((ConnectionRefusedError(), True), (TimeoutError(), False),
                                (ConnectionResetError(), False)):
            with self.subTest(error=type(error).__name__):
                with patch.object(registry, 'proxy', side_effect=error):
                    self.assertEqual(registry.connection_refused('http://localhost:12345/'), expected)


if __name__ == '__main__':
    unittest.main()
