import math
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch, Mock
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.planning.random_crossing import CrossingGenerator, approach_pose
from scen_runner.morai.episode_link import SceneLink
from scen_runner.config import load_config


class CrossingTest(unittest.TestCase):
    def test_random_cases_cover_models_speeds_directions_and_timing(self):
        generator = CrossingGenerator(320)
        models, speeds, directions = set(), set(), set()
        for _ in range(300):
            plan = generator.next_plan()
            models.add(plan.models[0][1])
            speeds.add(plan.cfg['walk_speed_kmh'])
            directions.add(plan.direction_label)
            t = plan.timing
            expected_lateral = 6 if plan.direction_label.startswith('왼쪽') else 8
            x, y, _, heading = plan.ego.pose(t['cross_station_m'])
            lateral = -(plan.start[0]-x)*math.sin(heading) + (plan.start[1]-y)*math.cos(heading)
            self.assertAlmostEqual(lateral, expected_lateral if expected_lateral == 6 else -8)
            self.assertEqual(t['stop_timing_feasible'],
                             t['entry_s'] < t['stop_s'] < t['exit_s']
                             and t['front_gap_m'] >= t['required_gap_m'])
            self.assertLess(t['entry_s'], t['center_s'])
            self.assertGreater(t['exit_s'], t['center_s'])
            self.assertAlmostEqual(math.dist(plan.activation_xy, plan.start[:2]),
                                   plan.cfg['activation_distance_m'])
            self.assertAlmostEqual(math.dist(plan.start[:2], plan.end[:2]), plan.move_distance)
            rear = approach_pose(plan.ego, t['activation_station_m'] + 25/3.6*t['center_s'])
            front = (rear[0] + generator.vehicle.front_extent*math.cos(rear[3]),
                     rear[1] + generator.vehicle.front_extent*math.sin(rear[3]))
            self.assertLess(math.dist(front, plan.cross_xy), .1)
            upstream = approach_pose(plan.ego, t['activation_station_m'] - .5)
            self.assertGreater(math.dist(upstream[:2], plan.start[:2]),
                               plan.cfg['activation_distance_m'])
        self.assertEqual(models, {'Man1','Woman1','Boy1','Girl1','Deer1','Dog1','Ped_Pet1'})
        self.assertEqual(speeds, {15,16,17,18})
        self.assertEqual(len(directions), 6)

    def test_nearby_positions_report_inadequate_braking_without_relocating(self):
        generator = CrossingGenerator(1)
        plans = [generator.next_plan() for _ in range(30)]
        self.assertTrue(any(not p.timing['stop_timing_feasible'] for p in plans))
        self.assertEqual({p.timing['start_lateral_m'] for p in plans}, {6, 8})

    def test_seed_reproduces_scene_sequence(self):
        a, b = CrossingGenerator(42), CrossingGenerator(42)
        for _ in range(10):
            x, y = a.next_plan(), b.next_plan()
            self.assertEqual((x.models, x.start, x.end, x.cfg, x.timing),
                             (y.models, y.start, y.end, y.cfg, y.timing))

    def test_negative_station_extends_upstream(self):
        curve = CrossingGenerator(1).ego
        self.assertAlmostEqual(math.dist(approach_pose(curve, -5)[:2], curve.pose(0)[:2]), 5)

    def test_computed_values_sent_to_native_spawn_and_waypoints(self):
        plan = CrossingGenerator(3).next_plan()
        link = SceneLink(load_config())
        stub = Mock()
        stub.SpawnPedestrian.return_value = SimpleNamespace(status=1,custom_message='7')
        stub.SetPedestrianWaypoint.return_value = SimpleNamespace(status=1)
        link._sa = SimpleNamespace(_actor_stub=stub)
        aid = link.spawn_crossing(plan, with_route=False)
        link.register_crossing(aid, plan)
        spawn = stub.SpawnPedestrian.call_args.args[0]
        self.assertAlmostEqual(spawn.velocity, plan.cfg['walk_speed_kmh'])
        self.assertAlmostEqual(spawn.active_dist, plan.cfg['activation_distance_m'], places=4)
        self.assertFalse(spawn.start_action)
        route = stub.SetPedestrianWaypoint.call_args.args[0]
        self.assertEqual(len(route.waypoints.waypoints), 3)
        for point in route.waypoints.waypoints:
            self.assertEqual(point.speed, plan.cfg['walk_speed_kmh'])
        self.assertEqual([call[0] for call in stub.mock_calls], ['SpawnPedestrian','SetPedestrianWaypoint'])



if __name__ == '__main__':
    unittest.main()
