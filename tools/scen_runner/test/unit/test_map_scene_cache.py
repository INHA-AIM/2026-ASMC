"""Map reuse must preserve placement and never share mutable actor/checker state."""
import copy
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_catalog, load_config, load_yaml, section_config
from scen_runner.planning.feasibility import AvoidanceChecker
from scen_runner.planning.map_scene import (DistanceGrid, Marking, RouteFrame, StaticZoneScene,
                                   _scene_template, load_scene, reusable_scene)
from scen_runner.planning.static_plan import (POLICY_PATH, generate_scene, make_checkers,
                                     runtime_config)


def small_scene(cfg):
    route = RouteFrame(np.array([[0., 0., 0.], [20., 0., 0.]]), (0., 20.))
    grid = DistanceGrid(0., 0., 1., np.ones((3, 4), dtype=bool),
                        np.arange(12.).reshape(3, 4), np.full((3, 4), np.inf))
    marking = Marking('line', 'lane_marking', 'white', 'solid',
                      np.array([[0., 1.], [10., 1.]]), bool(cfg['forbidden_lines']))
    return StaticZoneScene(route, (0., 20.), 0., (20., 0.), 20., 1., [marking],
                           [np.array([[0., 0.], [10., 0.], [10., 10.]])], grid,
                           (0., 20.), dict(source='original'))


class SceneCacheTest(unittest.TestCase):
    def setUp(self):
        _scene_template.cache_clear()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.addCleanup(_scene_template.cache_clear)
        self.cfg = dict(map={}, zone=dict(view_s_range=[0., 20.]),
                        forbidden_lines=[dict(color='white')])
        for name in ('competition_metadata', 'global_path', 'drivable_map', 'lane_markings'):
            path = Path(self.directory.name) / name
            path.write_text('1')
            self.cfg['map'][name] = str(path)
        patcher = patch('scen_runner.planning.map_scene.load_scene', side_effect=small_scene)
        self.loader = patcher.start()
        self.addCleanup(patcher.stop)

    def test_shared_arrays_are_readonly_and_metadata_wrappers_are_private(self):
        first, second = reusable_scene(self.cfg), reusable_scene(copy.deepcopy(self.cfg))
        self.assertEqual(self.loader.call_count, 1)
        self.assertIsNot(first, second)
        self.assertIsNot(first.grid, second.grid)
        self.assertIsNot(first.route, second.route)
        self.assertIs(first.grid.edge_dist, second.grid.edge_dist)
        self.assertIs(first.route._tree, second.route._tree)
        for array in (first.grid.drivable, first.grid.edge_dist, first.grid.line_dist,
                      first.route.s, first.route.x, first.route._tree.data,
                      first.markings[0].points, first.drivable_polygons[0]):
            with self.assertRaises(ValueError):
                array.flat[0] = 99
        first.grid.x0 = 99.
        first.route.heading = np.zeros(1)
        first.markings[0].forbidden = False
        first.markings.clear()
        first.drivable_polygons.clear()
        first.sources['source'] = 'changed'
        self.assertEqual(second.grid.x0, 0.)
        self.assertGreater(len(second.route.heading), 1)
        self.assertTrue(second.markings[0].forbidden)
        self.assertEqual(len(second.drivable_polygons), 1)
        self.assertEqual(second.sources['source'], 'original')

    def test_each_geometry_source_invalidates_even_if_size_and_mtime_match(self):
        previous = reusable_scene(self.cfg)
        for name, filename in self.cfg['map'].items():
            with self.subTest(source=name):
                path = Path(filename)
                old = path.stat()
                path.write_text('2')
                os.utime(path, ns=(old.st_atime_ns, old.st_mtime_ns))
                current = reusable_scene(self.cfg)
                self.assertIsNot(previous.grid.edge_dist, current.grid.edge_dist)
                previous = current
        self.assertEqual(self.loader.call_count, 5)

    def test_zone_and_lane_rules_invalidate_but_unrelated_limits_do_not(self):
        original = reusable_scene(self.cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg['zone']['view_s_range'][0] = 1.
        other_zone = reusable_scene(cfg)
        self.assertIsNot(original.grid.edge_dist, other_zone.grid.edge_dist)
        cfg = copy.deepcopy(self.cfg)
        cfg['forbidden_lines'] = []
        tunnel = reusable_scene(cfg)
        self.assertFalse(tunnel.markings[0].forbidden)
        self.assertTrue(original.markings[0].forbidden)
        self.assertIsNot(tunnel.grid.edge_dist, original.grid.edge_dist)
        cfg = copy.deepcopy(self.cfg)
        cfg['limits'] = dict(max_speed=123.)
        self.assertIs(original.grid.edge_dist, reusable_scene(cfg).grid.edge_dist)
        self.assertEqual(self.loader.call_count, 3)

    def test_cache_does_not_grow_with_unbounded_zone_settings(self):
        for index in range(8):
            cfg = copy.deepcopy(self.cfg)
            cfg['zone']['view_s_range'][0] = float(index)
            reusable_scene(cfg)
        self.assertEqual(_scene_template.cache_info().currsize, 3)


class DistanceLookupTest(unittest.TestCase):
    def test_combined_lookup_preserves_boundaries_shape_and_outside_values(self):
        grid = DistanceGrid(-1., 2., .5, np.ones((3, 4), dtype=bool),
                            np.arange(12.).reshape(3, 4), np.arange(12., 24.).reshape(3, 4))
        for x, y in ((-1., 2.), (1., 2.), (-1.01, 2.), (.99, 3.49),
                     (np.array([[-1., -.5], [1., -2.]]), np.array([[2., 3.5], [3., 1.]]))):
            edge, line = grid.edge_distance(x, y), grid.line_distance(x, y)
            with patch.object(grid, '_index', wraps=grid._index) as index:
                combined_edge, combined_line = grid.distances(x, y)
                self.assertEqual(index.call_count, 1)
            np.testing.assert_array_equal(combined_edge, edge)
            np.testing.assert_array_equal(combined_line, line)


class RealPlacementTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_yaml(POLICY_PATH)
        cls.cfg = runtime_config(load_config(), cls.policy)
        cls.catalog = load_catalog()
        cls.cached = make_checkers(cls.cfg)
        # Keep the pre-optimization grid lookup for an independent comparison.
        class OriginalChecker(AvoidanceChecker):
            def _map_gaps(self, x, y, th):
                cx, cy = self.vehicle.circle_centers(np.ravel(x), np.ravel(y), np.ravel(th))
                edge = (self.scene.grid.edge_distance(cx, cy) - self.r).min(axis=1)
                line = (self.scene.grid.line_distance(cx, cy) - self.r).min(axis=1)
                return edge.reshape(np.shape(x)), line.reshape(np.shape(x))
        cls.original = {}
        for name in cls.cached:
            cfg = section_config(cls.cfg, name)
            cls.original[name] = OriginalChecker(load_scene(cfg), cfg)

    def test_real_route_markings_and_distance_grids_are_exactly_equal(self):
        for name, checker in self.cached.items():
            with self.subTest(section=name):
                scene, original = checker.scene, self.original[name].scene
                for field in ('s', 'x', 'y', 'z', 'heading', 'kappa'):
                    np.testing.assert_array_equal(getattr(scene.route, field), getattr(original.route, field))
                for field in ('drivable', 'edge_dist', 'line_dist'):
                    np.testing.assert_array_equal(getattr(scene.grid, field), getattr(original.grid, field))
                self.assertEqual(scene.grid.extent, original.grid.extent)
                self.assertEqual(scene.sources, original.sources)
                self.assertEqual(scene.zone_s, original.zone_s)
                self.assertEqual(scene.checkpoint_s, original.checkpoint_s)
                self.assertEqual(len(scene.markings), len(original.markings))
                for a, b in zip(scene.markings, original.markings):
                    self.assertEqual((a.marking_id, a.forbidden), (b.marking_id, b.forbidden))
                    np.testing.assert_array_equal(a.points, b.points)

    def test_seeded_placement_and_every_clearance_result_match_original(self):
        for seed, count in ((42, 1), (7, 2)):
            with self.subTest(seed=seed):
                before = generate_scene(self.cfg, self.catalog, self.policy, seed,
                    intersection_count=count, checkers=self.original, progress=lambda *_: None)
                after = generate_scene(self.cfg, self.catalog, self.policy, seed,
                    intersection_count=count, checkers=self.cached, progress=lambda *_: None)
                self.assertEqual([o.to_dict() for o in before[0]], [o.to_dict() for o in after[0]])
                self.assertEqual(before[1], after[1])

    def test_repeated_scene_has_fresh_checker_state_and_limits(self):
        self.cached['intersection']._cache['test'] = {'old_obstacle': True}
        self.addCleanup(self.cached['intersection']._cache.pop, 'test')
        cfg = copy.deepcopy(self.cfg)
        cfg['clearance']['obstacle_m'] = .9
        fresh = make_checkers(cfg)
        self.assertEqual(fresh['intersection']._cache, {})
        self.assertEqual(fresh['intersection'].gap_obstacle, .9)
        self.assertIs(fresh['intersection'].scene.grid.edge_dist,
                      self.cached['intersection'].scene.grid.edge_dist)


if __name__ == '__main__':
    unittest.main()
