import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml
from shapely.geometry import Point,Polygon

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from scen_runner.planning import signal_trial_geometry as geometry
from scen_runner.runtime.signal_trial_oracle import footprint


class GeometryTest(unittest.TestCase):
    def test_c091_conflict_contains_the_actual_post_line_recovery_pose(self):
        # The previous reviewed polygon closed diagonally across the first
        # metres after C091's line. The real car stopped there after the
        # overhead green light left camera view, so recovery never qualified.
        source=ROOT.parents[1]/'src/asmc_planner/config/behavior/signal_policy_geometry_kcity2025.yaml'
        bindings=yaml.safe_load(source.read_text())['reviewed_bindings']
        binding=next(b for b in bindings if b['stop_node_id']=='A1256W000807')
        conflict=Polygon(binding['conflict_polygon'])
        inner_body=footprint(-59.564,-154.905,1.419,3.845-.5,.79-.5,1.0805-.5)
        self.assertTrue(conflict.is_valid)
        self.assertTrue(conflict.intersects(inner_body))
        self.assertTrue(conflict.covers(Point(-59.55,-152)))

    def test_conflict_is_crossing_road_not_everything_after_stop_line(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)
            own=dict(idx='own',from_node_idx='start',points=[[20,0,0],[40,0,0]],width_start=3.5,width_end=3.5)
            cross=dict(idx='cross',from_node_idx='other',points=[[30,-10,0],[30,10,0]],width_start=3.5,width_end=3.5)
            alternative=dict(idx='same_origin_turn',from_node_idx='start',points=[[20,0,0],[35,10,0]],width_start=3.5,width_end=3.5)
            documents={
                'link_set.json':[own,cross,alternative],
                'traffic_light_set.json':[dict(idx='L',link_id_list=['own','same_origin_turn']),dict(idx='X',link_id_list=['cross'])],
                'intersection_controller_set.json':[dict(idx='J',TL=[['L'],['X']])],
                'intersection_controller_data.json':[dict(idx='J',TLState=[dict(lightcolor=['SG','R'],duration=30)])],
                'lane_boundary_set.json':[dict(idx='paint',lane_type=[530],points=[[20,-3,0],[20,3,0]])]}
            for name,data in documents.items():(path/name).write_text(json.dumps(data))
            with patch.object(geometry,'APPROACHES',[('N','L','own','J','straight')]):
                binding=geometry.build(path,[dict(idx='N',point=[20,0,0])])['reviewed_bindings'][0]
            conflict=Polygon(binding['conflict_polygon'])
            self.assertFalse(conflict.covers(Point(22,0)))
            self.assertTrue(conflict.covers(Point(30,0)))
            self.assertEqual(binding['source_conflicting_link_ids'],['cross'])
            self.assertEqual(binding['line_point'],[20.,0.])

    def test_surface_export_rejects_wrong_frame_or_missing_route(self):
        spec=importlib.util.spec_from_file_location('surface_export',ROOT/'export_signal_trial_surface.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        vertices=np.array([[-2,-2,0],[22,-2,0],[22,2,0],[-2,2,0]])
        indices=np.array([[0,1,2],[0,2,3]]);route=np.array([[0,0,0],[10,0,0],[20,0,0]])
        result=module.surface([('road',vertices,indices)],route)
        self.assertEqual(result['route_points_covered'],3)
        wrong=vertices.copy();wrong[:,0]-=50
        with self.assertRaisesRegex(ValueError,'uncovered'):module.surface([('road',wrong,indices)],route)


if __name__=='__main__':unittest.main()
