"""BDD binary inspection must coexist with production without claiming its contract."""
from pathlib import Path
import importlib.util
import json
import sys
from types import ModuleType
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import roslaunch

from asmc_camera.camera.geometry import letterbox_bgr

PACKAGE = Path(__file__).resolve().parents[2]


class BddExperimentTest(unittest.TestCase):
    def config(self, filename, *args):
        return roslaunch.config.load_config_default([(str(PACKAGE/'launch'/filename),list(args))],None,verbose=False)

    def test_producer_is_binary_and_has_no_family_bev_or_object_workers(self):
        cfg=self.config('bdd_lane.launch')
        self.assertEqual([n.name for n in cfg.nodes],['camera_lane_bdd'])
        params={k:v.value for k,v in cfg.params.items()}
        self.assertEqual(params['/camera_lane_bdd/model_config'],'large')
        self.assertEqual(params['/camera_lane_bdd/backend'],'twinlite')
        self.assertFalse(params['/camera_lane_bdd/publish_model_grid_road_marking'])
        self.assertEqual(params['/camera_lane_bdd/input_hw_by_view/front'],[544,960])
        self.assertEqual(params['/camera_lane_bdd/input_hw'],[480,640])
        self.assertEqual(params['/camera_lane_bdd/resize_interpolation'],'linear')
        self.assertEqual(params['/camera_lane_bdd/batch_wait_ms'],20.)
        remaps=dict(cfg.nodes[0].remap_args)
        expected=['/perception/camera/lane/status','/perception/camera/lane/set_mode',
                  '/camera_semantic_inference/inference_status','/camera_semantic_inference/set_inference_mode',
                  '/perception/camera/diagnostics','/perception/lane/boundary_family']
        expected += ['/perception/camera/{}/{}/probability'.format(v,h)
                     for v in ('front','left','right') for h in ('drivable','lane')]
        for path in expected:
            self.assertTrue(remaps[path].startswith('/experiments/twinlite_bdd/'),path)

    def test_official_input_is_explicit_and_invalid_resolution_fails(self):
        cfg=self.config('bdd_lane.launch','resolution:=official')
        self.assertEqual(cfg.params['/camera_lane_bdd/input_hw'].value,[384,640])
        self.assertFalse(any(k.startswith('/camera_lane_bdd/input_hw_by_view/') for k in cfg.params))
        with self.assertRaises(roslaunch.RLException):self.config('bdd_lane.launch','resolution:=bad')

    def test_production_and_experiment_have_distinct_node_names(self):
        cfg=roslaunch.config.load_config_default([(str(PACKAGE/'launch/camera.launch'),[]),
                    (str(PACKAGE/'launch/bdd_lane.launch'),[])],None,verbose=False)
        names=[n.name for n in cfg.nodes]
        self.assertEqual(len(names),len(set(names)))
        self.assertEqual(cfg.params['/camera_lane/backend'].value,'twinlite_tensorrt')

    def test_view_has_only_renderer_and_no_production_output(self):
        cfg=self.config('bdd_lane_view.launch')
        self.assertEqual([n.type for n in cfg.nodes],['semantic_visualizer_node.py'])
        self.assertEqual(cfg.params['/camera_lane_bdd_view/secondary_mode'].value,'lane')
        self.assertEqual(cfg.params['/camera_lane_bdd_view/lane_color_bgr'].value,[255,230,0])
        remaps=dict(cfg.nodes[0].remap_args)
        self.assertEqual(remaps['/perception/camera/debug/mosaic/compressed'],
                         '/experiments/twinlite_bdd/debug/mosaic/compressed')
        for v in ('front','left','right'):
            for h in ('lane','drivable'):
                path='/perception/camera/{}/{}/probability'.format(v,h)
                self.assertEqual(remaps[path],path.replace('/perception/camera','/experiments/twinlite_bdd'))

    def test_linear_preprocessing_matches_official_letterbox_equations(self):
        rng=np.random.RandomState(12)
        for hw,target in [((720,1280),(384,640)),((480,640),(384,640)),((719,1280),(544,960))]:
            source=rng.randint(0,256,size=hw+(3,),dtype=np.uint8)
            ratio=min(target[0]/hw[0],target[1]/hw[1])
            wh=(int(round(hw[1]*ratio)),int(round(hw[0]*ratio)))
            dw,dh=(target[1]-wh[0])/2,(target[0]-wh[1])/2
            ref=cv2.resize(source,wh,interpolation=cv2.INTER_LINEAR)
            ref=cv2.copyMakeBorder(ref,int(round(dh-.1)),int(round(dh+.1)),int(round(dw-.1)),int(round(dw+.1)),cv2.BORDER_CONSTANT,value=(114,114,114))
            expected=np.ascontiguousarray(ref[:,:,::-1].transpose(2,0,1)).astype(np.float32)/255
            actual,geometry=letterbox_bgr(source,target_hw=target,interpolation='linear')
            np.testing.assert_allclose(actual,expected,rtol=0,atol=6e-8)
            self.assertEqual(geometry.target_hw,target)
        with self.assertRaises(ValueError):letterbox_bgr(source,interpolation='invalid')

    @unittest.skipUnless(Path('/opt/baselines/TwinLiteNetPlus/model/model.py').is_file()
                         and importlib.util.find_spec('torch'), 'optional official source/PyTorch fixture')
    def test_repeated_scales_accept_the_same_namespace_and_reject_foreign_source(self):
        from asmc_camera.training.twinlite.external_model import load_external_twinlite, ExternalTwinLiteError
        large,commit=load_external_twinlite('large')
        medium,again=load_external_twinlite('medium')
        self.assertEqual(commit,again)
        self.assertGreater(sum(p.numel() for p in large.parameters()),sum(p.numel() for p in medium.parameters()))
        foreign=ModuleType('model');foreign.__path__=['/tmp/unrelated-model']
        with patch.dict(sys.modules,{'model':foreign}), self.assertRaises(ExternalTwinLiteError):
            load_external_twinlite('large')

    def test_bdd_provenance_preserves_binary_classes_and_original_tensor_identity(self):
        manifest=json.loads((PACKAGE/'models/lane_segmentation/twinlite-large-bdd100k.source.json').read_text())
        self.assertEqual(manifest['task_spec']['heads'],['drivable','lane'])
        self.assertEqual(manifest['task_spec']['class_counts'],{'drivable':2,'lane':2})
        self.assertTrue(manifest['all_source_tensors_identical'])
        self.assertFalse(manifest['training_performed'])


if __name__=='__main__':unittest.main()
