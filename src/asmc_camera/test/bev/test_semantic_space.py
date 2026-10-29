"""S3 free-space projection: no raw RGB dependency or invented visibility."""
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
import yaml
from asmc_camera.bev.calibration import load_calibration_snapshot, calibration_id
from asmc_camera.bev.comparison_projection import ComparisonProjection
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.runtime.comparison_frames import ComparisonFrames

class SemanticSpaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parents[2]
        _,cls.cameras=load_calibration_snapshot(root/'config/cameras_v3.yaml')
        cls.grid=BevGridSpec.from_mapping(yaml.safe_load((root/'config/bev/bev_grid.yaml').read_text()))
        cls.renderer=ComparisonProjection(cls.cameras,cls.grid)
        cls.planes={v:c.ground_plane.coefficients for v,c in cls.cameras.items()}
    def data(self):
        p={v:np.full((c.height,c.width),255,np.uint8) for v,c in self.cameras.items()}
        m={v:np.zeros_like(a,dtype=bool) for v,a in p.items()}
        return p,m
    def test_excluded_camera_pixels_never_become_free(self):
        p,m=self.data()
        for v in m:m[v][:]=True
        out=self.renderer.drivable(self.planes,p,m)
        self.assertFalse(np.any(out['coverage']==255))
        self.assertTrue(np.any(out['coverage']==128))
        self.assertFalse(np.any(out['drivable']))
    def test_low_probability_is_not_filled_by_other_views(self):
        p,m=self.data();p['front'][:]=0
        out=self.renderer.drivable(self.planes,p,m)
        r,c=map(int,[100,80]) # x10,y0, strong front support
        self.assertEqual(out['coverage'][r,c],255)
        self.assertEqual(out['drivable'][r,c],0)
    def test_visibility_margin_only_removes_support(self):
        p,m=self.data();m['front'][580:]=True
        narrow=self.renderer.drivable(self.planes,p,m,1)
        wide=self.renderer.drivable(self.planes,p,m,4)
        self.assertTrue(np.all((wide['coverage']!=255)|(narrow['coverage']==255)))
        self.assertTrue(np.any((narrow['coverage']==255)&(wide['coverage']!=255)))
        with self.assertRaises(ValueError):self.renderer.drivable(self.planes,p,m,100)

    def test_masks_and_camera_shape_rejected(self):
        p,m=self.data();m['front']=m['front'][:-1]
        with self.assertRaises(ValueError):self.renderer.drivable(self.planes,p,m)
    def test_shared_plane_changes_projection_without_mutation(self):
        p,m=self.data();p['front'][:360]=0
        original=p['front'].copy()
        a=self.renderer.drivable(self.planes,p,m)
        b=self.renderer.drivable({v:(.03,0.,-.36) for v in p},p,m)
        self.assertFalse(np.array_equal(a['coverage'],b['coverage']))
        np.testing.assert_array_equal(original,p['front'])
    def test_semantic_join_needs_exact_all_views_not_rgb(self):
        ids={v:calibration_id(c) for v,c in self.cameras.items()}
        cache=ComparisonFrames(self.cameras,ids)
        def header(v,t=10,seq=1):return NS(stamp=NS(to_nsec=lambda:t),frame_id=v,seq=seq)
        ctx=NS(views=list(ids),image_headers=[header(v) for v in ids],calibration_ids=list(ids.values()),
            plane_coefficients=[x for v in ids for x in self.planes[v]],header=header('base_link'),
            producer_session_id='s',projection_session_id='p',config_id='c',generation=1)
        cache.add_context(ctx)
        for v in list(ids)[:2]:cache.add_semantic(v,(NS(header=header(v)),))
        self.assertIsNone(cache.latest(require_raw=False))
        v=list(ids)[2];cache.add_semantic(v,(NS(header=header(v)),))
        self.assertIsNone(cache.latest())
        self.assertIsNotNone(cache.latest(require_raw=False))
        self.assertIsNone(cache.latest(require_raw=False))
if __name__=='__main__':unittest.main()
