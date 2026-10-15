import math
from types import SimpleNamespace as NS
import unittest
import numpy as np
from asmc_camera.bev.spline_fitting import bspline_basis,_bspline_basis_uncached,_cached_basis,_open_uniform_knots
from asmc_camera.bev.lane_tracking import _predict_y
from asmc_camera.bev.boundary_candidates import _track_separation,_track_y_at_x


class ComputeReuseTest(unittest.TestCase):
    def test_spline_basis_is_exact_owned_and_cache_is_bounded(self):
        _cached_basis.cache_clear();rng=np.random.RandomState(7)
        for controls in (4,6,12,24):
            knots=_open_uniform_knots(controls,3)
            for parameters in (np.linspace(0,1,180),rng.uniform(-.1,1.1,100)):
                expected=_bspline_basis_uncached(parameters,knots,3,controls)
                result=bspline_basis(parameters,knots,3,controls)
                np.testing.assert_array_equal(result,expected)
                result[:]=999
                np.testing.assert_array_equal(bspline_basis(parameters,knots,3,controls),expected)
        self.assertGreater(_cached_basis.cache_info().hits,0)
        for i in range(100):bspline_basis(np.array([i/101.]),_open_uniform_knots(4,3),3,4)
        self.assertLessEqual(_cached_basis.cache_info().currsize,64)
        knots=_open_uniform_knots(6,3).astype(np.float32)
        np.testing.assert_array_equal(bspline_basis([0,.3,1],knots,3,6),_bspline_basis_uncached([0,.3,1],knots,3,6))

    def test_recent_mean_reuse_keeps_exact_predictions(self):
        rng=np.random.RandomState(9)
        for n in range(1,25):
            for _ in range(20):
                nodes=[NS(xy=p) for p in rng.normal(size=(n,2))]
                x=rng.normal()
                if n<3:expected=(float(nodes[-1].xy[1]),0.)
                else:
                    recent=np.asarray([p.xy for p in nodes[-6:]],np.float64)
                    centered=recent[:,0]-float(np.mean(recent[:,0]));denom=float(np.dot(centered,centered))
                    slope=0. if denom<=1e-9 else float(np.dot(centered,recent[:,1]-np.mean(recent[:,1])))/denom
                    intercept=float(np.mean(recent[:,1])-slope*np.mean(recent[:,0]))
                    expected=(slope*float(x)+intercept,slope)
                self.assertEqual(_predict_y(nodes,x),expected)

    def test_vector_interpolation_matches_scalar_sort_and_median_exactly(self):
        rng=np.random.RandomState(11)
        for n in (4,12,40,100):
            for _ in range(40):
                # Include unsorted duplicate x coordinates and different extents.
                a=NS(track=NS(xy=np.column_stack([rng.randint(-2,30,n),rng.normal(size=n)])),reference_y_m=-2.)
                b=NS(track=NS(xy=np.column_stack([rng.randint(-2,30,n),rng.normal(size=n)])),reference_y_m=2.)
                lo=max(a.track.xy[:,0].min(),b.track.xy[:,0].min());hi=min(a.track.xy[:,0].max(),b.track.xy[:,0].max())
                if hi-lo<1:expected=b.reference_y_m-a.reference_y_m
                else:
                    xs=np.linspace(lo,hi,max(3,int(math.ceil(hi-lo))+1))
                    expected=float(np.median([_track_y_at_x(b.track,x)-_track_y_at_x(a.track,x) for x in xs]))
                self.assertEqual(_track_separation(a,b),expected)


if __name__=='__main__':unittest.main()
