import unittest
import numpy as np
from tools.perception.calibrate_stationary_lane_plane import fit_plane, mean_quaternion
from tools.perception.review_stationary_lane_plane import calibrated_plane
from tools.perception.test_review_family_motion import camera, q_axis
from tools.perception.review_family_motion import rotation


class StationaryPlaneTest(unittest.TestCase):
    def test_ransac_plane_recovers_known_road_with_object_outliers(self):
        rng=np.random.RandomState(19);xy=rng.uniform([-5,-3],[15,3],(600,2))
        z=.01*xy[:,0]-.02*xy[:,1]-.36+rng.normal(0,.001,600)
        z[::5]+=.3
        fit=fit_plane(np.c_[xy,z])
        np.testing.assert_allclose(fit['coefficients'],[.01,-.02,-.36],atol=.001)
        self.assertGreater(fit['inlier_ratio'],.79)

    def test_missing_or_nonfinite_ground_fails(self):
        for xyz in (np.empty((0,3)), np.full((100,3),np.nan)):
            with self.assertRaises(ValueError):fit_plane(xyz)

    def test_quaternion_mean_handles_sign_and_rejects_missing(self):
        q=q_axis(0,3)
        np.testing.assert_allclose(rotation(mean_quaternion([q,-q])),rotation(q),atol=1e-12)
        for values in ([],[[0,0,0,0]],[[0,0,np.nan,1]]):
            with self.assertRaises(ValueError):mean_quaternion(values)

    def test_measured_rest_is_preserved_including_real_rest_tilt(self):
        n=np.array([.01,-.02,1.]);n/=np.linalg.norm(n);q=q_axis(0,3)
        corrected=calibrated_plane(camera(),q,q,n,.36)
        np.testing.assert_allclose(corrected.ground_plane.coefficients,
                                   [-n[0]/n[2],-n[1]/n[2],-.36/n[2]],atol=1e-12)
        self.assertEqual(corrected.rotation_deg,camera().rotation_deg)

    def test_world_normal_is_transported_by_full_rotations(self):
        ref=q_axis(2,90);now=q_axis(0,4);n=np.array([.01,.02,1]);n/=np.linalg.norm(n)
        result=calibrated_plane(camera(),now,ref,n,.36).ground_plane
        actual=np.array([-result.dz_dx,-result.dz_dy,1.]);actual/=np.linalg.norm(actual)
        np.testing.assert_allclose(rotation(now)@actual,rotation(ref)@n,atol=1e-12)
        for h in (-.1,float('nan')):
            with self.assertRaises(ValueError):calibrated_plane(camera(),now,ref,n,h)


if __name__=='__main__':unittest.main()
