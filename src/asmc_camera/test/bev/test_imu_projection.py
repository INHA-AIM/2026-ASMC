from dataclasses import replace
from pathlib import Path
import unittest
import numpy as np
import yaml
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from asmc_camera.bev.imu_projection import attitude_camera, correct_evidence


class ImuProjectionTest(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[2]/'config/cameras_v3.yaml'
        self.camera=calibrations_from_dict(yaml.safe_load(path.read_text()))['front']
        self.projector=ModelGridGroundProjector(self.camera,model_hw=(544,960))

    def test_yaw_only_and_imu_mount_are_not_extra_camera_pitch(self):
        q=[0,0,np.sin(.3),np.cos(.3)]
        corrected=attitude_camera(self.camera,q)
        self.assertEqual(corrected.ground_plane.coefficients,self.camera.ground_plane.coefficients)
        self.assertEqual(corrected.rotation_deg,self.camera.rotation_deg)
        q=[0,np.sin(.01),0,np.cos(.01)]
        corrected=attitude_camera(self.camera,q,imu_mount_rotation_deg=(0,np.degrees(.02),0))
        np.testing.assert_allclose(corrected.ground_plane.coefficients,self.camera.ground_plane.coefficients,atol=1e-12)

    def test_sparse_remap_recovers_known_ray_plane_intersection(self):
        camera=attitude_camera(self.camera,[0,np.sin(.01),0,np.cos(.01)])
        truth=np.array([[3.,1.],[7.,-1.],[10.,0.]])
        pixels,_=camera.project_ground(truth)
        homogeneous=np.column_stack((pixels,np.ones(len(pixels)))) @ np.linalg.inv(self.camera.ground_to_image_homography).T
        nominal=homogeneous[:,:2]/homogeneous[:,2:]
        evidence=MetricEvidence(nominal.astype(np.float32),np.ones(3,np.uint8),np.ones(3,np.float32),np.ones(3,np.float32),np.ones(3,np.uint8))
        result=correct_evidence(self.projector,evidence,camera)
        np.testing.assert_allclose(result.xy,truth,atol=2e-6)
        np.testing.assert_array_equal(result.confidence,evidence.confidence)
        self.assertIs(correct_evidence(self.projector,evidence,self.camera),evidence)

    def test_excessive_tilt_and_invalid_height_are_not_approximated(self):
        with self.assertRaises(ValueError): attitude_camera(self.camera,[0,np.sin(.2),0,np.cos(.2)])
        with self.assertRaises(ValueError): attitude_camera(replace(self.camera,
            ground_plane=replace(self.camera.ground_plane,z_at_origin_m=0)),[0,0,0,1])


if __name__=='__main__':unittest.main()
