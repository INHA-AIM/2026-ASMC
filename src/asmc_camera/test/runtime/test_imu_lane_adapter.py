from pathlib import Path
from dataclasses import replace
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import rospy
import numpy as np
import yaml
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from asmc_msgs.msg import CameraModelGridTiming, SensorFrameTiming
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.boundary_family_output import BoundaryFamilyMemberOutput, build_boundary_family_output
from asmc_camera.bev.imu_projection import attitude_camera
from asmc_camera.bev.family_fit_policy import FamilyFitPolicy
from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode
from asmc_camera.runtime.sensor_timing import ImuTimeline


class ImuLaneAdapterTest(unittest.TestCase):
    def setUp(self):
        node=self.node=LaneVectorizerNode.__new__(LaneVectorizerNode)
        config=yaml.safe_load((Path(__file__).resolve().parents[2]/'config/cameras_v3.yaml').read_text())
        node.projectors={view:NS(camera=camera) for view,camera in calibrations_from_dict(config).items()}
        node.imu_config={};node.imu_lock=threading.Lock();node.imu_counts={};node.imu_last={}
        node.epoch_gate=NS(context=lambda:(True,3,0,1),enabled=True,mode=2,cutoff=0,
                           producer_session_id='lane',generation=3)
        node.imu_timeline=ImuTimeline(clock=lambda:1.)
        imu=SensorFrameTiming(bridge_session_id='b',stream_id='imu',sensor_topic='/imu',
            sensor_frame_id='imu',published_stamp=rospy.Time(1,1),sensor_wire_sequence=1,
            sensor_wire_sequence_valid=True,source_sec=10,source_valid=True,source_clock_domain='morai_sensor')
        node.imu_timeline.add_timing(imu)
        node.imu_timeline.add_imu(('/imu','imu',1000000001,1),(0,0,0,1),'imu')
        self.frames={}
        for view,projector in node.projectors.items():
            header=Header(seq=2,stamp=rospy.Time(1),frame_id='camera_'+view)
            raw=SensorFrameTiming(bridge_session_id='b',stream_id='cam',sensor_topic=projector.camera.image_topic,
                sensor_frame_id=header.frame_id,published_stamp=header.stamp,sensor_wire_sequence=123,
                sensor_wire_sequence_valid=True,source_sec=10,source_valid=True,source_clock_domain='morai_sensor')
            timing=CameraModelGridTiming(source_timing_valid=True,sensor_timing=raw,
                                         image_header=Header(seq=123,stamp=header.stamp,frame_id=header.frame_id))
            self.frames[view]=NS(header=header,stamp_ns=10**9,timing=timing)

    def test_all_three_views_get_exact_pose_and_preserve_nominal_plane_at_identity(self):
        cameras=self.node._attitude_cameras(self.frames)
        self.assertEqual(set(cameras),set(self.frames))
        self.assertEqual(self.node.imu_counts,{'corrected':1})
        for view,camera in cameras.items():
            self.assertEqual(camera.ground_plane.coefficients,self.node.projectors[view].camera.ground_plane.coefficients)

    def test_missing_one_source_or_wrong_wire_key_falls_back_for_all_views(self):
        self.frames['right'].timing.source_timing_valid=False
        self.assertEqual(self.node._attitude_cameras(self.frames),{})
        self.assertFalse(self.node.imu_last['used'])
        self.frames['right'].timing.source_timing_valid=True
        self.frames['right'].timing.sensor_timing.sensor_wire_sequence=99
        self.assertEqual(self.node._attitude_cameras(self.frames),{})
        self.assertEqual(self.node.imu_last['reason'],'source_identity_mismatch')
        self.frames['right'].timing.sensor_timing.sensor_wire_sequence=123
        self.frames['right'].timing.sensor_timing.sensor_wire_sequence_valid=False
        self.assertEqual(self.node._attitude_cameras(self.frames),{})

    def test_width_policy_tracks_complete_correction_missing_view_and_recovery(self):
        policy = FamilyFitPolicy({'width_prior_strength': .35})
        expected = [(True, 0.), (False, .35), (True, 0.)]
        for available, prior in expected:
            self.frames['right'].timing.source_timing_valid = available
            cameras = self.node._attitude_cameras(self.frames)
            selected = policy.select(bool(cameras), self.node.imu_last['reason'])
            self.assertEqual(selected.parameters['width_prior_strength'], prior)
            self.assertEqual(len(cameras), 3 if available else 0)

    def test_wrong_grid_sequence_or_model_session_is_rejected_before_decoding(self):
        node=self.node;node.inputs={'front':0};node.pair_errors={'front':0};node.fit_work=Mock()
        header=Header(seq=4,stamp=rospy.Time(1),frame_id='camera_front')
        image=Image(header=header)
        timing=CameraModelGridTiming(header=header,view='front',producer_session_id='old',generation=3,
            class_header=header,confidence_header=header,image_header=header,grid_wire_sequences_valid=True)
        node._view_callback('front',image,image,timing)
        self.assertEqual(node.pair_errors['front'],1)
        node.fit_work.offer.assert_not_called()

    def test_published_xyz_uses_this_frame_plane_without_changing_nominal_config(self):
        node=self.node
        for key in ('frame_id','operating_domain','boundary_family_algorithm_id',
                    'shared_family_representation','boundary_family_config_sha256'):
            setattr(node,key,'fixture')
        camera=node.projectors['front'].camera
        node.ground_plane=camera.ground_plane
        q=(0,np.sin(.01),0,np.cos(.01))
        plane=attitude_camera(camera,q).ground_plane
        fields={key:0 for key in BoundaryFamilyMemberOutput.__dataclass_fields__}
        fields.update(polyline_xy=np.array([[1.,2.],[5.,2.]]),support_state=np.array([1,1]))
        output=replace(build_boundary_family_output(None,status_reason='fixture'),
                       boundaries=(BoundaryFamilyMemberOutput(**fields),))
        message=node._boundary_family_message(output,Header(stamp=rospy.Time(1)),plane)
        np.testing.assert_allclose([p.z for p in message.boundaries[0].polyline],
                                   plane.xyz(fields['polyline_xy'])[:,2],atol=1e-7)
        self.assertEqual(node.ground_plane,camera.ground_plane)


if __name__=='__main__':unittest.main()
