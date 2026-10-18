#!/usr/bin/env python3
"""Actual ROS order/identity tests; publishes observations, never vehicle control."""
import collections
import copy
import importlib.util
from pathlib import Path
import time
import unittest

import cv2
import numpy as np
import rospy
import rospkg
import rostest
from asmc_msgs.msg import CameraGroundProjection, InferenceStatus
from asmc_camera.bev.calibration import load_calibration_snapshot, calibration_id
from asmc_camera.bev.imu_projection import attitude_camera
from lane_boundary_adapter.msg import CameraBBox, CameraBBoxFrame, CameraBBoxLaneResult
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

spec = importlib.util.spec_from_file_location('fixture', str(Path(__file__).with_name('lane_family_fixture.py')))
fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
HASH = '2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a'


class ImuBBoxTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('imu_bbox_test')
        _, cls.cameras = load_calibration_snapshot(
            Path(rospkg.RosPack().get_path('asmc_camera'))/'config/cameras_v3.yaml')
        cls.jpeg = cv2.imencode('.jpg', np.zeros((720, 1280, 3), np.uint8))[1].tobytes()
        cls.results = collections.deque(maxlen=200)
        cls.sub = rospy.Subscriber('/planning/camera_bbox_lane/result', CameraBBoxLaneResult,
                                  cls.results.append, queue_size=20)
        cls.family_pub = rospy.Publisher('/perception/lane/boundary_family', fixture.LaneBoundaryFamily, queue_size=8)
        cls.context_pub = rospy.Publisher('/perception/lane/ground_projection', CameraGroundProjection, queue_size=8)
        cls.status_pub = rospy.Publisher('/perception/camera/lane/status', InferenceStatus, queue_size=1)
        cls.frame_pub = rospy.Publisher('/planning/camera_bbox_lane/frame', CameraBBoxFrame, queue_size=8)
        cls.sequence = 0
        end = time.monotonic()+8
        while time.monotonic()<end and any(p.get_num_connections()==0 for p in
                (cls.family_pub, cls.context_pub, cls.status_pub, cls.frame_pub)):
            time.sleep(.02)
        cls.active()
        time.sleep(.12)

    @classmethod
    def active(cls, mode=InferenceStatus.ACTIVE):
        cls.status_pub.publish(InferenceStatus(header=Header(stamp=rospy.Time.now()),
            producer_session_id='lane-test', generation=1, mode=mode, loaded=True))

    def sample(self, corrected=True):
        type(self).sequence += 1
        stamp = rospy.Time.now()-rospy.Duration(.04)
        q = (.015, .02, 0., np.sqrt(1-.015**2-.02**2))
        cameras = {v: attitude_camera(c, q) if corrected else c for v,c in self.cameras.items()}
        headers = [Header(seq=self.sequence, stamp=stamp+rospy.Duration(i*.01), frame_id='camera_'+v)
                   for i,v in enumerate(self.cameras)]
        context = CameraGroundProjection(header=Header(seq=333, stamp=headers[-1].stamp, frame_id='base_link'),
            producer_session_id='lane-test', generation=1, projection_session_id='family-test',
            config_id=HASH, imu_corrected=corrected, reason='corrected' if corrected else 'imu_missing',
            views=list(self.cameras), image_headers=headers,
            calibration_ids=[calibration_id(c) for c in self.cameras.values()],
            plane_coefficients=[x for c in cameras.values() for x in c.ground_plane.coefficients])
        lane = fixture.family()
        lane.header = Header(seq=777, stamp=context.header.stamp, frame_id='base_link')
        lane.config_sha256 = HASH
        plane = cameras[list(cameras)[-1]].ground_plane
        for member in lane.boundaries:
            for p in member.polyline:
                p.z = plane.z_at_origin_m+plane.dz_dx*p.x+plane.dz_dy*p.y
        boxes = []
        for y in (0., 3.5):
            pixel, _ = cameras['front'].project_ground(np.array([[10., y]]))
            u,v = pixel[0]
            boxes.append(CameraBBox(xmin=u-12, xmax=u+12, ymin=v-50, ymax=v, label='car'))
        # Top-level publisher seq is intentionally unrelated to original JPEG.
        frame = CameraBBoxFrame(header=Header(seq=999, stamp=stamp, frame_id='camera_front'),
            valid=True, image=CompressedImage(header=copy.deepcopy(headers[0]), format='jpeg', data=self.jpeg), boxes=boxes)
        return lane, context, frame

    def feed(self, fault='', corrected=True, reverse=False, duration=.75):
        self.results.clear()
        end = time.monotonic()+duration
        while time.monotonic()<end:
            self.active()
            lane, context, frame = self.sample(corrected)
            if fault=='sequence': context.image_headers[0].seq += 1
            if fault=='calibration': context.calibration_ids[0] = 'wrong'
            if fault=='xyz': lane.boundaries[0].polyline[0].z += .2
            # Exercise both orders and let frames arrive before slower fitting.
            self.frame_pub.publish(frame)
            if reverse:
                self.family_pub.publish(lane); time.sleep(.015)
                if fault!='missing': self.context_pub.publish(context)
            else:
                if fault!='missing': self.context_pub.publish(context)
                time.sleep(.015); self.family_pub.publish(lane)
            time.sleep(.05)
        time.sleep(.12)
        return list(self.results)

    def assert_classified(self, results):
        valid = [m for m in results if m.valid and len(m.assignments)==2]
        self.assertGreaterEqual(len(valid), 3, [m.reason for m in results][-8:])
        for message in valid:
            self.assertEqual(list(message.assignments),
                             [CameraBBoxLaneResult.OWN, CameraBBoxLaneResult.ADJACENT])
        return valid

    def test_01_corrected_nominal_and_both_callback_orders(self):
        for corrected in (True, False, True):
            for reverse in (False, True):
                self.assert_classified(self.feed(corrected=corrected, reverse=reverse))

    def test_02_missing_identity_calibration_and_xyz_never_use_static_plane(self):
        for fault in ('sequence', 'calibration', 'xyz', 'missing'):
            results = self.feed(fault=fault, duration=.9)
            self.assertTrue(results)
            self.assertFalse(any(m.valid for m in results), fault)
        self.assert_classified(self.feed())

    def test_03_standby_blocks_observations_and_active_recovers(self):
        self.active(InferenceStatus.STANDBY);time.sleep(.12)
        self.results.clear()
        for _ in range(4):
            lane, context, frame = self.sample()
            self.context_pub.publish(context);self.family_pub.publish(lane);self.frame_pub.publish(frame)
            time.sleep(.06)
        self.assertFalse(any(m.valid for m in self.results))
        self.assert_classified(self.feed())


if __name__ == '__main__':
    rostest.rosrun('lane_boundary_adapter', 'camera_bbox_imu_projection', ImuBBoxTest)
