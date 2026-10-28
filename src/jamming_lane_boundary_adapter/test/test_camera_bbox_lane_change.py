#!/usr/bin/env python3
"""LiDAR-only obstacle consumer contract; executed only by an explicit rostest."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rospy
import rostest
from asmc_msgs.msg import CameraObjectBoxArray, LaneBoundaryFamily, LidarObject, LidarObjectArray, LidarObservation
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Vector3
from lane_boundary_adapter.msg import CameraBBoxLaneResult
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Header, UInt8
from visualization_msgs.msg import MarkerArray

spec = importlib.util.spec_from_file_location('fixture', str(Path(__file__).with_name('lane_family_fixture.py')))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class LaneChangeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rospy.core.is_initialized():
            rospy.init_node('lidar_obstacle_consumer_test')
        cls.lane_pub = rospy.Publisher('/perception/lane/boundary_family', LaneBoundaryFamily, queue_size=1)
        cls.lidar_pub = rospy.Publisher('/perception/lidar/observations_local', LidarObservation, queue_size=1)
        cls.box_pub = rospy.Publisher('/perception/camera/objects', CameraObjectBoxArray, queue_size=1)
        cls.classification_pub = rospy.Publisher('/planning/camera_bbox_lane/result', CameraBBoxLaneResult, queue_size=1)
        cls.ego_pub = rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=1)
        cls.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        cls.stop_pub = rospy.Publisher('/planning/camera_bbox_lane_change/emergency_stop', Bool, queue_size=1)
        cls.mode_pub = rospy.Publisher('/planning/mode', UInt8, queue_size=1, latch=True)
        cls.gps_return_pub = rospy.Publisher('/planning/mode/gps_return', Bool, queue_size=1, latch=True)
        cls.command = cls.diag = cls.markers = None
        cls.subscribers = []
        for topic, typ, attribute in [
                ('/bbox_change_test/command', CtrlCmd, 'command'),
                ('/planning/camera_bbox_lane_change/diagnostics', DiagnosticArray, 'diag'),
                ('/planning/camera_bbox_lane_change/markers', MarkerArray, 'markers')]:
            cls.subscribers.append(rospy.Subscriber(topic, typ, lambda msg, a=attribute: setattr(cls, a, msg)))

    def setUp(self):
        type(self).command = type(self).diag = type(self).markers = None
        self.sequence = 0
        self.log = open('/tmp/' + self._testMethodName + '-lidar-only.log', 'w')
        # Lifecycle and discrete lane-change tests retain the existing state
        # machine; other tests exercise the default five-line/dynamic behavior.
        discrete = self._testMethodName == 'runTest' or self._testMethodName.startswith('test_legacy_')
        creep = self._testMethodName == 'test_no_clear_line_keeps_existing_creep_policy'
        self.process = subprocess.Popen([
            'roslaunch', 'lane_boundary_adapter', 'camera_bbox_lane_change.launch',
            'use_rviz:=false', 'start_classifier:=false', 'enable_control:=true',
            'use_lidar_obstacles:=true', 'control_topic:=/bbox_change_test/command',
            'expected_config_sha256:=' + fixture.HASH,
            'straddle_avoidance:=' + str(not discrete).lower(),
            'dynamic_obstacle_stop:=' + str(not discrete and not creep).lower(),
            'creep_speed_kph:=' + ('3.0' if creep else '0.0'),
            'creep_clear_distance:=0.0'], stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.gps_return_pub.publish(Bool(False))
        self.mode_pub.publish(UInt8(1))
        end = time.monotonic() + 10
        while time.monotonic() < end:
            if self.command is not None and self.lane_pub.get_num_connections() and self.lidar_pub.get_num_connections():
                break
            time.sleep(.03)
        self.assertIsNotNone(self.command)
        self.assertGreater(self.lidar_pub.get_num_connections(), 0)

    def tearDown(self):
        self.gps_return_pub.publish(Bool(False))
        os.killpg(self.process.pid, signal.SIGINT)
        self.process.wait(timeout=10)
        self.log.close()
        time.sleep(.15)

    def status(self):
        self.assertIsNotNone(self.diag)
        return self.diag.status[0].message, {v.key: v.value for v in self.diag.status[0].values}

    def feed(self, ys=(), shift=0., duration=.55, lidar=None, publish_lidar=True,
             valid=True, camera=True, gap=False, speed=0., track_id=7, session='fixture',
             generation=1, predicted=False, velocity=(0., 0.), velocity_valid=True,
             label='car', camera_boxes=(), stamp=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            now = rospy.Time.now()
            self.ego_pub.publish(EgoVehicleStatus(velocity=Vector3(speed, 0, 0)))
            self.imu_pub.publish(Imu())
            if camera:
                lanes = fixture.family(shift=shift, gap=gap)
                lanes.header.stamp = now
                self.lane_pub.publish(lanes)
            if camera_boxes:
                self.box_pub.publish(fixture.camera_boxes(Header(stamp=now, frame_id='camera_front'), camera_boxes))
                # Even a camera-only invalid/unknown classification cannot gate
                # this consumer; the camera is used by upstream class fusion.
                self.classification_pub.publish(CameraBBoxLaneResult(header=Header(stamp=now), valid=False, reason='camera_only_unknown'))
            if publish_lidar:
                boxes = lidar if lidar is not None else [(10., 12., y-.3, y+.3, 1.) for y in ys]
                array = LidarObjectArray(header=Header(stamp=now if stamp is None else stamp, frame_id='base_link'),
                    status=(LidarObjectArray.OBSERVED if boxes else LidarObjectArray.OBSERVED_EMPTY) if valid else LidarObjectArray.INPUT_INVALID,
                    reason='fixture')
                for i, (x0, x1, y0, y1, height) in enumerate(boxes):
                    obj = LidarObject(track_id=track_id+i,
                        observation_state=LidarObject.PREDICTED if predicted else LidarObject.OBSERVED,
                        miss_count=1 if predicted else 0, camera_class=label,
                        obb_corners=[Point(x, y+shift, z) for z in (0., height)
                                     for x, y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))],
                        length=x1-x0, width=y1-y0, height=height, source_point_count=50,
                        absolute_velocity_valid=velocity_valid, dynamic_valid=velocity_valid)
                    obj.absolute_velocity_mps.x, obj.absolute_velocity_mps.y = velocity
                    array.objects.append(obj)
                self.sequence += 1
                self.lidar_pub.publish(LidarObservation(objects=array, producer_session_id=session,
                    generation=generation, scan_sequence=self.sequence))
            time.sleep(.035)

    def following(self):
        self.assertGreater(self.command.velocity, 0., self.status())

    def stopped(self, reason=None):
        self.assertEqual(self.command.velocity, 0., self.status())
        self.assertGreater(self.command.brake, 0.)
        if reason:
            self.assertEqual(self.status()[0], reason)

    def road_markers(self):
        self.assertIsNotNone(self.markers)
        return [m for m in self.markers.markers if m.ns == 'road_obstacle']

    def test_camera_only_boxes_do_not_create_obstacles_or_stop(self):
        self.feed()
        self.feed(camera_boxes=[fixture.camera_box(100, 100, 1200, 700, 'person')])
        self.following()
        self.assertEqual(self.status()[1]['line_obstacles'], '0')
        self.assertEqual(self.status()[1]['object_camera_geometry_used'], 'false')
        self.assertEqual(self.road_markers(), [])

    def test_measured_length_and_width_are_not_six_meter_camera_boxes(self):
        self.feed([3.5])
        marker, = self.road_markers()
        self.assertAlmostEqual(max(p.x for p in marker.points)-min(p.x for p in marker.points), 2.)
        self.assertAlmostEqual(max(p.y for p in marker.points)-min(p.y for p in marker.points), .6)
        self.assertEqual(marker.id, 7)

    def test_same_id_moves_and_shrinks_without_old_position(self):
        self.feed(lidar=[(10,12,2.5,4.,1.)])
        self.feed(lidar=[(10,11,3.5,4.,1.)])
        marker, = self.road_markers()
        self.assertAlmostEqual(max(p.x for p in marker.points)-min(p.x for p in marker.points), 1.)
        self.assertEqual(self.status()[1]['line_obstacles'], '1')

    def test_retired_ids_do_not_accumulate_after_reacquisition(self):
        for identifier in (7, 8, 9):
            self.feed([3.5], track_id=identifier)
            self.assertEqual(self.status()[1]['line_obstacles'], '1')
            self.assertEqual([m.id for m in self.road_markers()], [identifier])

    def test_gps_denied_stationary_lidar_track_gets_local_motion_estimate(self):
        self.feed([3.5], velocity_valid=False, duration=.8)
        self.following()
        self.assertEqual(self.status()[1]['dynamic_known_motion'], '1')
        self.assertEqual(self.status()[1]['dynamic_unknown_motion'], '0')

    def test_camera_lane_following_does_not_require_a_new_lidar_scan(self):
        self.feed(publish_lidar=False, duration=.8)
        self.following()
        self.assertEqual(self.status()[1]['lidar_fresh'], 'false')

    def test_session_restart_replaces_old_session(self):
        self.feed([3.5])
        self.feed([3.5], session='restarted', generation=2)
        self.assertEqual(self.status()[1]['lidar_source'], 'restarted/2')
        self.assertEqual(self.status()[1]['line_obstacles'], '1')

    def test_fresh_empty_snapshot_releases_retired_tracks(self):
        self.feed([3.5])
        self.feed()
        self.assertEqual(self.status()[1]['line_obstacles'], '0')
        self.following()

    def test_prediction_is_preserved_but_velocity_is_unknown(self):
        self.feed([3.5])
        self.feed([3.5], predicted=True)
        self.assertEqual(self.status()[1]['line_obstacles'], '1')
        self.assertEqual(self.status()[1]['lidar_predicted_objects'], '1')
        self.assertEqual(self.status()[1]['dynamic_known_motion'], '0')
        self.assertEqual(self.status()[1]['dynamic_unknown_motion'], '1')

    def test_invalid_scan_does_not_clear_memory_as_empty(self):
        self.feed([3.5])
        self.feed(valid=False)
        self.assertEqual(self.status()[1]['lidar_fresh'], 'false')
        self.assertEqual(self.status()[1]['line_obstacles'], '1')
        self.assertEqual(self.status()[1]['lidar_fresh'], 'false')
        self.feed()
        self.following()

    def test_missing_scan_and_duplicate_stamp_are_not_clear_observations(self):
        self.feed([3.5])
        self.feed(publish_lidar=False, duration=.8)
        self.assertEqual(self.status()[1]['lidar_fresh'], 'false')
        self.assertEqual(self.status()[1]['line_obstacles'], '1')
        self.feed()
        self.following()
        self.feed(stamp=rospy.Time.now(), duration=.8)
        self.assertEqual(self.status()[1]['lidar_fresh'], 'false')

    def test_crossing_risk_uses_motion_for_every_class(self):
        for label in ('person', 'car', 'dog', 'unknown'):
            self.feed(lidar=[(3,3.5,3.,3.5,1.)], velocity=(0.,-3.), label=label)
            self.stopped('dynamic_path_conflict')
        self.feed(duration=2.)
        self.following()

    def test_lane_loss_and_emergency_still_stop(self):
        self.feed()
        self.feed(camera=False, duration=.8)
        self.stopped()
        self.feed()
        self.stop_pub.publish(Bool(True))
        self.feed()
        self.stopped('emergency_stop')
        self.stop_pub.publish(Bool(False))
        self.feed()
        self.following()

    def test_no_clear_line_keeps_existing_creep_policy(self):
        # Generic creep remains available with prediction disabled. Do not
        # confuse it with a just-committed side or an active safety stop.
        self.feed(duration=1.3)
        self.feed(lidar=[(7,8,-1.3,4.8,1.)])
        self.assertEqual(self.status()[0], 'creep:no_clear_line')
        self.assertAlmostEqual(self.command.velocity, 3.)

    def test_legacy_discrete_change_uses_lidar_lane_occupancy(self):
        self.feed()
        self.feed([0])
        self.assertEqual(self.status()[1]['change_state'], 'changing')
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed(shift=-3.5, duration=.8)
        self.assertEqual(self.status()[1]['change_completed'], 'true')
        self.following()


if __name__ == '__main__':
    rostest.rosrun('lane_boundary_adapter', 'camera_bbox_lane_change_integration', LaneChangeTest)
