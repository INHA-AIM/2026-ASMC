#!/usr/bin/env python3
"""Real YOLO messages -> bridge -> classifier -> controller, isolated ROS master."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import cv2
import numpy as np
import rospy
import rospkg
import rostest
from asmc_msgs.msg import LaneBoundaryFamily,LidarObject,LidarObjectArray
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray
from asmc_camera.bev.calibration import load_calibration_snapshot
from geometry_msgs.msg import PointStamped
from lane_boundary_adapter.msg import CameraBBoxLaneResult
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from asmc_msgs.msg import CameraObjectBoxArray
from sensor_msgs.msg import CompressedImage,Imu
from std_msgs.msg import Header,Bool,Int8,UInt8
from visualization_msgs.msg import MarkerArray,Marker

spec=importlib.util.spec_from_file_location('fixture',str(Path(__file__).with_name('lane_family_fixture.py')))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


class LaneChangeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rospy.core.is_initialized():
            rospy.init_node('bbox_change_integration')
        cls.camera=load_calibration_snapshot(Path(rospkg.RosPack().get_path('asmc_camera'))/'config/bev/cameras_v2.yaml')[1]['front']
        cls.jpeg=cv2.imencode('.jpg',np.zeros((720,1280,3),dtype=np.uint8))[1].tobytes()
        cls.lane_pub=rospy.Publisher('/perception/lane/boundary_family',LaneBoundaryFamily,queue_size=1)
        cls.image_pub=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=1)
        cls.box_pub=rospy.Publisher('/perception/camera/objects',CameraObjectBoxArray,queue_size=1)
        cls.stop_pub=rospy.Publisher('/planning/camera_bbox_lane_change/emergency_stop',Bool,queue_size=1)
        cls.ego_pub=rospy.Publisher('/Ego_topic',EgoVehicleStatus,queue_size=1)
        cls.imu_pub=rospy.Publisher('/imu',Imu,queue_size=1)
        cls.lidar_pub=rospy.Publisher('/perception/lidar/objects_local',LidarObjectArray,queue_size=1)
        # The legacy ACTIVE algorithm is exercised under its required mode.
        cls.mode_pub=rospy.Publisher('/planning/mode',UInt8,queue_size=1,latch=True)
        cls.mode_pub.publish(UInt8(1))
        cls.gps_return_pub=rospy.Publisher('/planning/mode/gps_return',Bool,queue_size=1,latch=True)
        cls.gps_return_pub.publish(Bool(False))
        cls.command=cls.diag=cls.target=cls.classification=None
        cls.subscribers=[]
        for topic,typ,attribute in [('/bbox_change_test/command',CtrlCmd,'command'),
            ('/planning/camera_bbox_lane_change/diagnostics',DiagnosticArray,'diag'),
            ('/planning/camera_bbox_lane_change/target',PointStamped,'target'),
            ('/planning/camera_bbox_lane_change/markers',MarkerArray,'markers'),
            ('/planning/camera_bbox_lane_change/reference_lane',Int8,'reference'),
            ('/planning/camera_bbox_lane/result',CameraBBoxLaneResult,'classification')]:
            cls.subscribers.append(rospy.Subscriber(topic,typ,lambda m,a=attribute:setattr(cls,a,m)))

    def setUp(self):
        type(self).command=type(self).diag=type(self).target=type(self).classification=None
        self.log=open('/tmp/'+self._testMethodName+'-bbox-change.log','w')
        self.process=subprocess.Popen(['roslaunch','lane_boundary_adapter','camera_bbox_lane_change.launch',
            'use_rviz:=false','enable_control:=true','control_topic:=/bbox_change_test/command',
            'ego_mask_enabled:=false',
            'projection_context_required:=false',
            'calibration_file:='+rospkg.RosPack().get_path('asmc_camera')+'/config/bev/cameras_v2.yaml',
            'expected_config_sha256:='+fixture.HASH,
            *(['avoid_clearance:=0.75'] if self._testMethodName.startswith('test_straddle_overflow') else []),
            *(['path_horizon:=10.0'] if self._testMethodName.startswith('test_straddle_far') else []),
            *(['lane_hold_time:=3.0'] if 'hold' in self._testMethodName else []),
            *(['use_lidar_obstacles:=true'] if 'lidar' in self._testMethodName else []),
            *(['avoid_line_overrun:=0.0'] if 'wall' in self._testMethodName else []),
            'lane_change_timeout:='+('10.0' if self._testMethodName.startswith('test_ignore') else '5.0'),
            'return_to_entry_lane:='+('true' if self._testMethodName.startswith('test_return') else 'false'),
            *([] if self._testMethodName.startswith(('test_ignore','test_repeat','test_return')) or 'hold' in self._testMethodName
              else ['ignore_bbox_during_lane_change:=false']),
            *([] if self._testMethodName.startswith(('test_repeat','test_return')) else
              ['one_lane_change_only:='+('false' if self._testMethodName=='test_clipped_cannot_start_second_change' else 'true')]),
            'hood_crop_bottom_ratio:='+('0.21' if self._testMethodName.startswith(('test_clipped','test_ignore')) else '0.0'),
            'far_obstacle_speed_kph:='+('0.0' if self._testMethodName in ('test_far_approach_can_be_disabled','test_ignore_far_zero_speed_during_change') else '5.0'),
            'lane_change_pause_timeout:='+('1.0' if self._testMethodName=='test_pause_timeout_latches' else '30.0')],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+10
        while (self.command is None or self.image_pub.get_num_connections()<1 or self.lane_pub.get_num_connections()<2) and time.monotonic()<end:
            time.sleep(.03)
        self.assertIsNotNone(self.command)

    def tearDown(self):
        self.gps_return_pub.publish(Bool(False))
        os.killpg(self.process.pid,signal.SIGINT)
        self.process.wait(timeout=10)
        self.log.close()
        time.sleep(.15)

    def status(self):
        self.assertIsNotNone(self.diag)
        return self.diag.status[0].message,{v.key:v.value for v in self.diag.status[0].values}

    def feed(self, ys=(),shift=0.,duration=.55,camera=True,bbox=True,stamp=None,gap=False,source_gap=False,target_gap=False,far=False,slope=0.,box_x=10.,raw_boxes=None,spans=(),speed=None,lidar=None,imu=None,label='car'):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            source=rospy.Time.now() if stamp is None else stamp
            if speed is not None:
                ego=EgoVehicleStatus();ego.velocity.x=speed
                self.ego_pub.publish(ego)
            if imu is not None:
                message=Imu();message.angular_velocity.z=imu
                self.imu_pub.publish(message)
            if lidar is not None:
                # (x0, x1, y0, y1, height) axis-aligned boxes in base_link; lanes shift like the camera fixture.
                array=LidarObjectArray(header=Header(stamp=rospy.Time.now(),frame_id='base_link'),
                    status=LidarObjectArray.OBSERVED if lidar else LidarObjectArray.OBSERVED_EMPTY,reason='test')
                for i,(x0,x1,y0,y1,h) in enumerate(lidar):
                    corners=[Point(x,y+shift,z) for z in (0.,h) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
                    array.objects.append(LidarObject(track_id=i,observation_state=LidarObject.OBSERVED,obb_corners=corners,
                        length=x1-x0,width=y1-y0,height=h,source_point_count=50))
                self.lidar_pub.publish(array)
            if camera:
                lane=fixture.family(shift=shift,gap=gap)
                for member in lane.boundaries:
                    if (source_gap and member.role==3) or (target_gap and member.role==1):
                        for i,point in enumerate(member.polyline):
                            if 4.0 <= point.x <= 9.0:member.support_state[i]=0
                for member in lane.boundaries:
                    for point in member.polyline:point.y += slope * point.x
                lane.header.stamp=rospy.Time.now()
                self.lane_pub.publish(lane)
            if bbox:
                boxes=[]
                for y in ys:
                    uv,_=self.camera.project_ground(np.array([[box_x,y+shift+slope*box_x]]))
                    u,v=uv[0]
                    boxes.append(fixture.camera_box(u-15,v-60,u+15,v,label))
                for y_min,y_max,x in spans:
                    # Wide box whose bottom edge spans the given ground interval (base_link y, +left).
                    uv,_=self.camera.project_ground(np.array([[x,y_max+shift],[x,y_min+shift]]))
                    (left,v),(right,_)=uv
                    boxes.append(fixture.camera_box(left,v-60,right,v,'car'))
                if far:
                    uv,_=self.camera.project_ground(np.array([[30.,shift]]))
                    u,v=uv[0]
                    boxes.append(fixture.camera_box(u-15,v-50,u+15,v,'car'))
                if raw_boxes is not None:boxes=raw_boxes
                header=Header(stamp=source,frame_id='camera_front')
                self.image_pub.publish(CompressedImage(header=header,format='jpeg',data=self.jpeg))
                self.box_pub.publish(fixture.camera_boxes(header,boxes))
            time.sleep(.05)
        time.sleep(.03)

    def following(self):
        # Driving command: MORAI velocity control (no ego speed) or pedal control (ego speed available).
        # A stop is type 1 with stop_brake (1.0); pedal deceleration never reaches it.
        self.assertIn(self.command.longlCmdType,(1,2),self.status())
        if self.command.longlCmdType==2:
            self.assertEqual(self.command.brake,0)
        else:
            self.assertLess(self.command.brake,1.0,self.status())
            self.assertGreater(self.command.velocity,0.,self.status())

    def stopped(self,reason=None):
        self.assertEqual(self.command.longlCmdType,1,self.status())
        self.assertEqual(self.command.brake,1)
        if reason:self.assertEqual(self.status()[0],reason)

    def test_return_clear_entry_lane_and_keep_target_obstacle_stop(self):
        self.feed()
        self.feed([0])
        self.feed([], shift=-3.5, duration=.65)
        self.assertEqual(self.status()[1]['entry_lane'], '1')
        self.assertEqual(self.status()[1]['change_completed'], 'true')
        # A blocked entry lane never becomes a voluntary return target, even
        # after the cooldown. Continue in the currently clear lane.
        self.feed([0], shift=-3.5, duration=4.1)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed([], shift=-3.5, duration=1.5)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'], '1')
        self.assertEqual(self.status()[1]['returning_to_entry'], 'true')
        self.assertEqual(self.status()[1]['bbox_stop_suppressed'], 'false')
        self.feed([0], shift=-3.5)
        self.stopped('target_lane_detected')
        # Exactly one fresh BBOX source frame (duplicates do not count) can
        # resume an already-authorized return after observing the target clear.
        self.feed([], shift=-3.5, duration=.2, stamp=rospy.Time.now())
        self.following()
        self.feed([], shift=0., duration=.65)
        self.assertEqual(self.status()[1]['change_completed'], 'true')
        self.assertEqual(self.status()[1]['returning_to_entry'], 'false')
        self.feed([], duration=4.1)
        self.assertEqual(self.status()[1]['locked_lane'], '1')

    def test_return_rejects_unresolved_or_stale_observations(self):
        self.feed()
        self.feed([0])
        self.feed([], shift=-3.5, duration=.65)
        self.feed([], shift=-3.5, far=True, duration=4.1)
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed([], shift=-3.5, duration=.6)
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed([], shift=-3.5, bbox=False, duration=.8)
        self.stopped()
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed([], shift=-3.5, duration=.65)
        self.assertEqual(self.status()[1]['locked_lane'], '0')
        self.feed([], shift=-3.5, duration=.9)
        self.assertEqual(self.status()[1]['locked_lane'], '1')

    def test_person_stops_three_seconds_then_changes_lane_without_restop(self):
        self.feed()
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.feed([0],label='person',duration=.5)
        self.stopped('person_stop')
        self.assertEqual(self.status()[1]['person_state'],'stop')
        self.feed([0],label='person',duration=2.8)
        # After 3 s the person no longer stops the vehicle but still occupies the lane.
        self.following()
        self.assertEqual(self.status()[1]['person_state'],'ignore')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.reference.data,0)

    def test_repeat_changes_use_new_own_lane_and_reset_completion(self):
        self.feed()
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed([0],shift=-3.5,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['change_completed'],'true')

        # A second obstacle is considered, but the existing 4 s cooldown still applies.
        self.feed([3.5],shift=-3.5)
        self.stopped('lane_change_cooldown')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed([],shift=-3.5,duration=4.1)
        self.following()
        self.assertEqual(self.status()[1]['change_state'],'following_lane')
        self.assertEqual(self.status()[1]['locked_lane'],'0')  # No automatic return.
        self.feed([0,3.5],shift=-3.5)
        self.stopped('adjacent_lane_detected_or_path_invalid')
        self.feed([3.5],shift=-3.5)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.assertEqual(self.status()[1]['change_state'],'changing')
        self.assertEqual(self.status()[1]['change_completed'],'false')
        self.assertEqual(self.reference.data,1)
        self.assertEqual(self.classification.own_lane,1)
        self.assertLess(self.command.steering,0)
        self.feed([3.5],shift=0.,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['change_completed'],'true')
        self.assertEqual(self.status()[1]['locked_lane'],'1')

        self.feed([],duration=4.1)
        self.following()
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.status()[1]['change_completed'],'false')
        self.assertEqual(self.reference.data,0)
        self.assertEqual(self.classification.own_lane,0)
        self.assertGreater(self.command.steering,0)
        self.feed([0],shift=-3.5,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['change_completed'],'true')
        self.assertEqual(self.status()[1]['locked_lane'],'0')

    def test_ignore_boxes_during_change_and_restore_after_completion(self):
        self.feed([0,3.5])
        self.stopped('adjacent_lane_detected_or_path_invalid')
        self.feed([0])
        self.following()
        self.feed([0,3.5])  # Target detections no longer brake during the maneuver.
        self.following()
        self.assertEqual(self.status()[1]['bbox_stop_suppressed'],'true')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed(raw_boxes=[fixture.camera_box(600,480,680,568,'car')])
        self.assertFalse(self.classification.valid)
        self.following()
        self.feed(bbox=False,duration=.7)  # Keep fresh lanes, lose BBOX input.
        self.following()
        self.feed(bbox=False,shift=-3.5,duration=.65)
        self.stopped()
        self.assertEqual(self.status()[1]['change_completed'],'true')
        self.assertEqual(self.status()[1]['bbox_stop_suppressed'],'false')
        self.feed([3.5],shift=-3.5)
        self.stopped('lane_change_already_completed')

    def test_ignore_far_zero_speed_during_change(self):
        self.feed(far=True)
        self.stopped('far_obstacle_approach_disabled')
        self.feed([0])
        self.following()
        self.feed(far=True)
        self.following()  # Far speed=0 must not indirectly generate a brake command.
        self.assertGreater(self.command.velocity,0.)
        self.assertLessEqual(self.command.velocity,15.)
        self.assertEqual(self.status()[0],'changing_without_bbox_stop')

    def test_ignore_bbox_keeps_lane_and_emergency_stops(self):
        self.feed()
        self.feed([0])
        self.following()
        self.feed(bbox=False,camera=False,duration=.7)
        self.stopped()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed([0,3.5])
        self.following()
        self.stop_pub.publish(Bool(True))
        self.feed([0,3.5])
        self.stopped('emergency_stop')
        self.stop_pub.publish(Bool(False))
        self.feed([0,3.5])
        self.following()
        self.feed(target_gap=True,shift=-2.5)
        self.stopped()
        self.assertEqual(self.status()[1]['locked_lane'],'0')

    def test_left_change_cross_boundary_complete_and_keep(self):
        self.stopped()
        self.feed()
        self.following()
        self.assertAlmostEqual(self.target.point.y,0)
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'0')
        self.assertEqual(self.status()[1]['change_state'],'changing')
        # Ownership changes before the vehicle moves or completion is reported.
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.status()[1]['change_completed'],'false')
        self.assertEqual(self.reference.data,0)
        self.assertGreater(self.command.steering,0)
        self.assertLessEqual(self.command.velocity,15)
        for shift in (-.8,-1.75,-2.5):
            self.feed([0],shift=shift,duration=.4)
            self.following()
            self.assertTrue(self.classification.valid,self.classification.reason)
            self.assertEqual(self.classification.own_lane,0)  # target reference; occupancy still maps to geometric lane
            self.assertEqual(self.status()[1]['target_lane'],'0')
            self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed([0],shift=-3.5,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.status()[1]['change_completed'],'true')
        self.assertEqual(self.status()[1]['change_state'],'following_lane')
        self.assertAlmostEqual(self.target.point.y,0)
        self.feed([3.5],shift=-3.5)
        self.stopped('lane_change_already_completed')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        pubs,_,_=rospy.get_master().getSystemState()[2]
        self.assertNotIn('/ctrl_cmd',dict(pubs))

    def test_right_change(self):
        self.feed(shift=-3.5)
        self.following()
        self.feed([3.5],shift=-3.5)
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'1')
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.assertEqual(self.status()[1]['change_completed'],'false')
        self.assertEqual(self.reference.data,1)
        self.assertEqual(self.classification.own_lane,1)
        self.assertLess(self.command.steering,0)

    def test_adjacent_detection_blocks_and_target_intrusion_stops(self):
        self.feed([3.5])
        self.following()  # detection only in adjacent lane does not request a change
        self.assertEqual(self.status()[1]['change_state'],'following_lane')
        self.feed([0,3.5])
        self.stopped('adjacent_lane_detected_or_path_invalid')
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.feed([1.75])
        self.stopped('adjacent_lane_detected_or_path_invalid')
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        self.feed([0])
        self.following()
        self.feed([0,3.5])
        self.stopped('target_lane_detected')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.reference.data,0)
        self.assertEqual(self.status()[1]['target_lane'],'0')
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'0')

    def test_duplicate_timeout_gap_and_emergency(self):
        self.feed()
        old=rospy.Time.now()
        self.feed([0],stamp=old,duration=.12)
        self.stopped('confirming_bbox_lane_change')
        self.feed([0],stamp=old,duration=.6)
        self.stopped()
        self.feed()
        self.following()
        self.feed(camera=False,duration=.7)
        self.stopped()
        self.feed()
        self.feed(bbox=False,duration=.7)
        self.stopped()
        self.feed(gap=True)
        self.stopped()
        self.feed()
        self.stop_pub.publish(Bool(True));self.feed()
        self.stopped('emergency_stop')
        self.stop_pub.publish(Bool(False));self.feed()
        self.following()

    def test_change_uses_target_after_source_is_occluded(self):
        self.feed()
        self.feed([0])
        self.following()
        # Once inside the target strip, source-only far occlusion cannot veto completion.
        self.feed([],shift=-3.3,source_gap=True,duration=.7)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.status()[1]['change_completed'],'true')

    def test_target_gap_still_blocks_change(self):
        self.feed()
        self.feed([0])
        self.following()
        self.feed([],shift=-2.5,target_gap=True)
        self.stopped()
        self.assertEqual(self.status()[1]['target_lane'],'0')
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.reference.data,0)

    def test_pause_does_not_consume_drive_timeout(self):
        self.feed()
        self.feed([0])
        self.following()
        self.stop_pub.publish(Bool(True))
        self.feed([0],duration=5.6)
        self.stopped('emergency_stop')
        self.stop_pub.publish(Bool(False))
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['change_state'],'changing')
        self.assertLess(float(self.status()[1]['change_drive_time_sec']),3.0)

    def test_pause_timeout_latches(self):
        self.feed()
        self.feed([0])
        self.stop_pub.publish(Bool(True))
        self.feed([0],duration=1.4)
        self.stopped('lane_change_pause_timeout_restart_required')
        self.stop_pub.publish(Bool(False))
        self.feed([])
        self.stopped('lane_change_pause_timeout_restart_required')

    def test_left_change_aligns_heading_when_lateral_step_is_reduced(self):
        self.feed()
        self.feed([0])
        self.following()
        self.feed([0],shift=-.91,slope=-.26)
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'0')
        self.assertLessEqual(self.command.velocity,15)
        self.feed([0],shift=-3.5,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['change_completed'],'true')

    def test_right_change_aligns_heading_when_lateral_step_is_reduced(self):
        self.feed(shift=-3.5)
        self.feed([3.5],shift=-3.5)
        self.following()
        self.feed([3.5],shift=-2.59,slope=.26)
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'1')
        self.feed([3.5],shift=0.,duration=.65)
        self.following()
        self.assertEqual(self.status()[1]['change_completed'],'true')

    def test_contact_at_observed_end_starts_change_and_blocks_target_intrusion(self):
        self.feed()
        self.feed([0],box_x=15.2)
        self.following()
        self.assertEqual(self.status()[1]['change_state'],'changing')
        self.assertEqual(self.status()[1]['target_lane'],'0')
        self.assertIn('contact_overlap',self.classification.box_reasons[0])
        self.feed([0,3.5],box_x=15.2)
        self.stopped('target_lane_detected')
        self.feed([0],box_x=15.2)
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'0')

    def test_clipped_known_source_continues_and_target_still_stops(self):
        normal=fixture.camera_box(600,480,680,560,'car')
        clipped=fixture.camera_box(600,480,680,568,'car')
        self.feed()
        self.feed(raw_boxes=[normal])
        self.following()
        self.assertEqual(self.status()[1]['change_state'],'changing')
        self.assertEqual(self.reference.data,0)
        self.assertEqual(self.classification.own_lane,0)
        own=[m for m in self.markers.markers if m.action==Marker.ADD and m.ns=='own_lane_centerpoint']
        self.assertEqual(len(own),1)
        self.assertAlmostEqual(own[0].pose.position.y,3.5)
        self.assertEqual(own[0].color.g,1.)
        self.feed(raw_boxes=[clipped])
        self.following()
        self.assertLessEqual(self.command.velocity,5.)
        self.assertEqual(list(self.classification.clipped_association),[True])
        target=fixture.camera_box(200,480,280,560,'car')
        self.feed(raw_boxes=[target])
        self.stopped('target_lane_detected')
        target.ymax=568
        self.feed(raw_boxes=[target])
        self.assertTrue(self.classification.valid,self.classification)
        self.assertEqual(list(self.classification.clipped_association),[True])
        self.stopped('target_lane_detected')

    def test_clipped_cannot_start_second_change(self):
        self.feed()
        self.feed([0])
        self.feed([0],shift=-3.5,duration=.65)
        self.assertEqual(self.status()[1]['change_completed'],'true')
        normal=fixture.camera_box(600,480,680,560,'car')
        self.feed(raw_boxes=[normal],shift=-3.5)
        normal.ymax=568
        self.feed(raw_boxes=[normal],shift=-3.5)
        self.assertEqual(list(self.classification.clipped_association),[True])
        self.stopped('clipped_obstacle_cannot_start_change')
        self.assertEqual(self.status()[1]['change_state'],'following_lane')

    def test_clipped_new_object_and_expired_track_stop(self):
        normal=fixture.camera_box(600,480,680,560,'car')
        clipped=fixture.camera_box(600,480,680,568,'car')
        self.feed()
        self.feed(raw_boxes=[clipped])
        self.stopped()
        self.assertFalse(self.classification.valid)
        self.feed(raw_boxes=[normal])
        self.following()
        self.feed(raw_boxes=[clipped],duration=.4)
        self.following()
        self.feed(raw_boxes=[clipped],duration=3.0)
        self.stopped()
        self.assertFalse(self.classification.valid)

    def test_far_approach_then_resolved_lane_change(self):
        self.feed(far=True)
        self.following()
        self.assertGreater(self.command.velocity,0)
        self.assertLessEqual(self.command.velocity,5)
        self.assertEqual(self.status()[0],'approaching_unresolved_far_obstacle')
        self.assertEqual(self.classification.own_state,CameraBBoxLaneResult.UNOBSERVED)
        self.assertEqual(self.classification.adjacent_state,CameraBBoxLaneResult.UNOBSERVED)
        self.assertEqual(self.status()[1]['change_state'],'following_lane')
        self.feed([0],far=True)
        self.stopped('far_obstacle_lane_unresolved')
        self.feed([0])
        self.following()
        self.assertEqual(self.status()[1]['change_state'],'changing')
        self.feed([],far=True)
        self.stopped('far_obstacle_lane_unresolved_during_change')
        self.feed([])
        self.following()
        self.assertEqual(self.status()[1]['target_lane'],'0')

    def test_far_box_does_not_override_gap_or_input_loss(self):
        self.feed(far=True)
        self.following()
        self.feed(far=True,gap=True)
        self.stopped()
        self.feed(far=True)
        self.following()
        self.feed(far=True,bbox=False,duration=.7)
        self.stopped()

    def test_far_approach_can_be_disabled(self):
        self.feed(far=True)
        self.stopped('far_obstacle_approach_disabled')

    # Line selection (ego speed available). Five lines from the camera lanes: 1=+5.25 (left road edge),
    # 2=+3.5 (left lane centre), 3=+1.75 (centre line), 4=0 (own lane centre), 5=-1.75 (right road edge).
    # One line unit = 1.75 m; candidates every 0.25 line. Judgement clearance: half width 0.946 + 0.5 + 0.05.
    def line_status(self):
        status=self.status()[1]
        return float(status['line_target']) if status['line_target']!='none' else None,status

    def test_straddle_adjacent_side_keeps_lane_with_offset(self):
        self.feed()
        self.assertEqual(self.status()[1]['locked_lane'],'1')
        # Centre-line obstacle mostly in the adjacent lane: 0.45 m into own lane -> line 4.25.
        self.feed(spans=[(1.3,3.0,10.)],speed=0.,duration=.8)
        self.assertEqual(self.classification.assignments[0],CameraBBoxLaneResult.BOTH)
        self.assertAlmostEqual(self.classification.contact_y_min[0],1.3,delta=.05)
        self.following()
        line,status=self.line_status()
        self.assertEqual(status['line_mode'],'true',status)
        self.assertAlmostEqual(line,4.25)
        self.assertEqual(status['locked_lane'],'1')
        self.assertLess(self.target.point.y,0.)
        self.assertLessEqual(self.command.velocity,8.)
        # Back to line 4 only after the whole obstacle (10 m + 6 m assumed length + rear 0.79 + 2 m) is passed.
        self.feed(speed=8.,duration=1.5)
        self.assertAlmostEqual(self.line_status()[0],4.25)
        self.feed(speed=8.,duration=1.5)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.0)
        self.assertAlmostEqual(self.target.point.y,0.,delta=.05)

    def test_straddle_overflow_below_occupancy_ratio_shifts_centre(self):
        self.feed()
        # 0.15 m overflow of a 2 m box: classified ADJACENT, still shifts with a 0.75 m clearance -> line 4.25.
        self.feed(spans=[(1.6,3.6,10.)],speed=0.,duration=.8)
        self.assertEqual(self.classification.assignments[0],CameraBBoxLaneResult.ADJACENT)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],4.25)
        self.feed(speed=8.,duration=3.0)
        # Fully inside the adjacent lane: enough room, stay on line 4.
        self.feed(spans=[(2.4,4.4,10.)],speed=0.,duration=.8)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],4.0)
        self.assertAlmostEqual(self.target.point.y,0.,delta=.05)

    def test_straddle_own_side_offsets_past_obstacle_without_lane_change(self):
        self.feed()
        # 1.55 m into the own lane: line 4.75 (body partly over the right edge line) beats the left lane.
        self.feed(spans=[(0.4,1.95,14.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.75)
        self.assertEqual(status['locked_lane'],'1')
        self.assertLess(self.command.steering,0)

    def test_straddle_centre_line_obstacle_moves_past_outer_line(self):
        self.feed()
        self.feed(spans=[(0.65,2.9,10.)],speed=0.,duration=.8)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],4.5)
        self.assertLess(self.command.steering,0)

    def test_straddle_centre_obstacle_uses_lane_line(self):
        self.feed()
        # Narrow obstacle in the own lane centre: line 3 and line 5 are equally far, right first -> line 5.
        self.feed(spans=[(-0.25,0.25,13.)],speed=0.,duration=.8)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],5.0)
        self.assertLess(self.command.steering,0)
        # The vehicle has moved onto line 5: lanes appear shifted left by 1.75 m.
        self.feed(shift=1.75,speed=8.,duration=1.0)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],5.0)
        self.feed(shift=1.75,speed=8.,duration=2.5)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.0)  # Passed (13+6+0.79+2 m): back to the lane centre.
        self.assertEqual(status['locked_lane'],'1')
        self.assertGreater(self.target.point.y,0.3)  # Heading back towards line 4 (+1.75 m in this frame).

    def test_straddle_obstacles_on_both_sides_use_gap_centre(self):
        self.feed()
        # Right obstacle blocks line 4 (y -0.8..-3.2); left obstacle covers up to line ~1.49 (y 4.4..6.0).
        # The nearest clear line 3.5 would graze the right obstacle; the gap centre is line ~2.97.
        self.feed(spans=[(-3.2,-0.8,13.),(4.4,6.0,13.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.97,delta=0.15)
        self.assertEqual(status['locked_lane'],'0')
        self.assertGreater(self.command.steering,0)

    def test_return_line_mode_back_to_entry_lane_after_pass(self):
        self.feed()
        # Own lane blocked: the left lane (line 2.25) becomes my lane while passing.
        self.feed(spans=[(-1.2,1.2,14.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.25)
        self.assertEqual(status['locked_lane'],'0')
        # The vehicle has moved onto line 2.25: lanes appear shifted right by 3.06 m.
        self.feed(shift=-3.0625,speed=8.,duration=1.0)
        self.feed(shift=-3.0625,speed=8.,duration=2.5)
        self.following()
        line,status=self.line_status()
        # Passed and clear: back to the entry lane centre, not the nearer left-lane centre (line 2).
        self.assertAlmostEqual(line,4.0)
        self.assertEqual(status['locked_lane'],'1')
        self.assertLess(self.target.point.y,-0.3)  # Heading back right towards line 4.

    def test_return_widened_entry_lane_obstacle_moves_further_left(self):
        self.feed()
        # Entry-lane obstacle: avoid onto line 2.25 in the left lane.
        self.feed(spans=[(-1.2,1.2,14.)],speed=0.,duration=.8)
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.25)
        # Closer views widen the remembered obstacle (y up to 2.3): 2.25 is blocked. Line 1.75 is
        # 3.94 m from the entry lane centre but 0.44 m from the current (left) lane centre.
        self.feed(spans=[(-2.0,2.3,14.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,1.75)
        self.assertEqual(status['locked_lane'],'0')
        # Passed and clear: back to the entry lane across 2.25 lines.
        self.feed(shift=-3.9375,speed=8.,duration=1.0)
        self.feed(shift=-3.9375,speed=8.,duration=2.5)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.0)
        self.assertEqual(status['locked_lane'],'1')

    def test_return_gps_return_skips_pass_margin(self):
        self.feed()
        self.feed(spans=[(-1.2,1.2,14.)],speed=0.,duration=.8)
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.25)
        # GPS is back: return as soon as the entry-lane path is clear, while the obstacle is
        # still remembered (before vehicle rear + avoid_pass_margin has passed it).
        self.gps_return_pub.publish(Bool(True))
        returned=None
        for _ in range(40):
            self.feed(shift=-3.0625,speed=4.,duration=.25)
            line,status=self.line_status()
            if line==4.0:
                returned=status
                break
        self.assertIsNotNone(returned,'no GPS-return to the entry lane')
        self.assertEqual(returned['gps_return'],'true')
        self.assertNotEqual(returned['line_obstacles'],'0')
        self.assertEqual(returned['locked_lane'],'1')

    def test_straddle_far_blocking_obstacle_uses_lane_change(self):
        self.feed()
        # Own lane fully blocked beyond the path reach: not ignored; the left lane (line 2.25) becomes my lane.
        self.feed(spans=[(-1.2,1.2,14.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.25)
        self.assertEqual(status['locked_lane'],'0')
        self.assertEqual(self.reference.data,0)
        self.assertGreater(self.command.steering,0)

    def test_straddle_own_lane_blocked_changes_lane_despite_far_box(self):
        self.feed()
        # Live scene: a parked car fills the own lane, a car further ahead is still unresolved.
        self.feed(spans=[(-1.6,1.6,14.)],far=True,speed=0.,duration=.8)
        self.assertEqual(self.classification.reason,'far_boxes_deferred')
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,2.0)
        self.assertEqual(status['locked_lane'],'0')
        self.assertEqual(status['change_state'],'following_lane')
        self.assertGreater(self.command.steering,0)

    def test_straddle_live_truck_follows_left_edge_line(self):
        self.feed()
        # Live scene mirrored: only the road edge line 5 clears a truck reaching 0.02 m from the lane centre.
        self.feed(spans=[(0.02,5.16,16.)],speed=0.,duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,5.0)
        self.assertIn('line 5.00',status['line_decision'])

    def test_straddle_far_unresolved_box_allows_own_lane_avoidance(self):
        self.feed()
        self.feed(spans=[(1.3,3.0,10.)],far=True,speed=0.,duration=.8)
        self.assertEqual(self.classification.reason,'far_boxes_deferred')
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.25)
        self.assertEqual(status['locked_lane'],'1')

    def test_straddle_lidar_only_obstacle_in_road_area(self):
        self.feed()
        # No camera box. LiDAR: 0.5 m object in the own lane centre; walls outside the road, an over-long piece.
        walls=[(-5.,15.,6.5,7.0,3.),(-5.,15.,-3.5,-3.0,3.),(-20.,20.,0.0,0.4,3.)]
        self.feed(speed=0.,lidar=[(13.,14.,-0.25,0.25,1.)]+walls,duration=1.0)
        self.assertEqual(self.classification.own_state,CameraBBoxLaneResult.NO_DETECTION)
        self.following()
        line,status=self.line_status()
        self.assertEqual(status['lidar_fresh'],'true',status)
        self.assertEqual(status['road_obstacles'],'1')
        # The wall at -3.0 m keeps the right side to the wall: line 3 (centre line) instead of line 5.
        self.assertAlmostEqual(line,3.0)
        # Measured LiDAR length is used for passing.
        self.feed(shift=-1.75,speed=8.,lidar=[],duration=1.5)
        self.assertAlmostEqual(self.line_status()[0],3.0)
        self.feed(shift=-1.75,speed=8.,lidar=[],duration=1.5)
        self.assertAlmostEqual(self.line_status()[0],4.0)

    def test_straddle_lidar_memory_follows_latest_scan_in_narrow_gap(self):
        self.feed()
        # Objects on both sides leave a 3.4 m gap around line 4 (needs about 3.0 m).
        left,right=(9.,10.,1.7,3.0,1.),(9.,10.,-3.0,-1.7,1.)
        self.feed(speed=0.,lidar=[left,right],duration=.8)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],4.0)
        # One wide transient measurement of the left object closes the gap.
        self.feed(speed=0.,lidar=[(9.,10.,0.3,3.0,1.),right],duration=.3)
        # The next scans show the true shape again: memory follows them instead of keeping the union.
        self.feed(speed=0.,lidar=[left,right],duration=.8)
        self.following()
        line,status=self.line_status()
        self.assertAlmostEqual(line,4.0,delta=0.15)
        self.assertEqual(status['lidar_fresh'],'true',status)

    def test_straddle_lidar_blocks_adjacent_that_camera_sees_clear(self):
        self.feed()
        # Camera: own-lane box, adjacent NO_DETECTION. LiDAR: a wide object also filling the adjacent lane.
        self.feed([0],speed=0.,lidar=[(12.,16.,-0.6,5.0,1.5)],duration=1.0)
        self.assertEqual(self.classification.adjacent_state,CameraBBoxLaneResult.NO_DETECTION)
        self.stopped('no_clear_line')
        self.assertEqual(self.status()[1]['locked_lane'],'1')

    def test_straddle_lidar_wall_opens_shoulder_corridor(self):
        self.feed()
        # The object leaves 1.8 m to the right road edge. Without a confirmed wall (and overrun 0) no line fits.
        blocker=(15.,19.,0.05,5.0,1.2)
        self.feed(speed=0.,lidar=[blocker],duration=1.0)
        self.stopped('no_clear_line')
        # LiDAR confirms the tunnel wall at -3.6 m: the corridor extends onto the shoulder -> line 5.
        wall=(-10.,40.,-3.9,-3.6,3.)
        self.feed(speed=0.,lidar=[blocker,wall],duration=1.0)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],5.0)
        self.assertLess(self.command.steering,0)

    def test_straddle_lane_hold_through_lane_loss(self):
        self.feed()
        self.feed(spans=[(1.3,3.0,10.)],speed=0.,imu=0.,duration=.8)
        self.following()
        self.assertAlmostEqual(self.line_status()[0],4.25)
        # Camera lanes lost (e.g. large heading during an avoidance): keep the last lanes moved by dead
        # reckoning (IMU yaw rate + speed) for at most 3 s / 8 m.
        self.feed(camera=False,bbox=False,speed=2.,imu=0.,duration=1.0)
        self.following()
        status=self.status()[1]
        self.assertEqual(status['lane_hold_active'],'true',status)
        self.assertAlmostEqual(self.line_status()[0],4.25)
        self.feed(camera=False,bbox=False,speed=2.,imu=0.,duration=3.0)
        self.stopped()
        self.assertEqual(self.status()[1]['lane_hold_active'],'false')

    def test_straddle_without_room_stops(self):
        self.feed()
        self.feed(spans=[(-0.3,2.9,10.)],speed=0.,duration=.8)
        self.stopped('no_clear_line')

    def test_pedal_speed_control_follows_ego_speed(self):
        self.feed()
        # Slower than the target: accelerator only.
        self.feed(speed=0.,duration=.6)
        self.assertEqual(self.command.longlCmdType,1)
        self.assertGreater(self.command.accel,0.)
        self.assertEqual(self.command.brake,0.)
        self.assertLessEqual(self.command.accel,0.5)
        # Much faster than the 20 km/h target: brake pedal, never the full stop brake.
        self.feed(speed=12.,duration=.6)
        self.assertEqual(self.command.longlCmdType,1)
        self.assertEqual(self.command.accel,0.)
        self.assertGreater(self.command.brake,0.)
        self.assertLessEqual(self.command.brake,0.5)
        # Without ego speed: MORAI velocity control fallback.
        self.feed(duration=.6)
        self.assertEqual(self.command.longlCmdType,2)
        self.assertAlmostEqual(self.command.velocity,20.,delta=.1)

    def test_straddle_without_ego_speed_keeps_legacy_stop(self):
        self.feed()
        self.feed(spans=[(1.3,3.0,10.)],duration=.8)
        self.stopped('adjacent_lane_detected_or_path_invalid')
        self.assertEqual(self.status()[1]['ego_speed_fresh'],'false')

    def test_change_timeout_latches(self):
        self.feed()
        self.feed([0],duration=5.8)
        self.stopped('lane_change_timeout_restart_required')
        self.feed([])
        self.stopped('lane_change_timeout_restart_required')


if __name__=='__main__':
    rostest.rosrun('lane_boundary_adapter','camera_bbox_lane_change_integration',LaneChangeTest)
