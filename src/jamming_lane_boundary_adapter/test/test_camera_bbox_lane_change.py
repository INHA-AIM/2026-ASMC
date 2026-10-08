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
from asmc_msgs.msg import LaneBoundaryFamily
from diagnostic_msgs.msg import DiagnosticArray
from asmc_camera.bev.calibration import load_calibration_snapshot
from geometry_msgs.msg import PointStamped
from lane_boundary_adapter.msg import CameraBBoxLaneResult
from morai_msgs.msg import CtrlCmd
from Msg.msg import Yolo_bbox,Yolo_bbox_arr
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header,Bool,Int8
from visualization_msgs.msg import MarkerArray,Marker

spec=importlib.util.spec_from_file_location('fixture',str(Path(__file__).with_name('lane_family_fixture.py')))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


class LaneChangeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('bbox_change_integration')
        cls.camera=load_calibration_snapshot(Path(rospkg.RosPack().get_path('asmc_camera'))/'config/bev/cameras_v2.yaml')[1]['front']
        cls.jpeg=cv2.imencode('.jpg',np.zeros((720,1280,3),dtype=np.uint8))[1].tobytes()
        cls.lane_pub=rospy.Publisher('/perception/lane/boundary_family',LaneBoundaryFamily,queue_size=1)
        cls.image_pub=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=1)
        cls.box_pub=rospy.Publisher('/planning/camera_bbox_lane/yolo',Yolo_bbox_arr,queue_size=1)
        cls.stop_pub=rospy.Publisher('/planning/camera_bbox_lane_change/emergency_stop',Bool,queue_size=1)
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
            'lane_change_timeout:='+('10.0' if self._testMethodName.startswith('test_ignore') else '5.0'),
            *([] if self._testMethodName.startswith(('test_ignore','test_repeat')) else ['ignore_bbox_during_lane_change:=false']),
            *([] if self._testMethodName.startswith('test_repeat') else
              ['one_lane_change_only:='+('false' if self._testMethodName=='test_clipped_cannot_start_second_change' else 'true')]),
            'hood_crop_bottom_ratio:='+('0.21' if self._testMethodName.startswith(('test_clipped','test_ignore')) else '0.0'),
            'far_obstacle_speed_kph:='+('0.0' if self._testMethodName in ('test_far_approach_can_be_disabled','test_ignore_far_zero_speed_during_change') else '5.0'),
            'lane_change_pause_timeout:='+('1.0' if self._testMethodName=='test_pause_timeout_latches' else '30.0')],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+10
        while (self.command is None or self.image_pub.get_num_connections()<1 or self.lane_pub.get_num_connections()<2) and time.monotonic()<end:
            time.sleep(.03)
        self.assertIsNotNone(self.command)

    def tearDown(self):
        os.killpg(self.process.pid,signal.SIGINT)
        self.process.wait(timeout=10)
        self.log.close()
        time.sleep(.15)

    def status(self):
        self.assertIsNotNone(self.diag)
        return self.diag.status[0].message,{v.key:v.value for v in self.diag.status[0].values}

    def feed(self, ys=(),shift=0.,duration=.55,camera=True,bbox=True,stamp=None,gap=False,source_gap=False,target_gap=False,far=False,slope=0.,box_x=10.,raw_boxes=None):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            source=rospy.Time.now() if stamp is None else stamp
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
                    boxes.append(Yolo_bbox(u-15,v-60,u+15,v,'car'))
                if far:
                    uv,_=self.camera.project_ground(np.array([[30.,shift]]))
                    u,v=uv[0]
                    boxes.append(Yolo_bbox(u-15,v-50,u+15,v,'car'))
                if raw_boxes is not None:boxes=raw_boxes
                header=Header(stamp=source,frame_id='camera_front')
                self.image_pub.publish(CompressedImage(header=header,format='jpeg',data=self.jpeg))
                self.box_pub.publish(Yolo_bbox_arr(header=header,objc=len(boxes),yolo_bbox_arr=boxes))
            time.sleep(.05)
        time.sleep(.03)

    def following(self):
        self.assertEqual(self.command.longlCmdType,2,self.status())
        self.assertEqual(self.command.brake,0)

    def stopped(self,reason=None):
        self.assertEqual(self.command.longlCmdType,1,self.status())
        self.assertEqual(self.command.brake,1)
        if reason:self.assertEqual(self.status()[0],reason)

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
        self.feed(raw_boxes=[Yolo_bbox(600,480,680,568,'car')])
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
        normal=Yolo_bbox(600,480,680,560,'car')
        clipped=Yolo_bbox(600,480,680,568,'car')
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
        target=Yolo_bbox(200,480,280,560,'car')
        self.feed(raw_boxes=[target])
        self.stopped('target_lane_detected')
        target.yMax=568
        self.feed(raw_boxes=[target])
        self.assertTrue(self.classification.valid,self.classification)
        self.assertEqual(list(self.classification.clipped_association),[True])
        self.stopped('target_lane_detected')

    def test_clipped_cannot_start_second_change(self):
        self.feed()
        self.feed([0])
        self.feed([0],shift=-3.5,duration=.65)
        self.assertEqual(self.status()[1]['change_completed'],'true')
        normal=Yolo_bbox(600,480,680,560,'car')
        self.feed(raw_boxes=[normal],shift=-3.5)
        normal.yMax=568
        self.feed(raw_boxes=[normal],shift=-3.5)
        self.assertEqual(list(self.classification.clipped_association),[True])
        self.stopped('clipped_obstacle_cannot_start_change')
        self.assertEqual(self.status()[1]['change_state'],'following_lane')

    def test_clipped_new_object_and_expired_track_stop(self):
        normal=Yolo_bbox(600,480,680,560,'car')
        clipped=Yolo_bbox(600,480,680,568,'car')
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

    def test_change_timeout_latches(self):
        self.feed()
        self.feed([0],duration=5.8)
        self.stopped('lane_change_timeout_restart_required')
        self.feed([])
        self.stopped('lane_change_timeout_restart_required')


if __name__=='__main__':
    rostest.rosrun('lane_boundary_adapter','camera_bbox_lane_change_integration',LaneChangeTest)
