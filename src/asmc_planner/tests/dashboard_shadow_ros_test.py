#!/usr/bin/env python3
"""Real ROS messages -> dashboard callbacks -> HTTP; no simulator/control."""
import json
import time
import unittest
from urllib.request import urlopen
from urllib.error import URLError

import rospy
import rostest
from asmc_msgs.msg import LaneBoundaryFamily, LaneBoundaryFamilyMember, LidarTrackingObservation, LidarObject, LidarTrackingState
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class DashboardTest(unittest.TestCase):
    def test_local_geometry_without_map_or_debug_and_invalidation(self):
        publishers = [
            rospy.Publisher('/perception/lane/boundary_family', LaneBoundaryFamily, queue_size=1),
            rospy.Publisher('/perception/lidar/tracks_local', LidarTrackingObservation, queue_size=1),
            rospy.Publisher('/planning/camera_bbox_lane_change/markers', MarkerArray, queue_size=1),
            rospy.Publisher('/planning/control_selector/diagnostics', String, queue_size=1)]
        deadline = time.monotonic() + 10
        while any(not p.get_num_connections() for p in publishers) and time.monotonic() < deadline:
            time.sleep(.03)
        self.assertTrue(all(p.get_num_connections() for p in publishers))
        def snapshot():
            with urlopen('http://127.0.0.1:18088/api/snapshot', timeout=2) as response:
                return json.load(response)
        # ROS subscriptions are installed before HTTP bind. Wait for readiness
        # before creating source stamps, rather than weakening freshness.
        deadline = time.monotonic() + 5
        while True:
            try:
                snapshot()
                break
            except URLError:
                if time.monotonic() >= deadline: raise
                time.sleep(.02)
        stamp = rospy.Time.now()
        family = LaneBoundaryFamily(valid=True, status_reason='fixture')
        family.header.stamp = stamp
        family.header.frame_id = 'base_link'
        family.boundaries = [LaneBoundaryFamilyMember(role=1,
            polyline=[Point32(1, 2, 0), Point32(12, 2, 0)], support_state=[1, 1])]
        tracks = LidarTrackingObservation()
        o = tracks.observation
        o.objects.header = family.header
        o.producer_session_id = 'fixture'; o.generation = 1; o.scan_sequence = 1
        o.config_id = 'cfg'; o.calibration_id = 'cal'
        o.objects.status = o.objects.OBSERVED
        box = LidarObject(track_id=7, classification='static_obstacle', camera_class='plastic_barrier')
        box.observation_state = box.OBSERVED
        for i in range(8):
            box.obb_corners[i] = Point(5 + (i & 1), 1 + ((i >> 1) & 1), (i >> 2) & 1)
        o.objects.objects = [box]
        tracks.tracking = [LidarTrackingState(track_id=7)]
        marker = Marker(ns='selected_centerline', type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.header = family.header; marker.points = [Point(0,0,0), Point(12,1,0)]
        for pub, msg in zip(publishers, [family, tracks, MarkerArray(markers=[marker]),
                                      String('mode=JAMMING selected_source=/ctrl_cmd/jamming command_fresh=false')]):
            pub.publish(msg)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            data = snapshot()
            if all(k in data['shadow'] for k in ('family', 'path', 'objects', 'selector')):
                break
            time.sleep(.01)
        scene = data['shadow']
        self.assertIsNone(data['ego'])  # No stale GPS pose fabricated.
        for key in ('family', 'path', 'objects'):
            self.assertTrue(scene[key]['available'], scene[key])
            self.assertEqual(scene[key]['frame_id'], 'base_link')
        self.assertEqual(scene['objects']['payload']['objects'][0]['camera_class'], 'plastic_barrier')
        self.assertEqual(scene['objects']['payload']['scan_sequence'], 1)
        self.assertEqual(scene['selector']['payload']['selected_source'], '/ctrl_cmd/jamming')
        self.assertEqual(scene['path']['payload']['lines'][0]['kind'], 'selected_centerline')
        # Explicit invalid input clears geometry before its previous sample expires.
        o.scan_sequence = 2; o.objects.header.stamp = rospy.Time.now()
        o.objects.status = o.objects.INPUT_INVALID; o.objects.objects = []; tracks.tracking = []
        publishers[1].publish(tracks)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            scene = snapshot()['shadow']
            if not scene['objects']['valid']: break
            time.sleep(.01)
        self.assertFalse(scene['objects']['available'])
        self.assertEqual(scene['objects']['payload'], {})
        time.sleep(.4)
        scene = snapshot()['shadow']
        self.assertFalse(scene['family']['available'])
        self.assertFalse(scene['path']['available'])
        self.assertTrue(scene['selector']['available'])  # 1 Hz status, not control freshness.
        for pub in publishers: pub.unregister()


if __name__ == '__main__':
    rospy.init_node('dashboard_shadow_test')
    rostest.rosrun('asmc_planner', 'dashboard_shadow_ros', DashboardTest)
