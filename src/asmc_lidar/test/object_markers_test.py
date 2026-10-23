#!/usr/bin/env python3
"""CPU-only tests of the read-only RViz adapter (no ROS master or detector)."""
import importlib.util
import io
from pathlib import Path
import unittest

import rospy
from asmc_msgs.msg import LidarObject, LidarObjectArray, LidarObservation
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

spec = importlib.util.spec_from_file_location('lidar_markers', Path(__file__).resolve().parents[1] / 'scripts/lidar_markers.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def observation(sequence=1, generation=1, session='first', status=LidarObjectArray.OBSERVED):
    result = LidarObservation(producer_session_id=session, generation=generation, scan_sequence=sequence)
    result.objects.header.frame_id = 'base_link'
    result.objects.header.stamp = rospy.Time(100 + sequence)
    result.objects.status = status
    obj = LidarObject(track_id=42, classification='vehicle', camera_class='truck', source_point_count=31)
    obj.center = Point(8., 2., 1.)
    # Tilted, non-planar in the vehicle frame: yaw-only reconstruction would
    # change these authoritative corners and erase attitude compensation.
    obj.obb_corners = [Point(7., 1., .1), Point(9., 1., .3), Point(9., 3., .5), Point(7., 3., .3),
                       Point(7., 1., 1.1), Point(9., 1., 1.3), Point(9., 3., 1.5), Point(7., 3., 1.3)]
    result.objects.objects = [obj]
    return result


def serialized(message):
    output = io.BytesIO()
    message.serialize(output)
    return output.getvalue()


class ObjectMarkersTest(unittest.TestCase):
    def test_original_corners_header_classification_and_input_are_preserved(self):
        message = observation()
        before = serialized(message)
        rendered = module.ObjectMarkers().render(message)
        box, text = [m for m in rendered.markers if m.action == Marker.ADD]
        expected = [message.objects.objects[0].obb_corners[i] for edge in module.EDGES for i in edge]
        self.assertEqual(box.points, expected)
        self.assertEqual(len(box.points), 24)
        self.assertEqual(box.header, message.objects.header)
        self.assertEqual(box.type, Marker.LINE_LIST)
        self.assertEqual(text.text, '#42 vehicle / truck  n=31')
        self.assertAlmostEqual(text.pose.position.z, 1.75)
        self.assertEqual(serialized(message), before)
        self.assertFalse(any(m.type == Marker.ARROW for m in rendered.markers if m.action == Marker.ADD))

    def test_empty_and_invalid_results_remove_existing_boxes(self):
        for status in (LidarObjectArray.OBSERVED_EMPTY, LidarObjectArray.INPUT_INVALID,
                       LidarObjectArray.POSE_UNAVAILABLE):
            builder = module.ObjectMarkers()
            builder.render(observation())
            result = builder.render(observation(sequence=2, status=status))
            self.assertEqual({(m.ns, m.id, m.action) for m in result.markers},
                             {('obb', 42, Marker.DELETE), ('label', 42, Marker.DELETE)})
            self.assertEqual(builder.keys, set())

    def test_duplicate_regressing_and_retired_results_cannot_restore_boxes(self):
        builder = module.ObjectMarkers()
        message = observation(sequence=4, generation=3)
        builder.render(message)
        self.assertIsNone(builder.render(message))
        self.assertIsNone(builder.render(observation(sequence=3, generation=3)))
        self.assertIsNone(builder.render(observation(sequence=5, generation=2)))
        regressed = observation(sequence=5, generation=3)
        regressed.objects.header.stamp = rospy.Time(100)
        self.assertIsNone(builder.render(regressed))

    def test_reset_and_producer_restart_clear_previous_display(self):
        builder = module.ObjectMarkers()
        builder.render(observation(sequence=9))
        for message in (observation(sequence=10, generation=2), observation(session='restart')):
            result = builder.render(message)
            self.assertEqual(result.markers[0].action, Marker.DELETEALL)
            self.assertEqual(len([m for m in result.markers if m.action == Marker.ADD]), 2)

    def test_bad_geometry_clears_the_object_instead_of_drawing_invalid_boxes(self):
        builder = module.ObjectMarkers()
        builder.render(observation())
        message = observation(sequence=2)
        message.objects.objects[0].obb_corners[0].z = float('nan')
        result = builder.render(message)
        self.assertFalse(any(m.action == Marker.ADD for m in result.markers))
        self.assertEqual(builder.keys, set())

    def test_input_silence_clear_keeps_the_deduplication_gate(self):
        builder = module.ObjectMarkers()
        message = observation()
        builder.render(message)
        self.assertEqual(builder.clear(message.objects.header).markers[0].action, Marker.DELETEALL)
        self.assertIsNone(builder.render(message))
        result = builder.render(observation(sequence=2))
        additions = [m for m in result.markers if m.action == Marker.ADD]
        self.assertEqual(len(additions), 2)
        self.assertTrue(all(abs(m.lifetime.to_sec() - .35) < 1e-9 for m in additions))


if __name__ == '__main__':
    unittest.main()
