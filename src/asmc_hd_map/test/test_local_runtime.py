"""ROS regression checks for the pose path and signed raster wire format."""
import importlib.util
from io import BytesIO
from pathlib import Path
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from rospy.numpy_msg import numpy_msg
    from asmc_msgs.msg import LaneAssociation
    spec = importlib.util.spec_from_file_location(
        "local_map_node", Path(__file__).resolve().parents[1] / "scripts/local_map_crop_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
except ImportError:
    module = None


@unittest.skipIf(module is None, "ROS Noetic generated messages required")
class LocalRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.node = module.LocalMapCropNode.__new__(module.LocalMapCropNode)
        self.node.context = Mock()
        self.node.pose_status = module.HdMapContext.POSE_VALID
        self.node.publish_crops = True
        self.node.lock = threading.RLock()
        self.node.meta = Obj(complete=True, operational=True, build_id="build", map_id="map", geometry_sha256="sha")
        self.node.global_lanes = Obj(build_id="build", map_id="map", geometry_sha256="sha")
        self.node.latest_gps_stamp = None
        self.node.latest_pose = (1., 2., 0., rospy.Time(100))
        self.node.latest_pose_received_monotonic = time.monotonic()
        self.node.latest_association = module.LaneAssociationResult(valid=True, state=1, lane_id="lane")
        self.node.pose_stale_timeout_sec = .5
        self.node.association_pub = Mock()
        self.node.tf_broadcaster = None
        self.node.lane_associator = Mock()
        self.node.lane_associator.update.return_value = self.node.latest_association
        self.clock = patch.object(rospy.Time, "now", return_value=rospy.Time(100))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_current_lane_does_not_wait_for_global_raster(self):
        self.node.global_grid_message = None
        self.node._set_pose(3., 4., 0., rospy.Time(100, 1000000))
        result = self.node.association_pub.publish.call_args[0][0]
        self.assertTrue(result.valid)
        self.assertEqual(result.reference_point.x, 3.)
        self.assertEqual(result.header.stamp, rospy.Time(100, 1000000))

    def test_duplicate_and_delayed_source_samples_do_not_switch_lane(self):
        self.node._set_pose(3., 4., 0., rospy.Time(100))
        self.node._set_pose(3., 4., 0., rospy.Time(90))
        self.assertEqual(self.node.latest_pose[:2], (1., 2.))
        self.node.lane_associator.update.assert_not_called()

    def test_source_age_invalidates_even_recently_received_pose(self):
        self.node.latest_pose = (1., 2., 0., rospy.Time(99))
        self.node._publish_association()
        result = self.node.association_pub.publish.call_args[0][0]
        self.assertFalse(result.valid)
        self.assertEqual(result.state, LaneAssociation.STATE_STALE)
        self.assertEqual(result.header.stamp, rospy.Time(99))

    def test_stale_crop_and_unsubscribed_crop_skip_raster_work(self):
        node = self.node
        node.last_publish = 0.
        node.minimum_period = .05
        node.global_grid_message = Obj()
        node.global_occupancy = np.zeros((2, 2), dtype=np.int8)
        node.lane_bounds = []
        node.lanes_pub = Mock()
        node.grid_pub = Mock()
        node.lanes_pub.get_num_connections.return_value = 0
        node.grid_pub.get_num_connections.return_value = 0
        with patch.object(module, "crop_ego_occupancy_grid") as crop:
            node._try_publish(True)
            node.latest_pose_received_monotonic -= 1.
            node.grid_pub.get_num_connections.return_value = 1
            node._try_publish(True)
            crop.assert_not_called()
        node.grid_pub.publish.assert_not_called()

    def test_invalid_gps_immediately_invalidates_lane_and_recovery_resets(self):
        message = Obj(status=0, header=Obj(stamp=rospy.Time(100, 1000000)))
        self.node._on_gps(message)
        self.assertEqual(self.node.pose_status, module.HdMapContext.POSE_GPS_UNAVAILABLE)
        self.assertFalse(self.node.association_pub.publish.call_args[0][0].valid)
        self.node.context.publish.assert_called_once()
        self.node._set_pose(30., 40., 0., rospy.Time(100, 2000000))
        self.assertEqual(self.node.pose_status, module.HdMapContext.POSE_VALID)
        self.assertEqual(self.node.lane_associator.reset.call_count, 2)
        self.assertEqual(self.node.latest_pose[:2], (30., 40.))

    def test_delayed_invalid_gps_cannot_invalidate_recovered_pose(self):
        self.node._on_gps(Obj(status=0, header=Obj(stamp=rospy.Time(99))))
        self.assertEqual(self.node.pose_status, module.HdMapContext.POSE_VALID)
        self.node.context.invalidate.assert_not_called()

    def test_old_valid_gps_cannot_recover_a_newer_outage(self):
        self.node._on_gps(Obj(status=0, header=Obj(stamp=rospy.Time(100, 2000000))))
        self.node._on_gps(Obj(status=1, header=Obj(stamp=rospy.Time(100, 1000000))))
        self.assertEqual(self.node.pose_status, module.HdMapContext.POSE_GPS_UNAVAILABLE)
        self.assertEqual(self.node.latest_pose[:2], (1., 2.))

    def test_production_mode_never_computes_crops(self):
        self.node.publish_crops = False
        with patch.object(module, "crop_ego_occupancy_grid") as crop:
            self.node._try_publish(True)
            crop.assert_not_called()

    def test_numpy_grid_roundtrip_preserves_signed_unknown_and_orientation(self):
        grid = numpy_msg(OccupancyGrid)()
        grid.info.width, grid.info.height = 3, 1
        grid.info.origin.orientation.z = np.sin(.3)
        grid.info.origin.orientation.w = np.cos(.3)
        grid.data = np.array([-1, 0, 100], dtype=np.int8)
        buffer = BytesIO()
        grid.serialize(buffer)
        decoded = OccupancyGrid().deserialize(buffer.getvalue())
        self.assertEqual(list(decoded.data), [-1, 0, 100])
        self.assertAlmostEqual(decoded.info.origin.orientation.z, np.sin(.3))


if __name__ == "__main__":
    unittest.main()
