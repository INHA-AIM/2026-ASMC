"""Contract checks with actual generated ROS messages, without a master."""
from io import BytesIO
import unittest
from unittest.mock import Mock, patch
try:
    import rospy
    from asmc_msgs.msg import HdMapMeta, MissionZone, MissionZoneArray
    from asmc_hd_map.msg import HdMapRoute, HdMapContext
    from geometry_msgs.msg import Point
    from asmc_hd_map.context_publisher import ContextPublisher
except ImportError:
    ContextPublisher = None


@unittest.skipIf(ContextPublisher is None, 'generated ROS messages required')
class ContextTest(unittest.TestCase):
    def setUp(self):
        clock = patch.object(rospy.Time, "now", return_value=rospy.Time(10))
        clock.start()
        self.addCleanup(clock.stop)
        with patch('rospy.Publisher'):
            self.context = ContextPublisher()
        meta = HdMapMeta()
        meta.header.frame_id = 'map'
        meta.complete = meta.operational = True
        meta.build_id, meta.map_id, meta.geometry_sha256 = 'build', 'map', 'sha'
        meta.source_ids = ['official_global_path', 'competition_metadata']
        meta.source_sha256 = ['pathsha', 'missionsha']
        route = HdMapRoute()
        route.header.frame_id = 'map'
        route.build_id, route.map_id, route.geometry_sha256 = 'build', 'map', 'sha'
        route.route_id, route.route_sha256 = 'official', 'pathsha'
        route.points = [Point(0, 0, 0), Point(100, 0, 0)]
        route.stations_m, route.length_m = [0, 100], 100
        zones = MissionZoneArray()
        zones.header.frame_id, zones.build_id, zones.route_id = 'map', 'build', 'official'
        z = MissionZone()
        z.id, z.status, z.mission_type = 'shadow', z.STATUS_OFFICIAL, z.TYPE_GPS_SHADOW
        z.route_station_start_m, z.route_station_end_m = 10, 50
        zones.source_sha256 = 'missionsha'
        zones.zones = [z]
        self.context.configure(meta=meta, route=route, missions=zones)
        self.association = Mock(valid=True, lane_id='lane', link_id='link', lane_number=1)

    def test_outage_clears_current_and_retains_explicitly_historical_shadow(self):
        pose = (20, 0, 0, rospy.Time(10))
        valid = self.context.publish(pose, HdMapContext.POSE_VALID, self.association, True)
        self.assertTrue(valid.route_valid)
        self.assertEqual(valid.route_station_m, 20)
        self.assertEqual([b.distance_m for b in valid.boundaries], [-10, 30])
        lost = self.context.publish(pose, HdMapContext.POSE_GPS_UNAVAILABLE, self.association, True)
        self.assertFalse(lost.pose_valid or lost.lane_valid or lost.route_valid)
        self.assertEqual(lost.active_mission_ids, [])
        self.assertEqual(lost.boundaries, [])
        self.assertEqual(lost.route_station_m, -1)
        self.assertEqual(lost.ego_pose.position.x, 0)
        self.assertEqual(lost.gps_shadow_zone_ids, ['shadow'])
        self.assertTrue(lost.gps_shadow_context_from_last_valid_pose)
        self.assertEqual(lost.last_valid_route_station_m, 20)
        self.assertIsNone(self.context.hint)
        recovered = self.context.publish((80, 0, 0, rospy.Time(11)), HdMapContext.POSE_VALID, self.association, True)
        self.assertEqual(recovered.route_station_m, 80)
        self.assertEqual(recovered.gps_shadow_zone_ids, [])
        self.assertFalse(recovered.gps_shadow_context_from_last_valid_pose)
        # Wire compatibility, including nested boundaries and source stamp.
        stream = BytesIO()
        valid.serialize(stream)
        decoded = HdMapContext().deserialize(stream.getvalue())
        self.assertEqual(decoded.boundaries[1].point.x, 50)
        self.assertEqual(decoded.pose_stamp, rospy.Time(10))

    def test_startup_loss_does_not_claim_a_shadow_zone(self):
        result = self.context.publish(None, HdMapContext.POSE_GPS_UNAVAILABLE, self.association, True)
        self.assertFalse(result.last_valid_context_available)
        self.assertEqual(result.gps_shadow_zone_ids, [])

    def test_same_snapshot_republication_preserves_historical_context(self):
        self.context.publish((20, 0, 0, rospy.Time(10)), HdMapContext.POSE_VALID, self.association, True)
        self.context.configure(meta=self.context.meta)
        lost = self.context.publish(None, HdMapContext.POSE_GPS_UNAVAILABLE, self.association, True)
        self.assertEqual(lost.gps_shadow_zone_ids, ['shadow'])

    def test_mixed_route_snapshot_disables_queries(self):
        self.context.route.route_sha256 = 'wrong'
        with self.assertRaises(ValueError):
            self.context.configure()
        self.assertIsNone(self.context.reference)
