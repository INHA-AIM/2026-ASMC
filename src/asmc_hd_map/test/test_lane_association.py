import math
import unittest

from asmc_hd_map.lane_association import (
    LaneAssociator,
    LaneGeometry,
    STATE_BOUNDARY,
    STATE_INSIDE,
    STATE_NEARBY,
    STATE_UNKNOWN,
)


def straight_lane(lane_id, x0, x1, **kwargs):
    return LaneGeometry.from_coordinates(
        lane_id=lane_id,
        link_id=lane_id.replace("lane:", ""),
        lane_number=int(kwargs.pop("lane_number", 1)),
        lane_number_scope_id="road",
        polygon_xy=[(x0, 0.0), (x1, 0.0), (x1, 20.0), (x0, 20.0)],
        centerline_xy=[((x0 + x1) / 2.0, 0.0), ((x0 + x1) / 2.0, 20.0)],
        **kwargs,
    )


class LaneAssociationTest(unittest.TestCase):
    def setUp(self):
        self.right = straight_lane(
            "lane:right", 0.0, 3.5, lane_number=3,
            left_adjacent_lane_id="lane:left",
        )
        self.left = straight_lane(
            "lane:left", -3.5, 0.0, lane_number=2,
            right_adjacent_lane_id="lane:right",
        )
        self.associator = LaneAssociator([self.left, self.right])

    def test_reference_point_must_penetrate_and_confirm_lateral_switch(self):
        initial = self.associator.update(1.75, 5.0, math.pi / 2.0)
        self.assertEqual((initial.lane_id, initial.state), ("lane:right", STATE_INSIDE))

        boundary = self.associator.update(0.0, 5.0, math.pi / 2.0)
        self.assertEqual(boundary.lane_id, "lane:right")
        self.assertEqual(boundary.state, STATE_BOUNDARY)

        first = self.associator.update(-0.35, 5.0, math.pi / 2.0)
        self.assertEqual(first.lane_id, "lane:right")
        self.assertEqual(first.state, STATE_NEARBY)
        self.assertTrue(first.transition_pending)

        second = self.associator.update(-0.35, 5.2, math.pi / 2.0)
        self.assertEqual((second.lane_id, second.state), ("lane:left", STATE_INSIDE))
        self.assertEqual(second.previous_lane_id, "lane:right")

    def test_small_departure_keeps_lane_but_reports_nearby(self):
        self.associator.update(1.75, 5.0, math.pi / 2.0)
        result = self.associator.update(4.0, 5.0, math.pi / 2.0)
        self.assertEqual((result.lane_id, result.state), ("lane:right", STATE_NEARBY))
        self.assertLess(result.polygon_clearance_m, 0.0)

    def test_large_departure_is_unknown(self):
        self.associator.update(1.75, 5.0, math.pi / 2.0)
        result = self.associator.update(7.0, 5.0, math.pi / 2.0)
        self.assertFalse(result.valid)
        self.assertEqual(result.state, STATE_UNKNOWN)

    def test_successor_transition_does_not_wait_for_lateral_hysteresis(self):
        first = LaneGeometry.from_coordinates(
            "lane:first", "first", 2, "road",
            [(0.0, 0.0), (3.5, 0.0), (3.5, 10.0), (0.0, 10.0)],
            [(1.75, 0.0), (1.75, 10.0)],
            successor_lane_ids=("lane:next",),
        )
        next_lane = LaneGeometry.from_coordinates(
            "lane:next", "next", 2, "road",
            [(0.0, 10.0), (3.5, 10.0), (3.5, 20.0), (0.0, 20.0)],
            [(1.75, 10.0), (1.75, 20.0)],
            predecessor_lane_ids=("lane:first",),
        )
        associator = LaneAssociator([first, next_lane])
        associator.update(1.75, 8.0, math.pi / 2.0)
        result = associator.update(1.75, 11.0, math.pi / 2.0)
        self.assertEqual(result.lane_id, "lane:next")

    def test_opposite_heading_is_not_associated(self):
        result = self.associator.update(1.75, 5.0, -math.pi / 2.0)
        self.assertFalse(result.valid)
        self.assertEqual(result.state, STATE_UNKNOWN)


if __name__ == "__main__":
    unittest.main()
