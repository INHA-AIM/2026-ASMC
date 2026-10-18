import unittest
from types import SimpleNamespace as S
from imu_runtime_metrics import imu_application, family_probes


class ImuRuntimeMetricsTest(unittest.TestCase):
    def row(self, event, at, stamp=10, session='a', **extra):
        return dict(event=event, monotonic=at, source_stamp_ns=stamp, session_id=session, **extra)

    def test_policy_before_window_and_unpublished_next_frame(self):
        rows = [self.row('imu_projection', .9, used=True, reason='corrected'),
                self.row('family_fit_policy', .95, mode='M1_source', joint_width_fit=True),
                self.row('family_publish_end', 1.01),
                self.row('imu_projection', 1.9, stamp=11, used=True, reason='corrected'),
                self.row('family_fit_policy', 2.05, stamp=11, mode='M1_source', joint_width_fit=True),
                self.row('family_publish_end', 2.1, stamp=11)]
        result = imu_application(rows, 1., 2., True)
        self.assertEqual((result['frames'], result['corrected'], result['joint_frames']), (1, 1, 1))
        self.assertFalse(result['unmatched'])

    def test_other_session_and_missing_policy_fail(self):
        rows = [self.row('imu_projection', .8, session='b', used=True),
                self.row('family_publish_end', 1.1)]
        result = imu_application(rows, 1., 2., True)
        self.assertEqual(len(result['unmatched']), 2)
        self.assertEqual(result['corrected'], 0)

    def test_geometry_probes_do_not_bridge_invalid_support(self):
        boundary = S(role=2, polyline=[S(x=x, y=x * .1, z=-.36) for x in (0, 5, 10, 15)],
                     support_state=[1, 1, 0, 1])
        result = family_probes(S(boundaries=[boundary]))
        self.assertEqual(result, [[2, 5., .5, -.36]])


if __name__ == '__main__': unittest.main()
