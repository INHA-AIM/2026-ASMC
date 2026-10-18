import unittest
from types import SimpleNamespace as NS
import numpy as np
from asmc_camera.runtime.sensor_timing import SensorTimingIndex, ImuTimeline, slerp


class Stamp:
    def __init__(self, value): self.value = value
    def to_nsec(self): return self.value


def metadata(ns, seq=1, session='s', **kwargs):
    row = dict(bridge_session_id=session, sensor_wire_sequence_valid=True,
               sensor_topic='/imu', sensor_frame_id='imu', published_stamp=Stamp(seq),
               sensor_wire_sequence=seq, stream_id='imu', source_clock_domain='morai_sensor',
               source_sec=ns//10**9, source_nsec=ns%10**9, source_valid=True,
               source_duplicate=False, source_conflict=False, source_regression=False)
    row.update(kwargs)
    return NS(**row)


def key(row):
    return row.sensor_topic, row.sensor_frame_id, row.published_stamp.to_nsec(), row.sensor_wire_sequence


class SensorTimingTest(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.timeline = ImuTimeline(clock=lambda:self.now, capacity=8)

    def add(self, ns, seq, q=(0.,0.,0.,1.), **flags):
        row = metadata(ns, seq, **flags)
        self.timeline.add_timing(row)
        self.timeline.add_imu(key(row), q, 'imu')
        return row

    def test_exact_bracket_no_extrapolation_and_receipt_freshness(self):
        a = self.add(10**9,1)
        b = self.add(1080000000,2,(0.,0.,np.sin(.05),np.cos(.05)))
        pose, reason = self.timeline.attitude(metadata(1040000000))
        self.assertEqual(reason,'interpolated'); self.assertEqual(pose.bracket_ms,80.)
        np.testing.assert_allclose(pose.quaternion,[0,0,np.sin(.025),np.cos(.025)],atol=1e-12)
        self.assertEqual(self.timeline.attitude(a)[1],'exact')
        self.assertEqual(self.timeline.attitude(metadata(1090000000))[1],'imu_not_bracketed')
        self.now += .351
        self.assertIsNone(self.timeline.attitude(a)[0])

    def test_late_metadata_joins_signal_but_does_not_refresh_it(self):
        row = metadata(10**9)
        self.timeline.add_imu(key(row),(0,0,0,1),'imu'); self.now += .2
        self.timeline.add_timing(row)
        self.assertIsNotNone(self.timeline.attitude(row)[0])
        self.now += .2
        self.assertIsNone(self.timeline.attitude(row)[0])

    def test_duplicate_pose_is_not_independent_and_conflict_is_excluded(self):
        self.add(10**9,1); self.add(10**9,2,source_duplicate=True)
        self.assertEqual(len(self.timeline.poses),1)
        self.add(10**9,3,source_conflict=True)
        self.assertIsNone(self.timeline.attitude(metadata(10**9))[0])

    def test_restart_retired_session_and_colliding_wire_key_fail_closed(self):
        self.add(10**9,1)
        self.add(2*10**9,2,session='new')
        self.assertEqual(self.timeline.attitude(metadata(10**9))[1],'bridge_session_mismatch')
        self.timeline.add_timing(metadata(10**9,1,session='s'))
        self.assertEqual(self.timeline.session,'new')
        index=SensorTimingIndex(clock=lambda:self.now)
        old=metadata(10**9,1); index.add(old)
        new=metadata(2*10**9,1,session='new'); index.add(new)
        self.assertIsNone(index.lookup(key(new)))

    def test_broad_bracket_invalid_source_and_frame_are_rejected(self):
        self.add(10**9,1); self.add(1200000000,2)
        self.assertEqual(self.timeline.attitude(metadata(1100000000))[1],'imu_bracket_too_wide')
        self.assertEqual(self.timeline.attitude(metadata(10**9,source_clock_domain='other'))[1], 'camera_source_invalid')
        self.assertIsNone(self.timeline.attitude(metadata(10**9),imu_frame_id='wrong')[0])

    def test_capacity_ttl_and_invalid_quaternion(self):
        for i in range(30): self.add(10**9+i*1000,i)
        self.assertLessEqual(len(self.timeline.index.entries),8)
        self.assertLessEqual(len(self.timeline.poses),8)
        self.timeline.add_imu(('a','b',1,1),(0,0,0,0),'imu')
        self.assertEqual(self.timeline.counters['invalid_quaternion'],1)
        self.now += 3
        self.timeline.attitude(metadata(10**9))
        self.assertEqual(len(self.timeline.poses),0)

    def test_slerp_sign_does_not_change_rotation(self):
        np.testing.assert_allclose(slerp([0,0,0,1],[0,0,0,-1],.5),[0,0,0,1])

    def test_source_regression_flushes_old_poses_and_requires_new_bracket(self):
        self.add(2 * 10**9, 1)
        self.add(10**9, 2, source_regression=True)
        self.assertEqual(len(self.timeline.poses), 0)
        self.assertIsNone(self.timeline.attitude(metadata(2 * 10**9))[0])
        self.add(1010000000, 3)
        self.assertIsNone(self.timeline.attitude(metadata(1050000000))[0])
        self.add(1090000000, 4)
        self.assertEqual(self.timeline.attitude(metadata(1050000000))[1], 'interpolated')


if __name__=='__main__': unittest.main()
