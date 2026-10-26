import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from analyze_signal_bag import check_binding_evidence, check_header_order, object_frame


class RecordedBindingTest(unittest.TestCase):
    def setUp(self):
        self.token=['controller','behavior',2]
        self.request=dict(token=self.token,semantic='C2',release_authorized=True)
        self.plan=dict(token=self.token,planner_epoch='planner',sequence=3,semantic='C2',accepted_valid=True)
        self.status=dict(token=self.token,planner_epoch='planner',plan_sequence=3,linked=True,
            actuated=True,brake=0.,steering=0.,accel=.1)

    def test_common_tracking_uses_atomic_geometry_header(self):
        from types import SimpleNamespace as N
        frame=N(header=N(frame_id='map'),objects=[N(track_id=42)])
        message=N(observation=N(objects=frame),tracking=[])
        self.assertIs(object_frame(message),frame)

    def test_historical_bag_decoder_does_not_require_generated_msg_package(self):
        from types import SimpleNamespace as N
        frame=N(header=N(frame_id='map'),objects=[])
        self.assertIs(object_frame(N(tracked=frame)),frame)
        self.assertIs(object_frame(frame),frame)

    def test_explicit_identity_joins_without_relying_on_arrival_times(self):
        self.assertTrue(check_binding_evidence([self.request],[self.plan],[self.status])['passed'])

    def test_latest_old_command_trajectory_cannot_substitute_for_current_token(self):
        self.plan['token']=['controller','behavior',1]
        self.assertFalse(check_binding_evidence([self.request],[self.plan],[self.status])['passed'])

    def test_matching_plan_id_does_not_hide_different_command_content(self):
        self.plan['semantic']='C1'
        self.assertFalse(check_binding_evidence([self.request],[self.plan],[self.status])['passed'])

    def test_prepared_path_does_not_allow_acceleration_before_release(self):
        self.request['release_authorized']=False
        self.assertFalse(check_binding_evidence([self.request],[self.plan],[self.status])['passed'])
        self.status.update(accel=0.,brake=1.)
        self.assertTrue(check_binding_evidence([self.request],[self.plan],[self.status])['passed'])

    def test_missing_request_evidence_is_incomplete(self):
        report=check_binding_evidence([],[self.plan],[self.status])
        self.assertFalse(report['evidence_complete']);self.assertFalse(report['passed'])

    def test_adjacent_recorder_arrival_swap_preserves_source_order(self):
        headers={};reorders=[];failures=[];identity=('/imu','imu')
        for source,seq,arrival in ((100.2284496,78,100.2290),
                                   (100.2279330,77,100.2294),
                                   (100.2698824,79,100.2710)):
            check_header_order(headers,reorders,failures,identity,source,seq,arrival)
        self.assertEqual([],failures)
        self.assertEqual(1,len(reorders))
        self.assertEqual(79,headers[identity][1])

    def test_source_regression_with_increasing_sequence_still_fails(self):
        headers={};reorders=[];failures=[];identity=('/imu','imu')
        check_header_order(headers,reorders,failures,identity,100.2,77,100.21)
        check_header_order(headers,reorders,failures,identity,100.1,78,100.22)
        self.assertEqual([],reorders)
        self.assertEqual(1,len(failures))


if __name__=='__main__':unittest.main()
