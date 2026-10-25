import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from analyze_yellow_signal_trial import failure_response,physical_response,held_without_permission,red_exit_validation


class FailureAcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.oracle=dict(signal_violations_or_uncertain=[dict(begin=10.,end=10.1)],
            road_departure_samples=0,collisions=[])
        self.events=[dict(kind=1,stamp=100.),dict(kind=2,stamp=101.)]

    def test_failure_response_does_not_become_violation_free_success(self):
        result=failure_response(self.oracle,self.events,[dict(monotonic=9.,brake=1.)])
        self.assertTrue(result['accepted']);self.assertFalse(result['violation_free_driving'])

    def test_late_maximum_brake_missing_records_or_normal_reclassification_fail(self):
        self.assertFalse(failure_response(self.oracle,self.events,[dict(monotonic=10.1,brake=1.)])['accepted'])
        self.assertFalse(failure_response(self.oracle,self.events[:1],[dict(monotonic=9.,brake=1.)])['accepted'])
        self.events.append(dict(kind=0,stamp=102.))
        self.assertFalse(failure_response(self.oracle,self.events,[dict(monotonic=9.,brake=1.)])['accepted'])

    def test_road_departure_still_fails_and_no_crossing_is_not_invented(self):
        self.oracle['road_departure_samples']=1
        self.assertFalse(failure_response(self.oracle,self.events,[dict(monotonic=9.,brake=1.)])['accepted'])

    def test_uncertain_sensor_crossing_is_retained_without_inventing_confirmed_event(self):
        events=[dict(kind=1,stamp=100.),dict(kind=3,stamp=101.)]
        result=failure_response(self.oracle,events,[dict(monotonic=9.,brake=1.)])
        self.assertTrue(result['accepted']);self.assertTrue(result['uncertainty_recorded'])
        self.assertFalse(result['confirmed_crossing_recorded'])
        self.oracle['observation_gaps']=[dict(begin=1.,end=2.)]
        self.assertFalse(failure_response(self.oracle,events,[dict(monotonic=9.,brake=1.)])['accepted'])
        self.oracle['signal_violations_or_uncertain']=[]
        self.assertFalse(failure_response(self.oracle,self.events,[dict(monotonic=9.,brake=1.)])['accepted'])


class PhysicalResponseTest(unittest.TestCase):
    def test_ongoing_ordinary_braking_is_distinct_from_a_maximum_escalation(self):
        request=dict(monotonic=10.,ros_time=100.,brake_decision_ros=100.,brake_level=1,
                     brake_speed=3.4,brake_acceleration=-.8)
        rows=[dict(state_query_interval=[9.98,9.99],speed=3.4,reported_acceleration=dict(x=-.8)),
              dict(state_query_interval=[10.08,10.09],speed=3.3,reported_acceleration=dict(x=-.78),
                   actual_actuation=dict(brake=1.))]
        result=physical_response(request,rows)
        self.assertAlmostEqual(result['response_upper_s'],.09)
        self.assertEqual(result['response_kind'],'continued_ordinary_deceleration')
        request['brake_level']=2
        self.assertIsNone(physical_response(request,rows)['response_upper_s'])

    def test_escalation_needs_extra_actual_deceleration_and_speed_loss(self):
        request=dict(monotonic=10.1,ros_time=100.1,brake_decision_ros=100.)
        def sample(t,v,a):
            return dict(state_query_interval=[t-.01,t],speed=v,reported_acceleration=dict(x=a))
        rows=[sample(9.99,5.,-1.),sample(10.05,4.95,-1.),
              sample(10.1,5.,-3.),sample(10.15,4.7,-3.)]
        self.assertAlmostEqual(physical_response(request,rows)['response_upper_s'],.15)
        request['brake_level']=2
        self.assertIsNone(physical_response(request,rows)['response_upper_s'])
        rows[-1]['actual_actuation']=dict(brake=1.)
        self.assertAlmostEqual(physical_response(request,rows)['response_upper_s'],.15)

    def test_elapsed_time_cannot_replace_actual_baseline_or_response(self):
        request=dict(monotonic=10.,ros_time=100.,brake_decision_ros=100.)
        self.assertIsNone(physical_response(request,[])['response_upper_s'])
        row=dict(state_query_interval=[9.98,9.99],speed=5.,reported_acceleration=dict(x=0.))
        self.assertIsNone(physical_response(request,[row])['response_upper_s'])


class SafeHeldOutcomeTest(unittest.TestCase):
    def setUp(self):
        self.oracle=[];self.ros=[]
        for i in range(41):
            t=10+i*.05
            self.oracle.append(dict(monotonic=t,speed=0.,actual_actuation=dict(brake=1.)))
            self.ros.extend([dict(monotonic=t,topic='status',brake=1.,accel=0.),
                dict(monotonic=t,topic='policy',data=dict(motion='RECOVERY_BRAKE',green_count=0,severity=2))])

    def test_unobservable_signal_has_a_verified_physical_hold_not_a_fabricated_finish(self):
        self.assertTrue(held_without_permission(self.ros,self.oracle))
        self.oracle[-1]['speed']=.2
        self.assertFalse(held_without_permission(self.ros,self.oracle))

    def test_positive_acceleration_permission_or_missing_evidence_never_qualifies(self):
        self.ros[-2]['accel']=.1
        self.assertFalse(held_without_permission(self.ros,self.oracle))
        self.ros[-2]['accel']=0.;self.ros[-1]['data']['green_count']=3
        self.assertFalse(held_without_permission(self.ros,self.oracle))
        self.assertFalse(held_without_permission(self.ros[:4],self.oracle))


class RedExitReplayTest(unittest.TestCase):
    def setUp(self):
        self.binding=dict(traffic_light_id='L',stop_node_id='S',green_color=16,exit_point=[30,0],exit_direction=[1,0])
        policy=dict(motion='EXITED',actual_control_allowed=True,stop_line='S',severity=0,route_generation=2,decision_ros=100.,
            causes={c:0 for c in ('SIGNAL','COLLISION','PEDESTRIAN','INPUT_INVALID','EXECUTION_FAILURE')})
        self.ros=[dict(topic='policy',monotonic=10.,data=policy)]
        self.oracle=[dict(monotonic=10.1,aspects={'L':1},road_ok=True,collisions=[],observation_complete=True,
            bounding_box=dict(center=[33,0,0],half_length=2.3,half_width=1.1,half_height=.8,roll=0.,pitch=0.))]

    def test_historical_stronger_red_hold_needs_actual_body_and_policy_proof(self):
        self.assertTrue(red_exit_validation(self.ros,self.oracle,[],self.binding)['verified'])
        self.assertFalse(red_exit_validation([],self.oracle,[],self.binding)['verified'])
        self.oracle[0]['bounding_box']['center'][0]=32.
        self.assertFalse(red_exit_validation(self.ros,self.oracle,[],self.binding)['verified'])

    def test_recorded_milestone_must_replay_before_green_phase_restoration(self):
        events=[dict(event='recovery_red_exit_confirmed',monotonic=10.2,policy_route_generation=2,
                     policy_decision_ros=100.,actual_rear_body_exit_clearance_m=.7),
                dict(event='signal_set_verified',light_id='L',color=16,requested_monotonic=10.3)]
        self.assertTrue(red_exit_validation(self.ros,self.oracle,events,self.binding)['verified'])
        events[-1]['requested_monotonic']=10.15
        self.assertFalse(red_exit_validation(self.ros,self.oracle,events,self.binding)['verified'])
