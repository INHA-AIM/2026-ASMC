import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.signal_trial_oracle import Oracle,red_recovery_exit_proof


def actor(x,y=0,speed=1):
    vec=lambda **kwargs:S(**kwargs)
    return S(transform=S(location=vec(x=x,y=y),rotation=vec(z=0)),
        bounding_box=S(location=vec(x=x+1.5,y=y),rotation=vec(z=0),extent=vec(x=1,y=2.3)),
        velocity=vec(x=speed*3.6,y=0,z=0),vehicle_state=S(collision_objects=['Ego']))


class OracleTest(unittest.TestCase):
    def make(self):
        return Oracle({'drivable_areas':[{'polygon':[[-10,-2],[50,-2],[50,2],[-10,2]]}]},
            [{'traffic_light_id':'L','approach_direction':[1,0],'line_point':[20,0],'green_color':16}])

    def test_green_crossing_and_native_velocity_units(self):
        oracle=self.make();oracle.observe(actor(16),1,{'L':16})
        row=oracle.observe(actor(18),2,{'L':16})
        self.assertAlmostEqual(row['speed'],1)
        self.assertTrue(oracle.result()['violation_free']);self.assertEqual(len(oracle.crossings),1)

    def test_color_transition_crossing_is_uncertain_not_normal(self):
        oracle=self.make();oracle.observe(actor(16),1,{'L':16});oracle.observe(actor(18),2,{'L':4})
        self.assertFalse(oracle.result()['violation_free'])
        self.assertEqual(oracle.crossings[0]['classification'],'uncertain')

    def test_finite_stop_line_rejects_distant_plane_crossing_on_full_lap(self):
        oracle=self.make()
        oracle.bindings[0]['source_stopline_points']=[[20,-1.75],[20,1.75]]
        oracle.observe(actor(16,15),1,{'L':1})
        oracle.observe(actor(18,15),2,{'L':1})
        self.assertEqual(oracle.crossings,[])
        self.assertEqual(oracle.violations,[])

    def test_finite_stop_line_keeps_real_red_wheel_crossing(self):
        oracle=self.make()
        oracle.bindings[0]['source_stopline_points']=[[20,-1.75],[20,1.75]]
        oracle.observe(actor(16),1,{'L':1})
        oracle.observe(actor(18),2,{'L':1})
        self.assertEqual(len(oracle.violations),1)
        self.assertEqual(oracle.violations[0]['classification'],'red_or_yellow')

    def test_road_departure_fails_even_without_collision_or_signal_violation(self):
        oracle=self.make();row=oracle.observe(actor(5,1.02),1,{'L':16})
        self.assertFalse(row['road_ok']);self.assertFalse(oracle.result()['violation_free'])
        self.assertGreater(row['outside_area_m2'],0)

    def test_pavement_holes_and_swept_body_are_not_filled_in(self):
        road={'drivable_areas':[{'polygon':[[-10,-3],[30,-3],[30,3],[-10,3]],
            'holes':[[[6,-1],[7,-1],[7,1],[6,1]]]}]}
        oracle=Oracle(road,[])
        self.assertTrue(oracle.observe(actor(0),1,{})['road_ok'])
        self.assertFalse(oracle.observe(actor(10),2,{})['road_ok'])
        self.assertFalse(oracle.result()['violation_free'])

    def test_roll_projects_upper_body_outside_road_even_if_flat_box_fits(self):
        oracle=self.make();vehicle=actor(0,.9)
        vehicle.bounding_box.extent.z=.82;vehicle.bounding_box.rotation.x=15
        self.assertFalse(oracle.observe(vehicle,1,{'L':16})['road_ok'])

    def test_long_observation_gap_is_unknown_not_a_fabricated_safe_drive(self):
        oracle=self.make();oracle.maximum_gap=.3
        oracle.observe(actor(0),1,{'L':16});row=oracle.observe(actor(0),1.31,{'L':16})
        self.assertTrue(row['road_ok']);self.assertFalse(row['observation_complete'])
        self.assertFalse(oracle.result()['violation_free'])


class RecoveryPhaseScopeTest(unittest.TestCase):
    def setUp(self):
        self.binding=dict(traffic_light_id='L',stop_node_id='S',exit_point=[30,0],exit_direction=[1,0])
        self.actual=dict(aspects={'L':1},road_ok=True,collisions=[],observation_complete=True,
            bounding_box=dict(center=[33,0,0],half_length=2.3,half_width=1.1,half_height=.8,roll=0.,pitch=0.))
        self.policy=dict(motion='EXITED',actual_control_allowed=True,stop_line='S',severity=0,
            causes={c:0 for c in ('SIGNAL','COLLISION','PEDESTRIAN','INPUT_INVALID','EXECUTION_FAILURE')},
            route_generation=2,decision_ros=100.)

    def test_full_actual_body_and_half_metre_exit_margin_are_required(self):
        proof=red_recovery_exit_proof(self.binding,self.actual,self.policy)
        self.assertAlmostEqual(proof['actual_rear_body_exit_clearance_m'],.7)
        self.actual['bounding_box']['center'][0]=32.79
        self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))

    def test_policy_exit_cannot_substitute_for_actual_clearance_or_failed_evidence(self):
        for field,value in [('road_ok',False),('collisions',['other']),('observation_complete',False)]:
            original=self.actual[field];self.actual[field]=value
            self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))
            self.actual[field]=original
        self.actual['aspects']['L']=16
        self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))

    def test_wrong_episode_and_ongoing_or_invalid_policy_cannot_release_test_phase(self):
        for field,value in [('motion','RECOVERY_CLEAR'),('stop_line','OTHER'),('actual_control_allowed',False)]:
            original=self.policy[field];self.policy[field]=value
            self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))
            self.policy[field]=original
        self.policy['causes']['INPUT_INVALID']=2
        self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))
        self.policy['causes']={}
        self.assertIsNone(red_recovery_exit_proof(self.binding,self.actual,self.policy))


if __name__=='__main__':unittest.main()
