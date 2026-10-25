import sys
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from types import SimpleNamespace as NS

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.morai.clock_contract import calibration_from_samples,accepted_rate,calibrate_local_windows
from scen_runner.runtime.signal_control import SignalControl
from scen_runner.runtime.signal_trial_fixture import StaticFixture


class ClockContractTest(unittest.TestCase):
    def test_source_phase_change_discards_whole_epoch_without_widening_bounds(self):
        error=ValueError('clock bounds do not intersect; source epoch changed')
        error.calibration_samples=[dict(lower_s=0,upper_s=.0001),dict(lower_s=.0015,upper_s=.0016)]
        accepted=dict(source_to_ros_offset_s=.00155,source_clock_uncertainty_s=.00105)
        with patch('scen_runner.morai.clock_contract._calibrate_local_windows_once',side_effect=[error,dict(accepted)]) as probe:
            result=calibrate_local_windows()
        self.assertEqual(probe.call_count,2)
        self.assertEqual(result['source_to_ros_offset_s'],accepted['source_to_ros_offset_s'])
        self.assertEqual(result['source_clock_uncertainty_s'],accepted['source_clock_uncertainty_s'])
        self.assertEqual(result['rejected_calibration_epochs'][0]['samples'],error.calibration_samples)

    def test_repeated_source_epoch_changes_fail_after_three_complete_batches(self):
        with patch('scen_runner.morai.clock_contract._calibrate_local_windows_once',side_effect=ValueError('clock bounds do not intersect')) as probe:
            with self.assertRaisesRegex(ValueError,'do not intersect'):calibrate_local_windows()
        self.assertEqual(probe.call_count,3)

    def test_combined_tick_and_frequency_allow_bounded_normal_chrony_correction(self):
        self.assertAlmostEqual(accepted_rate(dict(tick_us=10000,frequency_ppm=99.3))['total_rate_ppm'],99.3)
        self.assertAlmostEqual(accepted_rate(dict(tick_us=10001,frequency_ppm=-30.))['total_rate_ppm'],70.)
        for tick,frequency in ((10460,-36.233),(10000,251.),(10000,float('nan'))):
            with self.assertRaisesRegex(RuntimeError,'environment changed'):
                accepted_rate(dict(tick_us=tick,frequency_ppm=frequency))

    def test_intersecting_bounds_and_clock_step_rejection(self):
        rows=[dict(lower_s=-.002,upper_s=.004) for _ in range(8)]
        result=calibration_from_samples(rows)
        self.assertAlmostEqual(result['source_to_ros_offset_s'],.001)
        self.assertAlmostEqual(result['source_clock_uncertainty_s'],.004)
        rows[-1]=dict(lower_s=.005,upper_s=.006)
        with self.assertRaisesRegex(ValueError,'do not intersect'):calibration_from_samples(rows)

    def test_bounded_rate_projects_reads_to_one_instant_but_rejects_a_step(self):
        rows=[dict(lower_s=i*.0002-.00001,upper_s=i*.0002+.00001,monotonic_s=float(i)) for i in range(8)]
        with self.assertRaisesRegex(ValueError,'do not intersect'):calibration_from_samples(rows)
        result=calibration_from_samples(rows,250.)
        self.assertAlmostEqual(result['source_to_ros_offset_s'],.0014)
        rows[-1]['lower_s']+=.01;rows[-1]['upper_s']+=.01
        with self.assertRaisesRegex(ValueError,'do not intersect'):calibration_from_samples(rows,250.)


class SignalOwnerTest(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.phase=2;self.color=1;self.events=[];self.link=Mock()
        self.link.intersection_state.side_effect=lambda _:dict(phase=self.phase,colors={'L':self.color})
        self.link.set_intersection_phase.side_effect=self.set_phase
        self.link.set_traffic_light_color.side_effect=self.set_color
        self.link.traffic_light_color.side_effect=lambda _:self.color
        self.owner=SignalControl(self.link,self.events.append,lambda:self.now)
        self.owner.begin('L','J',0,16,30)

    def set_phase(self,_,phase):self.phase=phase
    def set_color(self,_,color,impulse=False):self.color=color

    def test_forced_yellow_is_not_overwritten_by_green_heartbeat(self):
        self.owner.set_aspect('L',4);self.link.set_traffic_light_color.reset_mock()
        self.now=2.;self.owner.tick()
        self.link.set_traffic_light_color.assert_not_called()
        self.assertEqual(self.color,4)
        self.owner.green('L');self.assertEqual(self.color,16)

    def test_readback_mismatch_and_expired_phase_fail_trial(self):
        self.owner.set_aspect('L',1);self.color=16
        with self.assertRaisesRegex(RuntimeError,'overwritten'):self.owner.tick()
        self.color=1;self.now=29.6
        with self.assertRaisesRegex(RuntimeError,'outlive'):self.owner.tick()

    def test_snapshot_precedes_writes_and_failed_restore_is_retained(self):
        self.assertEqual(self.events[0]['event'],'signal_phase_snapshot')
        self.assertEqual(self.events[0]['phase'],2)
        self.link.set_intersection_phase.side_effect=RuntimeError('offline')
        with self.assertRaisesRegex(RuntimeError,'restoration incomplete'):self.owner.restore()
        self.assertIn('J',self.owner.original)
        self.link.set_intersection_phase.side_effect=self.set_phase
        self.owner.restore();self.assertEqual(self.phase,2)
        self.assertFalse(self.owner.original)

    def test_tick_bounds_rpc_work_but_checks_all_phase_deadlines(self):
        self.owner.active['B']=dict(self.owner.active['L'])
        self.now=2.;self.link.set_traffic_light_color.reset_mock()
        self.owner.tick()
        self.assertEqual(self.link.set_traffic_light_color.call_count,1)
        self.assertEqual(self.owner.active['B']['phase_at'],0.)
        self.owner.tick()
        self.assertEqual(self.link.set_traffic_light_color.call_count,2)
        self.owner.active['B']['phase_at']=-28.;self.owner.tick_cursor=0
        with self.assertRaisesRegex(RuntimeError,'outlive'):self.owner.tick()


class FixtureTest(unittest.TestCase):
    def test_snapshot_before_write_and_failed_restore_remains_retryable(self):
        events=[];actor=NS(actor_info=NS(id=NS(value='2'),object_type=3),
            velocity=NS(x=0.,y=0.,z=0.),transform=NS(location=NS(x=10.,y=20.,z=1.),
                rotation=NS(x=0.,y=0.,z=90.)))
        link=Mock();link.states.return_value=[actor]
        def move(aid,kind,state):
            self.assertEqual(events[0]['event'],'static_fixture_snapshot')
            for k in ('x','y','z'):setattr(actor.transform.location,k,state['pose'][k])
            actor.transform.rotation.z=state['pose']['yaw']+1.35
        link.restore_actor.side_effect=move
        fixture=StaticFixture(link,events.append);fixture.park('2',30.,0.)
        self.assertEqual(actor.transform.location.x,40.)
        link.restore_actor.side_effect=RuntimeError('offline')
        with self.assertRaisesRegex(RuntimeError,'incomplete'):fixture.restore()
        self.assertIn('2',fixture.original)
        link.restore_actor.side_effect=move;fixture.restore()
        self.assertEqual(actor.transform.location.x,10.)
        self.assertFalse(fixture.original)

    def test_never_moves_an_unidentified_vehicle_or_pedestrian(self):
        link=Mock();link.states.return_value=[NS(actor_info=NS(id=NS(value='2'),object_type=1))]
        with self.assertRaisesRegex(RuntimeError,'static prop'):StaticFixture(link,Mock()).park('2',30.,0.)
        link.restore_actor.assert_not_called()


if __name__=='__main__':unittest.main()
