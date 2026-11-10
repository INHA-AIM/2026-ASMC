import unittest
from localization_assessment import assess,native_static_clearance,native_fixture_polygon,native_ego_polygon,confirm_static_return,completed_gps_recovery,maximum_stopped_interval,shadow_timeline_coverage,phase_speed_metrics


class AssessmentTests(unittest.TestCase):
    def test_moving_actor_pairing_does_not_treat_missing_frames_as_clear(self):
        from localization_assessment import moving_fixture_evidence
        def actor(x):
            return dict(id='person',pose=dict(x=x,y=0.,yaw=0.,pitch=0.,roll=0.),
                        box_center=[x,0.,0.],box_extents=[.2,.2,.8],speed_kmh=4.3)
        native=[dict(ros_time=t,ego=dict(pose=dict(x=0.,y=0.,yaw=0.))) for t in (0.,1.,2.)]
        frames=[dict(ros_time=t,actors=[actor(x)]) for t,x in ((0.,10.),(1.,0.))]
        result=moving_fixture_evidence(native,frames,lambda x,y,h:h)
        self.assertAlmostEqual(result['fixture_pairing_ratio'],2/3)
        self.assertEqual(result['native_obb_clearance_min_m'],0.)
        self.assertEqual(result['fixture_max_displacement_m']['person'],10.)
        empty=moving_fixture_evidence(native,[],lambda x,y,h:h)
        self.assertEqual(empty['fixture_pairing_ratio'],0.)
        self.assertIsNone(empty['native_obb_clearance_min_m'])

    def test_mixed_normal_speed_cannot_hide_shadow_overspeed_or_missing_mode(self):
        from localization_trial_policy import native_speed_limit
        self.assertEqual(native_speed_limit(0,15,30),31)
        self.assertEqual(native_speed_limit(1,15,30),16)
        self.assertIsNone(native_speed_limit(None,15,30))
        native=[dict(ros_time=t,monotonic=t,ego=dict(speed_kmh=v)) for t,v in [(0,30),(6,15),(7,17),(8,30)]]
        modes=[dict(ros_time=t,value=v) for t,v in [(0,0),(5,1),(8,0)]]
        a=phase_speed_metrics(native,modes,15,30)
        self.assertEqual(a['speed_violation_samples'],1)
        self.assertEqual(a['shadow_speed_max_kph'],17)
        self.assertEqual(phase_speed_metrics(native,[],15,30)['speed_unknown_mode_samples'],4)
    def test_change_only_mode_covers_the_entire_shadow_interval(self):
        native=[dict(ros_time=i,ego=dict(speed_kmh=15)) for i in range(11)]
        modes=[dict(ros_time=t,value=v) for t,v in [(0,0),(1,1),(10,0)]]
        decisions=[dict(ros_time=i-.05,value=dict(valid=i!=5)) for i in range(1,11)]
        a=shadow_timeline_coverage(native,modes,decisions)
        self.assertEqual(a['shadow_native_samples'],9)
        self.assertEqual(a['shadow_paired_samples'],9)
        self.assertEqual(a['invalid_or_missing_shadow_samples'],1)
        self.assertLess(a['shadow_valid_moving_ratio'],.95)
    def test_future_trace_and_stopped_missing_interval_cannot_hide_a_gap(self):
        native=[dict(ros_time=5,ego=dict(speed_kmh=0))]
        modes=[dict(ros_time=1,value=1)]
        decisions=[dict(ros_time=5.01,value=dict(valid=True))]
        a=shadow_timeline_coverage(native,modes,decisions)
        self.assertEqual(a['shadow_pairing_ratio'],0)
        self.assertEqual(a['invalid_or_missing_shadow_samples'],1)
        self.assertEqual(shadow_timeline_coverage(native,[],decisions)['mode_coverage_ratio'],0)
    def test_complete_short_stop_cannot_hide_in_speed_percentiles(self):
        rows=self.fixture()[0]
        self.assertEqual(maximum_stopped_interval(rows),0)
        for r in rows[60:64]:r['ego']['speed_kmh']=0
        self.assertAlmostEqual(maximum_stopped_interval(rows),.4)
        for r in rows[:10]:r['ego']['speed_kmh']=0
        self.assertAlmostEqual(maximum_stopped_interval(rows),.4)

    def test_recovery_rejects_stop_epoch_and_missing_timeline(self):
        rows=self.fixture()[1]
        self.assertIsNotNone(completed_gps_recovery(rows))
        for kind in ('stop','epoch','gap','short'):
            a=[dict(r) for r in rows]
            if kind=='stop':a[59]['source']='unavailable'
            if kind=='epoch':
                for r in a[59:]:r['motion_epoch']=2
            if kind=='gap':del a[54:59]
            if kind=='short':
                for r in a:r['stamp']=100+(r['stamp']-100)*.5
            self.assertIsNone(completed_gps_recovery(a),kind)

    def test_tracking_verification_does_not_hide_a_later_true_loss_recovery(self):
        a=self.fixture()[1]
        for r in a[5:8]:r['source']='recovery_verify'
        for r in a[8:10]:r['source']='recovery_blend'
        self.assertIsNotNone(completed_gps_recovery(a))
        sources=(['gps_anchored']*5+['dead_reckoning']*10+['recovery_verify']*5+
                 ['dead_reckoning']*2+['recovery_verify']*10+['recovery_blend']*10+['gps_anchored']*5)
        rows=[dict(stamp=10+i*.1,source=s) for i,s in enumerate(sources)]
        self.assertIsNotNone(completed_gps_recovery(rows))

    def test_return_requires_passing_all_fixtures_and_body_before_exit(self):
        fixture=[dict(box_center=[20.,0.,0.])]
        native=[dict(monotonic=i*.1,ego=dict(pose=dict(x=31+i*.1,y=0))) for i in range(10)]
        refs=[dict(projected=1,lateral_m=0,heading_deg=0) for _ in native]
        self.assertIsNotNone(confirm_static_return(native,refs,fixture,[50,0],[1,0]))
        self.assertIsNone(confirm_static_return(native,refs,[dict(box_center=[40.,0.,0.])],[50,0],[1,0]))
        self.assertIsNone(confirm_static_return(native,refs,fixture,[35,0],[1,0]))
    def test_projected_native_obb_does_not_treat_sphere_overlap_as_contact(self):
        actor=dict(box_center=[2.,2.1,1.],box_extents=[.6,.6,.6],
                   pose=dict(roll=0.,pitch=0.))
        ego=native_ego_polygon(0.,0.,0.);box=native_fixture_polygon(actor,0.)
        self.assertLess(native_static_clearance(0.,0.,0.,actor),0.)
        self.assertGreater(ego.distance(box),.4)
        actor['box_center']=[2.,0.,1.]
        self.assertEqual(ego.distance(native_fixture_polygon(actor,0.)),0.)

    def test_static_clearance_uses_native_physical_body_and_circumscribed_fixture(self):
        actor=dict(box_center=[2.,0.,1.],box_extents=[.5,.5,.5])
        self.assertLess(native_static_clearance(0,0,0,actor),0)
        self.assertGreater(native_static_clearance(0,3,0,actor),1.)
        rotated=dict(box_center=[0.,2.,1.],box_extents=[.5,.5,.5])
        import math
        self.assertAlmostEqual(native_static_clearance(0,3,0,actor),
                               native_static_clearance(-3,0,math.pi/2,rotated))

    def fixture(self):
        events=[dict(ros_time=100+i*.1,monotonic=10+i*.1,
                     ego=dict(speed_kmh=10,pose=dict(x=i,y=0))) for i in range(80)]
        decisions=[dict(stamp=e['ros_time'],valid=True,map_x=i,map_y=0,processing_ms=2,
                        source=('gps_anchored' if i<10 or i>=60 else 'dead_reckoning' if i<40 else
                                'recovery_verify' if i<50 else 'recovery_blend'),recovery_state='ok')
                   for i,e in enumerate(events)]
        reference=[dict(projected=1,lateral_m=0,heading_deg=0,map_inside=1) for e in events]
        receipts=[dict(monotonic=10+i*.05) for i in range(159)]
        result=dict(cap_kph=10,reason='distance_limit',native_contacts=[],cleanup_errors=[])
        recording=dict(counts_match=True,no_topic_recorded_by_both_groups=True,no_active_bag_files=True)
        return [events,decisions,reference,receipts,result,recording]

    def test_complete_run(self):self.assertTrue(assess(*self.fixture())['stage_passed'])
    def test_invalid_plans_do_not_hide_native_error(self):
        a=self.fixture();a[1][30]['valid']=False;a[2][30]['lateral_m']=.8
        r=assess(*a);self.assertFalse(r['gates']['lateral_max']);self.assertEqual(r['metrics']['invalid_or_missing_native_samples'],1)
    def test_missing_periods_are_failures(self):
        a=self.fixture();del a[1][20:50];r=assess(*a)
        self.assertFalse(r['gates']['pairing']);self.assertFalse(r['gates']['valid_moving'])
    def test_speed_and_recording_cannot_be_waived(self):
        a=self.fixture();a[0][70]['ego']['speed_kmh']=11.1;a[5]['counts_match']=False
        r=assess(*a);self.assertFalse(r['gates']['speed_overshoot']);self.assertFalse(r['gates']['recording'])
    def test_absent_recovery_is_not_pass(self):
        a=self.fixture()
        for d in a[1]:d['source']='dead_reckoning'
        self.assertFalse(assess(*a)['gates']['gps_transition'])
    def test_missing_reference_and_stopped_samples_count(self):
        a=self.fixture();a[0][20]['ego']['speed_kmh']=0;a[2].pop()
        r=assess(*a);self.assertFalse(r['gates']['full_native_reference']);self.assertEqual(r['metrics']['stopped_native_samples'],1)

if __name__=='__main__':unittest.main()
