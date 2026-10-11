import unittest
from data_collection.capture_collection import (
    allocation, footprint_clear, kph_to_mps, native_annotation_class, RecoveryMonitor, motion_summary,
    observed_components, collection_trial_passed, StableRestoration)


class CollectionTest(unittest.TestCase):
    def test_restore_requires_continuous_stop_and_correct_mode(self):
        gate=StableRestoration()
        self.assertFalse(gate.update(0,True,True,True))
        self.assertFalse(gate.update(.4,False,True,True))
        self.assertFalse(gate.update(.5,True,True,True))
        self.assertFalse(gate.update(.9,True,True,True))
        self.assertTrue(gate.update(1.01,True,True,True))
        self.assertFalse(gate.update(1.1,True,False,True))

    def test_motion_success_does_not_override_failed_stop(self):
        validation=dict(tracking_passed=True,speed_achieved=True,forward_progress_m=20,
                        exit_reason='route_complete')
        summary=dict(failed=0,succeeded=30)
        restored=dict(stopped=True,gear_matches=True,mode_matches=True)
        self.assertTrue(collection_trial_passed(validation,summary,30,restored,0))
        self.assertFalse(collection_trial_passed(validation,summary,30,{},0))
        self.assertFalse(collection_trial_passed(validation,summary,30,dict(restored,stopped=False),0))
        self.assertFalse(collection_trial_passed(validation,summary,30,restored,1))

    def test_components_require_observation_and_matching_source(self):
        row=dict(native_ids=['7'], api_name='Stroller', target_class='stroller',
                 placement_approved=True, identity_approved=True)
        schema={'Stroller': {'components': {
            '0': dict(gt_source_label='Pedestrian', canonical='person'),
            '1': dict(gt_source_label='Obstacle', canonical='stroller'),
            '2': dict(gt_source_label='Pedestrian', canonical='person')}}}
        result=observed_components([row], {'7': {'Pedestrian'}, '70001': {'Obstacle'}}, schema)
        self.assertEqual(result[0]['native_ids'], ['7', '70001'])
        self.assertEqual(native_annotation_class('7', result)[0], 'person')
        self.assertEqual(native_annotation_class('70001', result)[0], 'stroller')
        self.assertEqual(row['native_ids'], ['7'])
        unresolved=observed_components([row], {'70001': {'Vehicle'}}, schema)
        with self.assertRaises(ValueError):native_annotation_class('70001',unresolved,'Vehicle',[1,2,3,255])
        other=dict(row, native_ids=['70001'])
        with self.assertRaises(ValueError):
            observed_components([row, other], {'70001': {'Obstacle'}}, schema)

    def test_reused_numeric_gt_id_requires_source_kind_and_color(self):
        row=dict(native_ids=['3'],api_name='Steel',target_class='steel_barricade',
                 placement_approved=True,identity_approved=True)
        schema={'Steel':{'components':{'0':dict(gt_source_label='Obstacle',canonical='steel_barricade')}}}
        rows=observed_components([row],{'3':{'Obstacle','Pedestrian'}},schema,
            {'3':{'Obstacle':{(0,0,82,255)},'Pedestrian':{(0,246,0,255)}}})
        self.assertEqual(native_annotation_class('3',rows,'Obstacle',[0,0,82,255])[0],'steel_barricade')
        with self.assertRaises(ValueError):native_annotation_class('3',rows,'Pedestrian',[0,246,0,255])
        with self.assertRaises(ValueError):native_annotation_class('3',rows,'Obstacle',[0,0,90,255])
        ambiguous=observed_components([row],{'3':{'Obstacle'}},schema,
            {'3':{'Obstacle':{(0,0,82,255),(0,0,90,255)}}})
        with self.assertRaises(ValueError):native_annotation_class('3',ambiguous,'Obstacle',[0,0,82,255])
        reviewed=observed_components([row],{'3':{'Obstacle'}},schema,
            {'3':{'Obstacle':{(0,0,82,255),(0,0,90,255)}}}, {'3':{'Obstacle':(0,0,82,255)}})
        self.assertEqual(native_annotation_class('3',reviewed,'Obstacle',[0,0,82,255])[0],'steel_barricade')
        with self.assertRaises(ValueError):native_annotation_class('3',reviewed,'Obstacle',[0,0,90,255])

    def test_units(self):
        self.assertAlmostEqual(kph_to_mps(20),5.5555555556)
        self.assertAlmostEqual(kph_to_mps(30),8.3333333333)
        for v in (31,float('nan'),-1):
            with self.assertRaises(ValueError):kph_to_mps(v)

    def test_rollback_and_transient_speed_cannot_pass(self):
        rows=[dict(monotonic=i*.05,capture_active=False,speed_mps=20/3.6,
                   station_m=i*.3) for i in range(100)]
        self.assertTrue(motion_summary(rows,20)['speed_achieved'])
        for r in rows:r['station_m']=0
        self.assertFalse(motion_summary(rows,20)['speed_achieved'])
        for i,r in enumerate(rows):r.update(station_m=i*.3,speed_mps=20/3.6 if i==50 else 0)
        self.assertFalse(motion_summary(rows,20)['speed_achieved'])

    def test_allocation_over_twenty_and_balance(self):
        rows=allocation(120)
        self.assertEqual(rows,allocation(120))
        self.assertEqual(len(rows),120)
        self.assertEqual(sum(c in ('deer','motorcycle','bicycle') for c,a in rows),20)
        self.assertGreater(sum(c=='plastic_barrier' for c,a in rows),sum(c=='bollard' for c,a in rows))
        self.assertTrue(all(c not in ('car','dog','person') for c,a in rows))

    def test_new_layout_uses_explicit_updated_presence_and_asset_pool(self):
        from data_collection.capture_collection import PRESENCE,ASSETS
        presence=dict(PRESENCE,stroller=1200)
        pools={k:tuple(a for a in v if a!='RedBarrel') for k,v in ASSETS.items()}
        rows=allocation(120,presence,pools,1)
        self.assertLess(sum(c=='stroller' for c,a in rows),sum(c=='stroller' for c,a in allocation(120)))
        self.assertNotIn('RedBarrel',[a for c,a in rows])
        self.assertEqual(rows,allocation(120,presence,pools,1))
        with self.assertRaises(ValueError):allocation(120,{'box':1},pools)

    def test_full_footprint_and_other_route_segment(self):
        route=[(0,0,0),(100,0,0),(100,10,0),(0,10,0)]
        self.assertFalse(footprint_clear(route,(50,2),1,[]))
        self.assertFalse(footprint_clear(route,(50,8),1,[]))
        self.assertTrue(footprint_clear(route,(50,-5),1,[]))
        self.assertFalse(footprint_clear(route,(50,-5),1,[(50,-6,1)]))

    def test_recovery_requires_post_capture_state_and_path(self):
        m=RecoveryMonitor();m.capture_completed(10)
        self.assertEqual(m.check(10.1,9.9,True,2,10.05,True),'waiting')
        self.assertEqual(m.check(10.2,10.1,True,2,10.1,True),'ready')
        m.capture_completed(20)
        self.assertEqual(m.check(22,21.99,True,0,21.99,True),'fallback')
        p=RecoveryMonitor('pp');p.capture_completed(20)
        self.assertEqual(p.check(22,21.99,True,0),'stop')

    def test_native_identity_and_composite_not_guessed(self):
        row=dict(native_ids=['100','1000001'],target_class='stroller',api_name='BabyStroller1',
                 identity_approved=True,placement_approved=True)
        self.assertEqual(native_annotation_class('1000001',[row]),('stroller','BabyStroller1'))
        row['native_class_overrides']={'100':'person'}
        self.assertEqual(native_annotation_class('100',[row]),('person','BabyStroller1'))
        with self.assertRaises(ValueError):native_annotation_class('1000002',[row])
        with self.assertRaises(ValueError):native_annotation_class('100',[row,row])


if __name__=='__main__':unittest.main()
