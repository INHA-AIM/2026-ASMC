import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.contact_evidence import inspect_contacts,ContactTransitions


def body(aid,x,extents=(1.,2.,1.),speed=0.,kind=1):
    return S(actor_info=S(id=S(value=aid),object_type=kind),
             bounding_box=S(location=S(x=x,y=0.,z=1.),extent=S(x=extents[0],y=extents[1],z=extents[2])),
             velocity=S(x=speed,y=0.,z=0.))


class ContactEvidenceTest(unittest.TestCase):
    def sample(self,*states,contacts=('1',)):
        return dict(contacts=contacts,owned_ids={'1'},actor_states=states,query_s=.01)

    def test_native_contact_to_far_static_is_rejected_with_measured_distance(self):
        sample=self.sample(body('Ego',0),body('1',258,(.2,.2,.4),kind=3))
        record=inspect_contacts(sample,{'1'})[0]
        self.assertEqual(record['verdict'],'rejected')
        self.assertEqual(record['kind'],3);self.assertEqual(record['separation_m'],258.)

    def test_near_body_without_native_contact_cannot_create_collision(self):
        self.assertEqual(inspect_contacts(self.sample(body('Ego',0),body('1',1),contacts=()),{'1'}),[])

    def test_native_contact_is_accepted_at_broad_trigger_distance_and_finite_speed(self):
        record=inspect_contacts(self.sample(body('Ego',0,speed=25),body('1',8.3)),{'1'})[0]
        self.assertEqual(record['verdict'],'near')

    def test_preexisting_far_id_approaching_and_even_overlapping_is_not_new_contact(self):
        tracker=ContactTransitions()
        for index,distance in enumerate((256.6,13.94,6.3,3.,0.)):
            sample=self.sample(body('Ego',0),body('1',distance));sample['query_started_at']=float(index)
            verdict=tracker.observe(sample,{'1'},'Ego',eligible=index>0)[0]['verdict']
            self.assertNotIn(verdict,('accepted','ongoing'))
        self.assertEqual(verdict,'stale')

    def test_same_id_rearms_only_after_native_list_release_then_new_enter(self):
        tracker=ContactTransitions()
        sample=self.sample(body('Ego',0),body('1',258));sample['query_started_at']=0.
        tracker.observe(sample,{'1'},'Ego',False)
        sample=self.sample(body('Ego',0),body('1',3),contacts=());sample['query_started_at']=1.
        self.assertFalse(tracker.observe(sample,{'1'},'Ego',True))
        sample['contacts']=('1',);sample['query_started_at']=2.
        self.assertEqual(tracker.observe(sample,{'1'},'Ego',True)[0]['verdict'],'accepted')

    def test_query_error_does_not_release_quarantine(self):
        tracker=ContactTransitions()
        sample=self.sample(body('Ego',0),body('1',258));sample['query_started_at']=0.
        tracker.observe(sample,{'1'},'Ego',False)
        tracker.observe(dict(error='timeout',contacts=(),query_started_at=1.),{'1'},'Ego',True)
        sample=self.sample(body('Ego',0),body('1',3));sample['query_started_at']=2.
        self.assertEqual(tracker.observe(sample,{'1'},'Ego',True)[0]['verdict'],'stale')

    def test_old_response_with_empty_list_does_not_rearm_stale_id(self):
        tracker=ContactTransitions()
        sample=self.sample(body('Ego',0),body('1',258));sample['query_started_at']=10.
        tracker.observe(sample,{'1'},'Ego',False)
        empty=dict(contacts=(),query_started_at=9.,owned_ids={'1'},actor_states=())
        tracker.observe(empty,{'1'},'Ego',True)
        sample=self.sample(body('Ego',0),body('1',3));sample['query_started_at']=11.
        self.assertEqual(tracker.observe(sample,{'1'},'Ego',True)[0]['verdict'],'stale')

    def test_missing_or_invalid_bounding_box_is_unknown(self):
        self.assertEqual(inspect_contacts(self.sample(body('Ego',0)),{'1'})[0]['verdict'],'unknown')
        self.assertEqual(inspect_contacts(self.sample(body('Ego',0),body('1',1,extents=(0,0,0))),{'1'})[0]['verdict'],'unknown')

    def test_foreign_contact_and_ownership_added_after_query_are_excluded(self):
        sample=self.sample(body('Ego',0),body('1',1),contacts=('2',))
        row=inspect_contacts(sample,{'1'})[0]
        self.assertEqual(row['verdict'],'ignored');self.assertTrue(row['unowned'])
        self.assertFalse(row['owned_now'])
        sample['contacts']=('1',);sample['owned_ids']=set()
        row=inspect_contacts(sample,{'1'})[0]
        self.assertEqual(row['verdict'],'ignored');self.assertFalse(row['owned_at_query'])
        self.assertTrue(row['owned_now'])

    def test_leftover_contact_is_explained_without_accepting_it_as_current_owned_collision(self):
        sample=self.sample(body('Ego',0),body('1',1),body('2',20),contacts=('1',))
        sample.update(owned_ids={'2'},query_started_at=10.)
        tracker=ContactTransitions();rows=tracker.observe(sample,{'2'},'Ego',True)
        self.assertEqual(rows[0]['verdict'],'ignored');self.assertTrue(rows[0]['unowned'])
        self.assertFalse(tracker.confirmed)
        sample.update(contacts=('2',),query_started_at=11.,actor_states=[body('Ego',0),body('2',1)])
        self.assertEqual(tracker.observe(sample,{'2'},'Ego',True)[0]['verdict'],'accepted')


if __name__=='__main__':unittest.main()
