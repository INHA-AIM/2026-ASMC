import unittest
from localization_trial_policy import ready,offset_start,ClearContactWindow

class DeparturePolicyTests(unittest.TestCase):
    def test_udp_self_tuple_requires_matching_time_position_and_empty_native_contacts(self):
        from localization_trial_policy import udp_contact_classification as check
        ego=dict(pose=dict(x=10.,y=20.,z=1.),native_contacts=['Ego'])
        item=dict(unique_id=0,type=-1,position=[10.,20.,1.])
        message=dict(stamp=1.,collision_object=[item],collision_objecta=[])
        self.assertEqual(check(message,ego,1.1),'position_matched_self_status')
        self.assertEqual(check(message,ego,1.3),'unresolved')
        self.assertEqual(check(message,dict(ego,native_contacts=['Ego','42']),1.1),'native_contact_present')
        for update in (dict(unique_id=-100),dict(unique_id=42),dict(position=[20.,20.,1.])):
            self.assertEqual(check(dict(message,collision_object=[dict(item,**update)]),ego,1.1),'unresolved')

    def test_inherited_unknown_contact_never_releases_departure(self):
        gate=ClearContactWindow()
        for t in (0,.5,1.,10.):self.assertFalse(gate.observe(t,['Ego','-100'],'Ego'))
        self.assertFalse(gate.observe(10.1,['Ego'],'Ego'))
        self.assertFalse(gate.observe(10.2,['Ego'],'Ego'))
        self.assertTrue(gate.observe(10.7,['Ego'],'Ego'))
    def test_contact_or_retrograde_query_restarts_clear_window(self):
        gate=ClearContactWindow()
        self.assertFalse(gate.observe(1.,['Ego'],'Ego'))
        self.assertFalse(gate.observe(1.4,['Ego','42'],'Ego'))
        self.assertFalse(gate.observe(1.5,[],'Ego'))
        self.assertFalse(gate.observe(1.6,[],'Ego'))
        self.assertTrue(gate.observe(2.1,[],'Ego'))
        self.assertFalse(gate.observe(2.0,[],'Ego'))
    def test_only_recent_ready_plan_and_anchor_release(self):
        trace=dict(anchor_ready=True,source='gps_anchored',recovery_state='gps_anchored',anchor_samples=10)
        self.assertTrue(ready(10,True,9.9,trace,9.9))
        self.assertFalse(ready(10,False,9.9,trace,9.9))
        self.assertFalse(ready(10,True,9.7,trace,9.9))
        self.assertFalse(ready(10,True,9.9,trace,9.7))
        for key,value in [('anchor_ready',False),('source','dead_reckoning'),('anchor_samples',9),('recovery_state','reanchor_stop')]:
            self.assertFalse(ready(10,True,9.9,dict(trace,**{key:value}),9.9))
    def test_offset_limits_and_copy(self):
        pose=dict(x=43.76026,y=-476.4096,yaw=-136.55374)
        shifted=offset_start(pose,.2,2)
        self.assertAlmostEqual(((shifted['x']-pose['x'])**2+(shifted['y']-pose['y'])**2)**.5,.2)
        self.assertAlmostEqual(shifted['yaw']-pose['yaw'],2)
        self.assertEqual(pose['x'],43.76026)
        for lateral,heading in [(.201,0),(0,2.01),(float('nan'),0)]:
            with self.assertRaises(ValueError):offset_start(pose,lateral,heading)

if __name__=='__main__':unittest.main()
