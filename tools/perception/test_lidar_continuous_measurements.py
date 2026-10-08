import copy
import unittest
from tools.perception.lidar_continuous_measurements import compare,table,speed_events

class ContinuousMeasurementsTest(unittest.TestCase):
    def rows(self):return [dict(stamp_ns=i,ready=True,objects=[dict(id=7,center=[1.,2.,3.],confirmations=i)]) for i in (1,2,3)]
    def test_identical_state_and_end_to_end_history(self):self.assertTrue(compare(self.rows(),self.rows())['exact'])
    def test_missing_final_record_and_counter_corruption_fail(self):
        self.assertFalse(compare(self.rows(),self.rows()[:-1])['exact'])
        bad=copy.deepcopy(self.rows());bad[1]['objects'][0]['confirmations']=99
        self.assertFalse(compare(self.rows(),bad)['exact'])
    def test_wrong_time_and_duplicate_consumption_fail(self):
        for bad in ([self.rows()[0]]*2,[dict(self.rows()[0],stamp_ns=0)]):
            with self.assertRaises(ValueError):table(bad)
    def test_obb_corruption_and_nonfinite_kf_fail(self):
        bad=self.rows();bad[1]["objects"][0]["corners"]=[[1.,2.,3.]]
        self.assertFalse(compare(self.rows(),bad)["exact"])
        bad=self.rows();bad[0]["objects"][0]["kf_ca"]=[float("nan")]
        with self.assertRaises(ValueError):table(bad)
    def test_hidden_nonfinite_geometry_fail(self):
        bad=self.rows();bad[0]['objects'][0]['center'][0]=float('nan')
        with self.assertRaises(ValueError):table(bad)
    def test_capped_initialization_counter_is_not_a_cause(self):
        obj=dict(semantic=dict(id=1,center=[0.,0.,0.],lidar_misses=0),
            kf_ca=[0.,0.,0.,0.],kf_init_count=2,age=76)
        rows=[dict(event='tracking_state',stamp_ns=1,source_dt_sec=.1,objects=[obj])]
        later=copy.deepcopy(obj);later['kf_ca'][2]=3.;later['age']=77
        rows.append(dict(event='tracking_state',stamp_ns=2,source_dt_sec=.1,objects=[later]))
        self.assertEqual(speed_events(rows)[0]['cause'],'requires geometry/association/attitude review')
    def test_wrong_dt_is_rejected(self):
        row=dict(event='tracking_state',stamp_ns=200000000,previous_stamp_ns=100000000,
            source_dt_sec=.1,objects=[])
        self.assertEqual(speed_events([row]),[])
        for dt in (0.,-.1,float('nan'),.2):
            with self.assertRaises(ValueError):speed_events([dict(row,source_dt_sec=dt)])

if __name__=='__main__':unittest.main()
