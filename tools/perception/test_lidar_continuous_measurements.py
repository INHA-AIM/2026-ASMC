import copy
import unittest
from tools.perception.lidar_continuous_measurements import compare,table

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
    def test_hidden_nonfinite_geometry_fail(self):
        bad=self.rows();bad[0]['objects'][0]['center'][0]=float('nan')
        with self.assertRaises(ValueError):table(bad)

if __name__=='__main__':unittest.main()
