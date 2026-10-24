import unittest
import numpy as np
import pandas as pd
from compare import compare
class Tests(unittest.TestCase):
 def setUp(self):
  self.off=pd.DataFrame({'scene':['s']*3,'idx':[0,1,2],'seg':['1_0']*3,'gt_tid':[1]*3,'kfc':[2]*3,'pvx':[0,100,2],'pvy':[0]*3,'gvx':[0]*3,'gvy':[0]*3})
  self.on=self.off.iloc[[0,2]].copy();self.on.loc[self.on.idx==2,'pvx']=1
 def test_support_change(self):
  r,_=compare(self.off,self.on);self.assertEqual(r['all_valid_speed']['off']['mae'],34);self.assertEqual(r['common_valid_speed']['off']['mae'],1);self.assertEqual(r['common_valid_speed']['on']['mae'],.5);self.assertEqual(r['tp_n'],{'off':3,'on':2,'common':2})
 def test_duplicate(self):
  with self.assertRaises(ValueError):compare(self.off,pd.concat([self.on,self.on]))
 def test_gt_mismatch(self):
  with self.assertRaises(ValueError):compare(self.off,self.on.assign(gvx=1))
 def test_nan(self):
  on=self.on.copy();on.loc[on.idx==2,'pvx']=np.nan;r,_=compare(self.off,on);self.assertEqual(r['common_valid_speed']['off']['n'],1)
 def test_uninitialized(self):
  r,_=compare(self.off,self.on.assign(kfc=1));self.assertEqual(r['common_valid_speed']['off']['n'],0)
 def test_identical(self):
  r,_=compare(self.off,self.off);self.assertEqual(r['all_valid_speed']['off'],r['common_valid_speed']['on'])
if __name__=='__main__':unittest.main()
