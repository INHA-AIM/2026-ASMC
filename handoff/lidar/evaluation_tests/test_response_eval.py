#!/usr/bin/env python3
import unittest
import numpy as np
from response_eval import estimate,interpolate,timestamp_quality,trajectory_groups
import pandas as pd

class ResponseTests(unittest.TestCase):
    def setUp(self):
        self.t=np.r_[0,np.cumsum(np.tile([.08,.13,.10,.16],30))]
        speed=2+np.sin(self.t*1.7)+.3*self.t
        self.truth=np.column_stack([speed,np.zeros(len(speed))])
        self.q=np.arange(1,7,.05);self.mask=np.ones(len(self.q),dtype=bool)
    def test_real_time_delay_recovered_under_irregular_sampling(self):
        prediction=interpolate(self.t,self.truth,self.q-.2)
        r=estimate(prediction,self.truth,self.t,self.q,self.mask,'speed')
        self.assertAlmostEqual(r['seconds'],.2,places=7)
        self.assertFalse(r['boundary'])
    def test_flat_signal_is_not_reported_as_certain(self):
        truth=np.tile([3.,0.],(len(self.t),1));p=np.tile([3.,0.],(len(self.q),1))
        self.assertTrue(estimate(p,truth,self.t,self.q,self.mask,'speed')['ambiguous'])
    def test_large_gap_is_not_silently_interpolated(self):
        t=np.array([0.,.1,1.,1.1]);y=np.column_stack([t,t])
        self.assertTrue(np.isnan(interpolate(t,y,np.array([.5]))).all())
    def test_search_boundary_is_explicit(self):
        prediction=interpolate(self.t,self.truth,self.q-.8)
        self.assertTrue(estimate(prediction,self.truth,self.t,self.q,self.mask,'vector')['boundary'])

    def test_exact_sample_after_gap_and_singleton(self):
        t=np.array([0.,.1,1.,1.1]);y=np.column_stack([t,t])
        np.testing.assert_allclose(interpolate(t,y,np.array([1.,1.1,1.+5e-9])),[[1,1],[1.1,1.1],[1,1]])
        np.testing.assert_allclose(interpolate(np.array([1.]),np.array([[2.,3.]]),np.array([1.])),[[2,3]])
    def test_boundaries_and_tolerance(self):
        t=np.array([0.,.3]);y=np.column_stack([t,t])
        self.assertTrue(np.isfinite(interpolate(t,y,np.array([-5e-9,.15,.3+5e-9]))).all())
        self.assertTrue(np.isnan(interpolate(t,y,np.array([-2e-8,.3+2e-8]))).all())
    def test_identity_and_generation_boundaries(self):
        t=np.array([0.,.1,.2,.3]);y=np.column_stack([t,t])
        z=interpolate(t,y,np.array([.05,.15,.2,.25]),groups=[0,0,1,1])
        self.assertTrue(np.isnan(z[1]).all());self.assertTrue(np.isfinite(z[[0,2,3]]).all())
        rows=pd.DataFrame({'pid':[1,1,2,2,2],'age':[1,2,1,2,1]})
        self.assertEqual(trajectory_groups(rows).tolist(),[1,1,2,2,3])
    def test_duplicate_and_reverse_time_are_reported(self):
        t=np.array([0.,.1,.1,.05,.2]);y=np.column_stack([t,t])
        self.assertEqual(timestamp_quality(t)['duplicate_edges'],1)
        self.assertEqual(timestamp_quality(t)['nonmonotonic_edges'],1)
        self.assertTrue(np.isnan(interpolate(t,y,np.array([.1,.075]))).all())
    def test_fixed_shift_support(self):
        from response_eval import DELAY
        truth=np.column_stack([self.t,self.t])
        support=np.ones(len(self.q),dtype=bool)
        for d in DELAY:support &= np.isfinite(interpolate(self.t,truth,self.q-d)).all(axis=1)
        counts=[int(np.isfinite(interpolate(self.t,truth,self.q-d)[support]).all(axis=1).sum()) for d in DELAY]
        self.assertEqual(len(set(counts)),1)

    def test_evaluate_freezes_gt_events_queries_and_joint_support(self):
        import tempfile
        from pathlib import Path
        from response_eval import evaluate
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);scene=root/'inputs/s';scene.mkdir(parents=True)
            idx=np.arange(60);stamp=1000000000+idx*100000000
            pd.DataFrame({'idx':idx,'stamp_ns':stamp,'ego_yaw':0.}).to_csv(scene/'frames.csv',index=False)
            pd.DataFrame({'kind':'gt','class_id':0,'x':10.,'y':0.,'track_id':1,'idx':idx,
                          'stamp_ns':stamp,'vx':idx*.1,'vy':0.}).to_csv(scene/'gt.csv',index=False)
            arms={}
            for name in ['a','b']:
                path=root/name/'samples';path.mkdir(parents=True)
                rows=pd.DataFrame({'gt_tid':1,'idx':idx,'kfc':2,'pvx':idx*.1,'pvy':0.,
                                   'pid':np.where(idx<30,1,2),'age':idx+1,'obs_count':idx+1,'prev_miss':False})
                if name=='b':rows=rows[~rows.idx.between(35,42)]
                pd.to_pickle({'tp':rows},path/'s.pkl');arms[name]=path.parent
            d=evaluate(root/'inputs',arms,['s'])
            self.assertEqual(d['summary']['a']['gt_defined_events'],1)
            self.assertEqual(d['summary']['a']['query_common_n'],d['summary']['b']['query_common_n'])
            event=d['events'][0]
            self.assertEqual(len(event['query_relative_s']),event['query_n'])
            self.assertEqual(sum(event['common_query_mask']),event['common_query_n'])
            self.assertLess(event['common_query_n'],event['query_n'])
            self.assertEqual(event['arms']['a']['id_switches_in_event'],1)

    def test_nonadjacent_duplicates_and_ambiguous_tolerance(self):
        from response_eval import TIME_ATOL
        self.assertEqual(timestamp_quality([0.,.1,.05,.1])['duplicate_timestamps_n'],1)
        t=np.array([0.,1.5*TIME_ATOL]);y=np.column_stack([t,t])
        self.assertTrue(np.isnan(interpolate(t,y,np.array([.75*TIME_ATOL]))).all())

if __name__=='__main__':unittest.main()
