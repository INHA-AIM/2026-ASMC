import json
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'scripts'))
from dashboard_localization import LocalizationView,ExperimentView,map_trajectory_fresh

class PayloadTest(unittest.TestCase):
    def fixture(self):return dict(frame='map',stamp=10.,valid=True,selected=[[0,0],[1,0]],candidates=[dict(feasible=True,points=[[0,0],[1,1]])],obstacles=[])
    def test_current_geometry_expires_in_source_time(self):
        view=LocalizationView();view.put(json.dumps(self.fixture()))
        self.assertTrue(view.snapshot(10.1)['fresh'])
        self.assertFalse(view.snapshot(11)['fresh']);self.assertEqual(view.snapshot(11)['selected'],[])
        self.assertEqual(view.snapshot(9)['candidates'],[])
    def test_invalid_plan_never_retains_selected_path(self):
        view=LocalizationView();p=self.fixture();p['valid']=False;view.put(json.dumps(p))
        self.assertEqual(view.snapshot(10)['selected'],[])
    def test_bad_frame_nonfinite_and_unbounded_candidates_rejected(self):
        for kind in ['frame','nan','limit']:
            p=self.fixture()
            if kind=='frame':p['frame']='local/other'
            elif kind=='nan':p['selected']=[[float('nan'),0]]
            else:p['candidates']*=25
            view=LocalizationView();view.put(json.dumps(p));self.assertIsNone(view.snapshot(10),kind)
    def test_delayed_frame_cannot_replace_newer_geometry(self):
        view=LocalizationView();p=self.fixture();view.put(json.dumps(p));p['stamp']=9;view.put(json.dumps(p))
        self.assertEqual(view.snapshot(10)['stamp'],10)
    def test_trial_rollover_terminal_and_late_producer(self):
        view=LocalizationView();experiment=ExperimentView(view)
        def status(trial,phase,started):
            experiment.put(json.dumps(dict(schema='asmc.localization.experiment.v1',run_id='run',trial_id=trial,
                phase=phase,branch='research',revision='sha',master_uri='http://127.0.0.1:11371',started_at=started)))
        p=dict(self.fixture(),run_id='run',trial_id='one')
        status('one','running',1);view.put(json.dumps(p));self.assertTrue(view.snapshot(10)['fresh'])
        status('two','preparing',2);view.put(json.dumps(p));self.assertIsNone(view.snapshot(10))
        p['trial_id']='two';view.put(json.dumps(p));self.assertTrue(view.snapshot(10)['fresh'])
        status('two','restoring',2);view.put(json.dumps(p));self.assertIsNone(view.snapshot(10))
        status('two','completed',2);self.assertTrue(experiment.snapshot()['terminal'])
        status('one','running',1);self.assertEqual(experiment.snapshot()['trial_id'],'two')
    def test_receive_timeout_even_when_replay_clock_frozen(self):
        from unittest.mock import patch
        view=LocalizationView()
        with patch('dashboard_localization.time.monotonic',return_value=100):view.put(json.dumps(self.fixture()))
        with patch('dashboard_localization.time.monotonic',return_value=101):
            p=view.snapshot(10);self.assertFalse(p['fresh']);self.assertIsNone(p['ego'])
    def test_retired_motion_epoch_cannot_return(self):
        view=LocalizationView();p=dict(self.fixture(),motion_frame='odom/one');view.put(json.dumps(p))
        p.update(motion_frame='odom/two',stamp=10.1);view.put(json.dumps(p))
        p.update(motion_frame='odom/one',stamp=10.2);view.put(json.dumps(p))
        self.assertEqual(view.snapshot(10.2)['motion_frame'],'odom/two')
    def test_normal_path_requires_current_map_source_and_receipt(self):
        p=dict(valid=True,header=dict(stamp=10,frame_id='map'))
        self.assertTrue(map_trajectory_fresh(p,10.1,.1))
        for now,age in [(11,.1),(9,.1),(10,1),(10,-1)]:self.assertFalse(map_trajectory_fresh(p,now,age))
        p['header']['frame_id']='base_link';self.assertFalse(map_trajectory_fresh(p,10,.1))

if __name__=='__main__':unittest.main()
