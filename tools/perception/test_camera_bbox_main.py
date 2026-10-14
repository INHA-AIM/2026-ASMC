import copy
import unittest

from train_camera_bbox_pilot import next_epoch_fits, validate_resume
from run_camera_bbox_main import compare_metrics, consecutive_regressions, checkpoint_score, evaluation_epochs, regression_pause_required


class MainTrainingTest(unittest.TestCase):
    def test_regression_override_preserves_default_stop(self):
        self.assertTrue(regression_pause_required(['person']))
        self.assertFalse(regression_pause_required(['person'], True))
        self.assertFalse(regression_pause_required([]))

    def test_budget_boundary_median_margin_and_reserve(self):
        self.assertFalse(next_epoch_fits(10000,[]))
        self.assertTrue(next_epoch_fits(1800,[1000]))
        self.assertFalse(next_epoch_fits(1799,[1000]))
        self.assertTrue(next_epoch_fits(1800,[100000,1000,1000]))

    def test_evaluation_schedule(self):
        self.assertEqual(evaluation_epochs(12),[1,2,4,8,12])
        self.assertEqual(evaluation_epochs(13),[1,2,4,8,12,13])
        self.assertEqual(evaluation_epochs(9,2),[1,2,4,6,8,9])
        self.assertEqual(evaluation_epochs(2,2),[1,2])
        with self.assertRaises(ValueError):evaluation_epochs(10,0)

    def test_precision_regression_cannot_hide_behind_recall(self):
        baseline={'control':{'selection':{'class':{'classes':{
            'person':dict(gt=100,complete_gt=100,annotated_recall=.9,precision=.9,f1=.9,complete_fp=10),
            'box':dict(gt=30,complete_gt=30,annotated_recall=0.,precision=0.,f1=0.)}}}}}
        after=copy.deepcopy(baseline)
        after['control']['selection']['class']['classes']['person'].update(precision=.3,f1=.45,complete_fp=210)
        after['control']['selection']['class']['classes']['box']['f1']=.8
        result=compare_metrics(baseline,after,quality_policy=dict(precision_drop_limit=.05,f1_drop_limit=.02))
        self.assertEqual(result['regressions'],[])
        self.assertEqual(len(result['quality_regressions']),2)
        self.assertIsNone(checkpoint_score(result))
        self.assertFalse(regression_pause_required(result['regressions']))
        legacy=compare_metrics(baseline,after)
        self.assertEqual(checkpoint_score(legacy),.8)
        for data in (baseline,after):data['control']['selection']['class']['classes']['person']['complete_gt']=10
        self.assertFalse(compare_metrics(baseline,after,quality_policy=dict(f1_drop_limit=.02))['quality_regressions'])

    def test_partial_recall_and_complete_only_new_f1(self):
        baseline={'control':{'selection':{'class':{'classes':{'motorcycle':dict(gt=100,tp=90,annotated_recall=.9),
            'box':dict(gt=30,annotated_recall=0,complete_gt=0,f1=None)}}}}}
        after=copy.deepcopy(baseline)
        after['control']['selection']['class']['classes']['motorcycle'].update(tp=87,annotated_recall=.87)
        result=compare_metrics(baseline,after)
        self.assertEqual(result['regressions'],['motorcycle'])
        self.assertIsNone(result['supported_new_macro_f1'])
        self.assertIsNone(checkpoint_score(result))
        self.assertEqual(consecutive_regressions(result,result),['motorcycle'])
        self.assertEqual(consecutive_regressions({},result),[])

    def test_threshold_boundary_and_small_samples(self):
        baseline={'control':{'selection':{'class':{'classes':{'person':dict(gt=100,annotated_recall=.9),
            'dog':dict(gt=10,annotated_recall=1.)}}}}}
        after=copy.deepcopy(baseline)
        after['control']['selection']['class']['classes']['person']['annotated_recall']=.88
        after['control']['selection']['class']['classes']['dog']['annotated_recall']=0.
        self.assertEqual(compare_metrics(baseline,after)['regressions'],[])

    def test_resume_config_epoch_change_rejected(self):
        before=dict(model_key='yolo26l',epochs=40)
        after=dict(before,epochs=20)
        with self.assertRaisesRegex(ValueError,'epochs'):validate_resume(before,after)


if __name__=='__main__':unittest.main()
