import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from evaluate_coco7_models import evaluate, summary
from run_coco7_live_trials import measurement_gate, deadline_report
from b0_measurements import output_report
from fusion_log_accounting import parse_fusion_logs


def ann(label, verified=True):
    return dict(source_class_name=label,subtype_verified=verified,xyxy=[0,0,10,10],actor_id='a')


class Coco7EvaluationTest(unittest.TestCase):
    def test_unknown_subtype_is_not_a_negative_or_a_positive(self):
        c,ignored,pairs,_,_=evaluate([ann('Vehicle',False)],
            [dict(source_class_name='car',confidence=.9,xyxy=[0,0,10,10])],.7)
        self.assertEqual(1,ignored);self.assertEqual([],pairs)
        self.assertIsNone(summary(c)['macro_f1'])
        self.assertEqual(0,c['car']['fp'])

    def test_raw_labels_are_not_combined_and_duplicates_are_false_positive(self):
        ps=[dict(source_class_name=l,confidence=.9,xyxy=[0,0,10,10]) for l in ('car','car','truck')]
        c,_,pairs,_,_=evaluate([ann('car')],ps,.7)
        self.assertEqual(1,len(pairs));self.assertEqual(1,c['car']['tp'])
        self.assertEqual(1,c['car']['fp']);self.assertEqual(1,c['truck']['fp'])

    def test_wall_latency_missing_partial_window_and_duplicates_fail(self):
        rows=[dict(arrival=i*.1+.025,stamp_ns=i+1,valid=True,wall_latency_ms=25.,source_age_ms=25.,input_anchor='matched') for i in range(50)]
        self.assertTrue(measurement_gate(output_report(rows,0,5,expected_seconds=5)))
        self.assertFalse(measurement_gate(output_report(rows,0,4.9,expected_seconds=5)))
        rows[25]['stamp_ns']=1
        self.assertFalse(measurement_gate(output_report(rows,0,5,expected_seconds=5)))

    def test_idle_planner_and_missing_tail_do_not_pass(self):
        self.assertFalse(deadline_report([],0,5,.25,19)['passed'])
        self.assertFalse(deadline_report([{'arrival':i*.05} for i in range(95)],0,5,.25,19)['passed'])
        self.assertTrue(deadline_report([{'arrival':i*.05} for i in range(100)],0,5,.25,19)['passed'])

    def test_fusion_reuse_accounting_is_backward_compatible_and_conserved(self):
        tail='Fusion sync tail: matched=3 dropped=1 offset median/p95/max=10/20/30 ms\n'
        legacy='Fusion sync total: matched=3 dropped=1 pending lidar/yolo=0/0'
        self.assertTrue(parse_fusion_logs(tail+legacy)[2])
        total='Fusion sync total: matched=3 dropped=1 fresh=2 reused=1 pending lidar/yolo=0/0'
        self.assertTrue(parse_fusion_logs(tail+total)[2])
        self.assertEqual(1,parse_fusion_logs(tail+total)[1]['reused'])
        self.assertFalse(parse_fusion_logs(tail+total.replace('reused=1','reused=2'))[2])


if __name__=='__main__':unittest.main()
