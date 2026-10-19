import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/asmc_camera/src'))
from asmc_camera.detection.mission import group_of
from asmc_camera.detection.output_nms import suppress
import evaluate_camera_bbox_pilot as e

class GroupEvaluation(unittest.TestCase):
    def row(self,supervision='complete'):
        return dict(id='frame',image_sha256='hash',split_group='scene',supervision=supervision,
                    annotations=[dict(training_class='car',xyxy=[0,0,20,20],supervision='positive')])
    def preds(self):return [dict(class_name='bus',confidence=.8,xyxy=[0,0,20,20]),dict(class_name='car',confidence=.7,xyxy=[0,0,20,20])]
    def test_group_duplicate_only_one_tp(self):
        with patch.object(e,'accepted_name',group_of):
            boxes,_=suppress(self.preds());counts,_=e.frame_counts(self.row(),boxes,.4,'accepted',True)
        self.assertEqual(counts['vehicle']['tp'],1)
        self.assertEqual(counts['vehicle']['complete_predictions'],2)
    def test_partial_does_not_make_fp(self):
        with patch.object(e,'accepted_name',group_of):
            counts,_=e.frame_counts(self.row('partial'),self.preds(),.4,'accepted',True)
        self.assertEqual(counts['vehicle']['tp'],1)
        self.assertEqual(counts['vehicle']['complete_predictions'],0)
    def test_positive_has_priority_over_ignore(self):
        row=self.row();row['annotations'].append(dict(training_class='car',xyxy=[0,0,20,20],supervision='ignore'))
        with patch.object(e,'accepted_name',group_of):
            counts,_=e.frame_counts(row,self.preds()[:1],.4,'accepted',True)
        self.assertEqual(counts['vehicle']['complete_tp'],1)
if __name__=='__main__':unittest.main()
