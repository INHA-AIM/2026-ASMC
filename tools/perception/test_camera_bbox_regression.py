import copy
from pathlib import Path
import unittest

from diagnose_camera_bbox_regression import (
    checkpoint_epoch, diagnose_frame, deterministic_review, regression_pairs, validate_names, verify_predictions)


class RegressionTest(unittest.TestCase):
    def row(self, labels=('motorcycle',)):
        return dict(id='image', image_sha256='image-sha', view='unknown', source='source',
            annotations=[dict(training_class=n, xyxy=[i*100,0,i*100+40,60]) for i,n in enumerate(labels)])

    def pred(self, name='car', confidence=.8, box=None):
        return dict(class_name=name,confidence=confidence,xyxy=box or [0,0,40,60])

    def test_class_order_and_text_order(self):
        validate_names({'0':'person','1':'motorcycle'},['person','motorcycle'])
        with self.assertRaisesRegex(ValueError,'order'):validate_names(['motorcycle','person'],['person','motorcycle'])
        with self.assertRaises(ValueError):validate_names({1:'person'},['person'])

    def test_confusion_is_not_geometry_missing(self):
        d=diagnose_frame(self.row(),[self.pred()])[0]
        self.assertEqual(d['category'],'other_class');self.assertEqual(d['iou'],1)
        self.assertEqual(diagnose_frame(self.row(),[])[0]['category'],'position_missing_at_saved_floor')

    def test_threshold_and_localization(self):
        self.assertEqual(diagnose_frame(self.row(),[self.pred('motorcycle',.4)])[0]['category'],'correct')
        self.assertEqual(diagnose_frame(self.row(),[self.pred('motorcycle',.39)])[0]['category'],'confidence_below_threshold')
        self.assertEqual(diagnose_frame(self.row(),[self.pred('motorcycle',.8,[20,0,60,60])])[0]['category'],'box_localization_error')

    def test_paired_threshold_applies_to_both_phases(self):
        before=[self.pred('motorcycle',.3)]
        self.assertFalse(regression_pairs(self.row(),before,[],.4))
        pairs=regression_pairs(self.row(),before,[],.25)
        self.assertEqual(len(pairs),1)
        self.assertEqual(pairs[0][0]['category'],'correct')

    def test_one_prediction_cannot_explain_two_objects(self):
        row=self.row(('motorcycle','motorcycle'));row['annotations'][1]['xyxy']=[2,0,42,60]
        d=diagnose_frame(row,[self.pred('motorcycle')])
        self.assertEqual(sum(a['category']=='correct' for a in d),1)
        self.assertEqual(sum(a['category']=='position_missing_at_saved_floor' for a in d),1)

    def test_duplicate_predictions_do_not_increase_tp(self):
        d=diagnose_frame(self.row(),[self.pred('motorcycle')]*2)
        self.assertEqual(len(d),1);self.assertEqual(d[0]['category'],'correct')

    def test_tiny_scope_and_partial_gt_never_mutate_data(self):
        row=self.row();row['annotations'][0]['xyxy']=[0,0,2,2];row['evaluation_scope']='recall_only'
        original=copy.deepcopy(row)
        self.assertFalse(diagnose_frame(row,[]));self.assertEqual(len(diagnose_frame(row,[],scoped=False)),1)
        self.assertEqual(row,original)

    def test_provenance_and_wrong_baseline(self):
        row=self.row();p=dict(id='image',image_sha256='image-sha',ego_mask_applied=False,predictions=[])
        m=dict(manifest_sha256='manifest',config_sha256='config',weights_sha256='baseline')
        verify_predictions([row],[p],m,'manifest','config','baseline')
        for replacement in [dict(m,weights_sha256='wrong'),dict(m,manifest_sha256='wrong')]:
            with self.assertRaises(ValueError):verify_predictions([row],[p],replacement,'manifest','config','baseline')
        with self.assertRaises(ValueError):verify_predictions([row],[dict(p,image_sha256='wrong')],m,'manifest','config')
        with self.assertRaises(ValueError):verify_predictions([row],[dict(p,ego_mask_applied=True)],m,'manifest','config')

    def test_partial_checkpoint_excluded(self):
        record=dict(completed_epochs=16)
        self.assertEqual(checkpoint_epoch(Path('epoch15.pt'),record,15),16)
        self.assertEqual(checkpoint_epoch(Path('initial.pt'),record,-1),0)
        with self.assertRaisesRegex(ValueError,'partial'):checkpoint_epoch(Path('epoch16.pt'),record,16)
        with self.assertRaises(ValueError):checkpoint_epoch(Path('epoch3.pt'),record,2)

    def test_all_rare_motorcycles_and_bounded_source_round_robin(self):
        rows=[]
        for source in ('a','b'):
            for i in range(8):rows.append(dict(self.row(('person',)),id=source+str(i),source=source))
        rows.append(dict(self.row(),id='motor'))
        moto,risk=deterministic_review(rows,20261011,6)
        self.assertEqual([r['id'] for r in moto],['motor'])
        self.assertEqual(len(risk),6);self.assertEqual(sum(r['source']=='a' for r in risk),3)
        self.assertEqual(risk,deterministic_review(list(reversed(rows)),20261011,6)[1])


if __name__=='__main__':unittest.main()
