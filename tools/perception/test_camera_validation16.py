import tempfile
import unittest
from pathlib import Path

from PIL import Image
from prepare_camera_validation16 import check_leak, check_review, deny_index, pixel_hash
from camera_pretrain_policy import CLASSES
from evaluate_camera_bbox_pilot import summarize


class Validation16Tests(unittest.TestCase):
    def test_fixed16_macro_keeps_small_classes_visible(self):
        row=dict(id='one',image_sha256='sha',split_group='scene',supervision='complete',
                 annotations=[dict(training_class=c,canonical=c,xyxy=[i*20,0,i*20+10,10],supervision='positive')
                              for i,c in enumerate(CLASSES)])
        predictions=[dict(id='one',image_sha256='sha',predictions=[dict(class_name=c,confidence=.9,xyxy=[i*20,0,i*20+10,10])
                     for i,c in enumerate(CLASSES)])]
        s=summarize([row],predictions,.4)
        self.assertEqual(s['macro_f1_fixed16'],1)
        self.assertIsNone(s['macro_f1'])  # Historical minimum sample rule unchanged.
        self.assertEqual(s['fixed16_small_sample_classes'],list(CLASSES))

    def test_all_train_exposures_denied(self):
        train = [{'id':'old','image_sha256':'sha','pixel_sha256':'pixels',
                  'source':'source-a','split_group':'layout-a','correlation_group':'sequence-a'}]
        deny=deny_index(train)
        for field in ('id','image_sha256','pixel_sha256','source','split_group','correlation_group'):
            row={'id':'new',field:train[0][field]}
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_leak(row,deny)
        check_leak({'id':'new','source':'other','split_group':'other-layout'},deny)

    def test_pixel_check_catches_reencoded_identical_content(self):
        with tempfile.TemporaryDirectory() as d:
            a,b=Path(d)/'a.png',Path(d)/'b.png'
            Image.new('RGB',(8,9),(22,33,44)).save(a,compress_level=0)
            Image.new('RGB',(8,9),(22,33,44)).save(b,compress_level=9)
            self.assertNotEqual(a.read_bytes(),b.read_bytes())
            self.assertEqual(pixel_hash(a),pixel_hash(b))

    def test_teacher_confidence_is_not_review(self):
        with self.assertRaises(ValueError):
            check_review({'confidence':.99,'annotation_complete':True})
        with tempfile.TemporaryDirectory() as d:
            evidence=Path(d)/'review.jpg';evidence.touch()
            row={'review_record':dict(decision='accept',all_targets_annotated=True,
                review_scope='camera_objects_v2_16',evidence=str(evidence),notes='RGB reviewed')}
            check_review(row)
            row['review_record']['all_targets_annotated']=False
            with self.assertRaises(ValueError):check_review(row)


if __name__=='__main__':unittest.main()
