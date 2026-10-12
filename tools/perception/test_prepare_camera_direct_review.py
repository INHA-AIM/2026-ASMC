import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from PIL import Image

from camera_data_audit import sha
from prepare_camera_direct_review import prepare


class DirectReviewTests(unittest.TestCase):
    def fixture(self, root):
        curated = root / 'curated'
        curated.mkdir()
        image = root / 'rgb.png'
        Image.new('RGB', (80, 40), '#62939b').save(image)
        row = dict(id='frame-a', image=str(image), image_sha256=sha(image),
                   source='roboflow:source/project', width=80, height=40,
                   annotations=[dict(id='ann-1', original_label='PE', disposition='candidate',
                                     xyxy=[10, 5, 30, 35])])
        (curated / 'curated.jsonl').write_text(json.dumps(row) + '\n')
        (curated / 'review-progress.json').write_text(json.dumps(dict(buckets=[
            dict(bucket=b, sample=dict(sample_ids=['frame-a']))
            for b in ('source|PE', 'source|other')])))
        return curated, image

    def test_shared_sample_has_one_panel_and_both_bucket_references(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            curated, image = self.fixture(root)
            before = sha(image)
            result = prepare(curated, ['source|PE', 'source|other'], root / 'out')
            manifest = json.loads((root / 'out/samples.json').read_text())
            self.assertEqual(result['samples'], 1)
            self.assertEqual(manifest['samples'][0]['buckets'], ['source|PE', 'source|other'])
            self.assertFalse(manifest['decisions_generated'])
            self.assertEqual(sha(image), before)
            self.assertEqual(manifest['sheet_sha256']['sheet-000.jpg'],
                             sha(root / 'out/sheet-000.jpg'))

    def test_changed_source_image_cannot_produce_review_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            curated, image = self.fixture(root)
            Image.new('RGB', (80, 40), 'red').save(image)
            with self.assertRaisesRegex(ValueError, 'SHA changed'):
                prepare(curated, ['source|PE'], root / 'out')
            self.assertFalse((root / 'out/samples.json').exists())

    def test_unknown_bucket_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            curated, _ = self.fixture(root)
            with self.assertRaisesRegex(ValueError, 'Unknown buckets'):
                prepare(curated, ['unknown'], root / 'out')
            self.assertFalse((root / 'out').exists())

    def test_all_images_includes_unsampled_rows_once_and_keeps_annotation_ids(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            curated, image = self.fixture(root)
            row = json.loads((curated / 'curated.jsonl').read_text())
            row['annotations'][0]['disposition'] = 'ambiguous'
            other = dict(row, id='frame-b')
            (curated / 'curated.jsonl').write_text('\n'.join(map(json.dumps, [row, other]))+'\n')
            bucket = row['source']+'|PE'
            (curated / 'review-progress.json').write_text(json.dumps(dict(buckets=[
                dict(bucket=bucket, sample=dict(sample_ids=['frame-a']))])))
            before = sha(image)
            result = prepare(curated, [bucket], root/'out', True, ['PE'])
            manifest = json.loads((root/'out/samples.json').read_text())
            crops = json.loads((root/'out/annotation-crops.json').read_text())
            self.assertEqual(result['samples'], 2)
            self.assertEqual(manifest['selection'], 'all_matching_images')
            self.assertEqual([r['id'] for r in crops], ['frame-a', 'frame-b'])
            self.assertEqual(crops[0]['annotation']['id'], 'ann-1')
            self.assertEqual(crops[0]['annotation']['xyxy'], [10, 5, 30, 35])
            self.assertEqual(manifest['crop_sheet_sha256']['annotation-crops-000.jpg'],
                             sha(root/'out/annotation-crops-000.jpg'))
            self.assertFalse(manifest['decisions_generated'])
            self.assertFalse((root/'out/reviews.json').exists())
            self.assertEqual(sha(image), before)

    def test_legacy_pillow_without_resampling_keeps_crop_metadata(self):
        class LegacyImageAPI:
            NEAREST = getattr(Image, 'Resampling', Image).NEAREST

            def __getattr__(self, name):
                if name == 'Resampling':
                    raise AttributeError(name)
                return getattr(Image, name)

        with mock.patch('prepare_camera_direct_review.Image', LegacyImageAPI()):
            self.test_all_images_includes_unsampled_rows_once_and_keeps_annotation_ids()

    def test_all_images_does_not_include_other_source_with_same_label(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            curated, _ = self.fixture(root)
            row = json.loads((curated/'curated.jsonl').read_text())
            bucket = 'roboflow:different/project|PE'
            (curated/'review-progress.json').write_text(json.dumps(dict(buckets=[
                dict(bucket=bucket, sample=dict(sample_ids=[]))])))
            result = prepare(curated, [bucket], root/'out', True, ['PE'])
            self.assertEqual(result['samples'], 0)
            self.assertEqual(json.loads((root/'out/annotation-crops.json').read_text()), [])


if __name__ == '__main__':
    unittest.main()
