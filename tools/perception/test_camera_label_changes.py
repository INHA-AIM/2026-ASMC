import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image

from camera_data_audit import sha
from report_camera_label_changes import report


class LabelChangeReportTests(unittest.TestCase):
    def fixture(self, root):
        image = root/'rgb.png'
        Image.new('RGB', (80, 40), 'blue').save(image)
        old = dict(id='frame', image=str(image), image_sha256=sha(image), width=80, height=40,
            source='roboflow:source/project', split='train', eligible=False, reasons=['mixed'],
            annotations=[dict(id='box-1', original_label='9', xyxy=[5, 5, 30, 30])])
        new = dict(old, eligible=True, reasons=[], annotations=[dict(old['annotations'][0],
            canonical='pe_firewall', training_class='plastic_barrier')])
        review = dict(image_sha256=sha(image), decision='accept',
                      overrides=[dict(id='box-1', canonical='pe_firewall')])
        for name, data in [('old', old), ('new', new)]:
            (root/name).write_text(json.dumps(data)+'\n')
        (root/'decisions').write_text(json.dumps(dict(frame=review)))
        return new

    def test_label_only_change_has_linked_provenance_and_no_automatic_approval(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            self.fixture(root)
            result=report(root/'old', root/'new', root/'decisions', root/'out')
            self.assertEqual(result['override_counts'], {'pe_firewall': 1})
            evidence=json.loads((root/'out/changes.json').read_text())
            self.assertFalse(evidence['automatic_approval'])
            self.assertEqual(evidence['rows'][0]['original_annotation_ids'], ['box-1'])
            self.assertEqual(evidence['rows'][0]['rendered_sha256'], sha(root/'out/frame.jpg'))

    def test_moved_split_or_changed_geometry_cannot_be_reported_as_preserved(self):
        for field, value in [('split', 'heldout'), ('annotations', [dict(
                id='box-1', original_label='9', xyxy=[0, 0, 30, 30])])]:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as d:
                root=Path(d)
                new=self.fixture(root)
                new[field]=value
                (root/'new').write_text(json.dumps(new)+'\n')
                with self.assertRaises(ValueError):
                    report(root/'old', root/'new', root/'decisions', root/'out')
                self.assertFalse((root/'out/changes.json').exists())


if __name__ == '__main__':
    unittest.main()
