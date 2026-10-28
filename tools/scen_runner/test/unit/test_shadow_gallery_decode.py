import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.records.analyze_shadow import decode_gallery_image

class GalleryPayloadTest(unittest.TestCase):
    def test_empty_or_invalid_gallery_does_not_abort_decision_analysis(self):
        self.assertEqual(decode_gallery_image(b''),(None,'empty_image_payload'))
        image,reason=decode_gallery_image(b'not an encoded camera image')
        self.assertIsNone(image)
        self.assertIn(reason,('invalid_image_payload','image_decode_exception'))
