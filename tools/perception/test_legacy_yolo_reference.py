import tempfile
import unittest
from pathlib import Path

from tools.perception.legacy_yolo_reference import require_legacy_yolo_checkout


class HistoricalCheckoutTest(unittest.TestCase):
    def test_partial_checkout_cannot_launch_removed_inference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            main = root / 'src/lidar+sensorfusion/src/Main'
            (main / 'YOLOWeight').mkdir(parents=True)
            (main / 'YOLOWeight/yolo11m.pt').touch()
            with self.assertRaisesRegex(RuntimeError, 'ae8dcbc0.*matching build/source'):
                require_legacy_yolo_checkout(root)

    def test_original_checkout_is_read_only_and_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            main = root / 'src/lidar+sensorfusion/src/Main'
            for relative in ('src/Camera/YOLO/YOLODetection.py', 'scripts/yolo_detection_node', 'YOLOWeight/yolo11m.pt'):
                path = main / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'original')
            require_legacy_yolo_checkout(root)
            self.assertEqual((main / 'YOLOWeight/yolo11m.pt').read_bytes(), b'original')
