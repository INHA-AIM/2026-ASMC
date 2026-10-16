"""Fail before launching historical Main inference that was removed from runtime."""
from pathlib import Path

REFERENCE_COMMIT = 'ae8dcbc0eb80f06f8c8db54742543aad369ef3cc'


def require_legacy_yolo_checkout(root):
    main = Path(root) / 'src/lidar+sensorfusion/src/Main'
    required = ('src/Camera/YOLO/YOLODetection.py', 'scripts/yolo_detection_node',
                'YOLOWeight/yolo11m.pt')
    if any(not (main / name).is_file() for name in required):
        raise RuntimeError(
            'Historical Main YOLO was removed. Reproduce this old comparison in an isolated '
            'checkout of ' + REFERENCE_COMMIT + ' with its matching build/source environment. '
            'For current inference use asmc_camera camera.launch; do not restore a second producer.')
