#!/usr/bin/env python3
"""Cache pre-NMS TensorRT predictions with the real packing/mask path, no training."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    import cv2
    import yaml
    from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
    from asmc_camera.detection.hood_mask import HoodMask
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--image-map', type=Path)
    p.add_argument('--host-data-root', help='Map a host-side data prefix to /data in this container')
    a = p.parse_args()
    image_map = json.loads(a.image_map.read_text()) if a.image_map else {}
    cfg = yaml.safe_load(a.config.read_text().replace('$(optenv ASMC_MODELS_ROOT /data/models)', '/data/models'))
    backend = BatchedYoloDetector(weights=cfg['weights'], expected_sha256=cfg['expected_sha256'],
        allowed_names=cfg['allowed_names'], conf_thres=.1, nms_iou=1., device='cuda:0', require_cuda=True,
        runtime_backend='tensorrt', front_width=cfg['front_width'], batch_mode='2plus1',
        head_profile=cfg['head_profile'])
    package = Path(__file__).resolve().parents[2] / 'src/asmc_camera'
    masks = {v:HoodMask.load(True,package/'config/camera'/f) for v,f in (
        ('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml'))}
    backend.capture_pre_nms_evaluation = True
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x') as stream:
        for line in a.manifest.read_text().splitlines():
            row = json.loads(line)
            image_path = image_map.get(row['image_sha256'], row['image'])
            if a.host_data_root and image_path.startswith(a.host_data_root+'/'):
                image_path = '/data/'+image_path[len(a.host_data_root)+1:]
            content = Path(image_path).read_bytes()
            if hashlib.sha256(content).hexdigest() != row['image_sha256']:
                raise ValueError('image SHA changed: '+row['id'])
            image = cv2.imread(image_path)
            view = row.get('view','unknown')
            if view in masks:
                image = masks[view].apply(image)
            packing_view = view if view in masks else 'front'
            result = backend.infer_images({packing_view:image},[packing_view])[packing_view]
            predictions = backend.pre_nms_evaluation[packing_view]
            stream.write(json.dumps(dict(id=row['id'],image_sha256=row['image_sha256'],predictions=predictions,
                view=view, ego_mask_applied=view in masks))+'\n')
    a.output.with_suffix('.metadata.json').write_text(json.dumps(dict(backend=backend.info,
        mask_shas={v:m.config_sha256 for v,m in masks.items()}, cutoff=.1, nms_iou=1.,
        unknown_view='front packing, no assumed Ego mask', nms_coordinates='actual packed tensor coordinates before source clipping',
        manifest_sha256=hashlib.sha256(a.manifest.read_bytes()).hexdigest()),indent=2))


if __name__ == '__main__':
    main()
