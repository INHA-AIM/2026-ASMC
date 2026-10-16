#!/usr/bin/env python3
"""Cache fixed E8/front960 predictions and construct an offline Main parity input.

This is a deterministic transport/algorithm regression, not real-time latency or
GT detection evaluation. Prediction availability is the RGB bag record time,
not a measured inference completion time. Originals are never modified.
"""
import argparse
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag
import yaml

from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.messages import build_box_array
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
from asmc_camera.detection.ultralytics_backend import file_sha256


ROOT = Path(__file__).resolve().parents[2]
TOPICS = ['/imu', '/gps', '/Ego_topic', '/velodyne_points', '/image_jpeg/compressed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    camera = ROOT / 'src/asmc_camera'
    config_path = camera / 'config/camera/yolo26l_mission16_e8_three_view.yaml'
    config = yaml.safe_load(config_path.read_text())
    config['weights'] = config['weights'].replace('$(find asmc_camera)', str(camera))
    detector = BatchedYoloDetector(**config, direct_detect=True)
    mask = HoodMask.load(True, camera / 'config/camera/yolo_hood_mask_v1.yaml')
    groups = load_class_groups(yaml.safe_load((camera / 'config/camera/object_classes_mission16_v1.yaml').read_text())['class_groups'])
    spec = importlib.util.spec_from_file_location('handoff_adapter', ROOT / 'src/lidar+sensorfusion/src/Main/scripts/camera_objects_yolo_bbox_adapter.py')
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    adapter.check_message_build()
    count = boxes = empty = 0
    labels = {}
    destination = args.output / 'replay.bag'
    with rosbag.Bag(str(args.bag)) as source, rosbag.Bag(str(destination), 'w') as output, (args.output / 'predictions.jsonl').open('w') as records:
        for topic, message, recorded in source.read_messages(topics=TOPICS):
            output.write(topic, message, recorded)
            if topic != '/image_jpeg/compressed':
                continue
            image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Undecodable RGB at ' + str(message.header.stamp))
            result = detector.infer_images({'front': mask.apply(image)}, ['front'])['front']
            header = copy.deepcopy(message.header)
            header.frame_id = 'camera_front'
            array = build_box_array(header, image.shape[1], image.shape[0], True, result.detections, groups)
            wire = io.BytesIO()
            array.serialize(wire)
            array = CameraObjectBoxArray().deserialize(wire.getvalue())
            bbox = adapter.convert(array, config['allowed_names'], 0.0)
            if bbox is None:
                raise ValueError('Valid front prediction rejected by adapter')
            output.write('/perception/camera/objects', array, recorded)
            output.write('/YOLO/BBox', bbox, recorded)
            row = dict(stamp_ns=header.stamp.to_nsec(), image_sha256=hashlib.sha256(message.data).hexdigest(),
                       boxes=[dict(source_class_name=b.source_class_name, class_name=b.class_name,
                                   confidence=b.confidence, xyxy=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in array.boxes])
            records.write(json.dumps(row)+'\n')
            count += 1
            boxes += bbox.objc
            empty += bbox.objc == 0
            for b in array.boxes:
                labels[b.source_class_name] = labels.get(b.source_class_name, 0) + 1
    summary = dict(bag=str(args.bag), bag_sha256=file_sha256(args.bag), replay_bag=str(destination),
                   frames=count, boxes=boxes, empty=empty, labels=labels,
                   engine_sha256=config['expected_sha256'], config_sha256=file_sha256(config_path),
                   mask_sha256=mask.config_sha256, front_width=config['front_width'],
                   confidence=config['conf_thres'], nms_iou=config['nms_iou'],
                   availability='RGB_record_time_logical_offline_not_latency_acceptance')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
