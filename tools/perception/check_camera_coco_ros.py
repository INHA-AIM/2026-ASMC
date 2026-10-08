#!/usr/bin/env python3
"""GPU-free expanded-label lifecycle and optional actual legacy adapter check."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_camera.detection.backend import Detection
from asmc_camera.nodes.object_detection_node import ObjectDetectionNode
from asmc_camera.msg import InferenceStatus
from asmc_camera.srv import SetInferenceMode
from asmc_msgs.msg import CameraObjectBoxArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['baseline', 'all'], default='all')
    parser.add_argument('--adapter', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    deadline = time.monotonic()+15
    while True:
        try:
            rospy.get_master().getPid()
            break
        except Exception:
            if time.monotonic()>deadline:
                raise
            time.sleep(.1)
    rospy.set_param('/use_sim_time', False)
    rospy.init_node('camera_coco_ros_fixture')
    all_config = args.profile=='all'
    names_path = root/('src/asmc_camera/config/camera/yolo11m_coco_all.yaml' if all_config else 'src/asmc_camera/config/camera/yolo11m_coco.yaml')
    groups_path = root/('src/asmc_camera/config/camera/object_classes_coco_all.yaml' if all_config else 'src/asmc_camera/config/camera/object_classes.yaml')
    names = yaml.safe_load(names_path.read_text())['allowed_names']
    rospy.set_param('~class_groups', yaml.safe_load(groups_path.read_text())['class_groups'])
    rospy.set_param('~managed', True)
    rospy.set_param('~initial_mode', 0)
    rospy.set_param('~hood_mask', True)
    rospy.set_param('~hood_mask_config', str(root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'))

    class Backend:
        info = dict(device='CPU_fixture', weights_sha256='not_a_model')
        empty = False
        def detect(self, image):
            detections = [] if self.empty else [Detection(1.25, 2.5, 10.75, 20., name, .875) for name in names]
            return detections, dict(inference_ms=0.)

    backend = Backend()
    node = ObjectDetectionNode(backend)
    statuses, outputs = [], []
    subscriptions = [rospy.Subscriber('~inference_status', InferenceStatus, statuses.append, queue_size=100),
                     rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray, outputs.append, queue_size=100)]
    publisher = rospy.Publisher('/image_jpeg/compressed', CompressedImage, queue_size=5)
    service = rospy.ServiceProxy('~set_inference_mode', SetInferenceMode)
    jpeg = cv2.imencode('.jpg', np.full((24, 32, 3), 180, np.uint8))[1].tobytes()
    checks = {}
    def wait(predicate, description):
        end = time.monotonic()+5
        while time.monotonic()<end:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError(description)
    def send(seq, data=jpeg):
        # The managed worker discards an input arriving before its next slot.
        # A stepped fixture must supply its next fresh frame after that slot.
        time.sleep(.06)
        header = Header(seq=seq, stamp=rospy.Time.now(), frame_id='camera_front')
        publisher.publish(CompressedImage(header=header, format='jpeg', data=data))
        return header
    try:
        wait(lambda: publisher.get_num_connections()>0 and bool(statuses), 'ROS connections')
        send(1)
        time.sleep(.1)
        checks['standby_no_compute'] = node.control.state.successful_batches==0
        reply = service(1, 20.)
        for seq in range(2, 5):
            before = node.control.state.successful_batches
            send(seq)
            wait(lambda: node.control.state.successful_batches>before, 'PREPARE new frame')
        wait(lambda: statuses[-1].generation==reply.generation and statuses[-1].prepared, 'PREPARE acknowledged generation')
        checks['prepare_ready_no_operational_results'] = statuses[-1].pipeline_ready and not outputs
        service(2, 20.)
        header = send(5)
        wait(lambda: any(m.header.stamp==header.stamp for m in outputs), 'ACTIVE output')
        output = next(m for m in outputs if m.header.stamp==header.stamp)
        wire = io.BytesIO()
        output.serialize(wire)
        restored = CameraObjectBoxArray().deserialize(wire.getvalue())
        checks['source_stamp_frame_dimensions_labels_order'] = (restored.header.stamp==header.stamp and
            restored.header.frame_id==header.frame_id and restored.valid and
            (restored.image_width, restored.image_height)==(32, 24) and
            [b.source_class_name for b in restored.boxes]==names)
        checks['coordinates_confidence_preserved'] = all(
            (b.xmin,b.ymin,b.xmax,b.ymax,b.confidence)==(1.25,2.5,10.75,20.,.875) for b in restored.boxes)
        if all_config:
            mapped = {b.source_class_name:b.class_name for b in restored.boxes}
            checks['infrastructure_preserved_with_coarse_group'] = mapped['traffic light']==mapped['stop sign']=='obstacle'
        if args.adapter:
            path = root/'src/lidar+sensorfusion/src/Main/scripts/camera_objects_yolo_bbox_adapter.py'
            spec = importlib.util.spec_from_file_location('coco_actual_adapter', path)
            adapter = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(adapter)
            converted = adapter.convert(restored)
            expected = [name for name in names if name in ('bus','truck','person','car','dog','bicycle','motorcycle')]
            checks['actual_adapter_source_classes_header_integer_coordinates'] = (converted.header==restored.header and
                converted.objc==len(expected) and [b.classes for b in converted.yolo_bbox_arr]==expected and
                all((b.xMin,b.yMin,b.xMax,b.yMax)==(1,2,10,20) for b in converted.yolo_bbox_arr))
        backend.empty = True
        empty_header = send(6)
        wait(lambda: any(m.header.stamp==empty_header.stamp for m in outputs), 'valid empty')
        empty = next(m for m in outputs if m.header.stamp==empty_header.stamp)
        checks['normal_empty'] = empty.valid and not empty.boxes
        invalid_header = send(7, b'broken jpeg')
        wait(lambda: any(m.header.stamp==invalid_header.stamp for m in outputs), 'invalid JPEG')
        invalid = next(m for m in outputs if m.header.stamp==invalid_header.stamp)
        checks['decode_failure_source_header_invalid'] = not invalid.valid and not invalid.boxes and invalid.header.frame_id=='camera_front'
        if args.adapter:
            checks['adapter_empty_invalid_wrong_frame'] = (adapter.convert(empty).objc==0 and
                adapter.convert(invalid) is None)
            wrong = CameraObjectBoxArray().deserialize(wire.getvalue())
            wrong.header.frame_id='camera_left'
            checks['adapter_empty_invalid_wrong_frame'] &= adapter.convert(wrong) is None
        service(0, 0.)
        wait(lambda: not outputs[-1].valid, 'STANDBY invalidation')
        checks['standby_invalidates_ready'] = not node.control.state.snapshot()['prepared']
    finally:
        node.shutdown()
        publisher.unregister()
        for subscription in subscriptions:
            subscription.unregister()
    print(json.dumps(dict(profile=args.profile, adapter=args.adapter, checks=checks, passed=all(checks.values())), indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
