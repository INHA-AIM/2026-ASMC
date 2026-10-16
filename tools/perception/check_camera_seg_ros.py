#!/usr/bin/env python3
"""GPU-free three-view seg lifecycle, header/mask and legacy adapter contract."""
import importlib.util
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.backend import Detection
from asmc_camera.detection.segmentation import SegmentationFrame
from asmc_camera.nodes.object_detection_node import ObjectDetectionNode
from asmc_msgs.srv import SetInferenceMode


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adapter',action='store_true',help='requires the nested Msg build')
    parser.add_argument('--batched',action='store_true',help='exercise the opt-in batch worker with the same lifecycle contracts')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    deadline=time.monotonic()+15
    while True:
        try:rospy.get_master().getPid();break
        except Exception:
            if time.monotonic()>deadline:raise
            time.sleep(.1)
    rospy.set_param('/use_sim_time',False)
    rospy.init_node('camera_seg_ros_fixture')
    views=['front','left','right']
    rospy.set_param('~views',views);rospy.set_param('~seg_experiment',True)
    rospy.set_param('~managed',True);rospy.set_param('~initial_mode',0)
    rospy.set_param('~debug_masks',True);rospy.set_param('~hood_mask',True)
    rospy.set_param('~hood_mask_config',str(root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'))
    rospy.set_param('~class_groups',yaml.safe_load((root/'src/asmc_camera/config/camera/object_classes_seg_experiment.yaml').read_text())['class_groups'])
    names=['person','dog','car','wooden crate']
    class Backend:
        batched=args.batched
        info=dict(device='CPU_fixture',weights_sha256='fixture')
        empty=False
        fail=False
        input_black=[]
        def infer_images(self,images,order):
            return {view:self.predict_frame(images[view]) for view in order}
        def predict_frame(self,image,include_masks=True):
            if self.fail:raise RuntimeError('injected inference failure')
            self.input_black.append(bool(np.any(np.all(image==0,axis=2))))
            masks=[np.zeros(image.shape[:2],bool) for _ in names]
            for i,mask in enumerate(masks):mask[2:10,2+i*4:5+i*4]=True
            return SegmentationFrame([] if self.empty else [Detection(1.25,2.5,10.75,20.,n,.875) for n in names],
                [] if self.empty else masks,dict(inference_ms=0.))
    backend=Backend();node=ObjectDetectionNode(backend)
    outputs=[];overlays=[]
    subs=[rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,outputs.append,queue_size=100)]
    for view in views:
        topic='/perception/camera/objects/debug/'+('instances' if view=='front' else view+'/instances')+'/compressed'
        subs.append(rospy.Subscriber(topic,CompressedImage,overlays.append,queue_size=100))
    topics=dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',right='/cam_right/image_jpeg/compressed')
    pubs={v:rospy.Publisher(topics[v],CompressedImage,queue_size=5) for v in views}
    service=rospy.ServiceProxy('/perception/camera/objects/set_mode',SetInferenceMode)
    jpeg=cv2.imencode('.jpg',np.full((24,32,3),180,np.uint8))[1].tobytes()
    checks={}
    def wait(test,description):
        end=time.monotonic()+5
        while time.monotonic()<end:
            if test():return
            time.sleep(.01)
        raise AssertionError(description)
    def send(seq,data=jpeg):
        time.sleep(.07);headers={}
        for view in views:
            h=Header(seq=seq,stamp=rospy.Time.now(),frame_id='camera_'+view);headers[view]=h
            pubs[view].publish(CompressedImage(header=h,format='jpeg',data=data))
        return headers
    def for_headers(messages,headers):
        stamps={h.stamp.to_nsec() for h in headers.values()}
        return [m for m in messages if m.header.stamp.to_nsec() in stamps]
    try:
        wait(lambda:all(p.get_num_connections() for p in pubs.values()),'input connections')
        send(1);time.sleep(.1);checks['standby_no_compute']=not backend.input_black
        service(1,20.)
        for seq in (2,3,4):
            before=len(backend.input_black);send(seq)
            wait(lambda:len(backend.input_black)>=before+3,'all PREPARE views')
        checks['all_views_prepared_no_operational_output']=node.control.state.snapshot()['prepared'] and not outputs
        checks['front_only_hood_mask']=sum(backend.input_black)==3 and len(backend.input_black)==9
        service(2,20.);headers=send(5)
        wait(lambda:len(for_headers(outputs,headers))==3,'ACTIVE all views')
        wait(lambda:len(for_headers(overlays,headers))==3,'mask overlays all views')
        messages=for_headers(outputs,headers)
        # ROS publishers assign their own Header.seq; compare source stamp/frame_id.
        checks['source_header_dimensions_names_groups']=all(m.header.stamp==headers[m.header.frame_id[7:]].stamp and m.valid and
            (m.image_width,m.image_height)==(32,24) and [b.source_class_name for b in m.boxes]==names and
            [b.class_name for b in m.boxes]==['person','animal','vehicle','obstacle'] for m in messages)
        checks['coordinates_confidence_kept']=all((b.xmin,b.ymin,b.xmax,b.ymax,b.confidence)==(1.25,2.5,10.75,20.,.875) for m in messages for b in m.boxes)
        if args.adapter:
            path=root/'src/lidar+sensorfusion/src/Main/scripts/camera_objects_yolo_bbox_adapter.py'
            spec=importlib.util.spec_from_file_location('seg_actual_adapter',path);adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
            front=next(m for m in messages if m.header.frame_id=='camera_front')
            converted=adapter.convert(front)
            checks['front_seven_class_policy']=converted.header==front.header and [b.classes for b in converted.yolo_bbox_arr]==['person','dog','car'] and all(adapter.convert(m) is None for m in messages if m.header.frame_id!='camera_front')
        backend.empty=True;empty=send(6)
        wait(lambda:len(for_headers(outputs,empty))==3,'valid empty')
        checks['empty_valid_all_views']=all(m.valid and not m.boxes for m in for_headers(outputs,empty))
        invalid=send(7,b'broken jpeg')
        wait(lambda:len(for_headers(outputs,invalid))==3,'decode failures')
        checks['decode_invalid_all_views']=all(not m.valid and not m.boxes for m in for_headers(outputs,invalid))
        backend.fail=True;failed=send(8)
        wait(lambda:len(for_headers(outputs,failed))==3,'inference failures')
        checks['inference_invalid_with_original_size']=all(not m.valid and (m.image_width,m.image_height)==(32,24) for m in for_headers(outputs,failed))
        service(0,0.);checks['standby_clears_prepared']=not node.control.state.snapshot()['prepared']
    finally:
        node.shutdown()
        for p in pubs.values():p.unregister()
        for s in subs:s.unregister()
    print(json.dumps(dict(checks=checks,passed=all(checks.values())),indent=2))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
