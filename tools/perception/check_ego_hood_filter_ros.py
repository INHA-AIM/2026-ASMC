#!/usr/bin/env python3
"""Generated ROS messages, mode guards and raw/filtered outputs with one CPU inference."""
import copy
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.backend import Detection
from asmc_camera.nodes.object_detection_node import ObjectDetectionNode
from asmc_msgs.srv import SetInferenceMode
from check_object_visualization_ros import wait


class Backend:
    info = dict(device='CPU fixture',weights_sha256='not a GPU model')
    def __init__(self):
        self.inputs=[]
        self.boxes=[Detection(10,80,110,100,'car',.95),Detection(5,60,30,99,'person',.9),Detection(20,20,110,99,'car',.8)]
    def detect(self,image):
        self.inputs.append(image.copy())
        return self.boxes,{}


def main():
    rosgraph.Master('/ego_filter_preflight').setParam('/use_sim_time',False)
    rospy.init_node('camera_object_detector')
    config=Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera/yolo_ego_hood_filter_v1.yaml'
    for k,v in dict(managed=True,initial_mode=0,target_hz=20.,hood_mask=False,
        ego_hood_filter=True,ego_hood_filter_config=str(config),debug_raw=True,
        class_groups=dict(groups=dict(person=['person'],vehicle=['car']))).items():rospy.set_param('~'+k,v)
    backend=Backend();node=ObjectDetectionNode(backend)
    raw=[];filtered=[];checks={}
    subscriptions=[rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,filtered.append,queue_size=10),
        rospy.Subscriber('/perception/camera/objects/debug/raw',CameraObjectBoxArray,raw.append,queue_size=10)]
    source=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=5)
    jpeg=cv2.imencode('.jpg',np.full((100,120,3),200,np.uint8))[1].tobytes()
    mode=rospy.ServiceProxy('/perception/camera/objects/set_mode',SetInferenceMode)
    def send(data=jpeg):
        h=Header(stamp=rospy.Time.now(),frame_id='camera_front')
        source.publish(CompressedImage(header=h,format='jpeg',data=data));return h
    def paired(h):
        a=[m for m in filtered if m.header.stamp==h.stamp];b=[m for m in raw if m.header.stamp==h.stamp]
        return (a[-1],b[-1]) if a and b else None
    try:
        wait(lambda:source.get_num_connections() and node.publisher.get_num_connections() and node.raw_publisher.get_num_connections(),'connected')
        send();time.sleep(.1);checks['standby_has_no_inference_or_outputs']=not backend.inputs and not raw and not filtered
        mode(1,20.)
        for i in range(3):send();wait(lambda:len(backend.inputs)>i,'prepare processed');time.sleep(.06)
        checks['prepare_filters_without_publishing']=node.control.state.snapshot()['prepared'] and not raw and not filtered
        checks['original_input_unmodified']=bool((backend.inputs[-1]==200).all())
        mode(2,20.);before=len(backend.inputs);h=send();wait(lambda:paired(h),'active pair');a,b=paired(h)
        checks['exactly_one_inference_for_both_outputs']=len(backend.inputs)==before+1
        checks['hood_removed_person_and_near_car_preserved']=len(b.boxes)==3 and a.boxes==b.boxes[1:]
        checks['header_dimensions_and_labels_preserved']=a.header.stamp==b.header.stamp==h.stamp and a.header.frame_id==b.header.frame_id=='camera_front' and (a.image_width,a.image_height)==(b.image_width,b.image_height)==(120,100) and a.boxes[0].source_class_name=='person' and a.boxes[1].source_class_name=='car'
        original=copy.deepcopy(backend.boxes);backend.boxes=backend.boxes[:1];time.sleep(.06);h=send();wait(lambda:paired(h),'hood only pair');a,b=paired(h)
        checks['all_excluded_is_valid_empty']=a.valid and not a.boxes and b.valid and len(b.boxes)==1
        backend.boxes=[];time.sleep(.06);h=send();wait(lambda:paired(h),'empty pair');a,b=paired(h)
        checks['normal_empty_raw_and_filtered_valid']=a.valid and b.valid and not a.boxes and not b.boxes
        backend.boxes=original;partition=node.processor.ego_hood_filter.partition
        def broken(*args):raise ValueError('injected post-filter failure')
        node.processor.ego_hood_filter.partition=broken;time.sleep(.06);h=send();wait(lambda:paired(h),'filter error');a,b=paired(h)
        checks['filter_error_keeps_raw_and_invalidates_operational']=not a.valid and b.valid and len(b.boxes)==3 and a.image_width==120 and node.filter_errors==1
        node.processor.ego_hood_filter.partition=partition
        time.sleep(.06);h=send(b'bad jpeg');wait(lambda:paired(h),'decode error');a,b=paired(h)
        checks['decode_error_invalidates_both']=not a.valid and not b.valid and a.header.stamp==b.header.stamp==h.stamp
        count=len(filtered);mode(0,0.);wait(lambda:len(filtered)>count and len(raw)>count,'standby invalidates both')
        checks['standby_invalidates_both']=not filtered[-1].valid and not raw[-1].valid and filtered[-1].header.stamp==raw[-1].header.stamp
        before=len(backend.inputs);send();time.sleep(.15);checks['standby_stops_inference']=len(backend.inputs)==before
        # Construction must reject conflicting modes before touching a GPU backend.
        rospy.set_param('~hood_mask',True);rospy.set_param('~hood_mask_config',str(config.with_name('yolo_hood_mask_v1.yaml')))
        try:ObjectDetectionNode(Backend());checks['conflicting_modes_rejected']=False
        except ValueError:checks['conflicting_modes_rejected']=True
    finally:
        node.shutdown();source.unregister()
        for s in subscriptions:s.unregister()
        rospy.signal_shutdown('done')
    print(json.dumps(dict(checks=checks,passed=all(checks.values())),indent=2))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
