#!/usr/bin/env python3
"""Real ROS worker/services/messages with a recording CPU detector backend."""
from pathlib import Path
import json
import time
import cv2
import numpy as np
import rospy
import rosgraph
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_camera.nodes.object_detection_node import ObjectDetectionNode
from asmc_camera.detection.backend import Detection
from asmc_msgs.srv import SetInferenceMode
from asmc_msgs.msg import CameraObjectBoxArray
from check_object_visualization_ros import wait


class RecordingBackend:
    info=dict(device='CPU fixture',weights_sha256='not a GPU model')
    def __init__(self):self.inputs=[];self.empty=False
    def detect(self,image):
        self.inputs.append(image.copy())
        return ([] if self.empty else [Detection(5,60,30,99,'person',.9)]),{}


def main():
    rosgraph.Master('/hood_mask_preflight').setParam('/use_sim_time',False)
    rospy.init_node('camera_object_detector')
    rospy.set_param('/use_sim_time',False)
    for key,value in dict(managed=True,initial_mode=0,target_hz=20.,hood_mask=True,ego_hood_filter=False,debug_raw=False,
        hood_mask_config=str(Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'),
        class_groups=dict(groups=dict(person=['person'],vehicle=['car']))).items():rospy.set_param('~'+key,value)
    backend=RecordingBackend();node=ObjectDetectionNode(backend=backend)
    outputs=[];checks={}
    sub=rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,outputs.append,queue_size=10)
    source=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=5)
    jpeg=cv2.imencode('.jpg',np.full((100,120,3),200,np.uint8))[1].tobytes()
    request=rospy.ServiceProxy('/perception/camera/objects/set_mode',SetInferenceMode)
    def send():
        h=Header(stamp=rospy.Time.now(),frame_id='camera_front')
        source.publish(CompressedImage(header=h,format='jpeg',data=jpeg));return h
    def matching(h):return [o for o in outputs if o.header.stamp==h.stamp]
    try:
        wait(lambda:source.get_num_connections() and node.publisher.get_num_connections(),'fixture connected')
        send();time.sleep(.1);checks['standby_no_inference']=not backend.inputs and not outputs
        checks['prepare_request_accepted']=request(1,20.).accepted
        for n in range(3):
            send();wait(lambda:len(backend.inputs)>n,'prepare processed');time.sleep(.06)
        checks['prepare_masked_without_operational_output']=node.control.state.snapshot()['prepared'] and not outputs
        checks['prepare_source_dimensions_and_mask']=backend.inputs[-1].shape==(100,120,3) and not backend.inputs[-1][95,60].any() and backend.inputs[-1][60,60].any().item()
        checks['active_request_accepted']=request(2,20.).accepted
        h=send();wait(lambda:matching(h),'active message')
        result=matching(h)[0]
        checks['original_coordinates_header_and_boundary_box']=(result.valid and result.header.frame_id==h.frame_id and
            (result.image_width,result.image_height)==(120,100) and len(result.boxes)==1 and
            result.boxes[0].source_class_name=='person' and result.boxes[0].class_name=='person' and result.boxes[0].ymax==99)
        backend.empty=True;time.sleep(.06);h=send();wait(lambda:matching(h),'empty message')
        checks['normal_empty_valid']=matching(h)[0].valid and not matching(h)[0].boxes
        apply=node.processor.hood_mask.apply
        def broken(image):raise ValueError('injected mask failure')
        node.processor.hood_mask.apply=broken
        time.sleep(.06);h=send();wait(lambda:matching(h),'mask failure message')
        result=matching(h)[0]
        checks['mask_failure_invalid_original_size']=not result.valid and not result.boxes and (result.image_width,result.image_height)==(120,100) and node.mask_errors==1
        node.processor.hood_mask.apply=apply
        count=len(outputs);request(0,0.)
        wait(lambda:len(outputs)>count,'standby invalidation')
        checks['standby_invalidates_prior_result']=not outputs[-1].valid and outputs[-1].header.stamp==h.stamp
        n=len(backend.inputs);send();time.sleep(.15);checks['standby_stops_repeated_inference']=len(backend.inputs)==n
    finally:
        node.shutdown();sub.unregister();source.unregister();rospy.signal_shutdown('done')
    print(json.dumps(dict(checks=checks,passed=all(checks.values())),indent=2))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
