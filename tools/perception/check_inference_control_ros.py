#!/usr/bin/env python3
"""GPU-free ROS lifecycle faults using the real object worker and generated API."""
import json
import threading
import time

import cv2
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_msgs.msg import InferenceStatus
from asmc_camera.msg import InferenceStatus as LegacyInferenceStatus
from asmc_camera.srv import SetInferenceMode as LegacySetInferenceMode
from asmc_msgs.srv import SetInferenceMode
from asmc_msgs.msg import CameraObjectBoxArray, CameraObjectObservation
from asmc_camera.nodes.object_detection_node import ObjectDetectionNode


class Backend:
    info = dict(device='CPU_fixture', weights_sha256='not_a_model')
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.block = False

    def detect(self, image):
        if self.block:
            self.entered.set()
            self.release.wait(3.)
            self.block = False
        return [], {'inference_ms': 0.}


def main():
    deadline=time.monotonic()+15
    while True:
        try:
            rospy.get_master().getPid();break
        except Exception:
            if time.monotonic()>=deadline:raise
            time.sleep(.1)
    rospy.set_param('/use_sim_time',True)
    rospy.init_node('inference_control_ros_fixture')
    rospy.set_param('~managed', True)
    rospy.set_param('~class_groups', {'groups':{'person':['person'],'vehicle':['car']}})
    clock_pub=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
    ticking=threading.Event()
    def tick():
        while not ticking.wait(.01):
            clock_pub.publish(Clock(clock=rospy.Time.from_sec(time.time())))
    clock_thread=threading.Thread(target=tick,daemon=True)
    clock_thread.start()
    while rospy.Time.now().to_nsec()==0:time.sleep(.01)
    backend=Backend()
    node=ObjectDetectionNode(backend)
    received, outputs, observations, legacy_status = [], [], [], []
    status_sub=rospy.Subscriber('/perception/camera/objects/status',InferenceStatus,received.append,queue_size=100)
    output_sub=rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,outputs.append,queue_size=100)
    observation_sub=rospy.Subscriber('/perception/camera/objects/observations',CameraObjectObservation,observations.append,queue_size=100)
    legacy_sub=rospy.Subscriber('/camera_object_detector/inference_status',LegacyInferenceStatus,legacy_status.append,queue_size=100)
    alias=rospy.ServiceProxy('/camera_object_detector/set_inference_mode',LegacySetInferenceMode)
    image_pub=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=5)
    service=rospy.ServiceProxy('/perception/camera/objects/set_mode',SetInferenceMode)
    checks={}
    def wait(predicate,timeout=4):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:time.sleep(.01)
        if not predicate():raise RuntimeError('ROS lifecycle fixture timed out')
    jpeg=cv2.imencode('.jpg',np.zeros((24,32,3),np.uint8))[1].tobytes()
    sequence=0
    def send(data=jpeg):
        nonlocal sequence
        sequence+=1
        image_pub.publish(CompressedImage(header=Header(seq=sequence,stamp=rospy.Time.now(),
            frame_id='camera_front'),format='jpeg',data=data))
        time.sleep(.11)
    try:
        wait(lambda:image_pub.get_num_connections()>0 and received)
        initial=received[-1].generation
        checks['invalid_request_no_generation_change']=not service(8,10).accepted and service(0,0).generation==initial
        send()
        checks['standby_no_compute']=node.control.state.successful_batches==0
        service(1,10)
        for _ in range(3):send()
        wait(lambda:received[-1].prepared)
        checks['prepare_empty_detection_ready_without_output']=received[-1].pipeline_ready and not outputs
        service(2,10)
        send()
        wait(lambda:any(m.valid for m in outputs))
        checks['active_empty_detection_valid']=outputs[-1].valid and not outputs[-1].boxes
        wait(lambda:observations and legacy_status)
        checks['observation_same_header_and_generation']=observations[-1].detections.header.stamp == outputs[-1].header.stamp and observations[-1].detections.header.frame_id == outputs[-1].header.frame_id and observations[-1].generation==received[-1].generation
        checks['status_observation_session_matches']=received[-1].producer_session_id==observations[-1].producer_session_id
        # Several state snapshots can share the same paused/quantized ROS stamp.
        # Match the complete payload rather than crossing every equal-stamp pair.
        def same_payload(a,b):
            return all(getattr(a,field)==getattr(b,field) for field in LegacyInferenceStatus.__slots__)
        wait(lambda:any(same_payload(a,b) for a in received for b in legacy_status))
        checks['legacy_status_same_payload']=any(same_payload(a,b) for a in received for b in legacy_status)
        checks['legacy_status_md5_preserved']=LegacyInferenceStatus._md5sum=='e106e808a882b2b0140346746e164145'
        checks['legacy_service_md5_preserved']=LegacySetInferenceMode._md5sum=='94a789aab65be7cc8d09e8f777708430'
        checks['observation_has_stable_identity']=bool(observations[-1].producer_session_id and observations[-1].calibration_id and observations[-1].model_id and observations[-1].config_id)
        checks['alias_same_state']=alias(2,0).generation==service(2,0).generation
        backend.block=True
        blocked_stamp=rospy.Time.now()
        image_pub.publish(CompressedImage(header=Header(seq=100,stamp=blocked_stamp,
            frame_id='camera_front'),data=jpeg))
        wait(backend.entered.is_set)
        generation=service(0,0).generation
        backend.release.set()
        time.sleep(.3)
        checks['old_inflight_result_withheld']=not any(m.header.stamp==blocked_stamp for m in outputs)
        checks['standby_invalidation']=not outputs[-1].valid
        service(1,10)
        for _ in range(3):send()
        wait(lambda:received[-1].prepared)
        send(b'broken jpeg')
        wait(lambda:not received[-1].prepared and received[-1].errors>0)
        checks['decode_failure_clears_ready']=True
        for _ in range(3):send()
        wait(lambda:received[-1].prepared)
        # Freeze ROS time together with the source. Readiness must still expire.
        ticking.set();clock_thread.join(timeout=1.)
        saved=rospy.Time.now()
        rospy.set_param('/use_sim_time',True)
        clock_pub.publish(Clock(clock=saved))
        before=len(received)
        wait(lambda:len(received)>before and not received[-1].prepared)
        checks['frozen_clock_input_stop_status_updates']=received[-1].reason in (
            'input_or_result_stale','source_image_stale')
    finally:
        backend.release.set()
        ticking.set();clock_thread.join(timeout=1.)
        node.shutdown()
        rospy.set_param('/use_sim_time',False)
        for sub in (status_sub,output_sub,observation_sub,legacy_sub):sub.unregister()
        image_pub.unregister();clock_pub.unregister()
    passed=bool(checks) and all(checks.values())
    print(json.dumps(dict(passed=passed,checks=checks)))
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
