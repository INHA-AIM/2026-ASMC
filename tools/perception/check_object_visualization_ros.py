#!/usr/bin/env python3
"""Actual ROS source pairing and header rejection for YOLO debug, without GPU/GPS/LiDAR."""
import copy
import argparse
from pathlib import Path
import json
import time
import cv2
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from asmc_msgs.msg import CameraObjectBoxArray,CameraObjectBox
from asmc_camera.nodes.object_visualizer_node import ObjectVisualizerNode
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue


def wait(predicate,description,timeout=10):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.02)
    raise AssertionError(description)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hood-mask',action='store_true')
    parser.add_argument('--ego-hood-filter',action='store_true')
    parser.add_argument('--debug-raw',action='store_true')
    args=parser.parse_args()
    debug_raw=args.debug_raw or args.ego_hood_filter
    master=rosgraph.Master('/object_overlay_preflight')
    def ready():
        try:master.getPid();return True
        except Exception:return False
    wait(ready,'master ready',15)
    master.setParam('/use_sim_time',False)
    rospy.init_node('object_overlay_ros_check',anonymous=True)
    rospy.set_param('~hood_mask',args.hood_mask)
    rospy.set_param('~ego_hood_filter',args.ego_hood_filter)
    rospy.set_param('~debug_raw',debug_raw)
    rospy.set_param('~ego_hood_filter_config',str(Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera/yolo_ego_hood_filter_v1.yaml'))
    rospy.set_param('~hood_mask_config',str(Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'))
    visualizer=ObjectVisualizerNode()
    diagnostic_publisher=rospy.Publisher('/perception/camera/objects/diagnostics',DiagnosticArray,queue_size=1,latch=True)
    def profile(enabled,sha):
        diagnostic_publisher.publish(DiagnosticArray(status=[DiagnosticStatus(name='camera_objects/inference',values=[
            KeyValue(key='hood_mask_enabled',value=str(enabled)),KeyValue(key='hood_mask_config_sha256',value=sha),
            KeyValue(key='ego_hood_filter_enabled',value=str(args.ego_hood_filter)),
            KeyValue(key='ego_hood_filter_config_sha256',value=visualizer.ego_hood_filter.config_sha256),
            KeyValue(key='debug_raw_enabled',value=str(debug_raw))])]))
    profile(args.hood_mask,visualizer.hood_mask.config_sha256)
    wait(lambda:visualizer.producer_profile is not None,'matching producer diagnostic')
    outputs=[]
    subscription=rospy.Subscriber('/perception/camera/objects/debug/image/compressed',CompressedImage,outputs.append,queue_size=10)
    images=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=5)
    objects=rospy.Publisher('/perception/camera/objects',CameraObjectBoxArray,queue_size=5)
    raw_publisher=(rospy.Publisher('/perception/camera/objects/debug/raw',CameraObjectBoxArray,queue_size=5)
                   if debug_raw else None)
    def publish(result):
        if raw_publisher:
            raw=copy.deepcopy(result)
            if result.valid and args.ego_hood_filter:
                raw.boxes.append(CameraObjectBox(xmin=20,ymin=135,xmax=230,ymax=159,class_name='vehicle',source_class_name='car',confidence=.9,confidence_valid=True))
            raw_publisher.publish(raw)
        objects.publish(result)
    wait(lambda:images.get_num_connections() and objects.get_num_connections() and visualizer.publisher.get_num_connections(),'overlay connected')
    jpeg=cv2.imencode('.jpg',np.zeros((160,240,3),np.uint8))[1].tobytes()
    checks={}
    def pair(seq):
        h=Header(seq=seq,stamp=rospy.Time.now(),frame_id='camera_front')
        return CompressedImage(header=h,format='jpeg',data=jpeg),CameraObjectBoxArray(header=copy.deepcopy(h),image_width=240,image_height=160,valid=True)
    image,result=pair(1)
    result.boxes=[CameraObjectBox(xmin=60,ymin=70,xmax=140,ymax=130,class_name='vehicle',source_class_name='car',confidence=.9,confidence_valid=True)]
    publish(result);time.sleep(.1);images.publish(image)
    wait(lambda:len(outputs)==1,'result-before-image paired')
    checks['exact_source_stamp_and_frame_id']=(outputs[-1].header.stamp==image.header.stamp and outputs[-1].header.frame_id==image.header.frame_id)
    checks['decodable_box_overlay']=cv2.imdecode(np.frombuffer(outputs[-1].data,np.uint8),1)[70,60].any().item()
    image,result=pair(2);images.publish(image);time.sleep(.05);publish(result)
    wait(lambda:len(outputs)==2,'valid empty result displayed');checks['empty_result_displayed']=True
    image,result=pair(3);result.header.stamp+=rospy.Duration(.01)
    images.publish(image);publish(result);time.sleep(.2)
    checks['different_stamp_not_drawn']=len(outputs)==2
    image,result=pair(4);result.header.frame_id='camera_left'
    images.publish(image);publish(result);time.sleep(.2)
    checks['different_camera_not_drawn']=len(outputs)==2
    image,result=pair(5);result.image_width=1280
    images.publish(image);publish(result);time.sleep(.2)
    checks['different_dimensions_not_drawn']=len(outputs)==2
    image,result=pair(6);result.valid=False;result.image_width=result.image_height=0
    images.publish(image);publish(result)
    wait(lambda:len(outputs)==3,'invalid result displayed');checks['invalid_result_displayed']=(outputs[-1].header.stamp==image.header.stamp and outputs[-1].header.frame_id==image.header.frame_id)
    if args.hood_mask:
        decoded=cv2.imdecode(np.frombuffer(outputs[-1].data,np.uint8),1)
        checks['mask_shading_original_dimensions']=decoded.shape==(160,240,3) and int(decoded[150,180,2])>int(decoded[150,180,0])+20
        profile(True,'different-configuration')
        wait(lambda:visualizer.producer_profile[1]=='different-configuration','different mask diagnostic')
        image,result=pair(7);images.publish(image);publish(result);time.sleep(.2)
        checks['mismatched_mask_profile_withheld']=len(outputs)==3
        profile(True,visualizer.hood_mask.config_sha256)
        wait(lambda:visualizer.producer_profile[1]==visualizer.hood_mask.config_sha256,'profile restored')
        image,result=pair(8);images.publish(image);publish(result)
        wait(lambda:len(outputs)==4,'overlay after matching profile restored')
        checks['matching_mask_profile_resumes']=True
    if args.ego_hood_filter:
        decoded=cv2.imdecode(np.frombuffer(outputs[0].data,np.uint8),1)
        pixel=decoded[135,20]
        checks['excluded_box_drawn_gray']=int(pixel.max())-int(pixel.min())<35 and int(pixel.max())>80
    if debug_raw:
        count=len(outputs);image,result=pair(9);objects.publish(result);images.publish(image);time.sleep(.2)
        checks['raw_same_frame_required']=len(outputs)==count
        count=len(outputs);image,result=pair(10);raw=copy.deepcopy(result);raw.header.frame_id='camera_left'
        raw_publisher.publish(raw);objects.publish(result);images.publish(image);time.sleep(.2)
        checks['wrong_raw_camera_rejected']=len(outputs)==count
        image,result=pair(11);raw=copy.deepcopy(result);raw.header.stamp+=rospy.Duration(.01)
        raw_publisher.publish(raw);objects.publish(result);images.publish(image);time.sleep(.2)
        checks['wrong_raw_stamp_rejected']=len(outputs)==count
    if raw_publisher:raw_publisher.unregister()
    subscription.unregister();images.unregister();objects.unregister();diagnostic_publisher.unregister()
    rospy.signal_shutdown('done')
    print(json.dumps(dict(checks=checks,passed=all(checks.values())),indent=2))
    return 0 if all(checks.values()) else 1

if __name__=='__main__':raise SystemExit(main())
