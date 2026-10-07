#!/usr/bin/env python3
"""Actual ROS source pairing and header rejection for YOLO debug, without GPU/GPS/LiDAR."""
import copy
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


def wait(predicate,description,timeout=10):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.02)
    raise AssertionError(description)


def main():
    master=rosgraph.Master('/object_overlay_preflight')
    def ready():
        try:master.getPid();return True
        except Exception:return False
    wait(ready,'master ready',15)
    master.setParam('/use_sim_time',False)
    rospy.init_node('object_overlay_ros_check',anonymous=True)
    visualizer=ObjectVisualizerNode()
    outputs=[]
    subscription=rospy.Subscriber('/perception/camera/objects/debug/image/compressed',CompressedImage,outputs.append,queue_size=10)
    images=rospy.Publisher('/image_jpeg/compressed',CompressedImage,queue_size=5)
    objects=rospy.Publisher('/perception/camera/objects',CameraObjectBoxArray,queue_size=5)
    wait(lambda:images.get_num_connections() and objects.get_num_connections() and visualizer.publisher.get_num_connections(),'overlay connected')
    jpeg=cv2.imencode('.jpg',np.zeros((160,240,3),np.uint8))[1].tobytes()
    checks={}
    def pair(seq):
        h=Header(seq=seq,stamp=rospy.Time.now(),frame_id='camera_front')
        return CompressedImage(header=h,format='jpeg',data=jpeg),CameraObjectBoxArray(header=copy.deepcopy(h),image_width=240,image_height=160,valid=True)
    image,result=pair(1)
    result.boxes=[CameraObjectBox(xmin=60,ymin=70,xmax=140,ymax=130,class_name='vehicle',source_class_name='car',confidence=.9,confidence_valid=True)]
    objects.publish(result);time.sleep(.1);images.publish(image)
    wait(lambda:len(outputs)==1,'result-before-image paired')
    checks['exact_source_stamp_and_frame_id']=(outputs[-1].header.stamp==image.header.stamp and outputs[-1].header.frame_id==image.header.frame_id)
    checks['decodable_box_overlay']=cv2.imdecode(np.frombuffer(outputs[-1].data,np.uint8),1)[70,60].any().item()
    image,result=pair(2);images.publish(image);time.sleep(.05);objects.publish(result)
    wait(lambda:len(outputs)==2,'valid empty result displayed');checks['empty_result_displayed']=True
    image,result=pair(3);result.header.stamp+=rospy.Duration(.01)
    images.publish(image);objects.publish(result);time.sleep(.2)
    checks['different_stamp_not_drawn']=len(outputs)==2
    image,result=pair(4);result.header.frame_id='camera_left'
    images.publish(image);objects.publish(result);time.sleep(.2)
    checks['different_camera_not_drawn']=len(outputs)==2
    image,result=pair(5);result.image_width=1280
    images.publish(image);objects.publish(result);time.sleep(.2)
    checks['different_dimensions_not_drawn']=len(outputs)==2
    image,result=pair(6);result.valid=False;result.image_width=result.image_height=0
    images.publish(image);objects.publish(result)
    wait(lambda:len(outputs)==3,'invalid result displayed');checks['invalid_result_displayed']=(outputs[-1].header.stamp==image.header.stamp and outputs[-1].header.frame_id==image.header.frame_id)
    subscription.unregister();images.unregister();objects.unregister()
    rospy.signal_shutdown('done')
    print(json.dumps(dict(checks=checks,passed=all(checks.values())),indent=2))
    return 0 if all(checks.values()) else 1

if __name__=='__main__':raise SystemExit(main())
