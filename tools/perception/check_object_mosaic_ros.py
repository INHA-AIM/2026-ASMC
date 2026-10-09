#!/usr/bin/env python3
"""CPU ROS check on a dedicated master; never inject synthetic live sensor data."""
import argparse
import json
import os
from pathlib import Path
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from asmc_camera.nodes.object_visualizer_node import ObjectMosaicNode


def wait(predicate, name, timeout=5):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.02)
    raise AssertionError(name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    if urlsplit(os.environ.get('ROS_MASTER_URI','http://localhost:11311')).port == 11311:
        parser.error('dedicated test master required; shared live11311 is forbidden')
    if a.output.exists():parser.error('refusing to overwrite prior evidence')
    rospy.init_node('object_mosaic_ros_check',anonymous=True)
    rospy.set_param('~panel_width',160);rospy.set_param('~panel_height',120)
    node=ObjectMosaicNode();outputs=[];checks={}
    publishers={v:rospy.Publisher(t,CompressedImage,queue_size=1) for v,t in node.TOPICS.items()}
    colors={'left':(160,0,0),'front':(0,160,0),'right':(0,0,160)}
    def publish(view,data=None,frame=None):
        image=np.full((120,160,3),colors[view],np.uint8)
        msg=CompressedImage(format='jpeg',data=data if data is not None else cv2.imencode('.jpg',image)[1].tobytes())
        msg.header.stamp=rospy.Time.now();msg.header.frame_id=frame or 'camera_'+view
        publishers[view].publish(msg)
        return msg
    def decoded():return cv2.imdecode(np.frombuffer(outputs[-1].data,np.uint8),1)
    def panel(view):
        n=('left','front','right').index(view)
        return decoded()[90:120,n*160+40:n*160+120]
    def has_color(view):return len(outputs)>0 and np.all(np.abs(panel(view).mean(axis=(0,1))-colors[view])<8)
    subscription=None
    try:
        time.sleep(.3)
        checks['no_subscriptions_without_viewer']=all(p.get_num_connections()==0 for p in publishers.values())
        subscription=rospy.Subscriber('/perception/camera/objects/debug/mosaic/compressed',CompressedImage,outputs.append,queue_size=1)
        wait(lambda:all(p.get_num_connections() for p in publishers.values()),'lazy subscriptions connect')
        publish('front');wait(lambda:has_color('front'),'front updates without sides')
        checks['missing_views_blank']=not panel('left').any() and not panel('right').any()
        publish('left');publish('right');wait(lambda:all(has_color(v) for v in colors),'left/front/right order')
        checks['layout_and_header']=(decoded().shape==(172,480,3) and outputs[-1].header.frame_id=='camera_objects_debug_mosaic')
        deadline=time.monotonic()+.8
        while time.monotonic()<deadline:
            publish('front');publish('right');time.sleep(.06)
        wait(lambda:has_color('front') and has_color('right') and not panel('left').any(),'stale left blank while other views continue')
        checks['stale_view_isolated']=True
        publish('left');wait(lambda:has_color('left'),'left recovery')
        with node.lock:stamp=node.latest['left'][0].header.stamp
        publish('left',frame='camera_front');time.sleep(.12)
        with node.lock:checks['wrong_camera_rejected']=node.latest['left'][0].header.stamp==stamp
        publish('left',data=b'broken JPEG');wait(lambda:not panel('left').any(),'invalid JPEG blank')
        publish('left');wait(lambda:has_color('left'),'recovery after invalid JPEG')
        checks['bad_jpeg_does_not_stop_worker']=node.thread.is_alive()
        subscription.unregister();subscription=None
        wait(lambda:all(p.get_num_connections()==0 for p in publishers.values()),'lazy subscriptions released')
        checks['viewer_close_unsubscribes']=True
    finally:
        if subscription:subscription.unregister()
        node.close()
        for p in publishers.values():p.unregister()
        rospy.signal_shutdown('mosaic check complete')
    checks['worker_stops']=not node.thread.is_alive()
    report=dict(checks=checks,passed=all(checks.values()),master=os.environ['ROS_MASTER_URI'])
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
