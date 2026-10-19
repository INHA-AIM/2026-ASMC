#!/usr/bin/env python3
"""CPU-only generated-message visualization checks. Use an isolated ROS master."""
import json
import os
import time
from urllib.parse import urlsplit
import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from asmc_msgs.msg import InferenceStatus
from asmc_msgs.msg import CameraObjectBoxArray,CameraObjectObservation
from asmc_camera.nodes.object_view_node import ObjectViewNode
from asmc_camera.nodes.semantic_visualizer_node import SemanticVisualizerNode


def main():
    if urlsplit(os.environ.get('ROS_MASTER_URI','http://localhost:11311')).port == 11311:
        raise RuntimeError('isolated test master required; shared live11311 is forbidden')
    rospy.init_node('object_view_fixture',anonymous=True)
    rospy.set_param('~ego_mask_configs',dict.fromkeys(('left','front','right'),''))
    rospy.set_param('~ego_mask_enabled',dict.fromkeys(('left','front','right'),False))
    rospy.set_param('~panel_width',128);rospy.set_param('~panel_height',96)
    rospy.set_param('~camera_topics',ObjectViewNode.TOPICS);rospy.set_param('~views',['left','front','right'])
    node=ObjectViewNode();semantic=SemanticVisualizerNode()
    pubs={v:rospy.Publisher(t,CompressedImage,queue_size=1) for v,t in node.TOPICS.items()}
    observations=rospy.Publisher('/perception/camera/objects/observations',CameraObjectObservation,queue_size=3)
    status=rospy.Publisher('/perception/camera/objects/status',InferenceStatus,queue_size=1,latch=True)
    diagnostics=rospy.Publisher('/perception/camera/objects/diagnostics',DiagnosticArray,queue_size=1,latch=True)
    profiles={v:dict(enabled=False,config_sha256='') for v in pubs}
    diagnostics.publish(DiagnosticArray(status=[DiagnosticStatus(name='camera_objects/inference',
        values=[KeyValue(key='ego_mask_by_view',value=repr(profiles))])]))
    status.publish(InferenceStatus(generation=2,managed=True,mode=2))
    time.sleep(.4)
    checks=dict(idle_no_image_subscriptions=not node.images,
                idle_no_decode=node.counts['decode']==0,
                idle_semantic_no_large_inputs=all(
                    not group.active and not group.subscribers
                    for group in semantic.synchronizers))
    frames=[]
    sink=rospy.Subscriber('/perception/camera/objects/debug/mosaic/compressed',CompressedImage,
        lambda m:frames.append(m),queue_size=1)
    colors=dict(left=(0,180,0),front=(180,0,0),right=(0,0,180))
    blobs={v:cv2.imencode('.jpg',np.full((96,128,3),c,np.uint8))[1].tobytes() for v,c in colors.items()}
    def send(session='fixture',generation=2,valid=True):
        stamp=rospy.Time.now()
        for view,p in pubs.items():
            header=Header(stamp=stamp,frame_id='camera_'+view)
            p.publish(CompressedImage(header=header,format='jpeg',data=blobs[view]))
            observations.publish(CameraObjectObservation(detections=CameraObjectBoxArray(header=header,
                image_width=128,image_height=96,valid=valid),producer_session_id=session,generation=generation,
                calibration_id=view,model_id='fixture',config_id='fixed'))
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        send();time.sleep(.05)
    checks['one_final_encode_no_intermediate']=node.counts['mosaic_encode']==node.counts['mosaic_publish'] and node.counts['individual_encode']==0
    checks['three_view_render']=node.counts['render']>10 and all(len(q)<=8 for q in node.cache.images.values())
    image=cv2.imdecode(np.frombuffer(frames[-1].data,np.uint8),cv2.IMREAD_COLOR)
    checks['left_front_right_order']=all(np.argmax(image[125,64+128*i])==np.argmax(colors[v]) for i,v in enumerate(('left','front','right')))
    time.sleep(.7)
    image=cv2.imdecode(np.frombuffer(frames[-1].data,np.uint8),cv2.IMREAD_COLOR)
    checks['input_stop_blanks_old_panels']=image[70:].max()<10
    status.publish(InferenceStatus(generation=3,managed=True,mode=0));time.sleep(.25)
    send(generation=2);time.sleep(.2)
    checks['standby_clears_old_observations']=not node.rendered
    status.publish(InferenceStatus(generation=4,managed=True,mode=2));time.sleep(.25)
    send(generation=2);time.sleep(.15)
    checks['old_generation_rejected']=node.cache.counts['old_epoch']+node.cache.counts['status_generation_mismatch']>0 and not node.rendered
    deadline=time.monotonic()+.6
    while time.monotonic()<deadline:
        send(session='restarted',generation=4);time.sleep(.05)
    checks['new_session_recovers']=bool(node.rendered) and node.cache.identity[0]=='restarted'
    sink.unregister()
    deadline=time.monotonic()+3.
    while node.images and time.monotonic()<deadline:
        send(session='restarted',generation=4);time.sleep(.05)
    time.sleep(.15)
    before=node.counts['decode'];send(session='restarted',generation=4);time.sleep(.2)
    checks['disconnect_stops_image_work']=not node.images and node.counts['decode']==before
    node.close();semantic.close()
    checks['shutdown_joins_worker']=(not node.render_worker.thread.is_alive() and
        not semantic.render_worker.thread.is_alive() and
        not semantic.demand_inputs.worker.is_alive())
    checks={k:bool(v) for k,v in checks.items()}
    result=dict(passed=all(checks.values()),checks=checks,counters=dict(node.counts))
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
