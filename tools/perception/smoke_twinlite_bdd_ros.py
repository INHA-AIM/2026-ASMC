#!/usr/bin/env python3
"""Exercise binary BDD inference + demand renderer on a private ROS master."""
import argparse
from collections import Counter, deque
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=11497)
    p.add_argument('--seconds',type=float,default=12)
    a=p.parse_args()
    if not 11400<=a.port<=11999:raise ValueError('private test master port required')
    if not 0<a.seconds<=60:raise ValueError('smoke duration must be in (0, 60] seconds')
    a.output.mkdir(parents=True,exist_ok=False)
    uri='http://127.0.0.1:{}'.format(a.port)
    master=xmlrpc.client.ServerProxy(uri)
    try:
        master.getPid('/bdd_smoke_preflight')
    except OSError:
        pass
    else:
        raise RuntimeError('private test master already exists')
    os.environ['ROS_MASTER_URI']=uri
    os.environ['ROS_LOG_DIR']=str(a.output/'roslogs')
    core=None;parents=[]
    log=(a.output/'roscore.log').open('w')
    try:
        core=subprocess.Popen(['roscore','-p',str(a.port)],stdout=log,stderr=subprocess.STDOUT)
        end=time.monotonic()+20
        while True:
            try:
                master.getPid('/bdd_smoke_wait');break
            except OSError:
                if time.monotonic()>end:raise RuntimeError('master start timeout')
                time.sleep(.1)
        import cv2
        import numpy as np
        import rospy
        import roslaunch
        import rospkg
        from sensor_msgs.msg import Image,CompressedImage
        from asmc_msgs.msg import InferenceStatus
        rospy.init_node('bdd_smoke',disable_signals=True)
        package=Path(rospkg.RosPack().get_path('asmc_camera'))
        uuid=roslaunch.rlutil.get_or_generate_uuid(None,False)
        roslaunch.configure_logging(uuid)
        counts=Counter();bad=Counter();statuses=deque(maxlen=256);latest_mosaic=[None];sent=set()
        views=('front','left','right')
        expected_hw={'front':(720,1280),'left':(480,640),'right':(480,640)}
        def probability(msg,view):
            counts[view]+=1
            if (msg.height,msg.width)!=expected_hw[view] or msg.encoding!='mono8':
                bad['shape/encoding:'+view]+=1
                return
            if msg.header.stamp.to_nsec() not in sent:bad['unknown_stamp:'+view]+=1
            if msg.step!=msg.width or len(msg.data)!=msg.height*msg.step:
                bad['invalid_payload:'+view]+=1
        subscribers=[rospy.Subscriber('/experiments/twinlite_bdd/'+v+'/lane/probability',Image,probability,callback_args=v,queue_size=1) for v in views]
        def mosaic(msg):
            counts['mosaic']+=1;latest_mosaic[0]=msg
            if msg.header.stamp.to_nsec() not in sent:bad['unknown_mosaic_stamp']+=1
        subscribers.append(rospy.Subscriber('/experiments/twinlite_bdd/debug/mosaic/compressed',CompressedImage,mosaic,queue_size=1))
        subscribers.append(rospy.Subscriber('/experiments/twinlite_bdd/status',InferenceStatus,
            lambda m:statuses.append(dict(prepared=m.prepared,pipeline_ready=m.pipeline_ready,mode=m.mode,reason=m.reason,session=m.producer_session_id)),queue_size=1))
        for launch in ('bdd_lane.launch','bdd_lane_view.launch'):
            parent=roslaunch.parent.ROSLaunchParent(uuid,[str(package/'launch'/launch)])
            parent.start();parents.append(parent)
        validation=Path('/data/dataset_versions/twinlite_morai_v6_road_marking_actor_negative/val.jsonl')
        first=json.loads(validation.open().readline());run=first['run_id'];frame=first['frame_id']
        rows=[json.loads(x) for x in validation.open()]
        rows={r['view']:r for r in rows if r['run_id']==run and r['frame_id']==frame}
        raw={v:cv2.imread('/data/'+rows[v]['image']) for v in views}
        if any(im is None for im in raw.values()):raise RuntimeError('missing fixture')
        jpeg={v:cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes() for v,im in raw.items()}
        paths={'front':'/image_jpeg/compressed','left':'/cam_left/image_jpeg/compressed','right':'/cam_right/image_jpeg/compressed'}
        pubs={v:rospy.Publisher(paths[v],CompressedImage,queue_size=1) for v in views}
        # Allow GPU load/warmup and demand subscriptions before the timed stream.
        deadline=time.monotonic()+40
        while not all(pub.get_num_connections()>=1 for pub in pubs.values()):
            if time.monotonic()>deadline:raise RuntimeError('inference input subscription timeout')
            time.sleep(.1)
        finish=time.monotonic()+a.seconds
        while time.monotonic()<finish:
            stamp=rospy.Time.now();sent.add(stamp.to_nsec())
            for view,pub in pubs.items():
                msg=CompressedImage();msg.header.stamp=stamp;msg.header.frame_id='camera_'+view;msg.format='jpeg';msg.data=jpeg[view];pub.publish(msg)
            time.sleep(1/15)
        state=master.getSystemState('/bdd_smoke_check')[2]
        public_publishers=[topic for topic,nodes in state[0] if topic.startswith('/perception/') or topic.startswith('/camera_semantic_inference/')]
        public_services=[name for name,nodes in state[2] if name.startswith('/perception/') or name.startswith('/camera_semantic_inference/')]
        image=latest_mosaic[0]
        if image is not None:
            decoded=cv2.imdecode(np.frombuffer(image.data,np.uint8),cv2.IMREAD_COLOR)
            if decoded is None:bad['invalid_mosaic']+=1
            else:cv2.imwrite(str(a.output/'mosaic.jpg'),decoded)
        result=dict(master_uri=uri,input_hz=15,duration_seconds=a.seconds,source_samples={v:r['sample_id'] for v,r in rows.items()},
                    received=dict(counts),validation_errors=bad,public_publishers=public_publishers,public_services=public_services,
                    prepared_seen=any(s['prepared'] for s in statuses),pipeline_ready_seen=any(s['pipeline_ready'] for s in statuses),
                    status=list(statuses),scope='Fixed RGB replay through real ROS/GPU/renderer; not live MORAI performance acceptance')
        result['pass']=all(counts[v]>10 for v in views) and counts['mosaic']>5 and result['prepared_seen'] and not result['pipeline_ready_seen'] and not (bad or public_publishers or public_services)
        (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='status'},indent=2))
        if not result['pass']:raise RuntimeError('BDD ROS smoke failed; see report')
    finally:
        for parent in reversed(parents):parent.shutdown()
        if core is not None:
            core.terminate()
            try:core.wait(timeout=10)
            except subprocess.TimeoutExpired:core.kill();core.wait()
        log.close()


if __name__=='__main__':main()
