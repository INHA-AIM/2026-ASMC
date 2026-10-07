#!/usr/bin/env python3
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import cv2,numpy as np,rospy,rosgraph
from sensor_msgs.msg import CompressedImage
from asmc_camera.msg import InferenceStatus

def main():
    parser=argparse.ArgumentParser(description='GPU bag smoke test for all debug image publishers; not rate acceptance.')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--bag',type=Path,default=Path('/data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag'))
    parser.add_argument('--start',type=float,default=44.)
    parser.add_argument('--duration',type=float,default=20.)
    args=parser.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    rosgraph.Master('/debug_profile_preflight').setParam('/use_sim_time',True)
    rospy.init_node('camera_debug_profile_check',disable_signals=True)
    topics={'mosaic':'/perception/camera/debug/mosaic/compressed','bev':'/perception/bev/debug/comparison/compressed','yolo':'/perception/camera/objects/debug/image/compressed','resnet':'/perception/traffic_light/debug/image/compressed'}
    received={k:[] for k in topics};statuses={};processes=[];streams=[]
    subscribers=[]
    for key,topic in topics.items():
     subscribers.append(rospy.Subscriber(topic,CompressedImage,lambda m,k=key:received[k].append(m),queue_size=20,buff_size=16<<20))
    for key,node in [('twinlite','camera_semantic_inference'),('yolo','camera_object_detector'),('resnet','traffic_light_infer')]:
     subscribers.append(rospy.Subscriber('/'+node+'/inference_status',InferenceStatus,lambda m,k=key:statuses.update({k:m}),queue_size=5))
    def run(name,command):
     h=(out/(name+'.log')).open('w');streams.append(h)
     p=subprocess.Popen(command,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    report={'scope':'debug_profile_four_image_publishers_smoke_not_performance_acceptance','errors':[]}
    try:
     run('camera',['roslaunch','asmc_camera','perception.launch','profile:=debug'])
     for node in ('camera_semantic_inference','camera_object_detector','traffic_light_infer'):
      rospy.wait_for_service('/'+node+'/set_inference_mode',timeout=90)
     bag=run('bag',['rosbag','play','--clock','-s',str(args.start),'-u',str(args.duration),str(args.bag)])
     bag.wait(timeout=args.duration+30.);time.sleep(.25)
     report['images']={k:{'messages':len(v),'decodable':bool(v and cv2.imdecode(np.frombuffer(v[-1].data,np.uint8),1) is not None),'last_source_stamp_ns':v[-1].header.stamp.to_nsec() if v else None} for k,v in received.items()}
     report['model_modes']={k:v.mode for k,v in statuses.items()}
     report['all_active']=len(statuses)==3 and all(v.mode==2 for v in statuses.values())
     report['passed']=report['all_active'] and all(v['messages']>0 and v['decodable'] for v in report['images'].values())
    except Exception as exc:report.update(passed=False);report['errors'].append(str(exc))
    finally:
     for p in reversed(processes):
      if p.poll() is None:
       os.killpg(p.pid,signal.SIGINT)
       try:p.wait(timeout=15)
       except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
     for h in streams:h.close()
     (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    
    return 0 if report.get("passed") else 1

if __name__=="__main__":raise SystemExit(main())
