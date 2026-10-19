#!/usr/bin/env python3
"""E8/E22 comparison using the existing isolated B0 recorder and fusion path."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from xmlrpc.client import ServerProxy
import yaml
from run_b0_matrix import stop

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--phase',choices=['compare','engine'],required=True);a=p.parse_args();w=a.workspace
 if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11314':raise ValueError('isolated master 11314 required')
 out=w/('bags-'+a.phase);out.mkdir(exist_ok=False)
 c=yaml.safe_load((w/'config/experiments/camera_lidar_b0_v1.yaml').read_text());c['conditions'].update(ros_master_port=11314,target_hz=dict(twinlite=10.,yolo=15.,resnet18=10.))
 (out/'conditions.yaml').write_text(yaml.safe_dump(c))
 jobs=[(e,b,r) for r in (1,2) for b in ('primary','shadow') for e in ((8,22) if a.phase=='compare' else (8,))]
 results=[]
 for e,bag,repeat in jobs:
  label=f'e{e}-{bag}-r{repeat}';core=child=None
  with (out/(label+'.log')).open('w') as log,(out/(label+'.core.log')).open('w') as clog:
   try:
    try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/mission16')
    except OSError:pass
    else:raise RuntimeError('existing master; refusing takeover')
    core=subprocess.Popen(['roscore','-p','11314'],stdout=clog,stderr=subprocess.STDOUT,start_new_session=True)
    end=time.monotonic()+20
    while True:
     try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/mission16');break
     except OSError:
      if time.monotonic()>end:raise TimeoutError('master')
      time.sleep(.2)
    config=w/(f'epoch{e}-pt.yaml' if a.phase=='compare' else 'epoch8-engine.yaml')
    cmd=[sys.executable,str(w/'tools/perception/run_camera_lidar_b0.py'),'--config',str(out/'conditions.yaml'),
      '--phase','fusion','--profile','C1','--bag-id',bag,'--repeat',str(repeat),'--output',str(out/label),
      '--objects-config',str(config),'--objects-class-groups-config',str(w/'src/asmc_camera/config/camera/object_classes_mission16_v1.yaml'),
      '--objects-views','all','--objects-side-ego-mask','--objects-min-rate','9.5','--fusion-yolo-policy','unique',
      '--camera-class-profile','mission16_v1','--record-boxes']
    child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);rc=child.wait(timeout=180)
    report=json.loads((out/label/'report.json').read_text()) if (out/label/'report.json').exists() else {}
    results.append(dict(epoch=e,bag=bag,repeat=repeat,returncode=rc,report=str(out/label/'report.json'),passed=report.get('passed',False),failures=report.get('failures',[])))
   except Exception as ex:results.append(dict(epoch=e,bag=bag,repeat=repeat,error=str(ex),passed=False))
   finally:
    if child:stop(child)
    if core:stop(core)
    (out/'index.json').write_text(json.dumps(results,indent=2));print(json.dumps(results[-1]),flush=True)
   # A missing report or load failure must be fixed before spending time on repetitions.
   if not (out/label/'report.json').exists() or report.get('error'):break
if __name__=='__main__':main()
