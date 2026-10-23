#!/usr/bin/env python3
import os,sys,json,subprocess,signal,time,xmlrpc.client,hashlib,argparse,importlib.util
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--data',required=True,type=Path);p.add_argument('--out',required=True,type=Path);p.add_argument('--setup',default=os.environ.get('LIDAR_SETUP'));p.add_argument('--inner',action='store_true');a=p.parse_args()
if not a.inner:raise SystemExit(subprocess.call(['unshare','--user','--map-root-user','--net','--pid','--fork','--mount-proc',sys.executable,__file__,*sys.argv[1:],'--inner']))
if not a.setup:raise SystemExit('Set LIDAR_SETUP to the compiled nested devel/setup.bash')
subprocess.run(['ip','link','set','lo','up'],check=True)
repo=Path(__file__).resolve().parents[2];a.data=a.data.resolve();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
manifest=json.loads((a.data/'MANIFEST.json').read_text())
for name,meta in manifest['files'].items():
 if hashlib.sha256((a.data/name).read_bytes()).hexdigest()!=meta['sha256']:raise SystemExit('Data hash mismatch: '+name)
import shlex
env=dict(x.decode().split('=',1) for x in subprocess.check_output(['bash','-c','source /opt/ros/noetic/setup.bash && source '+shlex.quote(a.setup)+' && env -0']).split(b'\0') if b'=' in x)
env.update(ROS_MASTER_URI='http://127.0.0.1:24789',ROS_IP='127.0.0.1',ROS_HOME=str(a.out/'ros'),PYTHONDONTWRITEBYTECODE='1')
env['ROS_PACKAGE_PATH']=str(repo/'src')+':'+str(repo/'src/lidar+sensorfusion/src')+':'+env.get('ROS_PACKAGE_PATH','')
env.pop('ROS_HOSTNAME',None)
import yaml
params=yaml.safe_load((Path(__file__).parent/'params.yaml').read_text())
for k,v in params.items():
 if isinstance(v,str) and v.startswith('@REPO@/'):params[k]=str(repo/v[len('@REPO@/'):])
param=a.out/'params.yaml';param.write_text(yaml.safe_dump(params))
prefix=Path(a.setup).resolve().parent
binary=prefix/'lib/Main/lidar_dataset_replay';library=prefix/'lib/libmyHeader.so'
cmd=[str(binary),'--bag',str(a.data/'inference.bag'),'--params',str(param),'--scene','actual_lap','--max-frames','1156','--out',str(a.out/'out.jsonl'),'--points-out',str(a.out/'points.bin'),'--track-trace',str(a.out/'trace.jsonl'),'--run-id','handoff_baseline']
for v in ['lidar_kf_reacquire_elapsed_init=true','lidar_kf_ego_translation_comp=true','lidar_yaw_ema_enabled=true','lidar_obb_long_axis_state=false','lidar_association_innovation_gate=false','lidar_point_support_ownership=false','lidar_observation_quality_r=false','lidar_observation_obb_center=false','lidar_kf_jerk_q=false','lidar_ego_gap_reset=false','lidar_kf_two_point_init_cov=false','planner_udp_enabled=false','imu_yaw_is_true_enu=true']:cmd+=['--set',v]
log=(a.out/'roscore.log').open('w');master=subprocess.Popen(['roscore','-p','24789'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 for _ in range(100):
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/probe')[0]==1:break
  except Exception:time.sleep(.1)
 with (a.out/'run.log').open('w') as f:subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
finally:os.killpg(master.pid,signal.SIGINT);master.wait(timeout=20);log.close()
def rows(path):
 r=[json.loads(x) for x in path.open()]
 for x in r:x.pop('timing',None)
 return r
old=rows(a.data/'expected.jsonl');new=rows(a.out/'out.jsonl');different=[i for i,(x,y) in enumerate(zip(old,new)) if x!=y]
spec=importlib.util.spec_from_file_location('ev',repo/'tools/experiments/lidar_obb_quality_temporal/eval_lib.py');ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
contract=json.loads((repo/'tools/experiments/lidar_obb_quality_temporal/EVAL_CONTRACT.json').read_text())
original=ev.segment_gt
def segment(gt,frames,c):
 x=original(gt,frames,c)
 if 'gt_generation' in x:x['seg']=x.seg+'_g'+x.gt_generation.astype(int).astype(str)
 return x
ev.segment_gt=segment
metrics={}
for arm,pred,points in [('new',a.out/'out.jsonl',a.out/'points.bin'),('expected',a.data/'expected.jsonl',a.out/'points.bin')]:
 sample=ev.eval_scene(contract,str(a.data/'evaluation'),str(pred),'actual_lap',str(points));metrics[arm]=ev.compute_metrics([sample],contract)
 tp=sample['tp'];z=tp[tp.idx.between(1086,1155)];v=z[(z.kfc>=2)&ev.np.isfinite(z[['pvx','pvy','gvx','gvy']]).all(axis=1)]
 errors=ev.np.hypot(v.pvx-v.gvx,v.pvy-v.gvy)
 metrics[arm]['event_window_1086_1155']={'TP':len(z),'velocity_n':len(v),'velocity_mae':float(errors.mean()) if len(v) else None,'velocity_max':float(errors.max()) if len(v) else None,'definition':'evaluation-only slice; inference always replays full prefix0..1155'}

(a.out/'evaluation.json').write_text(json.dumps(metrics,indent=2))
result={'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'binary_build_source':'a6fcf88294db2e44fe303580e54c1f3fcff9065e','command':cmd,'n_expected':len(old),'n_actual':len(new),'different_frames':different,'numeric_tolerance':0,'timing_excluded':True,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),'data_manifest_sha256':hashlib.sha256((a.data/'MANIFEST.json').read_bytes()).hexdigest(),'result':'PASS' if len(old)==len(new)==1156 and not different else 'FAIL','limits':'same exact outputs; evaluation reuses new unchanged detection point sets for expected comparison; no fresh dependency installation or MORAI/full-load test'}
(a.out/'RUN_MANIFEST.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
raise SystemExit(0 if result['result']=='PASS' else 1)
