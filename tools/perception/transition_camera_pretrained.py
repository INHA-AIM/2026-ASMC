#!/usr/bin/env python3
"""Durable epoch-closeout -> verified dataset -> fresh pretrained launch.

Run on the Docker host, not in a container with a mounted Docker socket. This
program never modifies old weights or auto-approves an incomplete dataset.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time


def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()


def valid_closeout(old, epoch=8):
 marker=old/'operator-closeout/_EVALUATED';result=old/'operator-closeout/result.json'
 if not marker.exists():return False
 if marker.read_text().strip()!=sha(result):raise ValueError('closeout marker identity mismatch')
 r=json.loads(result.read_text())
 if r['epoch']!=epoch or sha(r['checkpoint'])!=r['sha256']:raise ValueError('epoch checkpoint identity mismatch')
 return True


def valid_data(dataset, ready):
 marker=dataset/'_SUCCESS'
 if not marker.exists() or not ready.exists():return False
 meta=dataset/'pretrain.json';gate=json.loads(ready.read_text())
 if marker.read_text().strip()!=sha(meta) or gate['dataset_metadata_sha256']!=sha(meta):raise ValueError('dataset gate mismatch')
 if not gate.get('passed'):raise ValueError('dataset review not accepted')
 for path,digest in gate['required_files'].items():
  if sha(path)!=digest:raise ValueError('ready artifact changed: '+path)
 return True


def main(a):
 request=json.loads(a.request.read_text());old=Path(request['old_run']);new=Path(request['new_run']);dataset=Path(request['dataset'])
 state=new/'transition.json';deadline=time.monotonic()+request.get('wait_timeout_seconds',43200)
 def update(status,**kw):
  value=dict(status=status,updated_utc=datetime.now(timezone.utc).isoformat(),**kw)
  tmp=state.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(state)
 def docker(*parts):return subprocess.run(['docker',*map(str,parts)],check=True,capture_output=True,text=True).stdout
 try:
  while True:
   closed=valid_closeout(old);ready=valid_data(dataset,Path(request['ready_file']))
   if closed and ready:break
   if time.monotonic()>deadline:raise TimeoutError('new run not ready within authorized transition window')
   update('waiting',old_epoch8_evaluated=closed,new_dataset_verified=ready,new_training_started=False)
   time.sleep(20)
  for path,digest in request['frozen_files'].items():
   if sha(path)!=digest:raise ValueError('launch identity changed: '+path)
  existing=subprocess.run(['docker','inspect',request['new_container']],capture_output=True,text=True)
  if existing.returncode==0:raise ValueError('new container already exists; do not silently resume')
  info=json.loads(docker('inspect',request['old_container']))[0]
  cmd=' '.join(info['Config'].get('Cmd') or [])
  if str(old) not in cmd or 'run_camera_bbox_main.py' not in cmd:raise ValueError('old container ownership mismatch')
  if info['State']['Running']:
   update('stopping_evaluated_old_run')
   docker('stop','--time','10',request['old_container'])
  update('launching_original_pretrained')
  cid=docker(*request['docker_run_arguments']).strip()
  time.sleep(5)
  info=json.loads(docker('inspect',request['new_container']))[0]
  if not info['State']['Running']:raise RuntimeError('new container exited; inspect its preserved logs')
  update('launched',container_id=cid,new_training_started=True,
         initialization='original pretrained; 200-batch smoke discarded before main',dataset_metadata_sha256=sha(dataset/'pretrain.json'))
 except BaseException as exc:
  update('failed',error=type(exc).__name__+': '+str(exc),new_training_started=False)
  raise


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--request',type=Path,required=True)
 main(p.parse_args())
