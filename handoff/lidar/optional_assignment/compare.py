#!/usr/bin/env python3
"""One-binary OFF/ON replay. Completion is not acceptance."""
import argparse,json,os,subprocess,sys
from pathlib import Path
import numpy as np
import pandas as pd
HERE=Path(__file__).resolve().parent

def summary(df,s=''):
 mask=(df['kfc'+s]>=2)&np.isfinite(df[[c+s for c in ['pvx','pvy','gvx','gvy']]]).all(axis=1)
 d=df[mask];v=np.hypot(d['pvx'+s]-d['gvx'+s],d['pvy'+s]-d['gvy'+s])
 return {'n':len(v),'mae':float(v.mean()) if len(v) else None,'p95':float(np.percentile(v,95)) if len(v) else None,'p99':float(np.percentile(v,99)) if len(v) else None,'max':float(v.max()) if len(v) else None}

def compare(off,on):
 keys=['scene','idx','seg','gt_tid']
 if off.duplicated(keys).any() or on.duplicated(keys).any():raise ValueError('Duplicate GT/time keys')
 pair=off.merge(on,on=keys,suffixes=('_off','_on'),validate='one_to_one')
 mask=np.ones(len(pair),dtype=bool)
 for s in ['_off','_on']:mask&=(pair['kfc'+s]>=2)&np.isfinite(pair[[c+s for c in ['pvx','pvy','gvx','gvy']]]).all(axis=1)
 common=pair[mask].copy()
 for c in ['gvx','gvy']:
  if not np.array_equal(common[c+'_off'].to_numpy(),common[c+'_on'].to_numpy()):raise ValueError('GT differs at same key')
 objects=[{'seg':str(seg),'gt_tid':int(tid),'off':summary(d,'_off'),'on':summary(d,'_on')} for (seg,tid),d in common.groupby(['seg','gt_tid'])]
 return {'all_valid_speed':{'off':summary(off),'on':summary(on)},'common_valid_speed':{'off':summary(common,'_off'),'on':summary(common,'_on')},'tp_n':{'off':len(off),'on':len(on),'common':len(pair)},'objects':objects},common

def main():
 p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--setup',default=os.environ.get('LIDAR_SETUP'));a=p.parse_args()
 if not a.setup:p.error('Set LIDAR_SETUP or --setup to a fresh build')
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);manifests={};tables={};metrics={}
 for arm in ['off','on']:
  cmd=[sys.executable,str(HERE.parent/'run.py'),'--data',str(a.data.resolve()),'--out',str(a.out/arm),'--setup',str(Path(a.setup).resolve())]
  if arm=='on':cmd+=['--assignment-allow-unmatched']
  with (a.out/(arm+'_runner.log')).open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
  manifests[arm]=json.loads((a.out/arm/'RUN_MANIFEST.json').read_text());tables[arm]=pd.read_csv(a.out/arm/'tp.csv');metrics[arm]=json.loads((a.out/arm/'evaluation.json').read_text())['new']
 assert manifests['off']['result']=='PASS'
 assert manifests['on']['result']=='COMPLETED_NOT_ACCEPTED'
 for key in ['source_commit','binary_sha256','library_sha256','data_manifest_sha256']:
  if manifests['off'][key]!=manifests['on'][key]:raise RuntimeError('OFF/ON differ: '+key)
 if manifests['off']['source_dirty'] or manifests['on']['source_dirty']:raise RuntimeError('Dirty source; do not claim pinned commit replay')
 report,pairs=compare(tables['off'],tables['on']);pairs.to_csv(a.out/'COMMON_SPEED.csv',index=False)
 report.update(status='COMPLETED_NOT_ACCEPTED',source_commit=manifests['off']['source_commit'],all_metrics=metrics,limits='Development prefix0..1155, partial GT. No static/independent/full-load or fixed-support response verification. Fewer matched updates is not a success criterion.')
 (a.out/'COMPARISON.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'common_valid_speed':report['common_valid_speed']}))
if __name__=='__main__':main()
