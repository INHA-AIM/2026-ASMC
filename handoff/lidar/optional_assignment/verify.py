#!/usr/bin/env python3
import argparse,hashlib,json,os,subprocess,tempfile
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[2];SRC=REPO/'src/lidar+sensorfusion/src/Main/src'
def objective(c,a):
 js=[j for j in a if j>=0];assert len(js)==len(set(js));assert all(0<=c[i,j]<3 for i,j in enumerate(a) if j>=0)
 return sum(c[i,j] for i,j in enumerate(a) if j>=0)+1.5*(sum(c.shape)-2*len(js))
def reference(c):
 n,m=c.shape;b=np.full((n+m,n+m),1.5);b[:n,:m]=c;b[n:,m:]=0
 ii,jj=linear_sum_assignment(b);a=[-1]*n
 for i,j in zip(ii,jj):
  if i<n and j<m and c[i,j]<3:a[i]=int(j)
 return a
def brute(c):
 def visit(i,used,cost):
  if i==len(c):return cost+1.5*(c.shape[1]-len(used))
  options=[visit(i+1,used,cost+1.5)]
  for j in range(c.shape[1]):
   if j not in used and c[i,j]<3:options.append(visit(i+1,used|{j},cost+c[i,j]))
  return min(options)
 return visit(0,set(),0)
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--sanitize',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
 rng=np.random.default_rng(20261024)
 matrices=[np.array([[1e6,2.21594644],[2.36760306,.61019522]]),np.array([[.2,2.8],[2.7,.3]]),np.array([[2.99]]),np.full((3,2),1e6),np.full((4,5),1.5),np.empty((0,3)),np.empty((3,0))]
 for _ in range(600):
  n,m=rng.integers(1,7,2);c=rng.uniform(0,3,(n,m));c[rng.random((n,m))<.45]=1e6;matrices.append(c)
 fixture=json.loads((HERE/'frozen_cost_cases.json').read_text());fixed=[]
 for case in fixture['cases']:
  c=np.full((case['n'],case['m']),1e6)
  for i,(_,_,edges) in enumerate(case['rows']):
   for j,cost in edges:c[i,j]=cost
  fixed.append(c)
 jobs=[(1,c) for c in matrices]+[(mode,c) for c in fixed for mode in (0,1)]
 with tempfile.TemporaryDirectory() as tmp:
  tmp=Path(tmp);(tmp/'Global').mkdir();(tmp/'Global/Global.hpp').write_text('#include <vector>\n#include <cstdlib>\n#include <cfloat>\n')
  cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-I'+str(tmp),'-I'+str(SRC),str(HERE/'probe.cpp'),str(SRC/'Lidar/HungarianAlg/HungarianAlg.cpp'),'-o',str(tmp/'probe')]
  if a.sanitize:cmd+=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
  r=subprocess.run(cmd,capture_output=True,text=True);(a.out/'build.log').write_text(r.stdout+r.stderr);r.check_returncode()
  data=''.join(f'{mode} {c.shape[0]} {c.shape[1]}\n'+' '.join(format(float(x),'.17g') for x in c.flat)+'\n' for mode,c in jobs)
  r=subprocess.run([str(tmp/'probe')],input=data,capture_output=True,text=True,timeout=60);(a.out/'stderr.log').write_text(r.stderr);r.check_returncode()
  output=[[int(x) for x in l.split()][1:] for l in r.stdout.splitlines()];assert len(output)==len(jobs)
 for i,c in enumerate(matrices):
  assert len(output[i])==len(c);assert abs(objective(c,output[i])-objective(c,reference(c)))<1e-8
  if i<107:assert abs(objective(c,output[i])-brute(c))<1e-8
 assert output[0]==[-1,1]
 changes=[];counts=[0,0]
 for k,(case,c) in enumerate(zip(fixture['cases'],fixed)):
  off,on=output[len(matrices)+2*k:len(matrices)+2*k+2];assert off==[r[1] for r in case['rows']],case['scan']
  assert abs(objective(c,on)-objective(c,reference(c)))<1e-7
  diff=[{'id':r[0],'off':x,'on':y} for r,x,y in zip(case['rows'],off,on) if x!=y]
  if diff:changes.append({'scan':case['scan'],'changes':diff})
  counts[0]+=sum(x>=0 for x in off);counts[1]+=sum(x>=0 for x in on)
 files=[HERE/'verify.py',HERE/'probe.cpp',HERE/'frozen_cost_cases.json',SRC/'Lidar/Tracking/OptionalAssignment.hpp',SRC/'Lidar/HungarianAlg/HungarianAlg.cpp']
 result={'status':'PASS_SOLVER_ONLY','generated_cases':len(matrices),'exhaustive_objective_cases':107,'frozen_frames':len(fixed),'legacy_assignments_reproduced':True,'matched_edges_off_on':counts,'changed_frames':changes,'sanitizers':a.sanitize,'ASAN_OPTIONS':os.environ.get('ASAN_OPTIONS',''),'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],'source_sha256':{str(f.relative_to(REPO)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},'limits':'Real C++ solver compiled with STL-only Global.hpp shim; not ROS build. Frozen costs, no state feedback. No IDSW/speed/recall/runtime adoption claim.'}
 (a.out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['source_sha256','changed_frames']}))
if __name__=='__main__':main()
