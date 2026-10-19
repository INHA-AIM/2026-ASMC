import json,hashlib,argparse
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
from scipy.optimize import milp,Bounds,LinearConstraint
parser=argparse.ArgumentParser(description='Propose source-aware roles; never modifies datasets or starts training.')
parser.add_argument('--pool',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--allow-small-selection-dog',action='store_true');args=parser.parse_args()
if args.output.exists():raise ValueError('immutable proposal exists')
root=args.pool;groups=defaultdict(list)
for role in ['train','selection','heldout']:
 for line in (root/(role+'.jsonl')).open():
  r=json.loads(line);groups[r['split_group']].append(r)
keys=sorted(groups);N=len(keys);classes=['person','dog','deer','bicycle','car','motorcycle','bus','truck','bollard','box','concrete_barrier','steel_barricade','traffic_drum','plastic_barrier','traffic_cone','stroller'];roles=['train','selection','heldout'];old={g:groups[g][0]['split'] for g in keys}
counts=np.array([[sum(a.get('supervision')=='positive' and a.get('training_class')==c for r in groups[g] for a in r['annotations']) for c in classes] for g in keys])
weights=np.array([len(groups[g]) for g in keys]);low=np.zeros(3*N);high=np.ones(3*N)
for i,g in enumerate(keys):
 if old[g]=='heldout':low[3*i+2]=1
 else:high[3*i+2]=0
 for forced,role in [('capture_scenario_family:comp_sample','train'),('capture_scenario_family:comp_sample_b','selection')]:
  if g==forced:low[3*i+roles.index(role)]=1
A=[];lo=[];hi=[]
for i,g in enumerate(keys):
 z=np.zeros(3*N);z[3*i:3*i+3]=1;A.append(z);lo.append(1);hi.append(1)
for k in range(3):
 for c in range(16):
  z=np.zeros(3*N);z[k::3]=counts[:,c];A.append(z);lo.append(100 if k==0 else (10 if k==1 and classes[c]=='dog' and args.allow_small_selection_dog else 20));hi.append(np.inf)
# Maintain adequate Capture vehicle training; true model diversity still unknown.
for c,minimum in [('car',1000),('bus',1000),('truck',1000)]:
 z=np.zeros(3*N);z[0::3]=[counts[i,classes.index(c)] if any(r['source'].startswith('capture:') for r in groups[g]) else 0 for i,g in enumerate(keys)];A.append(z);lo.append(minimum);hi.append(np.inf)
objective=np.array([weights[i]*(0 if role==old[g] else 1)+ (weights[i]*.01 if role=='selection' else 0) for i,g in enumerate(keys) for role in roles])
sol=milp(objective,integrality=np.ones(3*N),bounds=Bounds(low,high),constraints=LinearConstraint(np.array(A),lo,hi),options={'time_limit':30})
if sol.x is None:
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(status='infeasible',solver_message=sol.message,selection_minimum=20,heldout_preserved=True,capture_vehicle_train_minimum=1000),indent=2));raise SystemExit(sol.message)
chosen={g:roles[int(np.argmax(sol.x[3*i:3*i+3]))] for i,g in enumerate(keys)}
summ=defaultdict(Counter)
for i,g in enumerate(keys):
 role=chosen[g];summ[role]['candidate_images']+=len(groups[g])
 for c,n in zip(classes,counts[i]):summ[role][c]+=int(n)
report=dict(selection_dog_underpowered=args.allow_small_selection_dog, original_constraints_infeasible='selection dog >=20; no solution while holding heldout fixed and maintaining Capture vehicle train coverage',status='source_aware_proposal_not_export_or_approved_evaluation',groups=chosen,moved={g:dict(before=old[g],after=chosen[g],images=len(groups[g])) for g in keys if old[g]!=chosen[g]},counts=dict(summ),heldout_membership_unchanged=True,train_validation_must_be_rebuilt_and_RGB_reviewed=True,current_v10_has_seen_new_selection=True,restart_pretrained_required_for_this_comparison=True,active_training_changed=False,solver_message=sol.message)
out=args.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2));print(json.dumps({k:report[k] for k in ['moved','counts']},ensure_ascii=False))
