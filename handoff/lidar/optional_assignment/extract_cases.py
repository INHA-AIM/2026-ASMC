#!/usr/bin/env python3
"""Extract costs only, without GT. Optional baseline output checks row coverage."""
import argparse,collections,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,required=True);p.add_argument('--baseline-output',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
assert not a.out.exists()
rows=[json.loads(l) for l in a.baseline_output.open()]
frames=collections.defaultdict(list)
for l in a.trace.open():
 e=json.loads(l)
 if e['ev']=='assoc' and 1085<=e['scan']<=1155:frames[e['scan']].append(e)
assert sorted(frames)==list(range(1085,1156))
cases=[]
for scan,rs in sorted(frames.items()):
 assert rows[scan-1]['idx']==scan-1
 assert len(rs)==len(rows[scan-1]['tracks'])
 assert {r['id'] for r in rs}=={r['id'] for r in rows[scan-1]['tracks']}
 assert all(r['gate']==3 for r in rs)
 m=max([c['j']+1 for r in rs for c in r['cands']]+[r['assigned_j']+1 for r in rs]+[0])
 cases.append({'scan':scan,'n':len(rs),'m':m,'rows':[[r['id'],r['assigned_j'] if any(c['j']==r['assigned_j'] and not c['gated'] for c in r['cands']) else -1,[[c['j'],c['cost']] for c in r['cands'] if not c['gated']]] for r in rs]})
a.out.write_text(json.dumps({'source_trace_sha256':hashlib.sha256(a.trace.read_bytes()).hexdigest(),'source_output_sha256':hashlib.sha256(a.baseline_output.read_bytes()).hexdigest(),'limits':'Rounded 9-significant-digit baseline costs; frozen states, not closed loop. All previous track rows checked against saved output. Entirely forbidden trailing columns may be omitted; they cannot affect accepted matches.','cases':cases},separators=(',',':'))+'\n')
