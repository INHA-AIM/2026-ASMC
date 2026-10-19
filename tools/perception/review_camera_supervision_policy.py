#!/usr/bin/env python3
"""Read-only policy review. Candidate exports never replace active training/eval.

The 4px/64px2 rule is evaluated in native and square-960 coordinates. Range
requires a SHA-pinned spawn ledger, exact actor ID and a fresh request-time
Ego state. Semantic absence creates a review proposal, never an automatic
negative or an assertion that a scenario was unloaded.
"""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw

from analyze_camera_small_supervision import footprint, rejected
from camera_pretrain_policy import CLASSES, ROOT, load_policy, sha, transform_boxes, view_masks
from prepare_camera_pretrain import yolo_lines
from prepare_camera_spatial_background import semantic_grid
from prepare_camera_seg_gt import split_runs
sys.path.insert(0, str(ROOT/'src/data_collection/src'))
from data_collection.mask_baker import OFFICIAL_PALETTE_RGB

RULE = dict(tiny_short_px=4, tiny_area_px2=64, minimum_visible_pixels=4,
            scopes=['native', 'square_960_letterbox'], boundary='equal passes',
            treatment='ignore, never silently negative', range_cutoff_applied=False)
BASE_SAFE = ['asphalt', 'white_lane', 'yellow_lane', 'blue_lane', 'crosswalk',
             'stopline', 'sidewalk', 'sky']
EXTRA_SAFE = ['building', 'road_edge', 'road_sign', 'traffic_sign', 'traffic_light']
TARGET_SEMANTIC = ['vehicle','sedan','suv','truck','bus','van','wagon','mpv','pedestrian','obstacle']
CONDITIONS = {}


def read_rows(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip(): yield json.loads(line)


def save(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def save_rows(path, rows):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('w') as f:
        for r in rows: f.write(json.dumps(r, ensure_ascii=False)+'\n')


def weather(row):
    value = str(row.get('weather', 'unknown')).upper()
    if value in ('SUNNY','FOGGY'): return value
    source = row.get('source','').lower()
    if '_foggy_' in source: return 'FOGGY'
    if '_sunny_' in source: return 'SUNNY'
    run=row.get('provenance',{}).get('source_run') or (source[len('capture:'):] if source.startswith('capture:') else source)
    if run in CONDITIONS:return CONDITIONS[run]['weather']
    return 'unknown'


def recorded_conditions(data):
    found={}
    for p in sorted((data/'collection-evidence-20261012').glob('*.json')):
        if p.stat().st_size>4_000_000:continue
        try:j=json.loads(p.read_text())
        except (ValueError,OSError):continue
        for r in j.get('runs',[]) if isinstance(j,dict) else []:
            if not isinstance(r,dict):continue
            name=r.get('run_id');w=str(r.get('weather','')).upper()
            if name and w in ('SUNNY','FOGGY'):
                if name in found and found[name]['weather']!=w:raise ValueError('conflicting condition '+name)
                found[name]=dict(weather=w,hour=r.get('hour'),evidence=str(p),evidence_sha256=sha(p),
                    basis='recorded collection condition; not inferred from RGB appearance')
    return found


def band(distance):
    if distance is None: return 'unknown'
    return next((name for end,name in [(20,'0-20m'),(40,'20-40m'),(60,'40-60m')]
                 if distance < end), '60m+')


def palette_counts(semantic, ego):
    packed=(semantic[:,:,0].astype(np.uint32)+(semantic[:,:,1].astype(np.uint32)<<8)
            +(semantic[:,:,2].astype(np.uint32)<<16))
    values,counts=np.unique(packed[~ego],return_counts=True)
    return dict(zip(map(int,values),map(int,counts)))


def color_key(rgb):
    r,g,b=rgb
    return r+(g<<8)+(b<<16)


class RangeLinker:
    def __init__(self, data):
        self.data = data
        self.pinned = {}
        self.cache = {}
        for folder in ('collection-evidence-20261012','capture_layouts'):
            for p in (data/folder).rglob('*.jsonl'):
                if p.stat().st_size < 8_000_000: self.pinned.setdefault(sha(p), p)

    def state(self, row):
        p = row.get('provenance',{})
        expected = p.get('ledger_sha256')
        if not p.get('source_run') or not expected: return None
        run = Path(row['image']).parents[3]
        key = (str(run), expected)
        if key not in self.cache:
            ledger = run/'lineage/spawn-ledger.jsonl'
            if not ledger.is_file() or sha(ledger) != expected: ledger = self.pinned.get(expected)
            captures = run/'lineage/capture_manifest.jsonl'
            if ledger is None or not captures.is_file(): self.cache[key] = None
            else:
                actors = {}
                for a in read_rows(ledger):
                    if a.get('created') and a.get('placement_approved'):
                        for native in a.get('native_ids',[]):
                            actors[str(native)] = a.get('settled_location') or a.get('location')
                states = {str(c['sequence']):c for c in read_rows(captures) if c.get('success')}
                self.cache[key] = (actors, states)
        state = self.cache[key]
        if state is None: return None
        actors, states = state
        c = states.get(str(p.get('frame_id')),{}).get('state',{})
        age = c.get('ego_age_sec')
        loc = c.get('ego',{}).get('position')
        if age is None or not 0 <= age <= .2 or not loc: return None
        return actors, loc

    def distance(self, state, ann):
        if state is None: return None
        actors, loc = state
        obj = actors.get(str(ann.get('actor_id')))
        if not obj: return None
        return math.sqrt(sum((float(obj[k])-float(loc[k]))**2 for k in ('x','y','z')))


def proposed_row(row):
    r = deepcopy(row)
    for a in r['annotations']:
        if a.get('supervision')=='positive' and rejected(a['xyxy'],r['width'],r['height'],'short4_area64'):
            a.update(supervision='ignore', policy_review_previous_supervision='positive')
            a['supervision_reasons'] = list(a.get('supervision_reasons',[]))+['candidate_short4_area64']
    r['candidate_policy'] = RULE
    return r


def distribution(rows):
    counts = {c:Counter() for c in CLASSES}
    dimensions = {k:Counter() for k in ('weather','view','source','supervision')}
    for r in rows:
        dimensions['weather'][weather(r)]+=1
        for key in ('view','source','supervision'): dimensions[key][r.get(key,'unknown')]+=1
        present = set()
        for a in r['annotations']:
            c = a.get('training_class')
            if c not in counts: continue
            counts[c][a.get('supervision','unknown')]+=1
            if a.get('supervision')=='positive': present.add(c)
        for c in present: counts[c]['images']+=1
    groups = {c:len({r['split_group'] for r in rows if any(a.get('training_class')==c and a.get('supervision')=='positive'
              for a in r['annotations'])}) for c in CLASSES}
    return dict(images=len(rows), classes={c:dict(v,groups=groups[c]) for c,v in counts.items()}, dimensions=dimensions)


def balanced(rows, target=12, limit=300, correlation_cap=None):
    """Choose whole images, balancing class incidence and condition diversity.

    Never drop boxes, duplicate an image or move a train image into evaluation.
    Targets are aspirations, not fabricated per-class quotas.
    """
    pool = sorted(rows,key=lambda r:hashlib.sha256(('20261013:'+r['id']).encode()).hexdigest())
    labels = {r['id']:{a['training_class'] for a in r['annotations'] if a.get('supervision')=='positive'} for r in pool}
    freq=Counter(c for r in pool for c in labels[r['id']])
    counts=Counter();conditions=Counter();correlations=Counter();chosen=[]
    while pool and len(chosen)<limit:
        def score(r):
            names=labels[r['id']]
            gain=sum(max(0,target-counts[c])/max(1,min(freq[c],target)) for c in names)
            diversity=sum(1/(1+conditions[(c,weather(r),r['view'],r['source'])]) for c in names)
            variety=1/(1+correlations[r['correlation_group']])
            return gain*variety,diversity,-len(r['annotations'])
        available=[r for r in pool if correlation_cap is None or correlations[r['correlation_group']]<correlation_cap]
        if not available:break
        r=max(available,key=score)
        if score(r)[0]<=0:break
        pool.remove(r);chosen.append(r)
        correlations[r['correlation_group']]+=1
        for c in labels[r['id']]:
            counts[c]+=1;conditions[(c,weather(r),r['view'],r['source'])]+=1
    return chosen


def export_eval(path, rows):
    path.mkdir(parents=True)
    for r in rows:
        file=path/'labels'/(r['id']+'.txt');file.parent.mkdir(exist_ok=True)
        lines=yolo_lines(r)
        file.write_text('\n'.join(lines)+('\n' if lines else ''))
        # Preserve complete/partial and ignore separately from standard TXT.
        save(path/'supervision'/(r['id']+'.json'),dict(supervision=r['supervision'],annotations=r['annotations']))
        if file.read_text().splitlines()!=yolo_lines(r):raise ValueError('label roundtrip')
    save_rows(path/'manifest.jsonl', rows)
    save(path/'summary.json',distribution(rows))


def indexes(rows):
    return {key:{r[key] for r in rows if r.get(key)} for key in
            ('id','image_sha256','pixel_sha256','source_pixel_sha256','split_group','correlation_group')}


def assert_isolation(rows, denied):
    for r in rows:
        for key,values in denied.items():
            if r.get(key) in values:raise ValueError('train overlap '+key+': '+r['id'])
        if r.get('pixel_sha256') in denied['source_pixel_sha256'] or r.get('source_pixel_sha256') in denied['pixel_sha256']:
            raise ValueError('cross-field pixel train overlap')


def base(a):
    a.output.mkdir(parents=True,exist_ok=False)
    dataset=list(read_rows(a.dataset/'manifest.jsonl'))
    train=[r for r in dataset if r['split']=='train'];denied=indexes(train)
    linker=RangeLinker(a.data);counts=defaultdict(Counter);range_counts=defaultdict(Counter)
    case_candidates=defaultdict(list)
    for r in train:
        state=linker.state(r)
        for ann in r['annotations']:
            if ann.get('supervision')!='positive':continue
            drop=rejected(ann['xyxy'],r['width'],r['height'],'short4_area64')
            d=linker.distance(state,ann);b=band(d)
            for dim,value in [('class',ann['training_class']),('view',r['view']),('weather',weather(r)),('source',r['source'])]:
                counts[(dim,value)]['positive']+=1;counts[(dim,value)]['removed']+=drop
            for key in [('all',b),('class:'+ann['training_class'],b),('view:'+r['view'],b),('weather:'+weather(r),b)]:
                range_counts[key]['positive']+=1;range_counts[key]['removed']+=drop
            if drop and d is not None and d<60:
                key=ann['training_class']
                tb=transform_boxes([ann['xyxy']],r['width'],r['height'])[0]
                w,h=tb[2]-tb[0],tb[3]-tb[1]
                score=hashlib.sha256((r['id']+str(ann.get('id'))).encode()).hexdigest()
                case_candidates[key].append((score,r,ann,d,[float(w),float(h)]))
                case_candidates[key]=sorted(case_candidates[key],key=lambda x:x[0])[:3]
    report=dict(candidate_policy=RULE, dataset_manifest_sha256=sha(a.dataset/'manifest.jsonl'),
        dimensions=[dict(dimension=k[0],value=k[1],**v) for k,v in sorted(counts.items())],
        range=[dict(scope=k[0],band=k[1],**v) for k,v in sorted(range_counts.items())],
        range_definition='SHA-pinned exact actor ID; static pivot to request Ego age<=0.2s. Not optical depth, exposure-time ground truth or atmospheric visibility.',
        active_training_modified=False)
    save(a.output/'size-impact.json',report)
    cards=[];masks=view_masks(load_policy())
    for c,items in case_candidates.items():
        for i,(_,r,ann,d,wh) in enumerate(items):
            im=np.array(Image.open(r['image']).convert('RGB'))
            if r.get('ego_mask_applied'):im[masks[r['view']].geometry(r['height'],r['width'])[0]]=0
            im=Image.fromarray(im);draw=ImageDraw.Draw(im)
            draw.rectangle(ann['xyxy'],outline='#ff00ff',width=2)
            im.thumbnail((960,600));file=f'near-small-{c}-{i}.jpg';im.save(a.output/file)
            cards.append(dict(file=file,id=r['id'],annotation_id=ann.get('id'),class_name=c,range_m=d,
                              tensor_wh=wh,image_sha256=r['image_sha256'],source=r['source'],weather=weather(r),view=r['view']))
    save(a.output/'near-small-cases.json',cards)
    # Original val/test roles remain intact; all new exports are candidates.
    core=list(read_rows(a.validation/'manifest.jsonl'))
    assert_isolation(core,denied)
    by_id={r['id']:r for r in dataset if r['split']=='selection'}
    by_id.update({r['id']:r for r in core})
    selection=[proposed_row(r) for r in by_id.values()]
    test=[proposed_row(r) for r in dataset if r['split']=='heldout']
    assert_isolation(selection+test,denied)
    exports={}
    for role,pool in [('selection',selection),('test',test)]:
        export_eval(a.output/'evaluation-candidates'/(role+'-all'),pool)
        complete=[r for r in pool if r['supervision'] in ('complete','reviewed_negative')]
        chosen=balanced(complete)
        export_eval(a.output/'evaluation-candidates'/(role+'-balanced'),chosen)
        exports[role]=dict(all=distribution(pool),balanced=distribution(chosen),
                           missing_classes=[c for c in CLASSES if not any(x.get('training_class')==c and x.get('supervision')=='positive'
                               for r in chosen for x in r['annotations'])])
    shared=set(r['split_group'] for r in selection)&set(r['split_group'] for r in test)
    # Strict proposal: every group already touched by selection stays there.
    # Do not invent an unseen test by moving already-inspected selection images.
    strict_test=[r for r in test if r['split_group'] not in {x['split_group'] for x in selection}]
    export_eval(a.output/'evaluation-candidates/test-group-isolated',strict_test)
    save(a.output/'evaluation-policy.json',dict(candidate_policy=RULE,exports=exports,
        shared_selection_test_groups=sorted(shared),group_isolated_test=distribution(strict_test),
        existing_test_is_unseen=False,training_overlap=0,old_evaluation_preserved=True,
        note='Balanced test is descriptive only where selection already used its group. No reassignment can erase historical selection exposure; independent missing classes need reserved new groups. Unknown conditions are not guessed.'))
    save(a.output/'inputs.json',dict(dataset=str(a.dataset),validation=str(a.validation),job=str(a.job),
         dataset_sha256=sha(a.dataset/'manifest.jsonl'),validation_sha256=sha(a.validation/'manifest.jsonl'),
         created_utc=datetime.now(timezone.utc).isoformat(),candidate_policy=RULE))
    print(json.dumps(dict(stage='base',train_images=len(train),selection=len(selection),test=len(test),shared_groups=sorted(shared))),flush=True)


def frame_paths(row, data, cache):
    p=row.get('provenance',{});image=Path(row['image'])
    if p.get('source_run'):
        run=image.parents[3];key=(str(run),p.get('source_manifest_sha256'))
        if key not in cache:
            m=run/'manifest.jsonl'
            if sha(m)!=key[1]:
                matches=[x/'manifest.jsonl' for x in run.parent.glob(run.name+'-*')
                         if (x/'manifest.jsonl').is_file() and sha(x/'manifest.jsonl')==key[1]]
                if len(matches)!=1:raise ValueError('pinned frame manifest missing')
                m=matches[0]
            cache[key]={str(f['frame_id']):f for f in read_rows(m)}
        f=cache[key][str(p['frame_id'])]
        return run/f['paths']['semantic'][row['view']],run/f['paths']['instance'][row['view']],f
    if p.get('semantic'):return Path(p['semantic']),Path(p['instance']),None
    run=image.parents[3]
    return run/'frames/semantic'/row['view']/image.name,run/'frames/instance'/row['view']/image.name,None


def partition(a):
    """Group-separated developmental val/test proposal with explicit shortages.

    Historical selection exposure cannot be undone by repartitioning. This is
    a diagnostic split, never a newly unseen final test.
    """
    root=a.output/'evaluation-candidates'
    original=list(read_rows(root/'selection-all/manifest.jsonl'))
    original+=list(read_rows(root/'test-all/manifest.jsonl'))
    pool={}
    for r in original:
        if r['supervision'] not in ('complete','reviewed_negative'):continue
        # Selection's image-wide reviewed record takes precedence over the older
        # record for that same ID. Unrelated same-pixel label conflicts are held.
        pool.setdefault(r['id'],r)
    pixels={};conflicts=[];denied=set()
    for r in pool.values():
        key=r.get('source_pixel_sha256') or r['pixel_sha256']
        if key in pixels:
            old=pixels[key]
            if yolo_lines(old)!=yolo_lines(r):conflicts.append([old['id'],r['id']]);denied.update([old['id'],r['id']])
            else:denied.add(r['id'])
        else:pixels[key]=r
    pool=[r for r in pool.values() if r['id'] not in denied]
    group_rows=defaultdict(list)
    for r in pool:group_rows[r['split_group']].append(r)
    labels=lambda r:{x['training_class'] for x in r['annotations'] if x.get('supervision')=='positive'}
    group_counts={g:Counter(c for r in rs for c in labels(r)) for g,rs in group_rows.items()}
    remaining=Counter(c for r in pool for c in labels(r));test_counts=Counter();test_groups=set()
    while True:
        candidates=[]
        for g,cs in group_counts.items():
            if g in test_groups:continue
            if any(remaining[c]-n<min(6,remaining[c]+test_counts[c]) for c,n in cs.items()):continue
            gain=sum(min(n,max(0,6-test_counts[c])) for c,n in cs.items())
            if gain:candidates.append((gain,-sum(cs.values()),g))
        if not candidates:break
        g=max(candidates)[2];test_groups.add(g)
        remaining.subtract(group_counts[g]);test_counts.update(group_counts[g])
    val=balanced([r for r in pool if r['split_group'] not in test_groups],target=12)
    test=balanced([r for r in pool if r['split_group'] in test_groups],target=12)
    for role,rs in [('selection',val),('test',test)]:
        rs=deepcopy(rs)
        for r in rs:
            r['candidate_previous_split']=r['split'];r['split']='selection' if role=='selection' else 'heldout'
            r['evaluation_role']='group_separated_development_diagnostic_not_unseen'
        export_eval(root/(role+'-group-repartitioned'),rs)
    vg={r['split_group'] for r in val};tg={r['split_group'] for r in test}
    if vg&tg:raise ValueError('val/test group overlap')
    for k in ('id','image_sha256','pixel_sha256'):
        if {r.get(k) for r in val}&{r.get(k) for r in test}:raise ValueError('val/test '+k+' overlap')
    train=[r for r in read_rows(a.dataset/'manifest.jsonl') if r['split']=='train']
    assert_isolation(val+test,indexes(train))
    result=dict(selection=distribution(val),test=distribution(test),
        test_groups=sorted(tg),train_overlap=0,selection_test_group_overlap=0,
        same_pixel_conflicts_excluded=conflicts,duplicate_or_conflicting_rows_excluded=len(denied),
        classes_missing_test=[c for c in CLASSES if not test_counts[c]],
        status='candidate_development_split; historical exposure disclosed; not an unseen test',
        active_evaluation_changed=False)
    save(a.output/'group-repartition-proposal.json',result)
    print(json.dumps(dict(stage='partition',selection=len(val),test=len(test),missing=result['classes_missing_test'])),flush=True)


def resplit_group(row):
    """Conservative scenario families; never split weather rotations of a layout."""
    source=row['source'];group=row['split_group']
    if '_comp_sample_b_' in source:return 'capture_scenario_family:comp_sample_b'
    if '_comp_sample_' in source:return 'capture_scenario_family:comp_sample'
    # The six related Katri projects plus numeric-class jjj are conservatively
    # held together; this is not a claim every image in them is a duplicate.
    if source.startswith('roboflow:cone-z2vga/') or source=='roboflow:jjj-nx9di/katri_new_data-xkbev':
        return 'roboflow_family:katri_related_numeric_sources'
    return group


def resplit(a):
    from scipy.optimize import Bounds, LinearConstraint, milp
    source=list(read_rows(a.dataset/'manifest.jsonl'))
    existing={r['id']:r for r in source}
    # Prefer existing image-wide verified labels, preserving their lineage.
    existing.update({r['id']:r for r in read_rows(a.validation/'manifest.jsonl')})
    groups=defaultdict(list)
    for r in existing.values():groups[resplit_group(r)].append(proposed_row(r))
    keys=sorted(groups);n=len(keys);roles=('train','selection','heldout')
    weights=np.array([len(groups[g]) for g in keys],dtype=float)
    boxes=np.array([[sum(a.get('supervision')=='positive' and a.get('training_class')==c
                   for r in groups[g] for a in r['annotations']) for c in CLASSES] for g in keys],dtype=float)
    constraints=[];lo=[];hi=[]
    for i in range(n):
        z=np.zeros(3*n);z[3*i:3*i+3]=1;constraints.append(z);lo.append(1);hi.append(1)
    for role in range(3):
        z=np.zeros(3*n);z[role::3]=weights;constraints.append(z)
        lo.append(weights.sum()*(.6 if role==0 else .05));hi.append(weights.sum()*(1 if role==0 else .3))
        for c in range(len(CLASSES)):
            z=np.zeros(3*n);z[role::3]=boxes[:,c];constraints.append(z)
            lo.append(100 if role==0 else 20);hi.append(np.inf)
    # Minimize evaluation volume while ensuring whole-layout and 16-class
    # coverage. Counts do not certify complete GT or independent RGB diversity.
    objective=np.zeros(3*n)
    for i,g in enumerate(keys):
        for role in (1,2):
            tie=int(hashlib.sha256(f'20261013:{g}:{role}'.encode()).hexdigest()[:8],16)/2**32
            objective[3*i+role]=weights[i]+tie*.01
    solved=milp(objective,integrality=np.ones(3*n),bounds=Bounds(0,1),
        constraints=LinearConstraint(np.array(constraints),np.array(lo),np.array(hi)),
        options=dict(time_limit=30,mip_rel_gap=.001))
    if solved.x is None:
        save(a.output/'full-resplit-proposal.json',dict(status='infeasible_or_timeout',solver_message=solved.message,
            training_restarted=False));return
    chosen=np.argmax(solved.x.reshape(n,3),axis=1)
    partitions={r:[] for r in roles};mapping={}
    for i,g in enumerate(keys):
        role=roles[chosen[i]];mapping[g]=role
        for r in groups[g]:
            r['candidate_previous_split']=r['split'];r['candidate_original_split_group']=r['split_group']
            r['split']=role;r['split_group']=g
            r['candidate_train_eligible']=any(x.get('supervision')=='positive' for x in r['annotations']) or r['supervision']=='reviewed_negative'
            partitions[role].append(r)
    out=a.output/'full-resplit'
    out.mkdir(exist_ok=False)
    for role,rs in partitions.items():save_rows(out/(role+'.jsonl'),rs)
    # Check exact/pixel/lineage overlaps across all three prospective partitions.
    overlaps={}
    for i,left in enumerate(roles):
        for right in roles[i+1:]:
            overlaps[left+'|'+right]={k:len({r.get(k) for r in partitions[left] if r.get(k)}&
                {r.get(k) for r in partitions[right] if r.get(k)}) for k in
                ('id','image_sha256','pixel_sha256','source_pixel_sha256','split_group','correlation_group')}
    if any(v for checks in overlaps.values() for v in checks.values()):
        raise ValueError('full split exact/pixel/group overlap: '+str(overlaps))
    report=dict(status='proposal_only',rules=RULE,group_roles=mapping,overlaps=overlaps,
        splits={role:distribution(rs) for role,rs in partitions.items()},
        ignore_only_images={role:sum(not r['candidate_train_eligible'] for r in rs) for role,rs in partitions.items()},
        partial_images_needing_full16_review={role:sum(r['supervision']=='partial' for r in rs) for role,rs in partitions.items() if role!='train'},
        solver_message=solved.message,solver_gap=float(solved.mip_gap),
        class_minimum='train >=100 positive boxes; val/test >=20 each; these are counts, not quality acceptance',
        historical_evaluation_exposure=True,
        current_checkpoint_must_not_evaluate_new_split_as_unseen=True,
        restart_required_for_new_training_comparison=True,training_restarted=False,
        final_training_ready=False,
        required_before_restart=['review balanced val/test for all16 completeness; partial gives recall only',
            'complete background semantic/RGB review and final supervision contract',
            'recheck cross-source near-duplicate ancestry; exact hashes alone insufficient',
            'start a new run from original pretrained weights; never resume currently trained weights on new test'])
    save(a.output/'full-resplit-proposal.json',report)
    print(json.dumps(dict(stage='resplit',splits={k:len(v) for k,v in partitions.items()},
        nonempty16={k:all(any(a.get('training_class')==c and a.get('supervision')=='positive' for r in rs for a in r['annotations']) for c in CLASSES)
                    for k,rs in partitions.items()})),flush=True)


def panels(a):
    masks=view_masks(load_policy());result={}
    for role in ('selection','heldout'):
        pool=list(read_rows(a.output/'full-resplit'/(role+'.jsonl')))
        selected=balanced(pool,target=20,limit=300,correlation_cap=2)
        export_eval(a.output/'full-resplit'/(role+'-panel-diverse'),selected)
        cards=[];tiles=[];sheets=[]
        for i,r in enumerate(selected):
            if sha(r['image'])!=r['image_sha256']:raise ValueError('panel RGB SHA mismatch')
            im=np.array(Image.open(r['image']).convert('RGB'))
            if r.get('ego_mask_applied'):im[masks[r['view']].geometry(r['height'],r['width'])[0]]=0
            im=Image.fromarray(im);d=ImageDraw.Draw(im)
            for ann in r['annotations']:
                if ann.get('supervision') not in ('positive','ignore'):continue
                color='#00ff70' if ann['supervision']=='positive' else '#ff00ff'
                d.rectangle(ann['xyxy'],outline=color,width=2 if ann['supervision']=='positive' else 1)
                if ann['supervision']=='positive':d.text(tuple(ann['xyxy'][:2]),ann['training_class'],fill=color)
            im.thumbnail((1280,800));name=f'{role}-panel-diverse-{i:03d}.jpg';im.save(a.output/name,quality=90)
            thumb=im.copy();thumb.thumbnail((320,190));tile=Image.new('RGB',(334,224),'#202530');tile.paste(thumb,(0,0));td=ImageDraw.Draw(tile)
            td.text((0,191),f'{i:03d} {role} {weather(r)} {r["view"]}',fill='white')
            names=','.join(sorted({x['training_class'] for x in r['annotations'] if x.get('supervision')=='positive'}))
            td.text((0,205),names[:45],fill='white');tiles.append(tile)
            cards.append(dict(file=name,id=r['id'],image_sha256=r['image_sha256'],source=r['source'],
                split_group=r['split_group'],weather=weather(r),view=r['view'],classes=names,
                supervision=r['supervision'],review_decision='pending_image_wide_full16_review'))
        for offset in range(0,len(tiles),12):
            sheet=Image.new('RGB',(1002,896),'#202530')
            for j,t in enumerate(tiles[offset:offset+12]):sheet.paste(t,(j%3*334,j//3*224))
            name=f'{role}-panel-diverse-sheet-{offset//12:02d}.jpg';sheet.save(a.output/name,quality=90);sheets.append(name)
        result[role]=dict(distribution=distribution(selected),cards=cards,sheets=sheets,
            reviewed_complete_images=sum(r['supervision']=='complete' for r in selected),
            partial_images=sum(r['supervision']=='partial' for r in selected))
    save(a.output/'full-split-review-panels-diverse.json',result)
    print(json.dumps(dict(stage='panels',images={k:v['distribution']['images'] for k,v in result.items()})),flush=True)


def refine(a):
    current={r['id']:r for r in read_rows(a.dataset/'manifest.jsonl')}
    current_hashes={r[k] for r in current.values() for k in ('image_sha256','pixel_sha256','source_pixel_sha256') if r.get(k)}
    masks=view_masks(load_policy());valid_cache={};counts=Counter();negative_new=Counter();pixels=defaultdict(set)
    mapping=json.loads((a.output/'full-resplit-proposal.json').read_text())['group_roles']
    for r in read_rows(a.output/'background-scan.jsonl'):
        row=current.get(r.get('id'))
        if r.get('cells') and row and row['split']=='train':
            w,h=row['width'],row['height'];key=(w,h,r['view'])
            if key not in valid_cache:
                ego=masks[r['view']].geometry(h,w)[0]
                valid_cache[key]=semantic_grid(np.zeros((h,w,3),np.uint8),[],visible_known=np.ones((h,w),bool),ego=ego)
            valid=valid_cache[key];cells=r['cells']
            boxes=[x['xyxy'] for x in row['annotations'] if x.get('supervision') in ('positive','ignore')]
            boxes=footprint(transform_boxes(boxes,w,h),8)
            old_ignore=footprint(transform_boxes([x['xyxy'] for x in row['annotations'] if x.get('supervision')=='ignore'],w,h),8)
            old_pos=footprint(transform_boxes([x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive'],w,h),8)
            new_ignore=old_ignore|footprint(transform_boxes([x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive'
                 and rejected(x['xyxy'],w,h,'short4_area64')],w,h),8)
            new_pos=footprint(transform_boxes([x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive'
                 and not rejected(x['xyxy'],w,h,'short4_area64')],w,h),8)
            if row['supervision'] in ('complete','reviewed_negative'):
                before=int((valid&~old_ignore&~old_pos).sum())
                after=int((valid&~new_ignore&~new_pos).sum());unknown=0
            else:
                before=cells['current_geometric_background'];after=cells['proposed_geometric_background']
                unknown=cells['remaining_unknown']-int((~valid).sum())+int((boxes&~valid).sum())
            counts['images']+=1;counts['valid_cells']+=int(valid.sum())
            counts['current_background_cells']+=before;counts['proposed_background_cells']+=after
            counts['remaining_unknown_cells']+=max(0,unknown)
        if r.get('negative_candidate') and r.get('new_to_active_dataset'):
            family=resplit_group(dict(source='capture:'+r['run'],split_group='capture:'+r['run']))
            role=mapping.get(family,'unassigned')
            # Reservation follows the proposed whole-group assignment. Existing
            # current dataset duplicates remain excluded for a fresh split.
            if r['image_sha256'] in current_hashes or r['pixel_sha256'] in current_hashes:continue
            pixels[role].add(r['pixel_sha256']);negative_new[role]+=1
    save(a.output/'background-roi-refinement.json',dict(counts=counts,
        current_background_fraction=counts['current_background_cells']/max(1,counts['valid_cells']),
        proposed_background_fraction=counts['proposed_background_cells']/max(1,counts['valid_cells']),
        unknown_fraction=counts['remaining_unknown_cells']/max(1,counts['valid_cells']),
        denominator='conservative stride8 cells inside image excluding padding and Ego; positive box footprints excluded from background numerator',
        new_background_candidates_by_proposed_split=negative_new,
        unique_background_pixels_by_proposed_split={k:len(v) for k,v in pixels.items()},
        pending_rgb_review=True,recorded_conditions=CONDITIONS))
    # New condition counts are separate from the earlier frozen size census.
    cond=defaultdict(Counter)
    for r in current.values():
        if r['split']!='train':continue
        cond[weather(r)]['images']+=1
        for x in r['annotations']:
            if x.get('supervision')=='positive':
                cond[weather(r)]['positive']+=1
                cond[weather(r)]['removed']+=rejected(x['xyxy'],r['width'],r['height'],'short4_area64')
    save(a.output/'condition-coverage.json',dict(train=cond,registry=CONDITIONS,
        note='Declared collection conditions recovered from run records; unknown remains unknown. Original manifest weather fields unchanged.'))
    # Raw survey scope uses legacy Capture reservations for raw-only frames;
    # current training membership always comes from the active manifest.
    active_counts=Counter()
    for r in read_rows(a.output/'background-scan.jsonl'):
        row=current.get(r.get('id'))
        if r.get('cells') and row and row['split']=='train':
            for k,v in r['cells'].items():
                if isinstance(v,int):active_counts[k]+=v
            active_counts['images']+=1
            active_counts['images_with_added_background']+=r['cells']['added']>0
    bg=json.loads((a.output/'background-expansion.json').read_text())
    bg['existing_train_cells']=active_counts
    for name,key in [('current_geometric_background_fraction','current_geometric_background'),
                     ('proposed_geometric_background_fraction','proposed_geometric_background'),
                     ('remaining_unknown_fraction','remaining_unknown')]:
        bg[name]=active_counts[key]/max(1,active_counts['total'])
    bg['raw_survey_split_note']='Raw-only records use frozen legacy Capture roles; current training coverage uses actual active-manifest membership.'
    save(a.output/'background-expansion.json',bg)
    print(json.dumps(dict(stage='refine',roi_counts=counts)),flush=True)


def scan(a):
    cv2.setNumThreads(1)
    if not a.output.is_dir():raise ValueError('base stage required')
    policy=load_policy();masks=view_masks(policy);frame_cache={}
    rows=list(read_rows(a.dataset/'manifest.jsonl'))
    current={str(Path(r['image']).resolve()):r for r in rows}
    denied=indexes([r for r in rows if r['split']=='train'])
    eval_groups={r['split_group'] for r in rows if r['split']!='train'}
    eval_groups.update(r['split_group'] for r in read_rows(a.validation/'manifest.jsonl'))
    # Frozen 18-run roles, never shuffled again based on desired background count.
    legacy=[p for p in (a.data/'datasets').iterdir() if p.is_dir() and (p.name.startswith('kcity_') or '_comp_sample_' in p.name)
            and (p/'manifest.jsonl').is_file()]
    roles=split_runs([p.name for p in legacy])
    jobs={}
    for run in sorted(legacy):
        m=run/'manifest.jsonl';ms=sha(m)
        kind='competition_scenario' if '_comp_sample_' in run.name else 'user_reported_no_spawn_candidate'
        for frame in read_rows(m):
            for view in ('front','left','right'):
                paths=frame['paths']
                if not all(view in paths.get(k,{}) for k in ('intensity','semantic','instance')):continue
                rgb=(run/paths['intensity'][view]).resolve()
                role=roles[run.name]
                if 'capture:'+run.name in eval_groups:role='reserved_evaluation'
                jobs[str(rgb)]=dict(run=run.name,view=view,rgb=str(rgb),semantic=str(run/paths['semantic'][view]),
                    instance=str(run/paths['instance'][view]),frame=frame,frame_id=frame['frame_id'],
                    source_manifest=str(m),source_manifest_sha256=ms,split=role,kind=kind,row=current.get(str(rgb)))
    for r in rows:
        if not r['source'].startswith('capture:'):continue
        path=str(Path(r['image']).resolve())
        if path in jobs:continue
        sem,inst,f=frame_paths(r,a.data,frame_cache)
        jobs[path]=dict(run=r['source'].split(':',1)[1],view=r['view'],rgb=path,semantic=str(sem),instance=str(inst),
             frame=f,frame_id=r.get('provenance',{}).get('frame_id'),split=r['split'],kind='spawned_or_previously_curated',row=r)
    colors=[OFFICIAL_PALETTE_RGB[n] for n in TARGET_SEMANTIC]
    extra=[OFFICIAL_PALETTE_RGB[n] for n in EXTRA_SAFE]
    scanfile=a.output/'background-scan.jsonl'
    scan_rows=list(read_rows(scanfile)) if a.resume_scan and scanfile.is_file() else []
    completed={r['rgb'] for r in scan_rows}
    groups=defaultdict(list);aggregate=Counter()
    for r in scan_rows:
        aggregate[r['status']]+=1
        if r.get('negative_candidate'):
            aggregate['negative_candidates']+=1;groups[(r['run'],r['view'])].append(r)
    target_keys={color_key(c) for c in colors}
    known_keys={color_key(c) for c in OFFICIAL_PALETTE_RGB.values()}

    def process(job):
        out={k:v for k,v in job.items() if k not in ('row','frame')}
        out.update(status='checked',negative_candidate=False)
        try:
            sem=np.array(Image.open(job['semantic']).convert('RGB'))
            inst=Image.open(job['instance'])
            if inst.size!=(sem.shape[1],sem.shape[0]):raise ValueError('semantic instance shape mismatch')
            inst.verify()
            h,w=sem.shape[:2];ego=masks[job['view']].geometry(h,w)[0]
            histogram=palette_counts(sem,ego)
            out['target_semantic_pixels']=sum(v for k,v in histogram.items() if k in target_keys)
            out['candidate_expansion_pixels']={n:histogram.get(color_key(OFFICIAL_PALETTE_RGB[n]),0) for n in EXTRA_SAFE}
            out['unknown_palette_pixels']=sum(v for k,v in histogram.items() if k not in known_keys)
            row=job.get('row')
            if row:
                out.update(id=row['id'],supervision=row['supervision'],image_sha256=row['image_sha256'])
                if row.get('background_safe_grid'):
                    if sha(a.dataset/row['background_safe_grid'])!=row['background_safe_grid_sha256']:
                        raise ValueError('existing grid SHA mismatch')
                    with np.load(a.dataset/row['background_safe_grid']) as z:old=np.unpackbits(z['packed']).reshape(120,120).astype(bool)
                    if row['supervision'] in ('complete','reviewed_negative'):
                        old=np.ones_like(old) # Actual criterion includes padding/Ego negatives here.
                    more=semantic_grid(sem,extra,ego=ego) if any(out['candidate_expansion_pixels'].values()) else np.zeros_like(old)
                    oldboxes=[x['xyxy'] for x in row['annotations'] if x.get('supervision')=='ignore']
                    newboxes=oldboxes+[x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive' and
                         rejected(x['xyxy'],w,h,'short4_area64')]
                    positives=[x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive' and
                         not rejected(x['xyxy'],w,h,'short4_area64')]
                    ignore_old=footprint(transform_boxes(oldboxes,w,h),8)
                    ignore_new=footprint(transform_boxes(newboxes,w,h),8)
                    pos=footprint(transform_boxes(positives,w,h),8)
                    oldpos=footprint(transform_boxes([x['xyxy'] for x in row['annotations'] if x.get('supervision')=='positive'],w,h),8)
                    before=old&~ignore_old&~oldpos;after=(old|more)&~ignore_new&~pos
                    out['cells']=dict(total=old.size,current_geometric_background=int(before.sum()),
                        proposed_geometric_background=int(after.sum()),added=int((after&~before).sum()),
                        removed_by_new_ignore=int((before&~after).sum()),remaining_unknown=int((~(old|more|ignore_new|pos)).sum()),
                        limitation='stride8 conservative box-footprint census; actual assignment overrides and coarse-head gradients not measured')
            # No target-semantic pixels is independent of teacher confidence and
            # of whether a bbox file exists. RGB sample review is still pending.
            if not out['target_semantic_pixels']:
                rgb=Image.open(job['rgb']).convert('RGB')
                if rgb.size!=(w,h):raise ValueError('RGB GT shape mismatch')
                from capture_rgb_quality import validate_rendered_rgb
                validate_rendered_rgb(rgb)
                digest=sha(job['rgb'])
                pixels=hashlib.sha256(f'{w}x{h}:'.encode()+rgb.tobytes()).hexdigest()
                out.update(negative_candidate=True,image_sha256=digest,pixel_sha256=pixels,
                    semantic_sha256=sha(job['semantic']),instance_sha256=sha(job['instance']),
                    review_status='pending_rgb_group_review',new_to_active_dataset=job.get('row') is None,
                    forbidden_for_train=(job['split']!='train' or digest in denied['image_sha256'] or pixels in denied['pixel_sha256']))
            return out
        except (ValueError,OSError,KeyError) as exc:
            out.update(status='failed',reason=str(exc));return out

    pending=[j for k,j in jobs.items() if k not in completed]
    with scanfile.open('a' if a.resume_scan else 'w') as log,ThreadPoolExecutor(max_workers=a.workers) as pool:
        for i,r in enumerate(pool.map(process,pending)):
            log.write(json.dumps(r)+'\n');scan_rows.append(r);aggregate[r['status']]+=1
            if r.get('negative_candidate'):
                aggregate['negative_candidates']+=1;groups[(r['run'],r['view'])].append(r)
            if (i+1)%1000==0:log.flush();print(json.dumps(dict(stage='scan',checked=len(scan_rows),total=len(jobs),counts=aggregate)),flush=True)
    samples=[];sheets=[]
    for (run,view),pool in sorted(groups.items()):
        chosen=sorted(pool,key=lambda r:hashlib.sha256(('20261013:'+r['rgb']).encode()).hexdigest())[:min(12,len(pool))]
        sheet=Image.new('RGB',(1000,140*math.ceil(len(chosen)/5)),'#202530');d=ImageDraw.Draw(sheet)
        for i,r in enumerate(chosen):
            im=Image.open(r['rgb']).convert('RGB');im.thumbnail((196,112));x=i%5*200;y=i//5*140
            sheet.paste(im,(x,y));d.text((x+2,y+113),f'{i} {view} f={r["frame_id"]}',fill='white')
            samples.append(dict(run=run,view=view,frame_id=r['frame_id'],rgb=r['rgb'],image_sha256=r['image_sha256'],sheet=f'background-{run}-{view}.jpg',sample_index=i))
        name=f'background-{run}-{view}.jpg';sheet.save(a.output/name);sheets.append(name)
    candidate_rows=[r for r in scan_rows if r.get('negative_candidate')]
    unique={r['pixel_sha256']:r for r in candidate_rows if r['split']=='train' and r.get('new_to_active_dataset') and not r['forbidden_for_train']}
    totals=Counter();by_split=defaultdict(Counter);by_kind=defaultdict(Counter)
    for r in scan_rows:
        by_split[r['split']]['checked']+=1;by_kind[r['kind']]['checked']+=1
        if r.get('negative_candidate'):
            by_split[r['split']]['negative_candidates']+=1;by_kind[r['kind']]['negative_candidates']+=1
        if r.get('cells') and r['split']=='train':
            for k,v in r['cells'].items():
                if isinstance(v,int):totals[k]+=v
            totals['images']+=1;totals['images_with_added_background']+=r['cells']['added']>0
    save_rows(a.output/'negative-candidates.jsonl',candidate_rows)
    save(a.output/'background-samples.json',samples)
    save(a.output/'background-expansion.json',dict(counts=aggregate,by_split=by_split,by_kind=by_kind,
        new_train_candidates_before_exact_dedup=sum(r['split']=='train' and r.get('new_to_active_dataset') and not r['forbidden_for_train'] for r in candidate_rows),
        new_train_candidates_unique_pixels=len(unique),existing_train_cells=totals,
        current_geometric_background_fraction=totals['current_geometric_background']/max(1,totals['total']),
        proposed_geometric_background_fraction=totals['proposed_geometric_background']/max(1,totals['total']),
        remaining_unknown_fraction=totals['remaining_unknown']/max(1,totals['total']),
        proposed_additional_semantic_classes=EXTRA_SAFE,sheets=sheets,
        negatives_adopted=0,active_training_modified=False,
        note='Candidates pending RGB review; user no-spawn context supports run triage, not blanket negatives. Current-dataset overlap and evaluation reservations excluded from train additions. Expansion coverage is geometric stride8, not actual loss magnitude.'))
    print(json.dumps(dict(stage='scan_complete',checked=len(scan_rows),negative_candidates=len(candidate_rows),new_unique_train_candidates=len(unique))),flush=True)


def evaluate(a):
    from evaluate_camera_bbox_pilot import frame_counts
    from capture_size_scope import ignored_prediction_indices
    gt={r['id']:r for r in read_rows(a.validation/'manifest.jsonl')}
    linker=RangeLinker(a.data);result={};sem_cache={};frame_cache={}
    for r in gt.values():
        if not r['source'].startswith('capture:'):continue
        sem,_,_=frame_paths(r,a.data,frame_cache)
        if sem.is_file():sem_cache[r['id']]=np.array(Image.open(sem).convert('RGB'))
    lookup={tuple(rgb):name for name,rgb in OFFICIAL_PALETTE_RGB.items()}
    for folder in [a.job/'baseline-1280']+sorted(a.job.glob('epoch-*-1280')):
        file=folder/'predictions.jsonl'
        if not file.is_file():continue
        preds={r['id']:r for r in read_rows(file)}
        if set(preds)!=set(gt):raise ValueError('incomplete saved evaluation '+str(folder))
        model={}
        for policy in ('original','short4_area64'):
            metrics=defaultdict(Counter);classes=defaultdict(Counter);fp=Counter();semantic_fp=Counter()
            for old in gt.values():
                r=old if policy=='original' else proposed_row(old)
                item=preds[r['id']]
                if item['image_sha256']!=r['image_sha256']:raise ValueError('prediction RGB SHA mismatch')
                pp=[p for p in item['predictions'] if p['confidence']>=.4]
                counts,pairs=frame_counts(r,pp,.4)
                positive=[x for x in r['annotations'] if x.get('supervision')=='positive']
                matched={g for g,_,_ in pairs};matched_p={p for _,p,_ in pairs};state=linker.state(r)
                for i,ann in enumerate(positive):
                    b=band(linker.distance(state,ann))
                    metrics[b]['gt']+=1;metrics[b]['tp']+=i in matched
                    classes[ann['training_class']+'|'+b]['gt']+=1;classes[ann['training_class']+'|'+b]['tp']+=i in matched
                ignored=ignored_prediction_indices(pp,[x for x in r['annotations'] if x.get('supervision')=='ignore'],matched_p,dict(ignore_prediction_ioa=.5))
                if r['supervision'] in ('complete','reviewed_negative'):
                    fp['images']+=1;fp['ignored_predictions']+=len(ignored)
                    for i,p in enumerate(pp):
                        if i in matched_p or i in ignored:continue
                        fp['fp']+=1
                        fp['below_candidate_size']+=rejected(p['xyxy'],r['width'],r['height'],'short4_area64')
                        sem=sem_cache.get(r['id'])
                        if sem is not None:
                            x1,y1,x2,y2=p['xyxy'];cx=int(np.clip((x1+x2)/2,0,r['width']-1));cy=int(np.clip((y1+y2)/2,0,r['height']-1))
                            name=lookup.get(tuple(map(int,sem[cy,cx])),'unknown_palette')
                            category='current_safe' if name in BASE_SAFE else 'extra_background_candidate' if name in EXTRA_SAFE else 'target_or_unverified'
                            semantic_fp[category]+=1;semantic_fp['class:'+name]+=1
            for cc in (metrics,classes):
                for v in cc.values():v['recall']=v['tp']/v['gt'] if v['gt'] else None
            fp['fp_per_image']=fp['fp']/max(1,fp['images'])
            model[policy]=dict(range_recall=metrics,class_range_recall=classes,complete_image_false_positives=fp,
                 capture_fp_semantic_center=semantic_fp,
                 semantic_center_limitation='Diagnostic box-center attribution only; not a claim every pixel/feature cell in a prediction box is background.',
                 false_positive_range='not assigned: an unmatched prediction has no GT object distance')
        result[folder.name]=dict(policies=model,prediction_sha256=sha(file))
    save(a.output/'fixed-prediction-evaluation.json',dict(models=result,confidence=.4,models_reinferred=False,
        validation_sha256=sha(a.validation/'manifest.jsonl'),
        interpretation='Same saved RGB predictions, original versus proposed GT scope. Not retraining improvement; removed GT cannot improve the original-scope score. Range-unknown stays unknown.'))
    print(json.dumps(dict(stage='evaluate',models=list(result))),flush=True)


def report(a):
    size=json.loads((a.output/'size-impact.json').read_text());bg=json.loads((a.output/'background-expansion.json').read_text())
    ev=json.loads((a.output/'evaluation-policy.json').read_text());pred=json.loads((a.output/'fixed-prediction-evaluation.json').read_text())
    fresh=json.loads((a.output/'full-resplit-proposal.json').read_text())
    panels=json.loads((a.output/'full-split-review-panels-diverse.json').read_text())
    roi=json.loads((a.output/'background-roi-refinement.json').read_text())
    def table(headers,rows):
        return '<table><thead><tr>'+''.join('<th>'+html.escape(str(h))+'</th>' for h in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in r)+'</tr>' for r in rows)+'</tbody></table>'
    classes=[x for x in size['dimensions'] if x['dimension']=='class']
    doc='<html lang="ko"><meta charset="utf-8"><title>2026-10-13 · 4px/64px²·확실한 배경·평가 분포 검토</title><style>body{background:#19202b;color:#eee;font:16px system-ui;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #596679;padding:8px}a{color:#72c3ff}img{max-width:100%;margin:8px 0}p{max-width:1100px;line-height:1.7}</style><h1>2026-10-13 · 크기·배경·val/test 정책 검토</h1><p>후보 정책을 적용하기 전의 오프라인 분석. 현재 학습·데이터·기존 평가본은 변경하지 않았습니다. 4px/64px²는 원본과 960 tensor 모두 검사하며 미달 양성은 ignore 후보입니다. 거리 값은 actor 중심–요청 시점 Ego 근사이며 가시거리가 아닙니다.</p><a href="../index.html">갤러리 목차</a>'
    doc+='<h2>1. 전체 train 양성 영향</h2>'+table(['Class','현재 양성','제외 후보','비율'],[(x['value'],x['positive'],x['removed'],f"{100*x['removed']/x['positive']:.1f}%") for x in classes])
    doc+='<h2>2. 확실한 배경 확대 후보</h2><p>무스폰 수집 이력과 semantic GT를 함께 검토했습니다. 검출 부재나 BBox 파일 누락을 배경 증거로 사용하지 않았습니다. 아래 신규 후보와 추가 semantic 영역은 검수 전이며 아직 학습에 채택하지 않았습니다.</p>'+table(['항목','수치'],[(k,bg[k]) for k in ('new_train_candidates_before_exact_dedup','new_train_candidates_unique_pixels','current_geometric_background_fraction','proposed_geometric_background_fraction','remaining_unknown_fraction','negatives_adopted')])
    doc+='<p>위 전체 960 tensor 비율에는 padding이 포함됩니다. 실제 영상의 보수적인 feature cell만 분모로 잡고 차체와 padding을 빼면 다음과 같습니다.</p>'+table(['영상 영역 지표','비율'],[(k,f'{100*roi[k]:.2f}%') for k in ('current_background_fraction','proposed_background_fraction','unknown_fraction')])
    doc+='<p>새 전체 분할안에서는 신규 배경 후보가 '+html.escape(str(roi['unique_background_pixels_by_proposed_split']))+'장입니다. 현재 분할 기준 수와 다르며, 대상 semantic 부재를 확인한 제안이지 최종 채택이 아닙니다. 배경 이미지를 모두 추가해야 한다는 뜻도 아닙니다.</p>'
    doc+='<h2>3. 기존 예측의 거리별 recall·오검출</h2><p>confidence=0.40. 새 정책으로 GT 범위를 좁힌 점수와 기존 GT 점수를 함께 보존합니다. 미대응 예측에는 실제 거리가 없어 FP에 거리를 임의 부여하지 않았습니다.</p>'
    rows=[]
    for name,m in pred['models'].items():
        for policy,v in m['policies'].items():
            for b,c in v['range_recall'].items():rows.append([name,policy,b,c['gt'],c['tp'],f"{100*c['recall']:.1f}%"])
    doc+=table(['Model','GT policy','Range','GT','TP','Recall'],rows)
    doc+=table(['Model','GT policy','FP','FP/image'],[(name,policy,v['complete_image_false_positives']['fp'],round(v['complete_image_false_positives']['fp_per_image'],3)) for name,m in pred['models'].items() for policy,v in m['policies'].items()])
    doc+='<p>거리 연결이 없는 GT가 대부분이어서 위 거리별 수치로 16종의 60m 검출 성능을 수락할 수 없습니다. FP의 semantic 중심점 분석은 박스 전체나 실제 loss 영역의 배경 판정이 아닙니다.</p>'
    doc+='<h2>val/test 동일 기준·클래스 균형</h2><p>모든 대상 박스를 유지하고 이미지 단위로 균형을 조정합니다. 완전 주석만 precision 평가용 균형 묶음에 포함합니다. 기존 selection이 test의 두 그룹을 이미 사용해, 기존 test는 새로운 독립 시험으로 표시할 수 없습니다. 엄격하게 그룹을 분리한 test의 수량도 그대로 공개합니다.</p>'+table(['Class','Val images','Val positive','Test images','Test positive'],[(c,ev['exports']['selection']['balanced']['classes'][c].get('images',0),ev['exports']['selection']['balanced']['classes'][c].get('positive',0),ev['exports']['test']['balanced']['classes'][c].get('images',0),ev['exports']['test']['balanced']['classes'][c].get('positive',0)) for c in CLASSES])
    doc+='<p>기존 selection/test 공유 그룹: '+html.escape(', '.join(ev['shared_selection_test_groups']))+'. 그룹 격리 test: '+str(ev['group_isolated_test']['images'])+'장. 부족 클래스를 train에서 옮겨 채우지 않았습니다.</p>'
    doc+='<h2>전체를 재분배하는 새 분할 후보</h2><p>사용자 승인에 따라 새 출발을 위한 전체 분할안을 별도로 생성했습니다. 모든 날씨의 같은 스폰 layout, comp 시나리오 가족, 관련 숫자 라벨 Katri 소스를 한 그룹으로 유지했습니다. 현재 모델은 새 test 일부를 이미 학습했으므로 이 분할에서 독립 시험 모델로 사용할 수 없습니다. 새 학습은 원본 사전학습 가중치에서 시작해야 합니다.</p>'
    doc+=table(['분할 후보','전체 이미지','ignore만 남아 양성 학습에 넣지 않을 후보'],[(r,fresh['splits'][r]['images'],fresh['ignore_only_images'][r]) for r in ('train','selection','heldout')])
    doc+='<p>아래는 전체 평가 후보에서 클래스·조건·연속 구간 편중을 줄인 검수 패널입니다. 같은 correlation group은 최대 두 장이며 모든 대상 박스를 유지합니다. 아직 partial인 이미지는 전체 16종 누락 검수를 마치기 전 precision/F1에 사용하지 않습니다.</p>'
    doc+=table(['Class','새 train 양성','Val 패널 이미지/양성','Test 패널 이미지/양성'],[(c,
        fresh['splits']['train']['classes'][c].get('positive',0),
        str(panels['selection']['distribution']['classes'][c].get('images',0))+'/'+str(panels['selection']['distribution']['classes'][c].get('positive',0)),
        str(panels['heldout']['distribution']['classes'][c].get('images',0))+'/'+str(panels['heldout']['distribution']['classes'][c].get('positive',0))) for c in CLASSES])
    for role,panel in panels.items():
        doc+='<h3>'+role+' · '+str(panel['distribution']['images'])+'장</h3><p>날씨: '+html.escape(str(panel['distribution']['dimensions']['weather']))+' · 뷰: '+html.escape(str(panel['distribution']['dimensions']['view']))+' · partial '+str(panel['partial_images'])+'장</p>'
        for name in panel['sheets']:doc+='<a href="'+name+'"><img loading="lazy" src="'+name+'"></a>'
        doc+='<details><summary>개별 원본 크기 overlay</summary>'+''.join('<p><a href="'+r['file']+'">'+html.escape(r['file']+' | '+r['classes']+' | '+r['supervision'])+'</a></p>' for r in panel['cards'])+'</details>'
    doc+='<h2>60m 안에서 제외되는 작은 객체 사례</h2><p>자홍색은 새 기준에서 ignore 후보인 원본 양성입니다. 60m 안의 작은 사람·콘도 영향을 받으므로 근거리 recall 검토가 필요합니다.</p>'
    for r in json.loads((a.output/'near-small-cases.json').read_text()):doc+='<p>'+html.escape(f"{r['class_name']} · {r['range_m']:.1f}m · tensor {r['tensor_wh']} · {r['weather']} · {r['view']}")+'</p><img loading="lazy" src="'+r['file']+'">'
    doc+='<h2>배경 후보 표본</h2><p>날씨·run·뷰별 고정 seed 표본. 원본 경로·SHA·프레임은 background-samples.json에 있습니다.</p>'
    for name in bg['sheets']:doc+='<p>'+html.escape(name)+'</p><img loading="lazy" src="'+name+'">'
    doc+='<h2>재현 기록</h2>'+''.join('<p><a href="'+file+'">'+file+'</a></p>' for file in ['inputs.json','size-impact.json','background-expansion.json','background-roi-refinement.json','condition-coverage.json','evaluation-policy.json','group-repartition-proposal.json','full-resplit-proposal.json','full-split-review-panels-diverse.json','fixed-prediction-evaluation.json','near-small-cases.json','background-samples.json'])+'</html>'
    (a.output/'index.html').write_text(doc)
    print(json.dumps(dict(stage='report',output=str(a.output))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['base','partition','resplit','panels','refine','scan','evaluate','report'])
    p.add_argument('--data',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--validation',type=Path,required=True);p.add_argument('--job',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--resume-scan',action='store_true')
    a=p.parse_args();CONDITIONS.update(recorded_conditions(a.data));globals()[a.stage](a)
