#!/usr/bin/env python3
"""One entrypoint for provenance, CPU diagnostics, existing-weight and epoch audits.

Never trains or edits data. GPU phase requires explicit --gpu-idle-confirmed and
limits all child reevaluations together to one hour. Cached final predictions are
read-only. Human/visual review is recorded separately from automated checks.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import html
import json
from pathlib import Path
import subprocess
import sys
import time

from camera_data_audit import read_jsonl, save, sha, training_label
from evaluate_camera_bbox_pilot import summarize
from seg_measurements import iou, match_boxes

FOCUS = ('person', 'motorcycle')


def names_list(names):
    if isinstance(names, dict):
        keys = sorted(int(k) for k in names)
        if keys != list(range(len(names))):
            raise ValueError('class ID order is not contiguous')
        names = [names[k] if k in names else names[str(k)] for k in keys]
    return list(names)


def validate_names(names, expected):
    if names_list(names) != list(expected):
        raise ValueError('class ID/name order mismatch')


def checkpoint_epoch(path, record, stored_epoch):
    expected = 0 if Path(path).name == 'initial.pt' else int(Path(path).stem[len('epoch'):]) + 1
    if expected and (expected > record['completed_epochs'] or stored_epoch != expected - 1):
        raise ValueError('partial or incorrect checkpoint epoch')
    if not expected and stored_epoch != -1:
        raise ValueError('initial checkpoint epoch mismatch')
    return expected


def scoped_gt(row, scoped=True):
    values = []
    for index, a in enumerate(row['annotations']):
        label = training_label(a)
        if not label:
            continue
        w, h = a['xyxy'][2]-a['xyxy'][0], a['xyxy'][3]-a['xyxy'][1]
        long, short = (24, 12) if label in ('car', 'bus', 'truck') else (16, 8)
        if scoped and (max(w, h) < long or min(w, h) < short):
            continue
        values.append(dict(a, class_name=label, annotation_index=index, group=label))
    return values


def diagnose_frame(row, predictions, confidence=.4, scoped=True):
    """Class TP is authoritative. Geometry allocation is one-to-one diagnostic.

    Wrong-class vs low-confidence attribution follows class-aware valid matches,
    geometry at the fixed threshold, geometry below it, then residual IoU .1-.5.
    Predictions below saved inference floor .10 cannot be distinguished from no
    detection. No detector output can establish that a GT itself is wrong.
    """
    gt = scoped_gt(row, scoped)
    high = [dict(p, group=p['class_name']) for p in predictions if p['confidence'] >= confidence]
    low = [dict(p, group=p['class_name']) for p in predictions if .1 <= p['confidence'] < confidence]
    exact = match_boxes(gt, high, .5)
    result = {g: dict(category='correct', prediction=high[p], iou=v) for g, p, v in exact}
    used = {p for _, p, _ in exact}
    remaining = [g for g in range(len(gt)) if g not in result]
    candidates = [p for p in range(len(high)) if p not in used]
    for gs, ps, label in ((remaining, candidates, 'other_class'),):
        pairs = match_boxes([dict(gt[g], group='geometry') for g in gs],
                            [dict(high[p], group='geometry') for p in ps], .5)
        for g, p, v in pairs:
            result[gs[g]] = dict(category=label, prediction=high[ps[p]], iou=v)
            used.add(ps[p])
    remaining = [g for g in range(len(gt)) if g not in result]
    pairs = match_boxes([dict(gt[g], group='geometry') for g in remaining],
                        [dict(p, group='geometry') for p in low], .5)
    for g, p, v in pairs:
        result[remaining[g]] = dict(category='confidence_below_threshold', prediction=low[p], iou=v,
            low_prediction_same_class=low[p]['class_name'] == gt[remaining[g]]['class_name'])
    remaining = [g for g in range(len(gt)) if g not in result]
    candidates = [p for p in range(len(high)) if p not in used]
    pairs = match_boxes([dict(gt[g], group='geometry') for g in remaining],
                        [dict(high[p], group='geometry') for p in candidates], .1)
    for g, p, v in pairs:
        result[remaining[g]] = dict(category='box_localization_error', prediction=high[candidates[p]], iou=v)
    for g, a in enumerate(gt):
        result.setdefault(g, dict(category='position_missing_at_saved_floor', prediction=None, iou=0.))
        result[g].update(annotation_index=a['annotation_index'], gt_class=a['class_name'], gt_xyxy=a['xyxy'])
    return list(result.values())


def regression_pairs(row, before, after, confidence):
    """Compare both phases at one frozen threshold, including selection choices."""
    bs = {a['annotation_index']: a for a in diagnose_frame(row, before, confidence)}
    return [(bs[a['annotation_index']], a) for a in diagnose_frame(row, after, confidence)
            if a['gt_class'] in FOCUS and bs[a['annotation_index']]['category']=='correct'
            and a['category']!='correct']


def verify_predictions(rows, predictions, metadata, dataset_sha, config_sha, expected_weight=None):
    if metadata['manifest_sha256'] != dataset_sha or metadata['config_sha256'] != config_sha:
        raise ValueError('prediction configuration/manifest SHA mismatch')
    if expected_weight and metadata['weights_sha256'] != expected_weight:
        raise ValueError('wrong baseline/checkpoint SHA')
    by_id = {p['id']: p for p in predictions}
    if len(by_id) != len(predictions):
        raise ValueError('duplicate prediction image ID')
    if not {r['id'] for r in rows}.issubset(by_id):
        raise ValueError('missing evaluation image')
    for row in rows:
        p = by_id[row['id']]
        if p['image_sha256'] != row['image_sha256']:
            raise ValueError('image SHA mismatch')
        if bool(p['ego_mask_applied']) != (row.get('view') in ('front', 'left', 'right')):
            raise ValueError('unknown-view/mask policy mismatch')
    return by_id


def deterministic_review(train, seed, limit):
    motorcycle = [r for r in train if any(training_label(a) == 'motorcycle' for a in r['annotations'])]
    other = [r for r in train if r not in motorcycle and any(
        training_label(a) in ('car', 'bicycle', 'person') or
        any(k in str(a.get('original_label', '')).lower() for k in ('rider', 'vehicle', 'bike'))
        for a in r['annotations'])]
    groups = defaultdict(list)
    for r in other:
        groups[r['source']].append(r)
    for source in groups:
        groups[source].sort(key=lambda r: hashlib.sha256((str(seed)+r['id']).encode()).hexdigest())
    sample = []
    while len(sample) < limit and any(groups.values()):
        for source in sorted(groups):
            if groups[source] and len(sample) < limit:
                sample.append(groups[source].pop(0))
    return motorcycle, sample


def distribution(rows):
    result = {}
    for label in ('person', 'motorcycle', 'car', 'bicycle'):
        subset = [r for r in rows if any(training_label(a) == label for a in r['annotations'])]
        anns = [a for r in subset for a in r['annotations'] if training_label(a) == label]
        sizes = Counter('tiny' if min(a['xyxy'][2]-a['xyxy'][0], a['xyxy'][3]-a['xyxy'][1]) < 8
                        else 'small' if max(a['xyxy'][2]-a['xyxy'][0], a['xyxy'][3]-a['xyxy'][1]) < 32
                        else 'medium' if max(a['xyxy'][2]-a['xyxy'][0], a['xyxy'][3]-a['xyxy'][1]) < 96
                        else 'large' for a in anns)
        result[label] = dict(images=len(subset), objects=len(anns),
            sources=dict(Counter(r['source'] for r in subset)), sizes=dict(sizes),
            views=dict(Counter(r.get('view', 'unknown') for r in subset)),
            declared_complete=sum(bool(r.get('annotation_complete')) for r in subset),
            augmentation_families=len({(r['source'], r.get('augmentation_family') or r['id']) for r in subset}),
            split_groups=len({r['split_group'] for r in subset}),
            occlusion='unknown; not inferred from bbox size',
            scene_independence='augmentation family/group counts do not establish independent scenes')
    return result


def audit(args, cfg, dataset, paired, out):
    rows = list(read_jsonl(dataset/'manifest.jsonl'))
    eval_rows = [r for r in rows if r['split'] in ('selection', 'heldout')]
    manifest_sha, config_sha = sha(dataset/'manifest.jsonl'), sha(args.repo/cfg['pilot_config'])
    expected = names_list(json.loads((dataset/'summary.json').read_text())['classes'])
    validate_names(expected, cfg['classes'])
    for row in eval_rows:
        if sha(row['image']) != row['image_sha256']:
            raise ValueError('evaluation RGB changed')
    metadata, predictions, metrics, cases, cache_hashes = {}, {}, {}, [], {}
    for model in ('yolo26l', 'yoloe26l'):
        run = json.loads((paired/model/'run.json').read_text())
        validate_names(run['canonical_names'], expected)
        validate_names([n.replace(' ', '_') for n in run['text_names']], expected)
        if run['manifest_sha256'] != manifest_sha or run['config_sha256'] != config_sha:
            raise ValueError('training provenance mismatch')
        for width in (960, 1280):
            for phase in ('before', 'after'):
                key = '%s-%s-%s' % (model, phase, width)
                d = paired/key
                m = json.loads((d/'inference.json').read_text())
                ps = list(read_jsonl(d/'predictions.jsonl'))
                weight = (run['initial_weights_sha256'] if model == 'yolo26l' else
                          run['before_training_checkpoint']['sha256']) if phase == 'before' else run['checkpoints']['epoch15.pt']['sha256']
                p = verify_predictions(eval_rows, ps, m, manifest_sha, config_sha, weight)
                if metadata and m['mask_shas'] != next(iter(metadata.values()))['mask_shas']:
                    raise ValueError('cached evaluation mask SHA mismatch')
                allowed = {r['id'] for r in eval_rows}
                if m.get('temporal_manifest_sha256'):
                    if sha(dataset/'temporal-probe.jsonl') != m['temporal_manifest_sha256']:
                        raise ValueError('temporal manifest SHA mismatch')
                    allowed.update(r['id'] for r in read_jsonl(dataset/'temporal-probe.jsonl'))
                if set(p) != allowed:
                    raise ValueError('unexpected prediction membership')
                if m['front_width'] != width or m['side_shape'] != [480, 640]:
                    raise ValueError('input shape mismatch')
                metadata[key] = m
                cache_hashes[key] = {file:sha(d/file) for file in ('inference.json','predictions.jsonl','evaluation.json')}
                predictions[key] = p
                frozen = json.loads((d/'evaluation.json').read_text())['selection_confidence']
                metrics[key] = {}
                for split in ('selection', 'heldout'):
                    part = [r for r in eval_rows if r['split'] == split]
                    metrics[key][split] = {}
                    for threshold in sorted({.4, frozen}):
                        ds = [dict(id=r['id'], split=split, **a) for r in part
                              for a in diagnose_frame(r, p[r['id']]['predictions'], threshold) if a['gt_class'] in FOCUS]
                        metrics[key][split][str(threshold)] = dict(
                            classes={c: dict(Counter(a['category'] for a in ds if a['gt_class'] == c)) for c in FOCUS},
                            confusion=[dict(gt=g, prediction=n, count=v) for (g,n),v in Counter(
                                (a['gt_class'], a['prediction']['class_name']) for a in ds if a['category']=='other_class').items()],
                            exact=summarize(part, [p[r['id']] for r in part], threshold, pixel_scope=True),
                            all_gt=summarize(part, [p[r['id']] for r in part], threshold))
                    if phase == 'after':
                        before = predictions['%s-before-%s' % (model, width)]
                        for row in part:
                            for threshold in sorted({.4, frozen}):
                                for b, a in regression_pairs(row, before[row['id']]['predictions'],
                                                             p[row['id']]['predictions'], threshold):
                                    cases.append(dict(id=row['id'], split=split, model=model, width=width,
                                        threshold=threshold, before=b, after=a))
    train = [r for r in rows if r['split']=='train']
    motorcycle, risk = deterministic_review(train, cfg['seed'], cfg['additional_review_limit'])
    selected = {r['id']:r for r in motorcycle+risk}
    selected.update({r['id']:r for r in eval_rows if r['id'] in {c['id'] for c in cases}})
    save(out/'review-manifest.json', dict(motorcycle_ids=[r['id'] for r in motorcycle],
        risk_ids=[r['id'] for r in risk], regression_ids=sorted({c['id'] for c in cases}),
        rows=list(selected.values()), automated_check_is_not_visual_review=True))
    save(out/'cases.json', cases)
    save(out/'audit.json', dict(manifest_sha256=manifest_sha, config_sha256=config_sha,
        metadata=metadata, cache_hashes=cache_hashes, metrics=metrics, training_distribution=distribution(train),
        regression_comparisons=len(cases), regression_images=len({c['id'] for c in cases}),
        unknown_view_policy='no assumed mask', partial_gt_policy='recall only; unmatched detections never FP',
        confidence_floor=.1, thresholds_frozen=True, runtime_changes=False))
    print(json.dumps(dict(review_images=len(selected), regression_images=len({c['id'] for c in cases}),
                          distribution=distribution(train)), ensure_ascii=False))


def weights(args, cfg, dataset, paired, out):
    import inspect
    import torch
    import ultralytics
    from ultralytics.nn.tasks import BaseModel
    result = dict(torch=torch.__version__, ultralytics=ultralytics.__version__, models={})
    source = inspect.getsource(BaseModel._remap_cls_by_names)
    (out/'installed-class-remap.py.txt').write_text(source)
    result['remap_source_sha256'] = sha(out/'installed-class-remap.py.txt')
    pilot = json.loads((args.repo/cfg['pilot_config']).read_text())
    for name in ('yolo26l', 'yoloe26l'):
        run = json.loads((paired/name/'run.json').read_text())
        original = args.root/'data'/pilot['models'][name]
        initial = paired/name/'initial.pt'
        if sha(original) != run['initial_weights_sha256'] or sha(initial) != run['before_training_checkpoint']['sha256']:
            raise ValueError('initial weights SHA mismatch')
        a = torch.load(original, map_location='cpu'); b = torch.load(initial, map_location='cpu')
        a = a.get('ema') or a['model']; stored_epoch = b['epoch']; b = b.get('ema') or b['model']
        checkpoint_epoch(initial, run, stored_epoch)
        validate_names(b.names, cfg['classes'])
        sa, sb = a.state_dict(), b.state_dict()
        same, changed, missing = [], [], []
        for k,v in sb.items():
            if k not in sa or tuple(sa[k].shape) != tuple(v.shape):
                missing.append(k)
            elif torch.equal(sa[k].half(), v.half()): same.append(k)
            else: changed.append(k)
        rows = []
        # Inspect every terminal classification convolution, both one2many/one2one.
        for k,v in sb.items():
            if not (('.cv3.' in k or '.one2one_cv3.' in k) and k in sa and
                    v.ndim in (1,4) and v.shape[0]==16 and sa[k].shape[0]==len(a.names)):
                continue
            for label in cfg['classes']:
                if label not in names_list(a.names): continue
                si, ti = names_list(a.names).index(label), cfg['classes'].index(label)
                x,y = sa[k][si].half(), v[ti].half()
                rows.append(dict(tensor=k, label=label, original_id=si, target_id=ti,
                    shape_compatible=tuple(x.shape)==tuple(y.shape),
                    equal_fp16=torch.equal(x,y), max_abs=float((x.float()-y.float()).abs().max()) if x.shape==y.shape else None))
        checkpoints = []
        for epoch in cfg['completed_epochs']:
            p = initial if epoch==0 else paired/name/'fit/weights'/('epoch%d.pt' % (epoch-1))
            c = torch.load(p,map_location='cpu')
            completed=checkpoint_epoch(p,run,c['epoch']);model=c.get('ema') or c['model']
            validate_names(model.names,cfg['classes'])
            state=c.get('optimizer') or {};steps=[v.get('step') for v in state.get('state',{}).values() if 'step' in v]
            checkpoints.append(dict(completed_epochs=completed, path=str(p), sha256=sha(p),
                head=type(model.model[-1]).__name__,names=names_list(model.names),
                optimizer_step_max=max(float(s) for s in steps) if steps else None,
                ema_updates=c.get('updates'),amp_scale=(c.get('scaler') or {}).get('scale')))
            del c,model
        result['models'][name]=dict(original_sha256=sha(original),initial_sha256=sha(initial),
            original_head=type(a.model[-1]).__name__,initial_head=type(b.model[-1]).__name__,
            initial_names=names_list(b.names),text_names=run['text_names'],
            head_is_fused=getattr(b.model[-1],'is_fused',None),class_rows=rows,
            compatible_equal_tensors=len(same),compatible_changed_tensors=changed,non_directly_comparable_tensors=missing,
            checkpoints=checkpoints,initial_training_args_cls_remap=run['actual_args'].get('cls_remap'))
        del a,b,sa,sb
    save(out/'weights.json', result)
    print(json.dumps({n:dict(rows=len(m['class_rows']), unequal=sum(not r['equal_fp16'] for r in m['class_rows']),
        same=m['compatible_equal_tensors'],changed=len(m['compatible_changed_tensors'])) for n,m in result['models'].items()}))


def epochs(args, cfg, dataset, paired, out):
    if not args.gpu_idle_confirmed:
        raise ValueError('verify simulator and live inference stopped before GPU reevaluation')
    budget = min(args.gpu_budget_seconds, cfg['gpu_budget_seconds'])
    if budget<=0: raise ValueError('positive GPU budget required')
    started = time.monotonic(); jobs=[]
    save(out/'epoch-jobs.json',dict(status='running',budget_seconds=budget,jobs=jobs))
    for epoch in cfg['completed_epochs']:
        if epoch==16: continue  # Exact original final cache is verified in audit.
        for model in ('yolo26l','yoloe26l'):
            path = paired/model/'initial.pt' if epoch==0 else paired/model/'fit/weights'/('epoch%d.pt' % (epoch-1))
            dest = out/'epochs'/('%s-%02d' % (model,epoch))
            cmd=[sys.executable,str(args.repo/'tools/perception/evaluate_camera_bbox_pilot.py'),
                '--config',str(args.repo/cfg['pilot_config']),'--dataset',str(dataset),
                '--weights',str(path),'--output',str(dest),'--front-width','1280',
                '--baseline-policy','initialization','--fixed-confidence','.4','--skip-temporal']
            left=budget-(time.monotonic()-started)
            if left<=0:
                jobs.append(dict(model=model,epoch=epoch,status='budget_not_started'));continue
            if dest.exists(): raise ValueError('existing epoch output: preserve it and use a new experiment output')
            dest.parent.mkdir(parents=True,exist_ok=True)
            job=dict(model=model,epoch=epoch,command=cmd,weight_sha256=sha(path))
            t=time.monotonic()
            with (dest.parent/(dest.name+'.log')).open('w') as log:
                try:
                    r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=max(.01,left))
                    job.update(status='complete' if r.returncode==0 else 'failed',returncode=r.returncode)
                except subprocess.TimeoutExpired:job.update(status='budget_stopped_partial')
            job['wall_seconds']=time.monotonic()-t;jobs.append(job)
            save(out/'epoch-jobs.json',dict(status='running',budget_seconds=budget,jobs=jobs,
                elapsed_seconds=time.monotonic()-started))
    save(out/'epoch-jobs.json',dict(status='complete' if all(j['status']=='complete' for j in jobs) else 'incomplete',
        jobs=jobs,budget_seconds=budget,elapsed_seconds=time.monotonic()-started,training_performed=False))


def report(args, cfg, dataset, paired, out):
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    audit_data=json.loads((out/'audit.json').read_text())
    review=json.loads((out/'review-manifest.json').read_text()); by_id={r['id']:r for r in review['rows']}
    cases=json.loads((out/'cases.json').read_text())
    weights_data=json.loads((out/'weights.json').read_text()) if (out/'weights.json').exists() else {}
    epoch_jobs=json.loads((out/'epoch-jobs.json').read_text()) if (out/'epoch-jobs.json').exists() else {}
    eval_rows=[r for r in read_jsonl(dataset/'manifest.jsonl') if r['split'] in ('selection','heldout')]
    trajectory=[]
    for model in ('yolo26l','yoloe26l'):
        for epoch in cfg['completed_epochs']:
            d=paired/(model+'-after-1280') if epoch==16 else out/'epochs'/('%s-%02d' % (model,epoch))
            if not (d/'inference.json').exists(): continue
            if epoch!=16 and not any(j['model']==model and j['epoch']==epoch and j['status']=='complete'
                                    for j in epoch_jobs.get('jobs',[])): continue
            m=json.loads((d/'inference.json').read_text())
            wc=next(c for c in weights_data['models'][model]['checkpoints'] if c['completed_epochs']==epoch)
            p=verify_predictions(eval_rows,list(read_jsonl(d/'predictions.jsonl')),m,
                audit_data['manifest_sha256'],audit_data['config_sha256'],wc['sha256'])
            for split in ('selection','heldout'):
                part=[r for r in eval_rows if r['split']==split]
                diagnostics=[a for r in part for a in diagnose_frame(r,p[r['id']]['predictions'],.4) if a['gt_class'] in FOCUS]
                for label in FOCUS:
                    values=[a for a in diagnostics if a['gt_class']==label]
                    cs=Counter(a['category'] for a in values)
                    conf=[a['prediction']['confidence'] for a in values if a['prediction'] and a['iou']>=.5]
                    trajectory.append(dict(model=model,completed_epochs=epoch,split=split,gt_class=label,
                        gt=len(values),class_tp=cs['correct'],geometry_tp=cs['correct']+cs['other_class'],
                        categories=dict(cs),confusion=dict(Counter(a['prediction']['class_name'] for a in values
                            if a['category']=='other_class')),geometry_confidence_median=float(np.median(conf)) if conf else None,
                        checkpoint_sha256=wc['sha256'],optimizer_step_max=wc['optimizer_step_max'],
                        ema_updates=wc['ema_updates'],amp_scale=wc['amp_scale']))
    save(out/'trajectory.json',trajectory)
    if trajectory:
        canvas=Image.new('RGB',(1100,620),'white');draw=ImageDraw.Draw(canvas)
        for panel,label in enumerate(FOCUS):
            x0,y0=60+panel*550,60;draw.text((x0,y0-30),label+' heldout @ confidence .40',fill='black',font=font)
            draw.rectangle((x0,y0,x0+460,y0+460),outline='black')
            for percent in (0,25,50,75,100):
                y=y0+460-percent*4.6;draw.line((x0,y,x0+460,y),fill='#dddddd');draw.text((x0-35,y-7),str(percent),fill='black',font=font)
            for model,color in [('yolo26l','#c13436'),('yoloe26l','#0759a5')]:
                values=sorted((v for v in trajectory if v['model']==model and v['gt_class']==label and v['split']=='heldout'),key=lambda v:v['completed_epochs'])
                for metric,dashed in [('class_tp',False),('geometry_tp',True)]:
                    points=[(x0+v['completed_epochs']/16*460,y0+460*(1-v[metric]/v['gt'])) for v in values if v['gt']]
                    if len(points)>1:draw.line(points,fill=color,width=2 if dashed else 5)
                    for x,y in points:draw.ellipse((x-4,y-4,x+4,y+4),fill=color)
            for epoch in cfg['completed_epochs']:draw.text((x0+epoch/16*460-5,y0+467),str(epoch),fill='black',font=font)
        draw.text((60,565),'Red: YOLO26l | Blue: YOLOE | thick: exact class | thin: geometry (names ignored)',fill='black',font=font)
        canvas.save(out/'trajectory.png')
    pages=[]
    for group,ids in [('motorcycle-train',review['motorcycle_ids']),('risk-train',review['risk_ids']),('new-misses',review['regression_ids'])]:
        for start in range(0,len(ids),12):
            sheet=Image.new('RGB',(1800,1280),'#eef1f6');draw_sheet=ImageDraw.Draw(sheet);members=[]
            for i,id_ in enumerate(ids[start:start+12]):
                row=by_id[id_]
                if sha(row['image'])!=row['image_sha256']:raise ValueError('visual review source SHA mismatch')
                im=Image.open(row['image']).convert('RGB');d=ImageDraw.Draw(im)
                for a in row['annotations']:
                    label=training_label(a)
                    if not label:continue
                    color='#00ffff' if label in FOCUS else '#ffff00'
                    d.rectangle(a['xyxy'],outline=color,width=2)
                    d.text(tuple(a['xyxy'][:2]),str(a.get('original_label'))+'>'+label,fill=color,font=font)
                targets=[a for a in row['annotations'] if training_label(a) in FOCUS] or row['annotations']
                a=max(targets,key=lambda a:(a['xyxy'][2]-a['xyxy'][0])*(a['xyxy'][3]-a['xyxy'][1])) if targets else None
                crop=im
                if a:
                    x1,y1,x2,y2=a['xyxy'];pad=max(12,(x2-x1)*.3,(y2-y1)*.3)
                    crop=im.crop((max(0,int(x1-pad)),max(0,int(y1-pad)),min(im.width,int(x2+pad)),min(im.height,int(y2+pad))))
                x,y=(i%3)*600,(i//3)*320
                for pic,dx in [(im,0),(crop,300)]:
                    pic=pic.copy();pic.thumbnail((300,278));sheet.paste(pic,(x+dx,y+36))
                draw_sheet.text((x+3,y+2),'%d %s %s' % (start+i,id_[:10],row.get('view')),fill='black',font=font)
                draw_sheet.text((x+3,y+18),row['source'][-65:],fill='black',font=font)
                members.append(id_)
            filename='%s-%02d.jpg' % (group,start//12+1);sheet.save(out/filename,quality=93)
            pages.append(dict(group=group,file=filename,ids=members,sha256=sha(out/filename)))
    save(out/'review-sheets.json',pages)
    crop_items=[(by_id[id_],a) for id_ in review['motorcycle_ids'] for a in by_id[id_]['annotations']
                if training_label(a)=='motorcycle']
    crop_pages=[]
    for start in range(0,len(crop_items),48):
        sheet=Image.new('RGB',(1200,1440),'white');d=ImageDraw.Draw(sheet);members=[]
        for i,(row,a) in enumerate(crop_items[start:start+48]):
            raw=Image.open(row['image']).convert('RGB');x1,y1,x2,y2=a['xyxy']
            crop=raw.crop((max(0,x1-4),max(0,y1-4),min(raw.width,x2+4),min(raw.height,y2+4)))
            scale=min(145/crop.width,150/crop.height);crop=crop.resize((max(1,int(crop.width*scale)),max(1,int(crop.height*scale))))
            x,y=(i%8)*150,(i//8)*240;sheet.paste(crop,(x,y+40))
            d.text((x,y),str(start+i)+' '+row['id'][:6],fill='black',font=font)
            d.text((x,y+20),str(a['id'])+' '+a['original_label'],fill='black',font=font)
            members.append(dict(id=row['id'],annotation_id=a['id']))
        filename='motorcycle-crops-%02d.jpg' % (start//48+1);sheet.save(out/filename,quality=95)
        crop_pages.append(dict(file=filename,annotations=members,sha256=sha(out/filename)))
    save(out/'motorcycle-crops.json',crop_pages)
    # Every new before-TP -> after-FN case gets a full-resolution comparison.
    preds={}
    for model in ('yolo26l','yoloe26l'):
        for width in (960,1280):
            for phase in ('before','after'):
                key='%s-%s-%s' % (model,phase,width)
                preds[key]={p['id']:p for p in read_jsonl(paired/key/'predictions.jsonl')}
    case_html=[]
    for index,c in enumerate(cases):
        row=by_id[c['id']];images=[]
        for phase in ('before','after'):
            im=Image.open(row['image']).convert('RGB');d=ImageDraw.Draw(im)
            for a in scoped_gt(row):
                d.rectangle(a['xyxy'],outline='#00ffff',width=3)
                d.text(tuple(a['xyxy'][:2]),'GT '+a['class_name'],fill='#00ffff',font=font)
            key='%s-%s-%s' % (c['model'],phase,c['width'])
            threshold=c['threshold']
            for a in preds[key][row['id']]['predictions']:
                if a['confidence']<threshold:continue
                d.rectangle(a['xyxy'],outline='#ff3b85',width=2)
                d.text((a['xyxy'][0],a['xyxy'][1]+16),'%s %.3f' % (a['class_name'],a['confidence']),fill='#ff3b85',font=font)
            images.append(im)
        w=max(i.width for i in images);h=max(i.height for i in images)
        panel=Image.new('RGB',(2*w,h+30),'white');d=ImageDraw.Draw(panel)
        for i,im in enumerate(images):panel.paste(im,(i*w,30))
        d.text((5,5),'BEFORE / AFTER %.2f | %s %d | %s' % (c['threshold'],c['model'],c['width'],c['after']['category']),fill='black',font=font)
        filename='case-%04d.jpg' % index;panel.save(out/filename,quality=92)
        case_html.append('<details><summary>%s</summary><img loading="lazy" src="%s"></details>' %
            (html.escape('%d %s %s %s %s' % (index,c['id'][:10],c['split'],c['after']['gt_class'],c['after']['category'])),filename))
    table='<table><tr><th>Model</th><th>Epoch</th><th>Class</th><th>Exact TP/GT</th><th>Geometry TP</th><th>Wrong name</th></tr>'
    for v in trajectory:
        if v['split']!='heldout':continue
        table+='<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in [v['model'],v['completed_epochs'],v['gt_class'],
            '%d/%d'%(v['class_tp'],v['gt']),v['geometry_tp'],v['confusion']])+'</tr>'
    table+='</table>'
    stamp=time.strftime('%Y-%m-%d %H:%M:%S UTC',time.gmtime())
    links=''.join('<p><a href="%s">%s (%d)</a></p>' % (p['file'],p['file'],len(p['ids'])) for p in pages)
    links+=''.join('<p><a href="%s">%s (%d annotations)</a></p>' % (p['file'],p['file'],len(p['annotations'])) for p in crop_pages)
    (out/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>'+stamp+
        ' · person/motorcycle 회귀 진단</title><style>body{font:16px system-ui;margin:24px}img{max-width:100%}td,th{border:1px solid #aaa;padding:6px}table{border-collapse:collapse}details{margin:12px}</style>'+
        '<h1>'+stamp+' · 사람·이륜차 회귀 진단</h1><p>학습·운영 변경 없음. exact=클래스 일치; geometry=이름 무시 IoU≥0.5. partial GT는 recall만 평가. 신뢰도 .40 고정.</p>'+
        '<a href="../">전체 실험 목차</a>'+table+('<p><img src="trajectory.png"></p>' if trajectory else '')+
        '<h2>검수 시트 · 전체 영상과 최대 대상 crop</h2><p>cyan=person/motorcycle, yellow=다른 대상. 표본 생성과 실제 시각 검수 기록은 별도입니다.</p>'+links+
        '<h2>신규 누락 전체 비교</h2><p>cyan=GT, pink=예측. 위·아래 threshold 표시를 확인하세요. 같은 영상의 모델·해상도·threshold 대조도 포함합니다.</p>'+''.join(case_html)+'</html>',encoding='utf-8')
    losses={}
    for model in ('yolo26l','yoloe26l'):
        records=list(csv.DictReader((paired/model/'fit/results.csv').open()))
        losses[model]=[r for r in records if int(r['epoch']) in cfg['completed_epochs'] and int(r['epoch'])<=16]
    save(out/'learning-records.json',dict(records=losses,native_metrics_zero_are_placeholders=True,
        validation_disabled=True,training_loss_does_not_establish_generalization=True))
    save(out/'report.json',dict(generated_utc=stamp,trajectory=trajectory,review_sheets=len(pages),
        new_miss_comparisons=len(cases),new_miss_images=len(review['regression_ids']),
        status='diagnosis_only; not training/runtime acceptance',training_performed=False))
    print(json.dumps(dict(trajectory_rows=len(trajectory),sheets=len(pages),cases=len(cases))))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=('audit','weights','epochs','report'))
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path('config/experiments/camera_bbox_regression_v1.json'))
    p.add_argument('--output',type=Path)
    p.add_argument('--gpu-idle-confirmed',action='store_true')
    p.add_argument('--gpu-budget-seconds',type=int,default=3600)
    a=p.parse_args();a.repo=Path(__file__).resolve().parents[2]
    cfg=json.loads((a.repo/a.config).read_text());dataset=a.root/cfg['dataset'];paired=a.root/cfg['paired']
    out=a.output or a.root/cfg['output'];out.mkdir(parents=True,exist_ok=True)
    globals()[a.phase](a,cfg,dataset,paired,out)


if __name__=='__main__':
    main()
