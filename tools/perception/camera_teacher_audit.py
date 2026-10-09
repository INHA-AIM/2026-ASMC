#!/usr/bin/env python3
"""Offline X-teacher inference and heldout-gated label suggestions, never GT edits.

Reuses checkpoint-bound camera backend, view masks and cardinality-first matcher.
Incomplete GT may be rendered/inferred, but cannot produce teacher precision.
"""
import argparse
from collections import Counter
import html
import json
from pathlib import Path
import time

from camera_data_audit import (load_label_policy, label_metadata, read_jsonl, save,
    sha, training_names, training_label, write_jsonl)


def complete_gt(row, policy, scope=None):
    from capture_size_scope import partition
    verified_scope = row.get('review_scope') == policy.get('review_scope') or set(training_names(policy)) <= set(row.get('verified_target_classes', []))
    main,_=partition(row['annotations'],scope)
    return bool(verified_scope and row.get('annotation_complete') and not row.get('issues')
                and all(not a.get('issues') and a.get('disposition') != 'ambiguous'
                        and not a.get('actor_review',{}).get('human_confirmation_pending') for a in main))


def annotated_localization(rows, predictions, policy, confidence, scope=None):
    """Partial GT diagnostic: only confirmed annotations; no precision or approval."""
    from seg_measurements import match_boxes
    from capture_size_scope import partition
    counts={n:Counter(gt=0,matched=0) for n in training_names(policy)}
    for row in rows:
        gt=[label_metadata(dict(a),policy) for a in row['annotations'] if a.get('canonical') and not a.get('issues')]
        gt,ignored=partition(gt,scope)
        pred=[p for p in predictions[row['id']]['predictions'] if p['confidence']>=confidence]
        pairs=match_boxes([dict(a,group=a['training_class']) for a in gt],[dict(a,group=a['training_class']) for a in pred],.5)
        for a in gt:counts[a['training_class']].update(gt=1)
        for a in ignored:counts[a['training_class']].update(ignored_gt=1)
        for r,_,_ in pairs:counts[gt[r]['training_class']].update(matched=1)
    return dict(confidence=confidence,classes={n:dict(c,annotated_recall=c['matched']/c['gt'] if c['gt'] else None) for n,c in counts.items()},
        precision=None,teacher_approval=False,interpretation='Partial annotation diagnostic, including independently inspected actor candidates still pending human confirmation. Not dataset recall/precision or teacher acceptance.')


def prediction_label(prompt, policy):
    from camera_data_audit import classify
    canonical, disposition = classify(prompt, policy)
    if disposition != 'candidate':
        raise ValueError('unmapped teacher prompt: ' + prompt)
    return label_metadata(dict(canonical=canonical, source_prompt=prompt), policy)


def metrics(rows, predictions, policy, thresholds, level='training_class', scope=None):
    from seg_measurements import match_boxes
    from capture_size_scope import partition, ignored_prediction_indices
    counts = {name: Counter(gt=0, predictions=0, tp=0, fp=0, fn=0) for name in training_names(policy)}
    if level == 'consumer_group':
        counts = {g: Counter(gt=0, predictions=0, tp=0, fp=0, fn=0) for g in policy['groups']}
    elif level == 'source_semantic_label':
        counts = {}
    evaluated = 0;ignored_untyped=0
    for row in rows:
        if not complete_gt(row, policy,scope):
            continue
        if row['id'] not in predictions:
            raise ValueError('missing prediction for complete GT: ' + row['id'])
        evaluated += 1
        gt = [label_metadata(dict(a), policy) for a in row['annotations'] if a.get('canonical') or a.get('disposition')=='ambiguous']
        raw = predictions[row['id']]['predictions']
        pred = [dict(p) for p in raw if p['confidence'] >= thresholds.get(p['training_class'], 1.01)]
        gt = [dict(a, group=a[level]) for a in gt]
        pred = [dict(a, group=a[level]) for a in pred]
        gt,ignored=partition(gt,scope)
        pairs = match_boxes(gt, pred, .5)
        ignored_indices=ignored_prediction_indices(pred,ignored,{c for _,c,_ in pairs},scope)
        for a in gt: counts.setdefault(a['group'], Counter()).update(gt=1)
        for index,a in enumerate(pred):
            counts.setdefault(a['group'],Counter()).update(**{'ignored_predictions' if index in ignored_indices else 'predictions':1})
        for a in ignored:
            if a['group'] is None:ignored_untyped+=1
            else:counts.setdefault(a['group'],Counter()).update(ignored_gt=1)
        for r, c, _ in pairs: counts[gt[r]['group']].update(tp=1)
    for name, c in counts.items():
        tp, n, total = c['tp'], c['predictions'], c['gt']
        c.update(fp=n-tp, fn=total-tp)
        counts[name] = dict(c, precision=tp/n if n else None, recall=tp/total if total else None,
                           evaluable=bool(total))
    result=dict(evaluated_frames=evaluated, excluded_frames=len(rows)-evaluated, classes=counts)
    if scope is not None:result['ignored_untyped_gt']=ignored_untyped
    return result


def analyze(gt, predictions, policy, config, scope=None):
    predictions = list(predictions)
    by_id = {r['id']: r for r in predictions}
    if len(by_id) != len(predictions):
        raise ValueError('duplicate teacher frame')
    gt_ids = {r['id'] for r in gt}
    if len(gt_ids) != len(gt) or set(by_id) != gt_ids:
        raise ValueError('teacher/GT frame membership mismatch')
    for row in gt:
        if row['image_sha256'] != by_id[row['id']]['image_sha256']:
            raise ValueError('teacher/GT image SHA mismatch')
    selection = [r for r in gt if r.get('split', r.get('frozen_split')) == 'selection']
    heldout = [r for r in gt if r.get('split', r.get('frozen_split')) == 'heldout']
    sweeps = {}
    for value in config['confidence_sweep']:
        sweeps[value] = metrics(selection, by_id, policy, {c:value for c in training_names(policy)},scope=scope)
    thresholds, states = {}, {}
    for name in training_names(policy):
        eligible = [(c, result['classes'][name]) for c, result in sweeps.items()
                    if result['classes'][name]['gt'] >= config['minimum_gt']
                    and result['classes'][name]['predictions'] >= config['minimum_predictions']
                    and result['classes'][name]['precision'] >= config['minimum_precision']]
        if eligible:
            c, _ = max(eligible, key=lambda item:(item[1]['recall'], item[0]))
            thresholds[name] = c
            states[name] = dict(selection_threshold=c, status='heldout_pending')
        else:
            states[name] = dict(status='selection_insufficient_support_or_precision', approved=False)
    held = metrics(heldout, by_id, policy, thresholds,scope=scope)
    for name, threshold in thresholds.items():
        c = held['classes'][name]
        ok = bool(c['gt'] >= config['minimum_gt'] and c['predictions'] >= config['minimum_predictions']
                  and c['precision'] >= config['minimum_precision'])
        states[name].update(approved=ok, status='approved' if ok else 'heldout_insufficient_support_or_precision')
    fixed = {name:config['fixed_confidence'] for name in training_names(policy)}
    result=dict(schema_version=1, selection_sweep=[dict(confidence=c, **r) for c,r in sweeps.items()],
        heldout_frozen_thresholds=held, teacher_classes=states,
        fixed_selection={level:metrics(selection,by_id,policy,fixed,level,scope) for level in
            ('source_semantic_label','training_class','consumer_group')},
        fixed_heldout={level:metrics(heldout,by_id,policy,fixed,level,scope) for level in
            ('source_semantic_label','training_class','consumer_group')},
        approved_classes=[n for n,s in states.items() if s.get('approved')],
        partial_gt_diagnostic=annotated_localization(gt,by_id,policy,config['fixed_confidence'],scope),
        interpretation='No GT/low support is not approval. Source semantic, training and consumer scores are separate. Heldout never retunes thresholds.',
        training_performed=False, labels_modified=False)
    if scope is not None:result['evaluation_scope']=dict(scope)
    return result


def suggest(rows, predictions, approval, policy):
    from seg_measurements import match_boxes
    from capture_size_scope import in_scope,validate_scope
    scope=validate_scope(approval['evaluation_scope']) if approval.get('evaluation_scope') else None
    approved = approval['teacher_classes']
    result = []
    for row in rows:
        predicted = predictions.get(row['id'])
        if predicted is None:
            continue
        if row['image_sha256'] != predicted['image_sha256']:
            raise ValueError('suggestion image SHA mismatch')
        for field in ('checkpoint_sha256','embeddings_sha256','config_sha256','profile'):
            if field in approval and predicted.get(field) != approval[field]:
                raise ValueError('teacher approval/inference mismatch: ' + field)
        profile = policy['source_profiles'].get((row['source'][9:] if row['source'].startswith('roboflow:') else row['source']), {})
        from camera_data_audit import normalize
        ambiguous = [a for a in row['annotations'] if a['disposition']=='ambiguous'
                     and in_scope(a,scope)
                     and normalize(a['original_label']) not in profile.get('composite', {})]
        candidates = [p for p in predicted['predictions'] if approved.get(p['training_class'],{}).get('approved')
                      and p['confidence'] >= approved[p['training_class']]['selection_threshold']]
        pairs = match_boxes([dict(a,group='any') for a in ambiguous], [dict(p,group='any') for p in candidates],.5)
        for i,j,overlap in pairs:
            a,p = ambiguous[i],candidates[j]
            meanings = profile.get('mixed', {}).get(normalize(a['original_label']), [])
            known = {policy.get('training_map', {}).get(n,n) for n in meanings}
            if meanings and not any(n.startswith('unknown') for n in meanings) and p['training_class'] not in known:
                continue
            result.append(dict(image_id=row['id'],image_sha256=row['image_sha256'],annotation_id=a['id'],
                original_label=a['original_label'],original_category_id=a.get('original_category_id'),original_xyxy=a['xyxy'],
                proposed_training_class=p['training_class'],proposed_semantic_label=p['source_semantic_label'],
                teacher_prompt=p['source_prompt'],teacher_confidence=p['confidence'],iou=overlap,
                teacher_evaluation_scope=scope,teacher_evaluation_scope_sha256=approval.get('evaluation_scope_sha256'),
                requires_human_confirmation=True,decision='pending',source=row['source']))
    return result


def analysis_provenance(predictions, config, policy, config_path, gt_path, predictions_path):
    """Bind a re-analysis to one actual checkpoint, vocabulary and configuration."""
    binding={}
    for field in ('checkpoint_sha256','embeddings_sha256','config_sha256','profile'):
        values={r.get(field) for r in predictions}
        if len(values)!=1 or None in values:
            raise ValueError('missing or mixed prediction provenance: '+field)
        binding[field]=values.pop()
    if binding['config_sha256'] != sha(config_path) or binding['profile'] not in config['prompt_profiles']:
        raise ValueError('analysis config/profile mismatch')
    for row in predictions:
        for prediction in row['predictions']:
            if prediction['source_prompt'] not in config['prompt_profiles'][binding['profile']]:
                raise ValueError('prediction prompt outside bound profile')
            expected=prediction_label(prediction['source_prompt'],policy)
            if any(prediction.get(k)!=expected[k] for k in ('training_class','source_semantic_label','consumer_group')):
                raise ValueError('prediction label policy mismatch')
    binding.update(label_policy_sha256=policy['label_policy_file_sha256'],
        gt_sha256=sha(gt_path),predictions_sha256=sha(predictions_path))
    return binding


def infer(args):
    import cv2
    import numpy as np
    import torch
    from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
    from asmc_camera.detection.segmentation import SegmentationDetector, load_prompt_embeddings
    config = json.loads(args.config.read_text());policy = load_label_policy(args.labels)
    names = config['prompt_profiles'][args.profile]
    checkpoint_sha = sha(args.weights)
    cached, _ = load_prompt_embeddings(args.embeddings,checkpoint_sha)
    if cached != names: raise ValueError('teacher prompt/profile mismatch')
    rows = list(read_jsonl(args.gt))
    if len({r['id'] for r in rows}) != len(rows): raise ValueError('duplicate GT frame')
    args.output.mkdir(parents=True,exist_ok=False)
    masks = load_view_masks(['front','left','right'],HoodMask.load(True,args.hood_config),True,
        dict(left=args.hood_config.parent/'yolo_ego_mask_left_v1.yaml',right=args.hood_config.parent/'yolo_ego_mask_right_v1.yaml'))
    detector = None; latencies=[]
    (args.output/'masks').mkdir();(args.output/'gallery').mkdir()
    predicted=[];gallery=[]
    for index,row in enumerate(rows):
        width = args.front_width if row['view']=='front' else config['side_width']
        if detector is None:
            head=dict(config['head'],imgsz=width,half=True)
            detector=SegmentationDetector(weights=str(args.weights),model_kind='yoloe',prompt_embeddings=str(args.embeddings),
                expected_sha256=checkpoint_sha,conf_thres=.10,allowed_names=names,predict_overrides=head)
        detector.backend.predict_overrides['imgsz'] = width
        image_path=Path(row['image'])
        if args.image_prefix:
            image_path=args.image_root/image_path.relative_to(args.image_prefix)
        if sha(image_path) != row['image_sha256']: raise ValueError('RGB SHA changed')
        image=cv2.imread(str(image_path))
        if image is None: raise ValueError('RGB decode failed')
        started=time.monotonic()
        try:
            frame=detector.predict_frame(masks[row['view']].apply(image))
        except torch.cuda.OutOfMemoryError:
            save(args.output/'failure.json',dict(reason='CUDA_OOM',id=row['id'],front_width=args.front_width,
                completed=len(predicted),fallback_used=False))
            raise
        wall_ms=(time.monotonic()-started)*1000;latencies.append(wall_ms)
        preds=[]
        for d in frame.detections:
            ann=prediction_label(d.source_class_name,policy)
            ann.update(confidence=d.confidence,xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]);preds.append(ann)
        mask_file=None
        if len(frame.masks) != len(frame.detections):
            raise ValueError('bbox/mask count mismatch')
        if frame.masks:
            if any(m is None or m.shape != image.shape[:2] for m in frame.masks): raise ValueError('bbox/mask correspondence lost')
            mask_file='masks/'+str(index)+'.npz'
            np.savez_compressed(args.output/mask_file,masks=np.packbits(np.stack(frame.masks),axis=2),width=image.shape[1])
        record=dict(id=row['id'],image_sha256=row['image_sha256'],view=row['view'],source_key=row.get('source_key'),
            profile=args.profile,predictions=preds,masks=mask_file,wall_ms=wall_ms,timings=frame.timings)
        record.update(checkpoint_sha256=checkpoint_sha,embeddings_sha256=sha(args.embeddings),config_sha256=sha(args.config))
        predicted.append(record)
        with (args.output/'predictions.jsonl').open('a') as stream:stream.write(json.dumps(record)+'\n')
        if index%max(1,len(rows)//120)==0:
            overlay=image.copy()
            for a in row['annotations']:
                x,y,x2,y2=map(int,a['xyxy']);cv2.rectangle(overlay,(x,y),(x2,y2),(255,130,0),1)
                cv2.putText(overlay,str(a.get('canonical')),(max(0,x),max(12,y)),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,130,0),1)
            for d in preds:
                if d['confidence'] < config['fixed_confidence']:continue
                x,y,x2,y2=map(int,d['xyxy']);cv2.rectangle(overlay,(x,y),(x2,y2),(0,255,0),1)
                cv2.putText(overlay,d['source_prompt']+' %.2f'%d['confidence'],(max(0,x),max(12,y)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
            image_file='gallery/'+str(index)+'.jpg';cv2.imwrite(str(args.output/image_file),overlay)
            gallery.append('<article><p>'+html.escape(row['source']+' '+row['source_key'])+' · GT complete='+str(complete_gt(row,policy))+'</p><img src="'+image_file+'"></article>')
        if index%50==0: print('teacher frames',index+1,'/',len(rows),flush=True)
    summary=analyze(rows,predicted,policy,config)
    summary.update(checkpoint_sha256=checkpoint_sha,embeddings_sha256=sha(args.embeddings),gt_sha256=sha(args.gt),
        config_sha256=sha(args.config),profile=args.profile,front_width=args.front_width,
        label_policy_sha256=policy['label_policy_file_sha256'],predictions_sha256=sha(args.output/'predictions.jsonl'),
        latency_ms=dict(zip(('p50','p95','p99'),map(float,np.percentile(latencies,[50,95,99])))),
        cuda_peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20)
    save(args.output/'summary.json',summary)
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>2026-10-10 · YOLOE X '+args.profile+'</title><style>body{font:16px system-ui;max-width:1300px;margin:auto}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>2026-10-10 · YOLOE X '+args.profile+'</h1><p>GT 파랑 · 예측 초록. 미완전 주석은 precision·teacher 승인에 사용하지 않습니다.</p><a href="summary.json">수치·SHA</a>'+''.join(gallery))


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('infer');p.add_argument('--gt',type=Path,required=True);p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--embeddings',type=Path,required=True);p.add_argument('--profile',required=True);p.add_argument('--front-width',type=int,choices=[960,1280],default=1280)
    p.add_argument('--hood-config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--image-prefix',type=Path);p.add_argument('--image-root',type=Path)
    p=sub.add_parser('review');p.add_argument('--gt',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('analyze');p.add_argument('--gt',type=Path,required=True);p.add_argument('--predictions',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scope-config',type=Path);p.add_argument('--scope-profile')
    p=sub.add_parser('suggest');p.add_argument('--inventory',type=Path,required=True);p.add_argument('--predictions',type=Path,required=True);p.add_argument('--approval',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    for p in sub.choices.values():
        p.add_argument('--labels',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='infer' and bool(args.image_prefix) != bool(args.image_root):
        parser.error('image-prefix and image-root must be supplied together')
    if args.command=='infer':infer(args);return
    if args.output.exists():parser.error('preserve previous evidence; output exists')
    policy=load_label_policy(args.labels);config=json.loads(args.config.read_text())
    if args.command=='review':
        from camera_data_audit import build_gallery
        rows=list(read_jsonl(args.gt))
        for row in rows:
            row.update(eligible=complete_gt(row,policy),reasons=[] if complete_gt(row,policy) else ['capture_gt_scope_or_subtype_pending'])
        build_gallery(rows,{'capture_gt_review':[r['id'] for r in rows]},args.output,policy)
        page=args.output/'index.html'
        page.write_text(page.read_text().replace('Roboflow 데이터 감사 · 검수 대기','Capture 독립 GT · 검수 대기').replace(
            '<input id="q"','<p>Codex의 actor 종류 후보는 아직 사람 확인 전입니다. Teacher 예측은 표시하지 않습니다. 종류·보이는 대상의 누락을 직접 확인하고 체크한 기록만 내보내 주세요.</p><input id="q"'))
        return
    predictions=list(read_jsonl(args.predictions))
    if args.command=='analyze':
        scope=None;scope_profile=None
        if args.scope_config:
            from capture_size_scope import validate_scope
            scopes=json.loads(args.scope_config.read_text())
            if scopes['coordinate_space']!='original_image_pixels':raise ValueError('unsupported scope coordinates')
            scope_profile=args.scope_profile or scopes['primary_profile']
            scope=validate_scope(scopes['profiles'][scope_profile])
        elif args.scope_profile:parser.error('scope-profile requires scope-config')
        result=analyze(list(read_jsonl(args.gt)),predictions,policy,config,scope)
        result.update(analysis_provenance(predictions,config,policy,args.config,args.gt,args.predictions))
        if scope is not None:result.update(evaluation_scope_sha256=sha(args.scope_config),evaluation_scope_profile=scope_profile)
        save(args.output,result)
    else:
        approval=json.loads(args.approval.read_text())
        if approval.get('label_policy_sha256') != policy['label_policy_file_sha256'] or approval.get('config_sha256') != sha(args.config):
            raise ValueError('teacher approval policy/config mismatch')
        if len({r['id'] for r in predictions}) != len(predictions):
            raise ValueError('duplicate suggestion prediction frame')
        if any(not approval.get(f) for f in ('checkpoint_sha256','embeddings_sha256','profile')):
            raise ValueError('teacher approval missing checkpoint/vocabulary provenance')
        result=suggest(list(read_jsonl(args.inventory)),{r['id']:r for r in predictions},approval,policy)
        write_jsonl(args.output,result)


if __name__=='__main__':main()
