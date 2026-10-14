#!/usr/bin/env python3
"""Resume local bbox teachers, review semantic changes, export immutable v6.

Reuses the data audit's source policy, matcher, image facts and YOLO validator.
Teacher phrases are evidence, not truth. Actor inventories only select footage.
"""
import argparse
from collections import Counter, defaultdict
import copy
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import time

from camera_data_audit import (digest, export_dataset, image_facts, label_metadata,
    load_label_policy, normalize, read_jsonl, save, sha, training_names, valid_box, write_jsonl)


def phrase_label(phrase, policy):
    """Preserve unknown/multi-phrase text; never turn a wrong phrase into an alias."""
    from camera_data_audit import classify
    canonical, disposition = classify(phrase, policy)
    return label_metadata(dict(raw_phrase=phrase, canonical=canonical,
                               disposition=disposition), policy)


def verify_prediction(row, prediction, binding=None):
    if row['id'] != prediction['id'] or row['image_sha256'] != prediction['image_sha256']:
        raise ValueError('teacher image identity/SHA mismatch')
    if binding and prediction.get('binding') != binding:
        raise ValueError('teacher model/prompt/preprocessing binding mismatch')
    for p in prediction['predictions']:
        if not valid_box(p['xyxy'], row['width'], row['height']) or not math.isfinite(p['confidence']) or not 0 <= p['confidence'] <= 1:
            raise ValueError('teacher invalid geometry/confidence')


def provenance(name, config_path, models, source, contrast):
    config = json.loads(config_path.read_text())
    spec = config['models'][name]
    directory = models / 'teacher_v6'
    if name == 'yoloe_x':
        files = [models / spec['checkpoint'], models / spec['embeddings']]
    elif name == 'gdino_b':
        files = sorted(p for p in (directory / 'grounding-dino-base').iterdir() if p.suffix in ('.json', '.safetensors', '.txt'))
    else:
        files = [directory / spec['checkpoint'], source / 'mmdetection' / spec['config']]
        files += sorted((directory / 'bert-base-uncased').glob('*.json'))
        files += sorted((directory / 'bert-base-uncased').glob('*.safetensors'))
    if not files or any(not p.is_file() for p in files):
        raise ValueError('missing checkpoint/processor assets; prepare first')
    hashes = {str(p): sha(p) for p in files}
    source_commit = subprocess.check_output(['git','-C',str(source/'mmdetection'),'rev-parse','HEAD'],text=True).strip() if name=='mm_gdino_l' else None
    return dict(model=name, files_sha256=hashes, config_sha256=sha(config_path),source_commit=source_commit,
                preprocessing_profile='oom_contrast' if contrast else 'native',
                prompts=config['prompts'], bbox_only=True)


def completed_predictions(path):
    """Recover only an interrupted final append, preserving its bytes as evidence."""
    if not path.exists():return {}
    results={};position=0
    with path.open('rb') as stream:
        for line in stream:
            try:result=json.loads(line)
            except (ValueError,UnicodeDecodeError):
                if stream.read(1):raise ValueError('corrupt prediction before final record')
                shutil.copyfile(path,path.with_name('predictions-interrupted-%d.jsonl'%time.time_ns()))
                with path.open('r+b') as repair:repair.truncate(position)
                break
            if result['id'] in results:raise ValueError('duplicate completed inference record')
            results[result['id']]=result;position+=len(line)
    return results


def run_inference(args):
    import cv2
    from local_teacher_backend import LocalTeacher
    from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
    config = json.loads(args.config.read_text())
    policy = load_label_policy(args.labels)
    binding = provenance(args.model, args.config, args.models, args.source, args.contrast)
    binding['mask_files_sha256']={str(p):sha(p) for p in [args.hood_config]+[
        args.hood_config.parent/('yolo_ego_mask_%s_v1.yaml'%v) for v in ('left','right')]}
    rows = list(read_jsonl(args.manifest))
    if len({r['id'] for r in rows}) != len(rows):
        raise ValueError('duplicate input frame IDs')
    args.output.mkdir(parents=True, exist_ok=True)
    binding.update(input_manifest_sha256=sha(args.manifest))
    bound = args.output / 'binding.json'
    if bound.exists() and json.loads(bound.read_text()) != binding:
        raise ValueError('resume binding changed; use a new run directory')
    save(bound, binding)
    # Runtime binding remains reusable across manifests. Keep the executed code
    # and package environment separately, including every resumed session.
    import importlib.metadata
    code_directory=args.output/'code';code_directory.mkdir(exist_ok=True)
    code={}
    for path in (Path(__file__),Path(__file__).with_name('local_teacher_backend.py')):
        identity=sha(path);destination=code_directory/(identity+'-'+path.name)
        if not destination.exists():shutil.copyfile(path,destination)
        code[path.name]=identity
    versions={}
    for package in ('torch','transformers','ultralytics','mmcv','mmengine','numpy','opencv-python'):
        try:versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:versions[package]=None
    with (args.output/'execution-sessions.jsonl').open('a') as sessions:
        sessions.write(json.dumps(dict(started_at=datetime.now(timezone.utc).isoformat(),
            code_sha256=code,packages=versions,offline=True))+'\n')
    predictions_path = args.output / 'predictions.jsonl'
    done = completed_predictions(predictions_path)
    by_id = {r['id']: r for r in rows}
    if set(done) - set(by_id):
        raise ValueError('resume contains frames outside input')
    for key, result in done.items():
        verify_prediction(by_id[key], result, binding)
    inherited={}
    if args.reuse:
        previous_binding=json.loads((args.reuse/'binding.json').read_text())
        runtime=lambda b:{k:v for k,v in b.items() if k!='input_manifest_sha256'}
        if runtime(previous_binding)!=runtime(binding):raise ValueError('reuse teacher runtime differs')
        parent_sha=sha(args.reuse/'predictions.jsonl')
        for result in read_jsonl(args.reuse/'predictions.jsonl'):
            if result['id'] not in by_id or result['id'] in done:continue
            row=by_id[result['id']];verify_prediction(row,result,previous_binding)
            if sha(row['image'])!=row['image_sha256']:raise ValueError('reuse image changed')
            inherited[result['id']]=dict(result,binding=binding,inherited_from=dict(
                predictions=str(args.reuse/'predictions.jsonl'),sha256=parent_sha,original_binding=previous_binding))
    masks = load_view_masks(['front', 'left', 'right'], HoodMask.load(True, args.hood_config), True,
        {v: args.hood_config.parent / ('yolo_ego_mask_%s_v1.yaml' % v) for v in ('left', 'right')})
    detector = None
    started = time.monotonic()
    with predictions_path.open('a') as output:
        for row in rows:
            if row['id'] in done:
                continue
            if row['id'] in inherited:
                result=inherited[row['id']];output.write(json.dumps(result,ensure_ascii=False)+'\n');output.flush();done[row['id']]=result
                continue
            if args.limit and len(done) >= args.limit:
                break
            if args.max_seconds and time.monotonic() - started >= args.max_seconds:
                break
            if sha(row['image']) != row['image_sha256']:
                raise ValueError('source image changed before inference')
            image = cv2.imread(row['image'])
            if image is None or image.shape[:2] != (row['height'], row['width']):
                raise ValueError('source image decode/shape mismatch')
            if row.get('view') in masks:
                image = masks[row['view']].apply(image)
            if detector is None:
                detector = LocalTeacher(args.model, config, args.models, args.source, args.contrast)
            import torch
            torch.cuda.synchronize()
            tick = time.monotonic()
            try:
                predicted = detector.predict(image, row.get('view', 'unknown'))
                torch.cuda.synchronize()
                clipped=[]
                for proposal in predicted:
                    raw=proposal['xyxy']
                    if not all(math.isfinite(x) for x in raw):raise ValueError('non-finite native teacher box')
                    box=[max(0,min(row['width'],raw[0])),max(0,min(row['height'],raw[1])),
                         max(0,min(row['width'],raw[2])),max(0,min(row['height'],raw[3]))]
                    if not valid_box(box,row['width'],row['height']):continue
                    clipped.append(dict(proposal,xyxy=box,native_xyxy=raw))
                predicted=clipped
            except Exception as exc:
                with (args.output / 'failures.jsonl').open('a') as failures:
                    failures.write(json.dumps(dict(id=row['id'], error=type(exc).__name__, message=str(exc), binding=binding)) + '\n')
                raise  # Explicit separate contrast run, never silent downgrade.
            result = dict(id=row['id'], image_sha256=row['image_sha256'], binding=binding,
                mask_applied=row.get('view') in masks, elapsed_ms=(time.monotonic() - tick) * 1000,
                predictions=[dict(p, **{k:v for k,v in phrase_label(p['raw_phrase'], policy).items() if k != 'raw_phrase'}) for p in predicted])
            verify_prediction(row, result, binding)
            output.write(json.dumps(result, ensure_ascii=False) + '\n')
            output.flush()
            done[row['id']] = result
            if len(done) % 25 == 0:
                print(args.model, len(done), '/', len(rows), flush=True)
    save(args.output / 'summary.json', dict(completed=len(done), expected=len(rows), complete=len(done)==len(rows),
        remaining=len(rows)-len(done), elapsed_this_session_seconds=time.monotonic()-started,
        reused_predictions=sum('inherited_from' in r for r in done.values()),
        predictions_sha256=sha(predictions_path), training_performed=False))


def evaluate(rows, predictions, policy, threshold, mode='semantic'):
    """Complete in-scope images supply precision; partial ones only recall."""
    from seg_measurements import match_boxes
    from capture_size_scope import in_scope, ignored_prediction_indices
    counts = {n: Counter(gt=0, predictions=0, tp=0, fp=0) for n in training_names(policy)}
    partial = {n: Counter(gt=0, tp=0) for n in training_names(policy)}
    scenes = defaultdict(set)
    pooled_geometry = Counter(gt=0, predictions=0, tp=0)
    for row in rows:
        result = predictions.get(row['id'])
        if result is None:
            raise ValueError('missing evaluation prediction')
        verify_prediction(row, result)
        annotations = row['annotations']
        scope = row.get('evaluation_scope')
        gt = [a for a in annotations if a.get('training_class') and in_scope(a, scope)]
        unresolved = [a for a in annotations if a.get('disposition') == 'ambiguous' and in_scope(a, scope)]
        complete = bool(row.get('annotation_complete') and row.get('independent_gt_review') and not unresolved)
        pred = [p for p in result['predictions'] if p['confidence'] >= threshold]
        pairs = match_boxes([dict(a,group='geometry' if mode=='geometry' else a['training_class']) for a in gt],
                            [dict(a,group='geometry' if mode=='geometry' else a.get('training_class')) for a in pred], .5)
        ignored = [a for a in annotations if not in_scope(a, scope)]
        ignored_ids = ignored_prediction_indices(pred, ignored, {j for _,j,_ in pairs}, scope)
        if not complete:
            for a in gt:partial[a['training_class']].update(gt=1)
            for i,_,_ in pairs:partial[gt[i]['training_class']].update(tp=1)
            continue
        if mode == 'geometry':
            pooled_geometry.update(gt=len(gt), predictions=len(pred)-len(ignored_ids), tp=len(pairs))
        for a in gt:
            counts[a['training_class']].update(gt=1)
            scenes[a['training_class']].add(row.get('independent_scene', row['split_group']))
        for i,j,_ in pairs:
            counts[gt[i]['training_class']].update(tp=1)
        if complete:
            # Geometry precision is pooled: unmatched boxes have no defensible
            # true class. Do not assign them the matched GT's class after the fact.
            for j,p in enumerate(pred):
                if j not in ignored_ids and p.get('training_class'):
                    counts[p['training_class']].update(predictions=1)
            for j,p in enumerate(pred):
                if j not in ignored_ids and j not in {x[1] for x in pairs} and p.get('training_class'):
                    counts[p['training_class']].update(fp=1)
    values = {}
    for n,c in counts.items():
        denom=c['predictions']
        values[n]=dict(c, independent_scenes=len(scenes[n]), recall=c['tp']/c['gt'] if c['gt'] else None,
            precision=(denom-c['fp'])/denom if denom and mode=='semantic' else None)
    return dict(classes=values, partial_gt_localization={n:dict(c) for n,c in partial.items()}, threshold=threshold, mode=mode,
        pooled_geometry=dict(pooled_geometry, precision=pooled_geometry['tp']/pooled_geometry['predictions'] if pooled_geometry['predictions'] else None),
        precision_policy='Semantic precision requires independently complete GT. Geometry matching reports localization only; wrong names require separate identity evidence.')


def compare(args):
    rows=list(read_jsonl(args.manifest));policy=load_label_policy(args.labels)
    config=json.loads(args.config.read_text());reports={}
    for directory in args.runs:
        prediction_rows=list(read_jsonl(directory/'predictions.jsonl'))
        predictions={r['id']:r for r in prediction_rows}
        if len(predictions)!=len(prediction_rows):raise ValueError('duplicate comparison predictions')
        binding=json.loads((directory/'binding.json').read_text())
        for row in rows:verify_prediction(row,predictions[row['id']],binding)
        selection=[r for r in rows if r['split']=='selection']
        heldout=[r for r in rows if r['split']=='heldout']
        sweeps=[evaluate(selection,predictions,policy,c) for c in config['confidence_sweep']]
        states={}
        for name in training_names(policy):
            scenes={r.get('independent_scene',r['split_group']) for r in rows if
                r.get('annotation_complete') and r.get('independent_gt_review') and any(
                    a.get('training_class')==name for a in r['annotations'])}
            possible=[s for s in sweeps if (s['classes'][name]['precision'] or 0)>=config['minimum_precision']
                and s['classes'][name]['gt']>=config['minimum_gt']
                and s['classes'][name]['predictions']>=config['minimum_predictions']
                and len(scenes)>=config['minimum_scenes']]
            if not possible:
                states[name]=dict(approved=False,reason='selection_support_or_precision')
                continue
            best=max(possible,key=lambda s:(s['classes'][name]['recall'],s['threshold']))
            held=evaluate(heldout,predictions,policy,best['threshold'])['classes'][name]
            ok=(held['precision'] or 0)>=config['minimum_precision'] and held['gt']>=config['minimum_gt'] and held['predictions']>=config['minimum_predictions'] and len(scenes)>=config['minimum_scenes']
            # Different recordings/weather on the same route are not proof of
            # independent scenes. Keep statistical candidates, but never approve
            # automatic labeling from a run count alone.
            verified_scenes={r['independent_scene'] for r in rows if
                r.get('independent_scene_verified') and r.get('annotation_complete') and
                r.get('independent_gt_review') and any(a.get('training_class')==name for a in r['annotations'])}
            scene_ok=len(verified_scenes)>=config['minimum_scenes']
            states[name]=dict(approved=ok and scene_ok,statistical_thresholds_passed=ok,
                              threshold=best['threshold'],heldout=held,
                              independent_scenes=len(scenes),
                              verified_independent_scenes=len(verified_scenes),scene_count_basis='recording_groups_unless_explicitly_verified',
                              reason='approved' if ok and scene_ok else 'scene_independence_unverified' if ok else 'heldout_support_or_precision')
        reclass_sweeps={c:reclassification_metrics(selection,predictions,policy,c) for c in config['confidence_sweep']}
        reclass_states={}
        for name in training_names(policy):
            possible=[(c,v[name]) for c,v in reclass_sweeps.items() if
                (v[name]['precision'] or 0)>=config['minimum_precision'] and v[name]['gt']>=config['minimum_gt']
                and v[name]['predictions']>=config['minimum_predictions']]
            if not possible:
                reclass_states[name]=dict(approved=False,reason='selection_reclassification_support_or_precision');continue
            threshold,selected=max(possible,key=lambda v:(v[1]['recall'],v[0]))
            held=reclassification_metrics(heldout,predictions,policy,threshold)[name]
            scenes={r.get('independent_scene',r['split_group']) for r in rows if any(
                a.get('training_class')==name and str(a['id']) in r.get('verified_annotation_ids',[]) for a in r['annotations'])}
            ok=(held['precision'] or 0)>=config['minimum_precision'] and held['gt']>=config['minimum_gt'] and held['predictions']>=config['minimum_predictions'] and len(scenes)>=config['minimum_scenes']
            verified_scenes={r['independent_scene'] for r in rows if r.get('independent_scene_verified') and any(
                a.get('training_class')==name and str(a['id']) in r.get('verified_annotation_ids',[]) for a in r['annotations'])}
            scene_ok=len(verified_scenes)>=config['minimum_scenes']
            reclass_states[name]=dict(approved=ok and scene_ok,statistical_thresholds_passed=ok,
                threshold=threshold,heldout=held,independent_scenes=len(scenes),verified_independent_scenes=len(verified_scenes),
                scene_count_basis='recording_groups_unless_explicitly_verified',
                scope='rename_existing_boxes_only_no_new_box_approval',reason='approved' if ok and scene_ok else 'scene_independence_unverified' if ok else 'heldout_reclassification_support_or_precision')
        reports[binding['model']]=dict(binding=binding, selection_sweep=sweeps, teacher_classes=states,
            existing_box_reclassification=reclass_states,
            fixed_semantic=evaluate(rows,predictions,policy,config['fixed_confidence']),
            fixed_geometry=evaluate(rows,predictions,policy,config['fixed_confidence'],'geometry'))
    save(args.output,dict(models=reports,manifest_sha256=sha(args.manifest),config_sha256=sha(args.config),
                          training_performed=False,labels_modified=False))


def model_semantic(name):
    """Explicit known asset spelling only. Ambiguous/standing scooter stays unknown."""
    n=normalize(name)
    aliases={'cargobox':'cargo_box','woodbox':'wooden_box','pe firewall orange':'pe_firewall',
        'pe firewall':'pe_firewall','pe drum':'pe_drum','redbarrel':'redbarrel','yellowbarrel':'yellowbarrel',
        'trafficbarrel':'traffic_barrel','steel barricade':'steel_barricade','deer1':'deer'}
    if n in aliases:return aliases[n]
    if n.startswith('concrete barricade'):return 'concrete_barrier'
    if n.startswith(('bicycle ',)):return 'bicycle'
    if n in ('scooter1','scooter2','scooter 02','sportbike 01'):return 'motorcycle'
    if n.startswith('babystroller'):return 'stroller'
    if n.startswith('dog') and n[3:].isdigit():return 'dog'
    if n.startswith(('man','woman','boy','girl','worker ','police ')):return 'person'
    return None


def aim_inventory(root, config):
    result=[]
    for kind in config['aim_folders']:
        sessions=sorted(set(p.parent for pattern in ('*/SESSION.json','*/*/SESSION.json') for p in (root/kind).glob(pattern)))
        for session in sessions:
            report_path=session/'episode_000/actor_inventory_audit_v1/report.json'
            native_path=session/'NATIVE_CAPTURE_RESULT.json'
            report=json.loads(report_path.read_text()) if report_path.exists() else {}
            native=json.loads(native_path.read_text()) if native_path.exists() else {}
            quick=native.get('quick_native_capture') or {}
            counts={v:len(list((session/camera).glob('*_Intensity.png'))) for camera,v in config['views'].items()}
            expected=quick.get('counts',{})
            complete=bool(expected) and all(counts[v]==expected.get(camera.lower()) for camera,v in config['views'].items())
            models=report.get('models_by_instance',{})
            result.append(dict(session=str(session),kind=kind,models_by_instance=models,
                known_semantics=sorted({s for m in models.values() for s in [model_semantic(m)] if s}),
                image_counts=counts,expected_counts=expected,local_counts_complete=complete,
                metadata_present=bool(report),profile_off_not_empty=True,
                report_sha256=sha(report_path) if report_path.exists() else None,
                class_prior_only=True, image_visibility_confirmed=False))
    return result


def inventory(args):
    config=json.loads(args.config.read_text());rows=list(read_jsonl(args.curated))
    counts=Counter();classes=defaultdict(Counter);pending=Counter()
    for row in rows:
        counts['records']+=1;counts['accepted_images' if row['eligible'] else 'pending_images']+=1
        for ann in row['annotations']:
            if row['eligible'] and ann.get('training_class'):classes[ann['training_class']].update(images=0,objects=1)
            elif ann.get('disposition')=='ambiguous':pending[row['source']+'|'+str(ann['original_label'])]+=1
        for n in {a.get('training_class') for a in row['annotations'] if a.get('training_class')}:
            if row['eligible']:classes[n].update(images=1)
    aims=aim_inventory(args.aim,config) if args.aim else []
    args.output.mkdir(parents=True,exist_ok=False)
    save(args.output/'inventory.json',dict(parent_curated_sha256=sha(args.curated),roboflow_capture=dict(counts),
        classes=dict(classes),pending_mixed_annotations=dict(pending),aim_sessions=aims,
        aim_images=sum(sum(r['image_counts'].values()) for r in aims),
        recorded_actor_presence_is_not_visibility=True,training_performed=False))


def prepare_gt(args):
    """Independent actor RGB evidence supplies names; producer supplies geometry.

    Teacher selection/heldout roles are separate from original training splits.
    No final model evaluation is claimed on this annotation-assistance subset.
    """
    rows=list(read_jsonl(args.curated));config=json.loads(args.config.read_text());evidence_hashes={}
    if args.actor_reviews:
        from prepare_capture_training_data import validate_reviews
        from prepare_capture_teacher_gt import prepare
        reviews=json.loads(args.actor_reviews.read_text());validate_reviews(rows,reviews)
        rows=prepare([r for r in rows if r['source'].startswith('capture:')],load_label_policy(args.labels),reviews,{})
    selected={};groups=defaultdict(list)
    for row in rows:
        if not row['source'].startswith('capture:') or row.get('issues'):continue
        verified=[]
        for ann in row['annotations']:
            if ann.get('training_class') in ('person','dog','deer','truck','bus') and not ann.get('issues') and not ann.get('actor_review') and ann.get('disposition')=='candidate':
                verified.append(str(ann['id']))
            evidence=ann.get('actor_review',{})
            if (ann.get('training_class') and evidence.get('evidence_image_sha256') and
                not evidence.get('teacher_derived') and evidence.get('reviewer','').startswith('Codex independent Capture RGB')):
                path=evidence.get('evidence_image')
                if path not in evidence_hashes:evidence_hashes[path]=sha(path) if path and Path(path).is_file() else None
                if evidence_hashes[path]!=evidence['evidence_image_sha256']:raise ValueError('independent actor evidence SHA mismatch')
                verified.append(str(ann['id']))
        if not verified:continue
        copy_row=copy.deepcopy(row);copy_row['verified_annotation_ids']=verified
        copy_row['dataset_split']=row['split'];copy_row['annotation_complete']=False
        copy_row['independent_gt_review']=False;copy_row['independent_scene']=row['source']
        for name in {a['training_class'] for a in row['annotations'] if str(a['id']) in verified}:
            groups[(row['source'],name)].append(copy_row)
    sources=sorted({s for s,n in groups})
    held=set()
    # Keep source/run membership whole; assure at least one heldout source for
    # each multi-run class, without moving its original dataset assignment.
    for name in training_names(load_label_policy(args.labels)):
        available=sorted({s for s,n in groups if n==name},key=lambda s:digest(str(config['sample_seed'])+s))
        if len(available)>=2 and not set(available)&held:held.add(available[0])
    for (source,name),items in groups.items():
        practical=[r for r in items if any(str(a['id']) in r['verified_annotation_ids'] and a['training_class']==name
            and max(a['xyxy'][2]-a['xyxy'][0],a['xyxy'][3]-a['xyxy'][1])>=16
            and min(a['xyxy'][2]-a['xyxy'][0],a['xyxy'][3]-a['xyxy'][1])>=8 for a in r['annotations'])]
        practical=sorted(practical,key=lambda r:digest(str(config['sample_seed'])+r['id']))
        for row in practical[:args.per_class_run]:
            row['split']='heldout' if source in held else 'selection'
            row['evaluation_scope']=dict(min_long_edge=16,min_short_edge=8,ignore_prediction_ioa=.5)
            from capture_size_scope import in_scope
            original=row.get('original_annotation',{})
            complete=bool(original.get('bbox_file_present') and original.get('annotation_complete') and row.get('provenance_verified') and all(
                str(a['id']) in row['verified_annotation_ids'] or a.get('disposition')=='excluded' or not in_scope(a,row['evaluation_scope'])
                for a in row['annotations']))
            row['annotation_complete']=complete;row['independent_gt_review']=complete
            row['gt_basis']='original Capture geometry plus independently RGB-reviewed actor identities; unresolved in-scope objects disqualify precision'
            selected[row['id']]=row
    args.output.mkdir(parents=True,exist_ok=False)
    write_jsonl(args.output/'frames.jsonl',selected.values())
    save(args.output/'summary.json',dict(frames=len(selected),teacher_split_groups=len(sources),
        original_dataset_splits_unchanged=True,teacher_derived_gt=False,
        source_sha256=sha(args.curated),complete_in_scope_images=sum(r['annotation_complete'] for r in selected.values()),
        scope='independent actor names; complete original Capture geometry required for full-image precision',
        classes=dict(Counter(a['training_class'] for r in selected.values() for a in r['annotations'] if str(a['id']) in r['verified_annotation_ids']))))


def aim_queue(args):
    """Use original native associations, not sorted filename/index assumptions."""
    config=json.loads(args.config.read_text());inventory=json.loads(args.inventory.read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    candidates=[];skipped=[]
    for item in inventory['aim_sessions']:
        session=Path(item['session'])
        if not item['local_counts_complete'] or not item['metadata_present']:
            skipped.append(dict(session=str(session),reason='local_transfer_or_metadata_incomplete'));continue
        if not item['known_semantics']:
            skipped.append(dict(session=str(session),reason='no_known_target_prior_screen_later_not_verified_empty'));continue
        assoc=session/'episode_000/native_associations/associations.json'
        brackets=session/'episode_000/actor_inventory_audit_v1/frame_brackets.json'
        if not assoc.exists() or not brackets.exists():
            skipped.append(dict(session=str(session),reason='missing_native_frame_association'));continue
        frames=json.loads(assoc.read_text());actors={b['index']:b for b in json.loads(brackets.read_text())}
        usable=[f for f in frames if f['index'] in actors and
            (set(actors[f['index']].get('actor_ids_before',[]))|set(actors[f['index']].get('actor_ids_after',[])))-{'Ego'}]
        # All views at the same native frame remain one scene group. Initial
        # screening uses spread timestamps; expansion is a separate queue.
        indices=sorted({round(i*(len(usable)-1)/max(1,args.per_session-1)) for i in range(min(args.per_session,len(usable)))})
        group='aim:'+str(session.parent)+':'+session.name
        role=('heldout' if int(digest(str(20261009)+group)[:8],16)%10==0 else
              'selection' if int(digest(str(20261009)+group)[:8],16)%10==1 else 'train')
        for index in indices:
            frame=usable[index]
            for camera,view in config['views'].items():
                relative=frame['camera_files'].get(camera.lower())
                if not relative:continue
                path=session/relative
                if not path.is_file():continue
                row=dict(id=digest(group+relative),source=group,source_key=relative,image=str(path),
                    annotations=[],issues=[],frozen_split=role,split=role,split_group=group,
                    view=view,weather='unknown',tunnel=item['kind']=='tunnel_static_obstacle',
                    annotation_complete=False,provenance_verified=False,eligible=False,reasons=['teacher_proposal_review_required'],
                    actor_model_priors=item['models_by_instance'],known_semantics=item['known_semantics'],
                    native_association_sha256=sha(assoc),actor_inventory_sha256=item['report_sha256'],
                    source_stamp_ns=frame.get('scene_time_hypothesis_ns'),
                    model_presence_is_not_visibility=True)
                candidates.append(image_facts(row))
    write_jsonl(args.output/'frames.jsonl',candidates)
    save(args.output/'summary.json',dict(frames=len(candidates),sessions=len({r['source'] for r in candidates}),
        skipped=skipped,input_inventory_sha256=sha(args.inventory),training_performed=False,
        initial_screen_only=True,scene_group_policy='whole recording session and synchronized views; pHash/lineage review required before export'))


def bag_queue(args):
    """Extract spread frames only from explicitly nominated obstacle recordings."""
    import rosbag
    topics={'/image_jpeg/compressed':'front','/cam_left/image_jpeg/compressed':'left','/cam_right/image_jpeg/compressed':'right'}
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'images').mkdir()
    rows=[];sources=[]
    for path in args.bags:
        group='bag:'+path.parent.parent.name;bag_hash=sha(path)
        # These two recordings already supplied frozen baseline/fusion checks.
        protected=('lidar_manual_20260929_130623' in group or '20260912_003803_' in group)
        role='selection' if protected else ('heldout' if int(digest(str(20261009)+group)[:8],16)%10==0 else
               'selection' if int(digest(str(20261009)+group)[:8],16)%10==1 else 'train')
        next_ns={};count=0
        with rosbag.Bag(str(path)) as bag:
            for index,(topic,message,bag_stamp) in enumerate(bag.read_messages(topics=list(topics))):
                stamp=bag_stamp.to_nsec()
                if stamp<next_ns.get(topic,0):continue
                next_ns[topic]=stamp+1_000_000_000
                ident=digest(group+topic+str(index));image=args.output/'images'/(ident+'.jpg')
                image.write_bytes(bytes(message.data))
                row=dict(id=ident,source=group,source_key=topic+'/'+str(index),image=str(image.resolve()),
                    annotations=[],issues=[],split=role,frozen_split=role,split_group=group,
                    view=topics[topic],weather='foggy' if 'foggy' in group else 'sunny' if 'sunny' in group else 'unknown',
                    tunnel=True,annotation_complete=False,provenance_verified=True,eligible=False,
                    reasons=['teacher_proposal_review_required'],bag=str(path.resolve()),bag_sha256=bag_hash,
                    bag_stamp_ns=stamp,source_stamp_ns=message.header.stamp.to_nsec(),
                    original_header_frame_id=message.header.frame_id,previous_baseline_recording=protected)
                rows.append(image_facts(row));count+=1
        sources.append(dict(path=str(path),sha256=bag_hash,frames=count,previous_baseline_recording=protected))
    write_jsonl(args.output/'frames.jsonl',rows)
    save(args.output/'summary.json',dict(frames=len(rows),sources=sources,screen_period_seconds=1,
        training_performed=False,actor_presence_is_not_gt=True,all_other_bags_skipped_not_verified_empty=True))


def reclassification_metrics(rows,predictions,policy,threshold):
    """Existing-box naming quality, not whole-image detector precision.

    Only independently verified original boxes participate. Unmatched teacher
    boxes cannot become new labels through this route, and misses keep the GT.
    """
    from seg_measurements import match_boxes
    counts={n:Counter(gt=0,predictions=0,tp=0) for n in training_names(policy)}
    scenes=defaultdict(set)
    for row in rows:
        verify_prediction(row,predictions[row['id']])
        verified=set(row.get('verified_annotation_ids',[]))
        from capture_size_scope import in_scope
        gt=[a for a in row['annotations'] if str(a['id']) in verified and a.get('training_class') and in_scope(a,row.get('evaluation_scope'))]
        pred=[p for p in predictions[row['id']]['predictions'] if p['confidence']>=threshold]
        pairs=match_boxes([dict(a,group='any') for a in gt],[dict(p,group='any') for p in pred],.5)
        for a in gt:
            counts[a['training_class']].update(gt=1);scenes[a['training_class']].add(row.get('independent_scene',row['split_group']))
        for i,j,_ in pairs:
            target=pred[j].get('training_class')
            if target in counts:
                counts[target].update(predictions=1,tp=int(target==gt[i]['training_class']))
    return {n:dict(c,precision=c['tp']/c['predictions'] if c['predictions'] else None,
                   recall=c['tp']/c['gt'] if c['gt'] else None,independent_scenes=len(scenes[n])) for n,c in counts.items()}


def samples(args):
    """Deterministic mixed-box review queue; never silently rewrites source labels."""
    rows=list(read_jsonl(args.curated));config=json.loads(args.config.read_text())
    groups=defaultdict(list)
    for r in rows:
        if r['eligible'] or any('split' in reason or 'duplicate' in reason for reason in r['reasons']):continue
        if r.get('split')!='train':continue
        for label in {a['original_label'] for a in r['annotations'] if a['disposition']=='ambiguous'}:
            groups[r['source']+'|'+label].append(r)
    selected={}
    for key,items in sorted(groups.items()):
        items=sorted(items,key=lambda r:digest(str(config['sample_seed'])+r['id']))
        for r in items[:config['review_sample_images']]:selected[r['id']]=r
    args.output.mkdir(parents=True,exist_ok=False)
    write_jsonl(args.output/'frames.jsonl',selected.values())
    save(args.output/'summary.json',dict(groups={k:len(v) for k,v in groups.items()},
        frames=len(selected),source_sha256=sha(args.curated),config_sha256=sha(args.config)))


def apply_decision(row, decision, policy, predictions=None):
    """Independent RGB review may correct any teacher name; preserve original box."""
    review_state_issues=('review_pending','review_rejected')
    if any(issue not in review_state_issues for issue in row.get('issues',[])):
        raise ValueError('source structural issues require repair before semantic admission')
    result=copy.deepcopy(row)
    if row.get('issues'):
        result['prior_row_issues_before_individual_review']=list(row['issues'])
        result['issues']=[]
    if decision.get('image_sha256')!=row['image_sha256'] or not decision.get('evidence') or not decision.get('reviewer'):
        raise ValueError('review missing bound image/evidence/reviewer')
    if not decision.get('all_targets_annotated'):
        raise ValueError('new admission needs target-completeness review')
    changes={str(a['annotation_id']):a for a in decision.get('overrides',[])}
    if set(changes)-{str(a['id']) for a in row['annotations']}:
        raise ValueError('unknown original annotation ID')
    for a in result['annotations']:
        if str(a['id']) in changes:
            change=changes[str(a['id'])];target=change.get('canonical')
            if target not in training_names(policy) and target not in policy.get('training_map',{}) and target is not None:
                raise ValueError('unknown canonical target')
            if 'xyxy' in change and change['xyxy']!=a['xyxy']:
                raise ValueError('semantic correction cannot silently change original box')
            a.update(canonical=target,disposition='candidate' if target else 'excluded',
                issues=[i for i in a.get('issues',[]) if i not in ('capture_subtype_unknown','ambiguous_label','mixed_source_label')],
                semantic_review=change, original_semantic_before_review=a.get('canonical'))
        label_metadata(a,policy)
    known={str(a['id']) for a in result['annotations']}
    for addition in decision.get('additions',[]):
        ident=str(addition['id'])
        if ident in known or addition['canonical'] not in training_names(policy) or not valid_box(addition['xyxy'],row['width'],row['height']):
            raise ValueError('invalid added bbox/class/ID')
        origin=addition.get('annotation_origin','independently_reviewed_teacher_proposal')
        if origin not in ('independently_reviewed_teacher_proposal','independently_reviewed_manual_bbox'):
            raise ValueError('unknown reviewed annotation origin')
        a=dict(addition,original_label=None,original_category_id=None,disposition='candidate',
               annotation_origin=origin,issues=[])
        result['annotations'].append(label_metadata(a,policy));known.add(ident)
    if any(a.get('disposition')=='ambiguous' or a.get('issues') for a in result['annotations']):
        raise ValueError('unresolved annotation cannot be admitted')
    if not any(a.get('canonical') for a in result['annotations']) and not decision.get('negative_verified'):
        raise ValueError('empty teacher is not a verified negative')
    result.update(eligible=True,reasons=[],annotation_complete=True,provenance_verified=True,
                  review_scope=policy['review_scope'],review_basis='independent_teacher_review',
                  admission_tier='teacher_individually_reviewed',teacher_review=decision)
    return result


def admit_batch(rows, batch, policy, approval):
    """Sampled bulk admission only for a pinned, validated teacher cohort."""
    members={r['id']:r for r in rows}
    sample=batch.get('sample_reviews',{})
    expected=min(50,len(rows))
    ordered=sorted(members,key=lambda i:digest(str(20261009)+i))[:expected]
    if set(sample)!=set(ordered):raise ValueError('batch review sample membership mismatch')
    if any(sample[i].get('image_sha256')!=members[i]['image_sha256'] for i in sample):
        raise ValueError('batch sample image SHA mismatch')
    ok=sum(bool(r.get('correct') and r.get('all_targets_annotated') and r.get('evidence')) for r in sample.values())
    if not sample or ok/len(sample)<.95:raise ValueError('batch review below 95 percent')
    state=approval.get(batch['training_class'],{})
    if not state.get('approved'):raise ValueError('teacher class not independently approved')
    if not batch.get('reviewer') or not batch.get('cohort_binding'):raise ValueError('batch provenance missing')
    return dict(approved=True,sampled=len(sample),correct=ok,reviewer=batch['reviewer'],
                cohort_binding=batch['cohort_binding'],minimum_confidence=state['threshold'])


def export(args):
    policy=load_label_policy(args.labels);rows=list(read_jsonl(args.curated))
    decisions=json.loads(args.reviews.read_text())
    extra=list(read_jsonl(args.additional)) if args.additional else []
    if {r['id'] for r in rows}&{r['id'] for r in extra}:raise ValueError('additional frame IDs collide')
    all_rows=rows+extra;by_id={r['id']:r for r in all_rows}
    if len(by_id)!=len(all_rows):raise ValueError('duplicate source IDs')
    batch_reports=[]
    if args.batches:
        if not args.approval or not args.predictions:raise ValueError('batch admission requires approval and predictions')
        prediction_rows=list(read_jsonl(args.predictions));predictions={r['id']:r for r in prediction_rows}
        if len(predictions)!=len(prediction_rows):raise ValueError('duplicate cohort predictions')
        report=json.loads(args.approval.read_text())
        for batch in json.loads(args.batches.read_text()):
            members=[by_id[i] for i in batch['member_ids']]
            model=report['models'][batch['model']]
            if batch['cohort_binding']!=digest(json.dumps([(r['id'],r['image_sha256']) for r in members],sort_keys=True)):
                raise ValueError('batch cohort binding mismatch')
            if batch['predictions_sha256']!=sha(args.predictions):raise ValueError('batch predictions SHA mismatch')
            state=model['existing_box_reclassification']
            check=admit_batch(members,batch,policy,state);batch_reports.append(check)
            runtime=lambda b:{k:v for k,v in b.items() if k!='input_manifest_sha256'}
            for row in members:
                prediction=predictions[row['id']];verify_prediction(row,prediction)
                if runtime(prediction['binding'])!=runtime(model['binding']):raise ValueError('batch teacher runtime differs from validated teacher')
                review=batch.get('sample_reviews',{}).get(row['id'])
                if review and not review.get('correct'):continue
                if row['id'] in decisions:raise ValueError('individual and batch decision overlap')
                from seg_measurements import match_boxes
                ambiguous=[a for a in row['annotations'] if a['disposition']=='ambiguous']
                project=row['source'].replace('roboflow:','',1)
                composites=policy.get('source_profiles',{}).get(project,{}).get('composite',{})
                if any(normalize(a['original_label']) in composites or a.get('issues') for a in ambiguous):continue
                selected=[p for p in prediction['predictions'] if p.get('training_class')==batch['training_class'] and p['confidence']>=check['minimum_confidence']]
                pairs=match_boxes([dict(a,group='any') for a in ambiguous],[dict(p,group='any') for p in selected],.5)
                if not ambiguous or len(pairs)!=len(ambiguous):continue
                decisions[row['id']]=dict(image_sha256=row['image_sha256'],reviewer=batch['reviewer'],
                    evidence='Independently validated existing-box naming and sampled source cohort; original boxes retained',
                    all_targets_annotated=True,review_basis='sampled_teacher_cohort',batch=batch['cohort_binding'],
                    overrides=[dict(annotation_id=ambiguous[i]['id'],canonical=batch['training_class'],
                        teacher_raw_phrase=selected[j]['raw_phrase'],teacher_confidence=selected[j]['confidence'],
                        teacher_xyxy=selected[j]['xyxy'],teacher_iou=iou) for i,j,iou in pairs])
    if set(decisions)-set(by_id):raise ValueError('review IDs not in source inventory')
    changed=[]
    for index,row in enumerate(all_rows):
        if row['id'] not in decisions:continue
        if row.get('split')!='train':raise ValueError('v6 teacher expansion cannot modify frozen evaluation GT')
        if row['eligible']:raise ValueError('existing accepted v5 labels are immutable')
        if any('split' in reason or 'duplicate' in reason for reason in row.get('reasons',[])):
            raise ValueError('review cannot waive split/duplicate gate')
        all_rows[index]=apply_decision(row,decisions[row['id']],policy)
        if decisions[row['id']].get('review_basis')=='sampled_teacher_cohort':
            all_rows[index]['admission_tier']='teacher_sampled_cohort'
        changed.append(row['id'])
    # All original records, including excluded evaluation records, protect splits.
    roles=defaultdict(set);accepted=defaultdict(list)
    for row in all_rows:
        roles[row['pixel_sha256']].add(row['split'])
        if row['eligible']:accepted[row['pixel_sha256']].append(row)
    for pixel,group in accepted.items():
        old=[r for r in group if r['id'] not in changed]
        new=[r for r in group if r['id'] in changed]
        for row in new:
            if len(roles[pixel])>1 or old or row is not min(new,key=lambda r:r['id']):
                row.update(eligible=False,reasons=['v6_pixel_duplicate_or_split_conflict'])
    # Check new candidates against every original evaluation record, including
    # unaccepted ones. Preserve old admissions; ambiguous new near matches wait.
    protected=[r for r in all_rows if r['split']!='train' and r.get('phash')]
    near_pairs=[]
    for row in all_rows:
        if row['id'] not in changed or not row['eligible']:continue
        if not row.get('phash'):raise ValueError('new candidate missing perceptual hash')
        value=int(row['phash'],16)
        for other in protected:
            if other['split_group']==row['split_group']:continue
            distance=bin(value ^ int(other['phash'],16)).count('1')
            if distance<=6:
                near_pairs.append(dict(new_id=row['id'],evaluation_id=other['id'],distance=distance,confirmed=False))
                row.update(eligible=False,reasons=['v6_cross_split_similar_image_requires_review']);break
    args.output.mkdir(parents=True,exist_ok=False)
    write_jsonl(args.output/'curated.jsonl',all_rows)
    save(args.output/'reviews.json',decisions)
    save(args.output/'near-duplicate-review.json',near_pairs)
    export_dataset(all_rows,policy,args.dataset,True)
    before=[r for r in rows if r['eligible']];after=[r for r in all_rows if r['eligible']]
    summary=dict(parent_curated_sha256=sha(args.curated),reviews_sha256=sha(args.reviews),
        parent_curated=str(args.curated.resolve()),reviews_path=str(args.reviews.resolve()),
        labels_sha256=sha(args.labels),source_profiles_sha256=policy['source_profiles_sha256'],batch_reports=batch_reports,
        previous_images=len(before),images=len(after),new_images=len(after)-len(before),
        objects=sum(bool(a.get('canonical')) for r in after for a in r['annotations']),
        classes=dict(Counter(a['training_class'] for r in after for a in r['annotations'] if a.get('canonical'))),
        training_performed=False,existing_v5_quality_tiers_preserved=True,
        dataset=str(args.dataset.resolve()),verification=json.loads((args.dataset/'verification.json').read_text()))
    save(args.output/'summary.json',summary)


def registry(args):
    entries=[]
    superseded=set(getattr(args,'supersedes',[]) or [])
    for path in sorted(args.root.glob('camera_objects_bbox*/summary.json')):
        summary=json.loads(path.read_text());directory=path.parent
        verification=directory/'verification.json'
        curated=args.root.parent/'roboflow'/('curated-'+directory.name.replace('camera_objects_bbox_','',1))
        provenance_paths=[p for p in (curated/'summary.json',curated/'capture-summary.json') if p.exists()]
        provenance_data=[dict(path=str(p),sha256=sha(p),record=json.loads(p.read_text())) for p in provenance_paths]
        entries.append(dict(version=directory.name,path=str(directory),data_yaml=str(directory/'data.yaml'),
            summary_sha256=sha(path),manifest_sha256=summary.get('manifest_sha256'),
            images=summary.get('images'),classes=summary.get('classes'),splits=summary.get('splits'),
            labels_sha256=summary.get('labels_sha256'),curated=str(curated/'curated.jsonl') if curated.exists() else None,
            verification=json.loads(verification.read_text()) if verification.exists() else None,
            parent_provenance=provenance_data or None,
            frozen=True,status='current_adopted' if directory.name==args.current else
                'superseded_intermediate' if directory.name in superseded else 'historical_not_current'))
    if args.current not in {e['version'] for e in entries}:raise ValueError('current dataset version missing')
    if args.current in superseded or superseded-{e['version'] for e in entries}:
        raise ValueError('invalid superseded version list')
    save(args.output,dict(schema_version=1,current_adopted=args.current,
        updated_at=datetime.now(timezone.utc).isoformat(),versions=entries,
        image_storage='symlinks to immutable originals; copying export alone does not copy pixels'))


def coverage(args):
    """Image-level coverage; never count repeated frames as independent scenes."""
    policy=load_label_policy(args.labels);rows=list(read_jsonl(args.curated))
    parent={r['id']:r for r in read_jsonl(args.parent)} if args.parent else {}
    accepted=[r for r in rows if r['eligible']]
    report={}
    for name in training_names(policy):
        images=[r for r in accepted if any(a.get('training_class')==name for a in r['annotations'])]
        annotations=[a for r in images for a in r['annotations'] if a.get('training_class')==name]
        new=[r for r in images if r['id'] not in parent or not parent[r['id']]['eligible']]
        report[name]=dict(images=len(images),objects=len(annotations),
            new_images=len(new),new_objects=sum(a.get('training_class')==name for r in new for a in r['annotations']),
            splits=dict(Counter(r['split'] for r in images)),
            recording_groups=len({r['split_group'] for r in images}),
            source_projects=len({r['source'] for r in images}),
            source_subtypes=dict(Counter(a.get('source_subtype') or a.get('morai_asset_name') or
                'unconfirmed:'+str(a.get('source_semantic_label')) for a in annotations)),
            views=dict(Counter(r.get('view','unknown') for r in images)),
            weather=dict(Counter(r.get('weather','unknown') for r in images)),
            tunnel=dict(Counter('unknown' if r.get('tunnel') is None else str(r['tunnel']) for r in images)),
            exact_asset_subtype_requires_separate_evidence=True,
            support_status='no_data' if not images else 'coverage_only_not_training_acceptance')
    current={r['id']:r for r in rows}
    preserved=all(current.get(ident)==row for ident,row in parent.items() if row['eligible'])
    save(args.output,dict(classes=report,images=len(accepted),
        objects=sum(bool(a.get('training_class')) for r in accepted for a in r['annotations']),
        new_images=sum(r['id'] not in parent or not parent[r['id']]['eligible'] for r in accepted) if parent else None,
        parent_accepted_rows_unchanged=preserved,parent_sha256=sha(args.parent) if args.parent else None,
        curated_sha256=sha(args.curated),labels_sha256=sha(args.labels),training_performed=False,
        group_count_limitation='Recording/split groups are not a verified independent-scene count. Repeated routes/actors and unknown lineage require review.',
        pending_reasons=dict(Counter(reason for r in rows if not r['eligible'] for reason in r['reasons']))))


def gallery(args):
    from PIL import Image, ImageDraw
    rows=list(read_jsonl(args.manifest));pred={}
    if args.predictions:pred={r['id']:r for r in read_jsonl(args.predictions)}
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'images').mkdir()
    entries=[]
    for index,row in enumerate(rows):
        image=Image.open(row['image']).convert('RGB');draw=ImageDraw.Draw(image)
        for ann in row['annotations']:
            color='#00dbff' if ann.get('canonical') else '#ffb000'
            draw.rectangle(ann['xyxy'],outline=color,width=2)
            draw.text(tuple(ann['xyxy'][:2]),str(ann.get('training_class') or ann.get('original_label')),fill=color)
        for p in pred.get(row['id'],{}).get('predictions',[]):
            if p['confidence']<.4:continue
            draw.rectangle(p['xyxy'],outline='#ff36d8',width=2)
            draw.text((p['xyxy'][0],max(0,p['xyxy'][1]-12)),p['raw_phrase']+' %.2f'%p['confidence'],fill='#ff36d8')
        image.thumbnail((960,720));name=row['id']+'.jpg';image.save(args.output/'images'/name)
        entries.append('<article><h3>%s · %s</h3><p>%s</p><a href="images/%s"><img loading="lazy" src="images/%s" width="720"></a></article>' %
            (html.escape(row['id'][:12]),html.escape(row.get('source','')),html.escape(row.get('source_key','')),name,name))
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Local teacher v6 review</title><h1>Local teacher v6 review</h1><p>Cyan: mapped original GT; orange: unresolved original; magenta: teacher proposal, not truth.</p>'+''.join(entries))


def comparison_gallery(args):
    """Same RGB, independent GT and every teacher; no generated GT or model vote."""
    from PIL import Image,ImageDraw
    rows=list(read_jsonl(args.manifest));lookup={}
    for directory in args.runs:
        binding=json.loads((directory/'binding.json').read_text())
        lookup[binding['model']]={r['id']:r for r in read_jsonl(directory/'predictions.jsonl')}
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'images').mkdir()
    cards=[]
    for index,row in enumerate(rows):
        images=[]
        for name,pred in [('independent_gt',None)]+list(lookup.items()):
            if pred is not None:
                if row['id'] not in pred:raise ValueError('gallery missing teacher result; do not display it as a miss')
                verify_prediction(row,pred[row['id']])
            image=Image.open(row['image']).convert('RGB');draw=ImageDraw.Draw(image)
            annotations=row['annotations'] if pred is None else pred.get(row['id'],{}).get('predictions',[])
            for annotation in annotations:
                if pred is not None and annotation['confidence']<.4:continue
                color='#00dbff' if pred is None else '#ff36d8'
                draw.rectangle(annotation['xyxy'],outline=color,width=2)
                label=(annotation.get('training_class') or annotation.get('original_label','unknown')) if pred is None else annotation['raw_phrase']+' %.2f'%annotation['confidence']
                draw.text(tuple(annotation['xyxy'][:2]),str(label),fill=color)
            image.thumbnail((640,480));filename='%04d-%s.jpg'%(index,name);image.save(args.output/'images'/filename,quality=90)
            caption=name if pred is not None or row.get('independent_gt_review') else 'Original RGB / unverified annotations; not empty GT'
            images.append('<figure><figcaption>%s</figcaption><a href="images/%s"><img loading="lazy" src="images/%s"></a></figure>'%(caption,filename,filename))
        cards.append('<article><h3>%d · %s · %s</h3><div class="grid">%s</div></article>'%(index,html.escape(row['source']),html.escape(row['source_key']),''.join(images)))
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>2026-10-11 · Local teacher v6 same RGB comparison</title><style>body{font:15px system-ui}figure{margin:4px}img{max-width:100%}.grid{display:grid;grid-template-columns:repeat(2,1fr)}article{border-top:1px solid #ccc}</style><h1>2026-10-11 · Local teacher v6 same RGB comparison</h1><a href="../../index.html">실험 목차</a><p>독립 GT와 teacher 원본 이름/박스, confidence 0.40. 이름 차이와 위치 차이를 구분합니다. GT 없는 영상은 사례 검토입니다.</p>'+''.join(cards))


def admission_gallery(args):
    """Show accepted changes only, preserving raw teacher mistakes on the left."""
    from PIL import Image,ImageDraw
    before={r['id']:r for r in read_jsonl(args.before)}
    after=[r for r in read_jsonl(args.after) if r['eligible'] and
           (r['id'] not in before or not before[r['id']]['eligible'])]
    predictions={}
    for path in args.predictions:
        predictions.update({r['id']:r for r in read_jsonl(path)})
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'images').mkdir()
    cards=[]
    for index,row in enumerate(after):
        figures=[];original=before.get(row['id'],dict(annotations=[]))
        if row['id'] in predictions:verify_prediction(row,predictions[row['id']])
        for side in ('before','after'):
            image=Image.open(row['image']).convert('RGB');draw=ImageDraw.Draw(image)
            if side=='before':
                for a in original['annotations']:
                    draw.rectangle(a['xyxy'],outline='#ffb000',width=2)
                    draw.text(tuple(a['xyxy'][:2]),str(a['original_label']),fill='#ffb000')
                for p in predictions.get(row['id'],{}).get('predictions',[]):
                    if p['confidence']<.15:continue
                    draw.rectangle(p['xyxy'],outline='#ff36d8',width=2)
                    draw.text(tuple(p['xyxy'][:2]),p['raw_phrase']+' %.2f'%p['confidence'],fill='#ff36d8')
            else:
                for a in row['annotations']:
                    if not a.get('training_class'):continue
                    draw.rectangle(a['xyxy'],outline='#00dbff',width=2)
                    draw.text(tuple(a['xyxy'][:2]),a['training_class'],fill='#00dbff')
            image.thumbnail((640,480));filename='%04d-%s.jpg'%(index,side);image.save(args.output/'images'/filename,quality=92)
            figures.append('<figure><figcaption>%s</figcaption><a href="images/%s"><img loading="lazy" src="images/%s"></a></figure>'%(side,filename,filename))
        cards.append('<article><h3>%d · %s · %s</h3><div class="grid">%s</div><details><summary>검수 근거·원본 actor 기록</summary><pre>%s</pre></details></article>'%(
            index,html.escape(row['source']),html.escape(row['source_key']),''.join(figures),
            html.escape(json.dumps(dict(review=row.get('teacher_review'),basis=row.get('review_basis'),
                cohort=row.get('direct_cohort_evidence'),actor_model_priors=row.get('actor_model_priors')),ensure_ascii=False,indent=2))))
    title=html.escape(getattr(args,'title',None) or '2026-10-11 · v6 bbox labels before / after')
    (args.output/'index.html').write_text(('<!doctype html><meta charset="utf-8"><title>__TITLE__</title><style>body{font:15px system-ui}.grid{display:grid;grid-template-columns:repeat(2,1fr)}figure{margin:4px}img{max-width:100%}pre{white-space:pre-wrap}article{border-top:1px solid #aaa}</style><h1>__TITLE__</h1><a href="../../index.html">실험 목차</a><p>왼쪽: 주황 원본 주석 / 분홍 teacher 제안(confidence≥0.15, 제공된 경우). 오른쪽: 고정 표본 묶음 검수 또는 개별 RGB 검수로 채택한 학습 라벨. 검수 방식은 각 근거에 표시합니다. Teacher 미검출은 빈 정답이 아닙니다. 신규 채택 __COUNT__장. 원본 데이터·기존 평가 split은 유지했습니다.</p>'.replace('__COUNT__',str(len(after))).replace('__TITLE__',title))+''.join(cards))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    root=Path(__file__).resolve().parents[2]
    for name in ('inventory','samples','prepare-gt','aim-queue','bag-queue','infer','compare','export','registry','coverage','gallery','comparison-gallery','admission-gallery'):
        p=sub.add_parser(name)
        if name in ('inventory','samples','prepare-gt','export','coverage'):p.add_argument('--curated',type=Path,required=True)
        if name in ('inventory','samples','prepare-gt','aim-queue','infer','compare'):p.add_argument('--config',type=Path,default=root/'config/experiments/camera_local_teacher_v6.json')
        if name in ('infer','prepare-gt','compare','export','coverage'):p.add_argument('--labels',type=Path,default=root/'config/datasets/camera_object_labels_v2.json')
        p.add_argument('--output',type=Path,required=True)
        if name in ('infer','compare','gallery','comparison-gallery'):p.add_argument('--manifest',type=Path,required=True)
        if name=='inventory':p.add_argument('--aim',type=Path)
        if name=='prepare-gt':
            p.add_argument('--per-class-run',type=int,default=40);p.add_argument('--actor-reviews',type=Path)
        if name=='aim-queue':
            p.add_argument('--inventory',type=Path,required=True);p.add_argument('--per-session',type=int,default=12)
        if name=='bag-queue':p.add_argument('--bags',type=Path,nargs='+',required=True)
        if name=='infer':
            p.add_argument('--model',choices=['yoloe_x','gdino_b','mm_gdino_l'],required=True)
            p.add_argument('--models',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
            p.add_argument('--hood-config',type=Path,default=root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml')
            p.add_argument('--contrast',action='store_true');p.add_argument('--limit',type=int,default=0);p.add_argument('--max-seconds',type=float,default=0)
            p.add_argument('--reuse',type=Path)
        if name=='compare':p.add_argument('--runs',type=Path,nargs='+',required=True)
        if name=='comparison-gallery':p.add_argument('--runs',type=Path,nargs='+',required=True)
        if name=='export':
            p.add_argument('--reviews',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--additional',type=Path)
            p.add_argument('--batches',type=Path);p.add_argument('--approval',type=Path);p.add_argument('--predictions',type=Path)
        if name=='registry':
            p.add_argument('--root',type=Path,required=True);p.add_argument('--current',required=True)
            p.add_argument('--supersedes',nargs='*',default=[])
        if name=='coverage':p.add_argument('--parent',type=Path)
        if name=='admission-gallery':
            p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True)
            p.add_argument('--predictions',type=Path,nargs='*',default=[])
            p.add_argument('--title')
        if name=='gallery':p.add_argument('--predictions',type=Path)
    args=parser.parse_args()
    functions={'inventory':inventory,'samples':samples,'prepare-gt':prepare_gt,'aim-queue':aim_queue,'bag-queue':bag_queue,'infer':run_inference,'compare':compare,'export':export,'registry':registry,'coverage':coverage,'gallery':gallery,'comparison-gallery':comparison_gallery,'admission-gallery':admission_gallery}
    functions[args.command](args)


if __name__=='__main__':main()
