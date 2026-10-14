#!/usr/bin/env python3
"""Serial, resumable full-data training with external evaluation and a wall budget.

The original splits are immutable. Previously inspected pilot test images are
explicit development diagnostics, never a new blind final test.
"""
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

from camera_pretrain_policy import CLASSES, sha
from train_camera_bbox_pilot import next_epoch_fits, save

OLD_CLASSES=('person','dog','bicycle','car','motorcycle','bus','truck')
NEW_CLASSES=set(CLASSES)-set(OLD_CLASSES)
TOOLS=Path(__file__).resolve().parent


def read_rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def prepare_evaluation(dataset, legacy, output):
    import cv2
    from PIL import Image
    from camera_pretrain_policy import validate_dataset_contract, load_policy, view_masks
    from prepare_camera_pretrain import normalize_row
    meta=validate_dataset_contract(dataset,ignore_aware=True)
    originals=read_rows(dataset/'manifest.jsonl')
    train=[r for r in originals if r['split']=='train']
    train_files={r['image_sha256'] for r in train}
    train_pixels={r['pixel_sha256'] for r in train}
    train_groups={r['split_group'] for r in train}
    current={r['id']:r for r in originals if r['split']!='train'}
    candidates={r['id']:r for r in originals if r['split']=='selection'}
    historical=[r for r in read_rows(legacy/'manifest.jsonl') if r['split'] in ('selection','heldout')] if legacy else []
    policy=load_policy(dataset/'policy.json');masks=view_masks(policy)
    cache={'parent_shas':{str(legacy):sha(legacy/'manifest.jsonl')} if legacy else {}}
    aliases=json.loads((TOOLS.parents[1]/'config/datasets/camera_object_labels_v2.json').read_text())['training_map']
    for original in historical:
        if original['id'] in current:
            row=deepcopy(current[original['id']])
        else:
            old=deepcopy(original)
            for ann in old['annotations']:
                label=ann.get('training_class') or ann.get('canonical')
                ann['training_class']=aliases.get(label,label)
                if ann['training_class'] not in CLASSES:ann['disposition']='excluded'
            path=Path(old['image'])
            if sha(path)!=old['image_sha256']:raise ValueError('legacy RGB SHA mismatch')
            row=normalize_row(legacy,old,cv2.imread(str(path)),policy,masks,cache)
            for before,after in zip(original['annotations'],row['annotations']):
                label=before.get('training_class') or before.get('canonical')
                if aliases.get(label,label) not in CLASSES and before.get('disposition')!='excluded':
                    after['supervision']='ignore';after['training_class']=None
                    after['supervision_reasons']=['unresolved_original_label_not_background']
                    row['supervision']='partial'
            rgb=Image.open(path).convert('RGB')
            row['pixel_sha256']=hashlib.sha256(f'{rgb.width}x{rgb.height}:'.encode()+rgb.tobytes()).hexdigest()
        if row['id'] not in candidates:candidates[row['id']]=row
    rows=[];seen=set();excluded=[]
    for r in sorted(candidates.values(),key=lambda r:r['id']):
        if r['image_sha256'] in train_files or r['pixel_sha256'] in train_pixels or r['split_group'] in train_groups:
            raise ValueError('diagnostic/train image or lineage leak: '+r['id'])
        if r['pixel_sha256'] in seen:
            excluded.append(dict(id=r['id'],reason='exact_duplicate_diagnostic'))
            continue
        seen.add(r['pixel_sha256'])
        r=deepcopy(r)
        r['frozen_source_split']=r['split']
        r['evaluation_role']='previously_seen_development_diagnostic'
        r['split']='selection'  # Only this DERIVED evaluation manifest has a role alias.
        rows.append(r)
    output.mkdir(parents=True,exist_ok=False)
    (output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    save(output/'summary.json',dict(classes=list(CLASSES),images=len(rows),
        gt=dict(Counter(a['training_class'] for r in rows for a in r['annotations']
            if a['supervision']=='positive')),supervision=dict(Counter(r['supervision'] for r in rows)),
        original_splits=dict(Counter(r['frozen_source_split'] for r in rows)),
        parent_manifest_sha256=sha(dataset/'manifest.jsonl'),legacy_manifest_sha256=sha(legacy/'manifest.jsonl') if legacy else None,
        diagnostic_manifest_sha256=sha(output/'manifest.jsonl'),excluded=excluded,
        dataset_splits_changed=False,blind_test=False,train_leaks=0,
        policy='v8 positive/ignore/mask; no display cutoffs; previously inspected data'))
    return rows


def evaluation_epochs(maximum=40, interval_after_four=4):
    if maximum < 1 or interval_after_four < 1:
        raise ValueError('epoch maximum/interval must be positive')
    return sorted(e for e in {1,2,4,*range(4+interval_after_four,maximum+1,interval_after_four),maximum}
                  if e <= maximum)


def compare_metrics(baseline,current,minimum_new_gt=20,quality_policy=None):
    before=baseline['control']['selection']['class']['classes']
    after=current['control']['selection']['class']['classes']
    rows={};regressions=[];insufficient=[];quality_regressions=[]
    for name in CLASSES:
        b=before.get(name,{});a=after.get(name,{})
        recall=a.get('annotated_recall');reference=b.get('annotated_recall')
        delta=recall-reference if recall is not None and reference is not None else None
        sufficient=a.get('gt',0)>=20 and b.get('gt',0)==a.get('gt',0)
        rows[name]=dict(gt=a.get('gt',0),tp=a.get('tp',0),recall=recall,
            baseline_recall=reference,delta=delta,complete_gt=a.get('complete_gt',0),
            fp=a.get('complete_fp'),f1=a.get('f1'),precision=a.get('precision'),
            baseline_fp=b.get('complete_fp'),baseline_f1=b.get('f1'),baseline_precision=b.get('precision'),
            sufficient_regression_sample=sufficient)
        if (quality_policy and name in OLD_CLASSES and a.get('complete_gt',0)>=20 and
                a.get('complete_gt')==b.get('complete_gt')):
            for metric in ('precision','f1'):
                limit=quality_policy.get(metric+'_drop_limit')
                if limit is None:continue
                if limit<0:raise ValueError('quality drop limit must be nonnegative')
                value,reference_value=a.get(metric),b.get(metric)
                if value is not None and reference_value is not None and value-reference_value < -limit-1e-12:
                    quality_regressions.append(dict(class_name=name,metric=metric,
                        baseline=reference_value,current=value,delta=value-reference_value,limit=limit))
        if name in OLD_CLASSES:
            if not sufficient:insufficient.append(name)
            elif delta is not None and delta < -.02-1e-12:regressions.append(name)
    scores=[r['f1'] for name,r in rows.items() if name in NEW_CLASSES and
        r['complete_gt']>=minimum_new_gt and r['f1'] is not None]
    return dict(classes=rows,regressions=regressions,insufficient_old_classes=insufficient,
        quality_regressions=quality_regressions,
        quality_policy=quality_policy or {},quality_policy_scope='development candidate gate, not automatic training pause',
        supported_new_macro_f1=sum(scores)/len(scores) if scores else None,
        supported_new_classes=[name for name,r in rows.items() if name in NEW_CLASSES and
            r['complete_gt']>=minimum_new_gt and r['f1'] is not None],confidence=.4,
        macro_f1_fixed16=current['control']['selection']['class'].get('macro_f1_fixed16'),
        small_sample_classes=[n for n,r in rows.items() if r['complete_gt']<20],
        production_approved=False,blind_test=False)


def consecutive_regressions(previous,current):
    return sorted(set(previous.get('regressions',[])) & set(current.get('regressions',[])))


def regression_pause_required(classes, continue_on_regression=False):
    return bool(classes) and not continue_on_regression


def checkpoint_score(metrics):
    if metrics['regressions'] or metrics.get('quality_regressions') or metrics['supported_new_macro_f1'] is None:return None
    return metrics['supported_new_macro_f1']


def call(command,log):
    with Path(log).open('a') as stream:
        stream.write('\nCOMMAND '+json.dumps(list(map(str,command)))+'\n');stream.flush()
        return subprocess.run(list(map(str,command)),stdout=stream,stderr=subprocess.STDOUT).returncode


def evaluate(args,weights,folder,width=1280,confidence=.4,dataset=None):
    if (folder/'evaluation.json').exists():return json.loads((folder/'evaluation.json').read_text())
    rc=call([sys.executable,TOOLS/'evaluate_camera_bbox_pilot.py','--config',args.config,
        '--dataset',dataset or args.output/'evaluation','--weights',weights,'--output',folder,
        '--front-width',width,'--baseline-policy','initialization','--skip-temporal',
        '--selection-only',*(['--fixed-confidence',confidence] if confidence is not None else [])],
        args.output/'evaluation.log')
    if rc:raise RuntimeError('external evaluation failed: '+str(folder))
    return json.loads((folder/'evaluation.json').read_text())


def page(output,state):
    def val(v):return '—' if v is None else f'{100*v:.2f}%'
    summary=json.loads((output/'evaluation/summary.json').read_text()) if (output/'evaluation/summary.json').exists() else {}
    body='<p>Status: '+html.escape(state['status'])+'</p><p>'+html.escape(state.get('message',''))+'</p>'
    if (output/'main/run.json').exists():
        progress=json.loads((output/'main/run.json').read_text())
        body+='<pre>'+html.escape(json.dumps({k:progress.get(k) for k in
            ['status','completed_epochs','completed_batches','batch_seconds','last_loss','amp_skipped_steps',
             'finite_gradient_steps','epoch_seconds_estimate']},ensure_ascii=False,indent=2))+'</pre>'
    body+='<table><tr><th>Epoch</th><th>New macro F1</th><th>Recall regression</th><th>Precision/F1 review</th><th>Checkpoint</th></tr>'
    for entry in state.get('evaluations',[]):
        quality=', '.join(x['class_name']+':'+x['metric'] for x in entry['metrics'].get('quality_regressions',[]))
        body+='<tr><td>'+str(entry['epoch'])+'</td><td>'+val(entry['metrics']['supported_new_macro_f1'])+'</td><td>'+html.escape(', '.join(entry['metrics']['regressions']))+'</td><td>'+html.escape(quality)+'</td><td>'+html.escape(entry['checkpoint'])+'</td></tr>'
    body+='</table>'
    if state.get('evaluations'):
        body+='<h2>Latest class metrics · confidence 0.40</h2><table><tr><th>Class</th><th>GT/TP</th><th>Before recall</th><th>After recall</th><th>Δ</th><th>Complete FP</th><th>Complete F1</th></tr>'
        for name,r in state['evaluations'][-1]['metrics']['classes'].items():
            body+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in
                [name,f"{r['gt']}/{r['tp']}",val(r['baseline_recall']),val(r['recall']),val(r['delta']),r['fp'],val(r['f1'])])+'</tr>'
        body+='</table>'
    body+='<p>Previously inspected development diagnostics; not blind test or deployment acceptance. Partial GT supplies recall only. Sources/frames are correlated. Offline PT timing is not integrated latency.</p>'
    body+='<p><a href="gallery/index.html">GT / before / after · failures</a></p>' if (output/'gallery/index.html').exists() else ''
    body+='<details><summary>Evaluation provenance</summary><pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre></details>'
    stamp=state['started_utc']
    version=state.get('dataset_version','camera_objects_bbox_v8').split('_bbox_')[-1].split('-')[0]
    (output/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta http-equiv="refresh" content="60"><title>'+stamp+' · YOLO26l '+version+' full-data training</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;background:#101827;color:#f0f4fa}td,th{padding:8px;border:1px solid #526079}table{border-collapse:collapse}a{color:#5eddfa}pre{white-space:pre-wrap}</style><a href="../index.html">실험 목차</a><h1>'+stamp+' · YOLO26l '+version+' 본학습</h1>'+body+'</html>')


def gallery(output,folder):
    import cv2
    from evaluate_camera_bbox_pilot import frame_counts
    rows=read_rows(output/'evaluation/manifest.jsonl')
    before={p['id']:p for p in read_rows(output/'baseline-1280/predictions.jsonl')}
    after={p['id']:p for p in read_rows(folder/'predictions.jsonl')}
    selected=[]
    for row in rows:
        counts,_=frame_counts(row,after[row['id']]['predictions'],.4)
        priority=sum(c.get('complete_predictions',0)-c.get('complete_tp',0)+c.get('gt',0)-c.get('tp',0) for c in counts.values())
        selected.append((priority,row))
    destination=output/'gallery';destination.mkdir(exist_ok=True)
    cards=[]
    for _,row in sorted(selected,key=lambda p:(-p[0],p[1]['id']))[:32]:
        image=cv2.imread(row['image']);panels=[]
        for title,predictions in [('GT',[]),('Before · .40',before[row['id']]['predictions']),('After · .40',after[row['id']]['predictions'])]:
            canvas=image.copy()
            for ann in row['annotations']:
                if ann['supervision']=='excluded':continue
                color=(255,255,0) if ann['supervision']=='positive' else (255,0,255)
                x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(canvas,(x1,y1),(x2,y2),color,1)
                cv2.putText(canvas,str(ann.get('training_class') or 'ignore'),(x1,max(y1-3,12)),0,.38,color,1)
            for pred in predictions:
                if pred['confidence']<.4:continue
                x1,y1,x2,y2=map(int,pred['xyxy']);cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,210,255),2)
                cv2.putText(canvas,f"{pred['class_name']} {pred['confidence']:.2f}",(x1,max(y1+14,14)),0,.45,(0,210,255),1)
            canvas=cv2.resize(canvas,(640,round(640*canvas.shape[0]/canvas.shape[1])))
            cv2.putText(canvas,title,(10,22),0,.65,(255,255,255),2);panels.append(canvas)
        import numpy as np
        cv2.imwrite(str(destination/(row['id']+'.jpg')),np.hstack(panels))
        cards.append('<p>'+html.escape(row['source']+' · '+row['supervision'])+'</p><img loading="lazy" src="'+row['id']+'.jpg">')
    (destination/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>'+datetime.now(timezone.utc).isoformat()+' · YOLO26l v8 failures</title><style>body{background:#101827;color:white;font:16px sans-serif}img{max-width:100%}a{color:cyan}</style><a href="../index.html">본학습 결과</a><p>GT / before / after. Cyan positive, magenta ignore, orange prediction. Partial unmatched predictions are not FP. RGB panels show original images; inference uses frozen Ego masks.</p>'+''.join(cards))


def supervise(args):
    cfg=json.loads(args.config.read_text());deadline=time.time()+cfg['main']['budget_seconds']
    state_path=args.output/'supervisor.json'
    if state_path.exists():
        state=json.loads(state_path.read_text());deadline=state['deadline_unix']
        if state['config_sha256']!=sha(args.config):raise ValueError('supervisor configuration changed')
    else:
        state=dict(status='starting',started_utc=datetime.now(timezone.utc).isoformat(),deadline_unix=deadline,
            config_sha256=sha(args.config),dataset_metadata_sha256=sha(args.dataset/'pretrain.json'),
            dataset_version=args.dataset.name,
            evaluations=[],runtime_changes=False,production_approved=False)
    def update(status,message=''):
        state.update(status=status,message=message,updated_utc=datetime.now(timezone.utc).isoformat())
        save(state_path,state);page(args.output,state);print(json.dumps(dict(status=status,message=message)),flush=True)
    try:
        continue_on_regression = bool(getattr(args, 'continue_on_regression', False))
        state['regression_stop_enabled'] = not continue_on_regression
        state.setdefault('execution_history', []).append(dict(
            started_utc=datetime.now(timezone.utc).isoformat(),
            regression_stop_enabled=not continue_on_regression,
            supervisor_sha256=sha(Path(__file__))))
        state.pop('stop_classes', None)
        update('baseline_evaluation')
        weights=args.data_root/cfg['models']['yolo26l']
        baseline=evaluate(args,weights,args.output/'baseline-1280')
        if 'batch' not in state:
            for batch in cfg['training']['batch_candidates']:
                update('smoke',f'batch={batch}, 200 batches; discarded smoke weights')
                folder=args.output/f'smoke-b{batch}'
                rc=call([sys.executable,TOOLS/'train_camera_bbox_pilot.py','--config',args.config,
                    '--dataset',args.dataset,'--data-root',args.data_root,'--output',folder,
                    '--model','yolo26l','--batch',batch,'--epochs',cfg['main']['epochs'],
                    '--smoke-batches',200,'--max-seconds',3600,'--pretrain'],args.output/f'smoke-b{batch}.log')
                result=json.loads((folder/'run.json').read_text()) if (folder/'run.json').exists() else {}
                if rc==0 and result.get('smoke_complete') and result.get('finite_gradient_steps',0)>0:
                    state['batch']=batch;state['smoke']=result;break
                text=(args.output/f'smoke-b{batch}.log').read_text(errors='replace')
                if 'out of memory' not in text.lower() and 'OutOfMemoryError' not in text:
                    raise RuntimeError('non-OOM smoke failure; explicit fix required')
            else:raise RuntimeError('all explicit batch candidates failed')
            update('smoke_passed',f"batch={state['batch']}; epoch estimate {state['smoke']['epoch_seconds_estimate']:.0f}s")
        main=args.output/'main';resume=None
        if (main/'run.json').exists():
            record=json.loads((main/'run.json').read_text())
            if record.get('completed_epochs',0):resume=Path(record['resumable_checkpoint'])
        for target in evaluation_epochs(cfg['main']['epochs'],cfg['evaluation'].get('interval_after_four',4)):
            if any(e['epoch']==target for e in state['evaluations']):continue
            remaining=deadline-time.time()
            timing=state.get('epoch_seconds',[]) or [state['smoke']['epoch_seconds_estimate']]
            if not next_epoch_fits(remaining,timing,reserve=cfg['main']['reserve_seconds']):
                update('paused_budget','Insufficient budget for another complete epoch and evaluation');break
            update('training',f"through epoch {target}; batch={state['batch']}")
            command=[sys.executable,TOOLS/'train_camera_bbox_pilot.py','--config',args.config,
                '--dataset',args.dataset,'--data-root',args.data_root,'--output',main,
                '--model','yolo26l','--batch',state['batch'],'--epochs',cfg['main']['epochs'],
                '--stop-after-epoch',target,'--max-seconds',max(1,remaining),'--epoch-budget',
                '--reserve-seconds',cfg['main']['reserve_seconds'],'--pretrain']
            if resume:command+=['--resume',resume]
            rc=call(command,args.output/'training.log')
            if rc:raise RuntimeError('main training failed; inspect training.log')
            record=json.loads((main/'run.json').read_text());completed=record['completed_epochs']
            state['epoch_seconds']=state.get('epoch_seconds',[])+record['epoch_seconds']
            resume=main/'fit/weights'/f'epoch{completed-1}.pt'
            if not (args.output/'initialization-1280/evaluation.json').exists():
                evaluate(args,main/'initial.pt',args.output/'initialization-1280')
            folder=args.output/f'epoch-{completed:03d}-1280'
            update('checkpoint_evaluation',f'completed epoch {completed}')
            metrics=compare_metrics(baseline,evaluate(args,resume,folder),
                cfg['evaluation'].get('minimum_new_gt',20),cfg['evaluation'].get('existing_class_quality'))
            if cfg.get('regression_dataset'):
                evaluate(args,resume,args.output/f'regression-{completed:03d}',
                    dataset=Path(cfg['regression_dataset']))
            stop=consecutive_regressions(state['evaluations'][-1]['metrics'] if state['evaluations'] else {},metrics)
            entry=dict(epoch=completed,metrics=metrics,checkpoint=str(resume),sha256=sha(resume))
            state['evaluations'].append(entry)
            score=checkpoint_score(metrics)
            if score is not None and score>state.get('best_score',-1):
                destination=args.output/'best-development.pt';shutil.copy2(resume,destination)
                state.update(best_score=score,best_epoch=completed,best_checkpoint=str(destination),
                    best_checkpoint_sha256=sha(destination))
            gallery(args.output,folder);update('checkpoint_evaluated',f'epoch={completed}; regressions={metrics["regressions"]}')
            if regression_pause_required(stop, continue_on_regression):
                state['stop_classes']=stop;update('paused_regression','Two consecutive regression checks: '+', '.join(stop));break
            if stop:
                update('regression_warning','Continuing by explicit override; regressions: '+', '.join(stop))
            if record.get('budget_epoch_pause') or completed<target:
                update('paused_budget','Saved a full epoch checkpoint');break
        else:update('completed_schedule')
        if state.get('best_checkpoint') and deadline-time.time()>cfg['main']['reserve_seconds']:
            evaluate(args,Path(state['best_checkpoint']),args.output/'best-960',width=960)
            tuned=evaluate(args,Path(state['best_checkpoint']),args.output/'best-confidence',confidence=None)
            state['development_confidence']=tuned['selection_confidence']
            state['confidence_scope']='previously_seen_development; no production promotion'
            save(state_path,state);page(args.output,state)
        return state
    except BaseException as exc:
        state['error']=dict(type=type(exc).__name__,message=str(exc))
        update('failed',str(exc));raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--data-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--legacy',type=Path)
    parser.add_argument('--continue-on-regression',action='store_true',
        help='Report regression checks without pausing; retain budget and error stops.')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.action=='prepare':prepare_evaluation(args.dataset,args.legacy,args.output/'evaluation')
    else:supervise(args)


if __name__=='__main__':main()
