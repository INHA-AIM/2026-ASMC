#!/usr/bin/env python3
"""Versioned post-NMS metrics for saved pilot/v8-v11 predictions, preserving originals."""
import argparse
import fcntl
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import subprocess
import sys
import time
from camera_prediction_nms import POLICY, suppress


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()


def read(p):return json.loads(Path(p).read_text())
def rows(p):return [json.loads(x) for x in Path(p).read_text().splitlines() if x]
def save(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');tmp.replace(p)


def score_classes(summary):
    classes=summary['classes']
    names=('person dog deer bicycle car motorcycle bus truck bollard box concrete_barrier steel_barricade traffic_drum plastic_barrier traffic_cone stroller').split()
    vals=[classes.get(c,{}).get('f1') for c in names]
    summary['macro_f1_fixed16']=sum(vals)/16 if all(v is not None for v in vals) else None
    summary['complete_fp_total']=sum(v.get('complete_fp',0) for v in classes.values())
    return summary


def worker(job):
    source=Path(job['evaluator']);sys.path.insert(0,str(source.parent))
    from evaluate_camera_bbox_pilot import summarize, frame_counts
    manifest=Path(job['manifest']);folder=Path(job['folder']);out=Path(job['output'])
    original=rows(folder/'predictions.jsonl'); metadata=read(folder/'inference.json')
    gt=rows(manifest);pred_by={p['id']:p for p in original}
    if len(pred_by)!=len(original):raise ValueError('duplicate prediction image IDs')
    old=read(folder/'evaluation.json'); all_reports={}; reduced={}; removed_by={}
    for pred in original:
        boxes,removed=suppress(pred['predictions'],**dict(confidence=POLICY['confidence'],threshold=POLICY['iou']))
        reduced[pred['id']]=dict(pred,predictions=boxes);removed_by[pred['id']]=removed
    for role in ('selection','heldout'):
        part=[r for r in gt if r['split']==role]
        if not part:continue
        before=[pred_by[r['id']] for r in part]
        after=[reduced[r['id']] for r in part]
        raw=summarize(part,before,.4,pixel_scope=True)
        recorded=old['control'][role]['class']
        # Reproduce raw TP/FP/FN before changing postprocessing. Reject incompatible evaluators.
        for c in set(raw['classes'])|set(recorded['classes']):
            for key in ('gt','tp','complete_gt','complete_tp','complete_predictions'):
                if raw['classes'].get(c,{}).get(key,0)!=recorded['classes'].get(c,{}).get(key,0):
                    raise ValueError('raw evaluator mismatch '+role+'/'+c+'/'+key)
        processed=summarize(part,after,.4,pixel_scope=True)
        lost=[];gained=[]; frames=[]
        for r in part:
            a,_=frame_counts(r,pred_by[r['id']]['predictions'],.4,pixel_scope=True)
            b,_=frame_counts(r,reduced[r['id']]['predictions'],.4,pixel_scope=True)
            for c in set(a)|set(b):
                change=b.get(c,{}).get('tp',0)-a.get(c,{}).get('tp',0)
                if change: (gained if change>0 else lost).append(dict(id=r['id'],class_name=c,change=change))
            n=len(removed_by[r['id']])
            if n:frames.append(dict(id=r['id'],removed=n))
        all_reports[role]=dict(raw=score_classes(raw),post_nms=score_classes(processed),
            accepted_post_nms=summarize(part,after,.4,level='accepted',pixel_scope=True),
            removed=sum(x['removed'] for x in frames),affected_images=len(frames),
            lost_tp=lost,gained_tp=gained,frames=frames)
    result=dict(version=1,policy=POLICY,folder=str(folder),manifest=str(manifest),
        manifest_sha256=sha(manifest),predictions_sha256=sha(folder/'predictions.jsonl'),
        original_evaluation_sha256=sha(folder/'evaluation.json'),inference_sha256=sha(folder/'inference.json'),
        checkpoint_sha256=metadata['weights_sha256'],precision='FP16' if metadata.get('actual_fp16') else 'FP32',
        front_width=metadata['front_width'],evaluator=str(source),evaluator_sha256=sha(source),
        historical_evaluator_hash_exact=job['exact_evaluator'],raw_counts_reproduced=True,
        original_metrics_preserved=True,runtime_modified=False,roles=all_reports,
        postprocess_source_sha256=sha(Path(__file__).with_name('camera_prediction_nms.py')))
    save(out,result)


def discover(root):
    runs=[]
    for v in ('v8','v9','v10','v11'):
        runs.extend((root/'artifacts'/('camera-bbox-main-'+v)).glob('run-*'))
    folders=[p.parent for r in runs for p in r.rglob('inference.json') if (p.parent/'evaluation.json').exists()]
    pilot=root/'artifacts/camera-bbox-pilot-v1/paired-r1'
    folders.extend(p.parent for p in pilot.glob('*/inference.json') if (p.parent/'evaluation.json').exists())
    manifests=[p for r in runs for p in r.glob('evaluation/manifest.jsonl')]
    manifests.extend(root/'data/dataset_versions'/name/'manifest.jsonl' for name in (
        'camera_objects_bbox_validation16_v1-r2','camera_objects_bbox_pilot_v1-r4'))
    by_manifest={sha(p):p for p in manifests if p.exists()}
    sources=[r/'code/tools/perception/evaluate_camera_bbox_pilot.py' for r in runs]
    sources.append(Path(__file__).with_name('evaluate_camera_bbox_pilot.py'))
    by_source={sha(p):p for p in sources if p.exists()}
    jobs=[]
    for folder in sorted(set(folders)):
        meta=read(folder/'inference.json')
        if meta['manifest_sha256'] not in by_manifest:
            raise ValueError('unknown manifest '+str(folder))
        requested=meta.get('metric_evaluator_sha256',meta.get('evaluator_source_sha256'))
        # Older pilot lacks its frozen evaluator; accept current only if all original raw counts reproduce.
        evaluator=by_source.get(requested,Path(__file__).with_name('evaluate_camera_bbox_pilot.py'))
        jobs.append(dict(folder=str(folder),manifest=str(by_manifest[meta['manifest_sha256']]),
                         evaluator=str(evaluator),exact_evaluator=requested in by_source))
    return jobs,runs


def aggregate(root,out):
    jobs,runs=discover(root); results=[];errors=[]
    for job in jobs:
        folder=Path(job['folder']);rel=folder.relative_to(root/'artifacts')
        destination=out/'results'/rel/'metrics.json';job['output']=str(destination)
        cached=read(destination) if destination.exists() else None
        if (not cached or cached['predictions_sha256']!=sha(folder/'predictions.jsonl') or
                cached.get('postprocess_source_sha256')!=sha(Path(__file__).with_name('camera_prediction_nms.py')) or
                cached['original_evaluation_sha256']!=sha(folder/'evaluation.json')):
            request=out/'jobs'/rel/'request.json';save(request,job)
            result=subprocess.run([sys.executable,__file__,'--worker',str(request)],capture_output=True,text=True)
            if result.returncode:
                errors.append(dict(folder=str(folder),error=result.stderr[-2500:]));continue
            cached=read(destination)
        results.append(dict(path=str(destination.relative_to(out)),experiment=str(rel),**cached))
    payload=dict(updated_utc=datetime.now(timezone.utc).isoformat(),policy=POLICY,experiments=results,errors=errors,
                 comparison_rule='Compare epochs only within identical manifest/width/precision. No test-driven tuning.',
                 runtime_changed=False,original_scores_deleted=False)
    save(out/'summary.json',payload)
    render(out,payload)
    # A sidecar per run feeds the live dashboard; the training supervisor is not changed.
    for run in runs:
        selected=[r for r in results if Path(r['folder']).parent==run and
                  Path(r['folder']).name.startswith('epoch-') and r['front_width']==1280]
        base=next((r for r in results if Path(r['folder'])==run/'baseline-1280'),None)
        evals=[]
        if base and 'selection' in base['roles']:
            b=base['roles']['selection']['post_nms']['classes']
            for r in sorted(selected,key=lambda r:Path(r['folder']).name):
                m=r['roles']['selection']['post_nms'];cl={}
                for name,v in m['classes'].items():
                    cl[name]=dict(v,recall=v.get('annotated_recall'),delta=(v.get('annotated_recall') or 0)-(b.get(name,{}).get('annotated_recall') or 0))
                new=[v['f1'] for c,v in m['classes'].items() if c not in ('person','dog','bicycle','car','motorcycle','bus','truck') and v.get('complete_gt',0)>=20 and v.get('f1') is not None]
                regress=[c for c in ('person','dog','bicycle','car','motorcycle','bus','truck') if cl.get(c,{}).get('gt',0)>=20 and cl[c]['delta']<-.02]
                evals.append(dict(epoch=int(Path(r['folder']).name.split('-')[1]),metrics=dict(classes=cl,
                    macro_f1_fixed16=m['macro_f1_fixed16'],supported_new_macro_f1=sum(new)/len(new) if new else None,
                    regressions=regress),raw_macro_f1_fixed16=r['roles']['selection']['raw']['macro_f1_fixed16']))
        save(run/'post-nms-evaluations.json',dict(policy=POLICY,updated_utc=payload['updated_utc'],evaluations=evals,
            note='Class-aware output NMS added offline; runtime unchanged. Raw supervisor metrics preserved.',
            report='../../'+out.name+'/index.html'))
        index=run/'index.html'
        if index.exists():
            content=index.read_text()
            if 'id="post-nms-policy-note"' not in content:
                # Preserve the old page before adding an explicit policy cross-reference.
                archive=out/'original-pages'/run.parent.name/run.name
                archive.mkdir(parents=True,exist_ok=True)
                if not (archive/'index.original.txt').exists():(archive/'index.original.txt').write_text(content)
                note='<aside id="post-nms-policy-note" style="padding:15px;background:#3b3020;color:#ffe2a6">이 페이지의 기존 수치는 원본 NMS-free 출력입니다. <a style="color:#9edbff" href="live.html">중복 억제 후 지표·추세</a> 또는 <a style="color:#9edbff" href="../../'+out.name+'/index.html">과거 실험 재평가</a>에서 새 정책과 원본을 비교하세요. 운영 후처리는 변경하지 않았습니다.</aside>'
                if '<body' in content:
                    start=content.find('>',content.find('<body'))+1
                else:start=content.find('</style>')+len('</style>') if '</style>' in content else 0
                tmp=index.with_suffix('.nms.tmp');tmp.write_text(content[:start]+note+content[start:]);tmp.replace(index)
    return payload


def render(out,payload):
    cards=[]
    for exp in payload['experiments']:
        lines=[]
        for role,v in exp['roles'].items():
            def fmt(x):return '—' if x is None else f'{x*100:.2f}%'
            lines.append('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in [role,fmt(v['raw']['macro_f1_fixed16']),fmt(v['post_nms']['macro_f1_fixed16']),v['raw']['complete_fp_total'],v['post_nms']['complete_fp_total'],v['removed'],sum(-x['change'] for x in v['lost_tp'])])+'</tr>')
        cards.append('<article><h2>'+html.escape(exp['experiment'])+'</h2><p>'+html.escape(exp['precision']+' / front '+str(exp['front_width'])+' / GT SHA '+exp['manifest_sha256'][:12])+'</p><table><tr><th>분할</th><th>원본16 F1</th><th>NMS16 F1</th><th>원본FP</th><th>NMS FP</th><th>제거박스</th><th>감소TP</th></tr>'+''.join(lines)+'</table><details><summary>클래스별 수치</summary><pre>'+html.escape(json.dumps({role:v['post_nms']['classes'] for role,v in exp['roles'].items()},ensure_ascii=False,indent=2))+'</pre></details><a href="'+html.escape(exp['path'])+'">상세 근거 JSON</a></article>')
    out.mkdir(parents=True,exist_ok=True)
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>과거 본학습·파일럿 중복 억제 재평가</title><style>body{background:#101827;color:#eef4ff;font:15px/1.6 system-ui;margin:24px}a{color:#89d5ff}article{padding:18px;background:#1c293a;margin:16px 0;border-radius:8px}h2{font-size:18px}td,th{padding:6px 15px;border-bottom:1px solid #51657c}pre{overflow:auto;max-height:500px}.note{color:#ffc96e}</style><a href="../index.html">ASMC 실험 목록</a><h1>과거 본학습·파일럿 중복 억제 재평가</h1><p class="note">원본의 중복 FP 집계는 그대로 보존합니다. 아래 수치는 실제 출력에 같은 클래스 NMS(IoU&gt;0.7)를 추가한 별도 정책이며, 중복을 TP로 중복 인정하거나 평가에서 면제하지 않습니다. confidence 0.40·GT/ignore·일대일 IoU0.5 매칭은 유지합니다. 서로 다른 GT 버전의 F1은 직접 순위를 매기지 않습니다. 운영 후처리·학습은 변경하지 않았습니다.</p><p>GT가 부족한 클래스는 소표본이며 16종 GT가 모두 없으면 16종 F1을 만들지 않습니다. Heldout은 과거 평가가 있었던 결과만 고정 기준으로 재집계하며 임계값 선정에 사용하지 않습니다.</p><a href="summary.json">전체 요약 JSON</a>'''
    (out/'index.html').write_text(page+''.join(cards)+'</html>')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--worker',type=Path)
    p.add_argument('--root',type=Path);p.add_argument('--output',type=Path);p.add_argument('--watch-seconds',type=int,default=0)
    a=p.parse_args()
    if a.worker:worker(read(a.worker))
    else:
        lock=None
        if a.watch_seconds:
            a.output.mkdir(parents=True,exist_ok=True)
            lock=(a.output/'watch.lock').open('w')
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        deadline=time.monotonic()+a.watch_seconds
        while True:
            result=aggregate(a.root.resolve(),a.output.resolve())
            save(a.output/'watch-state.json',dict(updated_utc=result['updated_utc'],experiments=len(result['experiments']),
                errors=result['errors'],watching=a.watch_seconds>0,remaining_seconds=max(0,deadline-time.monotonic())))
            print(json.dumps(dict(experiments=len(result['experiments']),errors=result['errors'])),flush=True)
            if time.monotonic()>=deadline:break
            time.sleep(45)
