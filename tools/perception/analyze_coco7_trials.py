#!/usr/bin/env python3
"""Consolidate every trial, including failed repeats, without inventing missing GT."""
import argparse
from collections import Counter, defaultdict, deque
import html
import json
from pathlib import Path
from evaluate_coco7_models import evaluate


def temporal_report(gt, predictions, confidence):
    sequence=defaultdict(list)
    for p in predictions:
        row=gt[p['image']]
        _,_,pairs,known,_=evaluate(row['annotations'],p['predictions'],confidence)
        hits={known[g]['actor_id'] for g,_,_ in pairs}
        for a in known:
            sequence[row['run'],row['view'],a['actor_id']].append((int(row['frame']),a['actor_id'] in hits,a['source_class_name']))
    report=[]
    for key,values in sequence.items():
        values.sort();windows=deque(maxlen=5);stable=None;max_misses=current_misses=0;reentries=0;previous=None;contiguous=True;length=longest=0
        for frame,hit,label in values:
            if previous is not None and frame!=previous+1:
                windows.clear();contiguous=False;current_misses=0;length=0
            length+=1;longest=max(longest,length)
            windows.append(hit)
            if len(windows)==5 and sum(windows)>=3 and stable is None:stable=frame
            if hit:
                if current_misses:reentries+=1
                current_misses=0
            else:
                current_misses+=1;max_misses=max(max_misses,current_misses)
            previous=frame
        report.append(dict(run=key[0],view=key[1],actor_id=key[2],label=values[0][2],inputs=len(values),
            first_visible_frame=values[0][0],first_stable_frame=stable,
            stable_input_delay=stable-values[0][0] if stable is not None else None,
            maximum_consecutive_missed_visible_inputs=max_misses,reentries=reentries,contiguous_gt=contiguous,
            longest_contiguous_visible_inputs=longest,stability_evaluable=longest>=5))
    return dict(objects=report,not_stably_detected=sum(r['stability_evaluable'] and r['first_stable_frame'] is None for r in report),
                insufficient_contiguous_gt=sum(not r['stability_evaluable'] for r in report),
                interpretation='3 hits in 5 consecutive annotated source inputs; frame counts, not wall seconds; visibility gaps reset window')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    a=p.parse_args();root=a.root
    summaries=[json.loads(f.read_text()) for f in sorted((root/'quality').glob('*/summary.json'))]
    baseline={r['width']:next(t for t in r['thresholds'] if t['confidence']==.7)['false_positives'] for r in summaries if Path(r['model']['weights']).stem=='yolo11m'}
    quality=[]
    gt={r['image']:r for r in [json.loads(l) for l in (root/'gt-v2/frames.jsonl').read_text().splitlines()]}
    for r in summaries:
        model=Path(r['model']['weights']).stem
        allowed=[t for t in r['thresholds'] if t['false_positives']<=baseline[r['width']]]
        best=max(allowed,key=lambda t:(t['macro_f1'],t['confidence'])) if allowed else None
        fixed=next(t for t in r['thresholds'] if t['confidence']==.7)
        row=dict(model=model,width=r['width'],fixed=fixed,selected=best,baseline_fp_limit=baseline[r['width']],missing_gt_classes=r['missing_gt_classes'])
        if best:
            directory=root/'quality'/f'{model}-{r["width"]}-selection'
            temporal=temporal_report(gt,[json.loads(l) for l in (directory/'predictions.jsonl').read_text().splitlines()],best['confidence'])
            (directory/'temporal.json').write_text(json.dumps(temporal,indent=2)+'\n')
            row['temporal_summary']={k:v for k,v in temporal.items() if k!='objects'}
        quality.append(row)
    live=[];rejected=[]
    for parent in root.glob('live-*'):
        if not parent.is_dir():continue
        if (parent/'REJECTED_PROVENANCE.json').exists():
            rejected.append(dict(path=str(parent.relative_to(root)),reason=json.loads((parent/'REJECTED_PROVENANCE.json').read_text())))
            continue
        for path in parent.glob('*/report.json'):
            r=json.loads(path.read_text());live.append(dict(path=str(path.relative_to(root)),job=r['job'],passed=r['passed'],failures=r['failures'],
                measurements=r.get('measurements'),target_attainment=r.get('target_attainment'),resources=r.get('resources'),planning=r.get('planning')))
    report=dict(quality=quality,live=live,rejected_provenance=rejected,failed_trials=sum(not r['passed'] for r in live),trials=len(live))
    (root/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    body=['<!doctype html><meta charset="utf-8"><style>body{font:16px system-ui;margin:24px}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}img{max-width:95%}</style><h1>COCO7 · same images, live MORAI 4060 Ti</h1>',
          '<p>Four labelled classes in Capture GT; dog/bicycle/motorcycle remain inspection only. Failed repeats are retained. Minimum feasibility does not mean target 20 Hz.</p><p><a href="near-object-gallery/index.html">근거리 차량 · 960/1280 · PyTorch/TensorRT 비교</a></p>',
          '<table><tr><th>Model</th><th>Front width</th><th>F1 @0.7</th><th>FP @0.7 / limit</th><th>Frozen selection</th><th>Images</th></tr>']
    for r in quality:
        s=r['selected'];chosen=f'{s["confidence"]:.2f} / F1 {s["macro_f1"]:.3f}' if s else 'ineligible FP'
        body.append(f'<tr><td>{r["model"]}</td><td>{r["width"]}</td><td>{r["fixed"]["macro_f1"]:.3f}</td><td>{r["fixed"]["false_positives"]} / {r["baseline_fp_limit"]}</td><td>{chosen}</td><td><a href="quality/{r["model"]}-{r["width"]}-selection/gallery/index.html">GT / output / mask</a></td></tr>')
    body+=['</table><h2>All live trials</h2><table><tr><th>Condition</th><th>Output Hz front / left / right</th><th>VRAM MiB</th><th>Minimum gate</th></tr>']
    for r in live:
        m=r['measurements'] or {};rates=' / '.join(str(round(m.get('yolo_'+v,{}).get('hz',0),2)) for v in ('front','left','right'))
        memory=(r['resources'] or {}).get('gpu_memory_mib',{}).get('maximum')
        body.append(f'<tr><td><a href="{r["path"]}">{html.escape(r["path"].split("/")[-2])}</a></td><td>{rates}</td><td>{memory}</td><td>{"PASS" if r["passed"] else html.escape(str(r["failures"]))}</td></tr>')
    body+=['</table><h2>Fixed compute scene · original images</h2>']
    body+=['<h2>Rejected provenance · excluded from model measurement counts</h2><pre>'+html.escape(json.dumps(rejected,indent=2))+'</pre>']
    for view in ('front','left','right'):body.append(f'<h3>{view}</h3><img src="live-scene-rgb/{view}.jpg">')
    (root/'index.html').write_text(''.join(body))
    print(json.dumps(dict(quality=len(quality),live=len(live),failed_trials=report['failed_trials'])))


if __name__=='__main__':main()
