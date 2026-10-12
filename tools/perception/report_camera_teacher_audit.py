#!/usr/bin/env python3
"""Rebuild an X-teacher comparison from preserved predictions, not fresh inference."""
import argparse
import html
import json
from pathlib import Path

from camera_data_audit import load_label_policy, read_jsonl, save, sha
from camera_teacher_audit import annotated_localization, analysis_provenance


def report(root, gt_path, policy, config_path, environment):
    config=json.loads(config_path.read_text());gt=list(read_jsonl(gt_path));profiles=[]
    for profile in config['prompt_profiles']:
        folder=root/(profile+'-1280')
        summary=json.loads((folder/'summary.json').read_text())
        predictions=list(read_jsonl(folder/'predictions.jsonl'))
        binding=analysis_provenance(predictions,config,policy,config_path,gt_path,folder/'predictions.jsonl')
        for key in ('checkpoint_sha256','embeddings_sha256','config_sha256','profile','gt_sha256','predictions_sha256'):
            if summary.get(key) != binding[key]:
                raise ValueError('summary/predictions provenance mismatch: '+key)
        by_id={r['id']:r for r in predictions}
        if len(by_id)!=len(predictions) or set(by_id)!={r['id'] for r in gt}:
            raise ValueError('comparison frame membership mismatch')
        if any(r['image_sha256']!=by_id[r['id']]['image_sha256'] for r in gt):
            raise ValueError('comparison image SHA mismatch')
        profiles.append(dict(profile=profile,frames=len(predictions),latency_ms=summary['latency_ms'],
            cuda_peak_allocated_mib=summary['cuda_peak_allocated_mib'],approved_classes=summary['approved_classes'],
            checkpoint_sha256=summary['checkpoint_sha256'],embedding_sha256=summary['embeddings_sha256'],
            predictions_sha256=sha(folder/'predictions.jsonl'),summary_sha256=sha(folder/'summary.json'),
            partial_diagnostics=[annotated_localization(gt,by_id,policy,c) for c in (.10,.25,.40)],
            complete_selection_frames=summary['fixed_selection']['training_class']['evaluated_frames'],
            complete_heldout_frames=summary['fixed_heldout']['training_class']['evaluated_frames']))
    result=dict(schema_version=1,experiment_id=config['experiment_id'],gt_sha256=sha(gt_path),
        checkpoint_source=config['checkpoint_url'],profiles=profiles,**environment)
    # This report never approves a vocabulary based on partially labelled scenes.
    result.update(formal_teacher_acceptance=False,teacher_labels_applied=0,approved_classes=[],
        selection_reason='No complete 16-class Capture image review set in this probe. Independent actor references support diagnostics only.',
        deviations=['Fixed 317-frame probe from 5667 selection/heldout frames, every 20th plus independent actor references; full acceptance deferred until GT scope is complete.',
            'Actor subtype candidates were inspected by Codex, not confirmed by a human; not approved teacher GT.',
            'Diagnostic profiles include historically exposed heldout frames; not a fresh blind test. No confidence/vocabulary selected from heldout.',
            'No approved classes, so no mixed Roboflow labels were assigned from teacher.'],
        next=['Confirm independent GT subtypes and image completeness for priority classes.',
            'Run full selection sweep, freeze vocabulary/thresholds, then heldout evaluation.',
            'Use approved classes for one-to-one mixed-box suggestions; a human confirms every change.'])
    if any(p['approved_classes'] for p in profiles):
        raise ValueError('This diagnostic report cannot summarize an approved formal run')
    save(root/'summary.json',result)
    rows=[]
    for p in profiles:
        d=p['partial_diagnostics'][2]['classes'];lat=p['latency_ms']
        rows.append('<tr><td><a href="'+p['profile']+'-1280/index.html">'+html.escape(p['profile'])+'</a></td><td>'+
            '/'.join('%.1f'%lat[k] for k in ('p50','p95','p99'))+'</td>'+
            ''.join('<td>'+str(d[n]['matched'])+'/'+str(d[n]['gt'])+'</td>' for n in ('box','plastic_barrier','traffic_cone'))+
            '<td>보류</td></tr>')
    (root/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>2026-10-10 · YOLOE-26x teacher 검증</title>
<style>body{font:16px system-ui;max-width:1300px;margin:30px auto}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:8px}pre{white-space:pre-wrap}</style>
<h1>2026-10-10 · YOLOE-26x teacher 검증</h1><p>동일 프레임의 5개 실제 prompt 추론. 부분 GT 진단이며 정식 precision·전체 recall·teacher 승인이 아닙니다. 승인 클래스 0, 자동 변경 라벨 0. 지연은 단독 offline 추론이며 전체 시스템 지연과 다릅니다.</p>
<p><a href="../index.html">전체 실험 목차</a> · <a href="../camera-training-data-audit-v3/index.html">데이터 v3 감사</a> · <a href="capture-gt-review/index.html">독립 Capture GT 검토</a> · <a href="summary.json">수치·SHA·실행 제한</a></p>
<table><tr><th>prompt 실제 재추론</th><th>p50/p95/p99 ms</th><th>Box 매칭/기존 주석</th><th>Plastic barrier</th><th>Cone</th><th>teacher 승인</th></tr>'''+''.join(rows)+
        '</table><p>confidence 0.40, IoU≥0.5, 일대일 대응. Teacher 미검출은 빈 정답이 아닙니다. FP를 셀 수 없는 자료로 threshold를 선정하지 않았습니다.</p><pre>'+
        html.escape(json.dumps(result['deviations'],ensure_ascii=False,indent=2))+'</pre>')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','gt','labels','config','environment'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--evidence',type=Path)
    args=p.parse_args()
    result=report(args.root,args.gt,load_label_policy(args.labels),args.config,json.loads(args.environment.read_text()))
    if args.evidence:
        save(args.evidence/'summary.json',result)


if __name__=='__main__':main()
