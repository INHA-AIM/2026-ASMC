#!/usr/bin/env python3
"""Re-score saved teacher predictions with explicit tiny-GT ignore, without GPU."""
import argparse
from collections import Counter
import html
import json
from pathlib import Path

from PIL import Image, ImageDraw
from camera_data_audit import load_label_policy, read_jsonl, save, sha, write_jsonl, training_names
from camera_teacher_audit import annotated_localization, analysis_provenance, metrics
from capture_size_scope import partition, validate_scope


def report(args):
    if args.output.exists():
        raise ValueError('preserve earlier evidence: output exists')
    scope_config=json.loads(args.scope_config.read_text())
    if scope_config['coordinate_space']!='original_image_pixels':
        raise ValueError('pixel scope must not change with model tensor size')
    scopes={name:validate_scope(scope) for name,scope in scope_config['profiles'].items()}
    primary=scope_config['primary_profile'];scope=scopes[primary]
    config=json.loads(args.config.read_text());policy=load_label_policy(args.labels)
    rows=list(read_jsonl(args.gt));profiles=[];gallery_predictions=None
    for profile in config['prompt_profiles']:
        path=args.inference_root/(profile+'-1280')/'predictions.jsonl'
        predictions=list(read_jsonl(path))
        binding=analysis_provenance(predictions,config,policy,args.config,args.gt,path)
        by_id={r['id']:r for r in predictions}
        if len(by_id)!=len(predictions) or set(by_id)!={r['id'] for r in rows}:
            raise ValueError('prediction/GT membership mismatch')
        if any(r['image_sha256']!=by_id[r['id']]['image_sha256'] for r in rows):
            raise ValueError('prediction/GT RGB mismatch')
        thresholds={n:config['fixed_confidence'] for n in training_names(policy)}
        profiles.append(dict(profile=profile,provenance=binding,
            all_gt=annotated_localization(rows,by_id,policy,config['fixed_confidence']),
            scoped={name:annotated_localization(rows,by_id,policy,config['fixed_confidence'],s) for name,s in scopes.items()},
            complete_frame_primary=metrics(rows,by_id,policy,thresholds,scope=scope)))
        if profile=='rich21':gallery_predictions=by_id
    if gallery_predictions is None:
        raise ValueError('rich21 saved predictions required for the visual comparison')
    args.output.mkdir(parents=True);(args.output/'images').mkdir();(args.output/'raw').mkdir()
    manifest=[];gallery=[];geometry=Counter()
    for index,row in enumerate(rows):
        path=Path(row['image'])
        if sha(path)!=row['image_sha256']:raise ValueError('source RGB SHA changed')
        main,ignored=partition(row['annotations'],scope)
        geometry.update(all_gt=len(row['annotations']),main_gt=len(main),ignored_gt=len(ignored))
        (args.output/'raw'/(row['id']+path.suffix)).symlink_to(path.resolve())
        with Image.open(path) as image:
            base=image.convert('RGB')
            clean=base.copy();detail=base.copy()
            for overlay,show_ignored in ((clean,False),(detail,True)):
                draw=ImageDraw.Draw(overlay)
                if show_ignored:
                    for a in ignored:draw.rectangle(a['xyxy'],outline='#ff00ff',width=2)
                for a in main:
                    draw.rectangle(a['xyxy'],outline='#2695ff',width=2)
                    draw.text((a['xyxy'][0],max(0,a['xyxy'][1]-12)),a.get('canonical') or a['original_label'],fill='#2695ff')
                for p in gallery_predictions[row['id']]['predictions']:
                    if p['confidence']<config['fixed_confidence']:continue
                    draw.rectangle(p['xyxy'],outline='#28e859',width=2)
                    draw.text((p['xyxy'][0],max(0,p['xyxy'][1]-22)),p['source_prompt']+' %.2f'%p['confidence'],fill='#28e859')
            clean.save(args.output/'images'/('%04d-main.jpg'%index),quality=92)
            detail.save(args.output/'images'/('%04d-all.jpg'%index),quality=92)
        manifest.append(dict(id=row['id'],image_sha256=row['image_sha256'],scope=primary,
            main_annotation_ids=[a['id'] for a in main],ignored_annotation_ids=[a['id'] for a in ignored],
            reason='long_edge_or_short_edge_below_pixel_scope',training_labels_modified=False))
        gallery.append(dict(source=row['source'],source_key=row['source_key'],view=row['view'],
            main=len(main),ignored=len(ignored),labels=sorted({a.get('canonical') or a['original_label'] for a in row['annotations']}),
            image='images/%04d-main.jpg'%index,all_image='images/%04d-all.jpg'%index,
            raw_image='raw/'+row['id']+path.suffix))
    write_jsonl(args.output/'evaluation-scope.jsonl',manifest)
    result=dict(schema_version=1,experiment_id=scope_config['experiment_id'],frames=len(rows),
        scope_config=scope_config,scope_config_sha256=sha(args.scope_config),gt_sha256=sha(args.gt),
        primary_profile=primary,geometry_counts=dict(geometry),profiles=profiles,
        geometry_counts_scope='All source GT geometry including unresolved/excluded semantic types. Per-class localization uses known target annotations only.',
        interpretation='Known-annotation localization remains partial GT, not full recall/precision. Pixel scope is provisional; previously exposed data are not a new blind evaluation. Raw GT and old metrics are preserved.',
        capture_geometry_user_confirmed=True,formal_teacher_acceptance=False,training_labels_modified=False,
        operating_results_modified=False,gpu_inference_performed=False)
    result['tool_sha256']={name:sha(Path(__file__).parent/name) for name in
        ('report_capture_size_scope.py','capture_size_scope.py','camera_teacher_audit.py')}
    save(args.output/'summary.json',result);save(args.output/'gallery-data.json',gallery)
    if args.evidence:save(args.evidence,result)
    table=[]
    for p in profiles:
        cells=['<td>'+html.escape(p['profile'])+'</td>']
        for n in ('box','person','car','traffic_cone'):
            original=p['all_gt']['classes'][n];selected=p['scoped'][primary]['classes'][n]
            cells.append('<td>%d/%d → %d/%d (ignore %d)</td>'%(original['matched'],original['gt'],selected['matched'],selected['gt'],selected.get('ignored_gt',0)))
        table.append('<tr>'+''.join(cells)+'</tr>')
    page='''<!doctype html><meta charset="utf-8"><title>2026-10-10 · Capture GT 크기 범위 비교</title>
<style>body{font:16px system-ui;max-width:1400px;margin:25px auto}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}article{border:1px solid #ccc;padding:12px;margin:18px 0}img{max-width:100%}button,input{font:inherit;margin:8px;padding:6px}</style>
<h1>2026-10-10 · Capture GT 크기 범위 비교</h1><p>파랑: 크기 기준을 충족한 원본 GT · 초록: 저장된 rich21 예측(confidence 0.40) · <span style="color:#a000a0;font-weight:bold">자홍색 ■</span>: 극소형 ignore GT. 기본 화면은 자홍색 박스를 숨깁니다. 세부 종류 미확정 GT는 원본 Vehicle/Obstacle 이름을 표시합니다. 정량 표는 종류가 확인된 대상만 집계합니다. 원본 이미지·GT·기존 결과는 유지합니다.</p>
<p>긴 변 ≥__LONG__px, 짧은 변 ≥__SHORT__px를 모두 만족하면 주 평가 대상입니다. 정확히 경계값이면 포함하며 원본 영상 픽셀로 비교합니다. 검출 가능 한계나 최종 수락 기준으로 확정한 값은 아닙니다. 이미 노출된 자료의 시각 검토용 범위이며 새로운 blind 평가가 아닙니다.</p>
<p><a href="../index.html">전체 목차</a> · <a href="../camera-yoloe-x-teacher-v1/index.html">기존 전체 GT 진단</a> · <a href="summary.json">8/4·16/8·24/12 크기 대조 및 SHA</a></p>
<p>아래 표는 대응 수/검토된 종류의 GT 수입니다. 제외 후 개선 비율만으로 모델이 좋아졌다고 판단하지 않으며 teacher 승인·전체 precision은 여전히 보류입니다. 거리(m)는 추정하지 않았습니다. 학습용 TXT에서는 라벨을 삭제하지 않았습니다.</p>
<table><tr><th>prompt</th><th>box 전체→주 평가</th><th>person</th><th>car</th><th>cone</th></tr>__TABLE__</table>
<p><label><input type="checkbox" id="show">제외 GT도 표시 (자홍색)</label><input id="query" placeholder="box, person, run, view 검색"><button id="prev">이전</button><button id="next">다음</button></p><p id="count"></p><div id="gallery"></div>
<script>let rows=[],page=0;const el=id=>document.getElementById(id);function draw(){const selected=rows.filter(r=>JSON.stringify(r).toLowerCase().includes(el('query').value.toLowerCase()));el('gallery').replaceChildren();el('count').textContent=selected.length+'개 · 페이지 '+(page+1);for(const r of selected.slice(page*20,(page+1)*20)){const a=document.createElement('article'),p=document.createElement('p'),img=document.createElement('img'),link=document.createElement('a');p.textContent=r.source+' '+r.source_key+' / main '+r.main+' / ignore '+r.ignored;img.src=el('show').checked?r.all_image:r.image;link.href=img.src;link.target='_blank';link.append(img);const raw=document.createElement('a');raw.href=r.raw_image;raw.target='_blank';raw.textContent='주석 없는 원본';a.append(p,link,raw);el('gallery').append(a);}}el('show').onchange=draw;el('query').oninput=()=>{page=0;draw()};el('prev').onclick=()=>{page=Math.max(0,page-1);draw()};el('next').onclick=()=>{page++;draw()};fetch('gallery-data.json').then(r=>r.json()).then(r=>{rows=r;draw()});</script>'''
    page=page.replace('__LONG__','%g'%scope['min_long_edge']).replace('__SHORT__','%g'%scope['min_short_edge'])
    (args.output/'index.html').write_text(page.replace('__TABLE__',''.join(table)))
    print(json.dumps(dict(frames=len(rows),geometry_counts=dict(geometry),output=str(args.output))))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('gt','inference-root','config','labels','scope-config','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--evidence',type=Path)
    report(p.parse_args())


if __name__=='__main__':main()
