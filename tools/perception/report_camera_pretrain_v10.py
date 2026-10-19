#!/usr/bin/env python3
"""Read-only dataset coverage and review catalogue; never changes labels."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import html
import json
from pathlib import Path

from camera_pretrain_policy import CLASSES, sha


def build(dataset, output):
    summary = json.loads((dataset/'summary.json').read_text())
    reviews = json.loads((output/'reviews.json').read_text())
    rows = [json.loads(line) for line in (dataset/'manifest.jsonl').open()]
    counters = {}
    for split in ('train', 'selection', 'heldout'):
        subset = [r for r in rows if r['split'] == split]
        counters[split] = dict(
            images=len(subset), completeness=dict(Counter(r['supervision'] for r in subset)),
            source_families=dict(Counter(r['source'].split(':')[0] for r in subset)),
            mask_on=sum(bool(r.get('ego_mask_applied')) for r in subset),
            known_view_mask_off=sum(r['view'] in ('front','left','right') and
                                   not r.get('ego_mask_applied') for r in subset),
            positive=sum(a['supervision']=='positive' for r in subset for a in r['annotations']),
            ignore=sum(a['supervision']=='ignore' for r in subset for a in r['annotations']))
    coverage=[]
    for c in CLASSES:
        entry=dict(class_name=c, train=summary['splits']['train']['classes'][c],
                   epoch_image_exposures=summary['balance']['class_exposures'][c],
                   selection=summary['splits']['selection']['classes'][c],
                   heldout=summary['splits']['heldout']['classes'][c])
        coverage.append(entry)
    report=dict(dataset=dataset.name, dataset_metadata_sha256=sha(dataset/'pretrain.json'),
                generated_utc=datetime.now(timezone.utc).isoformat(), splits=counters,
                coverage=coverage, balance=summary['balance'],
                reviewed_gt_minimum=20, independent_scene_sufficiency=False,
                primary_eval_reviews={role:dict(Counter(r['decision'] for r in reviews[role]))
                                      for role in ('selection','heldout')},
                original_sources_immutable=True, operational_changes=False)
    (output/'review-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    style='body{font:16px sans-serif;background:#111827;color:#edf2fa;max-width:1500px;margin:30px auto;padding:16px}a{color:#67e8f9}table{border-collapse:collapse}td,th{border:1px solid #61718a;padding:8px}img{width:100%;max-width:1100px}pre{white-space:pre-wrap}details{margin:14px 0} .warn{color:#fbbf24}'
    title='2026-10-13 · v10 데이터 전수 구조검사·16종 평가 검수'
    body=f'<h1>{title}</h1><p><a href="../index.html">전체 실험 목차</a> · <a href="../camera-bbox-main-v10/run-20261013-r1/live.html">새 본학습 상태</a></p>'
    body+='<p>Train 39,391장 / val 212장 / test 116장. 학습 양성 148,778개 · ignore 230,546개. 전체 유효 학습 이미지를 매 epoch 사용합니다.</p>'
    body+='<p>원본 또는 960 tensor에서 짧은 변 &lt;4px 또는 면적 &lt;64px²이면 ignore입니다. 경계값은 양성으로 유지하고, 가시 instance 픽셀은 최소 4개입니다. 확인된 작은 물체를 배경 정답으로 바꾸지 않습니다. 거리 60m 필터는 적용하지 않았습니다.</p>'
    body+='<p>전방 1280×720 → 960×540, 좌우 640×480 → 960×720, 각각 960×960 letterbox. 세 뷰 Ego mask ON, 뷰가 미확인인 외부 영상에는 추정 mask를 적용하지 않습니다. 같은 정책을 train/val/test에 적용했습니다.</p>'
    body+='<p>검수된 배경 3,000장. Partial 자료에는 확인된 semantic 배경만 확장했고, etc·standing_object·Obstacle 영역은 배경으로 승인하지 않았습니다. 지도 고정 드럼/방벽은 actor GT에 없어도 RGB에 보이므로 무스폰 영상 전체를 자동 배경 처리하지 않았습니다.</p>'
    body+='<p class="warn">평가 종류별 GT는 20개 이상이지만 대부분 1~2개 배치/소스 그룹입니다. 연속 촬영 수를 독립 장면 수로 해석하지 않습니다. 기존 검토 이력이 있는 자료이며 새로운 비공개 시험은 아닙니다. 새 학습과의 이미지·알려진 계보 중복은 검사했습니다.</p>'
    body+='<table><tr><th>클래스</th><th>Train 등장 이미지</th><th>양성 박스</th><th>Epoch 노출 이미지</th><th>Val GT / 그룹</th><th>Test GT / 그룹</th></tr>'
    for r in coverage:
        cells=[r['class_name'],r['train']['images'],r['train']['positive'],r['epoch_image_exposures'],
               f"{r['selection']['positive']} / {r['selection']['groups']}",f"{r['heldout']['positive']} / {r['heldout']['groups']}"]
        body+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in cells)+'</tr>'
    body+='</table><p>같은 이미지의 여러 클래스를 중복 집계합니다. 희소 클래스 추가 반복은 이미지당 최대 2회이며, epoch 0은 총 41,728회 노출입니다.</p>'
    for name in ('final-verification.json','loader-verification.json','near-duplicate-review.json','review-summary.json','condition-registry.json','loss-tests.log'):
        body+=f'<p><a href="{name}">{name}</a></p>'
    body+='<details><summary>분할별 환경·뷰·출처와 완전성</summary><pre>'+html.escape(json.dumps(dict(counters=counters,dimensions={k:v['dimensions'] for k,v in summary['splits'].items()}),ensure_ascii=False,indent=2))+'</pre></details>'
    for role in ('selection','heldout'):
        for decision in ('accept','defer'):
            candidates=[r for r in reviews[role] if r['decision']==decision]
            body+=f'<details><summary>{role} · {decision} · {len(candidates)}장 원본 GT 검수</summary>'
            for r in candidates:
                body+='<p>'+html.escape(r['id']+' · '+r['reason'])+'</p><a href="'+html.escape(r['panel'],quote=True)+'"><img loading="lazy" src="'+html.escape(r['panel'],quote=True)+'"></a>'
            body+='</details>'
    (output/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>'+title+'</title><style>'+style+'</style>'+body+'</html>')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();build(a.dataset,a.output)
