#!/usr/bin/env python3
"""Render the fixed checkpoint decision and paired deployment evidence."""
import argparse
import html
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text()) if path.is_file() else None


def percent(value):
    return '—' if value is None else '{:.2f}%'.format(100 * value)


def table(headers, rows):
    return '<div class="scroll"><table><tr>'+''.join('<th>'+html.escape(str(h))+'</th>' for h in headers)+'</tr>'+''.join(
        '<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row)+'</tr>' for row in rows)+'</table></div>'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--history', type=Path, required=True)
    args = parser.parse_args()
    root = args.artifacts
    selection = read(root/'selection.json')
    history = read(args.history)
    status = read(root/'status.json') or {'state': '검증 진행 중 · 기본 모델 변경 전'}
    metrics = {name: read(root/folder/'metrics.json') for name, folder in (
        ('Medium v6', 'medium-evaluation'), ('Large E8', 'large-e8-evaluation'))}
    body = '<h1>TwinLite Large E8 · 선정과 배포 검증</h1>'
    body += '<p><a href="../index.html">ASMC 실험 목록</a> · <a href="../camera-seg-large-train-20261017-r1/index.html">E366 학습 기록</a></p>'
    body += '<p><strong>'+html.escape(status['state'])+'</strong></p>'
    body += '<p>선정: 완료된 1–8epoch의 SUNNY/FOGGY × 11/13/15시 균형 두 head 평균 IoU 최고점. Test를 보기 전에 E8과 SHA를 고정했습니다. 전방 544×960 B1 · 좌우 480×640 B2 · 같은 4종 road_marking/actor-negative 정답. 공개 제어·임계값·family 알고리즘은 변경하지 않습니다.</p>'
    body += '<p>FOGGY15 정지선은 val432장에 GT444픽셀로 희소합니다. 이 조건의 정지선 지표를 다른 차선과 구분합니다. Test17시는 이번 학습에 포함하지 않은 일반화 진단이며, epoch/threshold 재선정에 사용하지 않습니다.</p>'
    body += '<p>Test는 이번 Large 학습에서 미사용이지만 과거 Medium 개발에는 사용 이력이 있는 같은 K-City 자료입니다. 새로운 비공개·지리적으로 독립된 시험으로 설명하지 않습니다. 6조건 지표는 17시를 제외하며, pooled 지표는 test 전체 14,556장을 포함합니다.</p>'
    body += '<h2>완료 epoch 선정</h2>'+table(
        ['Epoch', '균형 두 head', 'drivable', '노면 표시'],
        [(r['epoch'], percent(r['validation']['six_condition_macro_mean_head_iou']),
          percent(r['validation']['six_condition_macro_head_scores']['drivable']),
          percent(r['validation']['six_condition_macro_head_scores']['road_marking'])) for r in history])
    points = [(55+i*85, 250-(r['validation']['six_condition_macro_mean_head_iou']-.84)*2300) for i,r in enumerate(history)]
    body += '<svg viewBox="0 0 720 300" role="img" aria-label="완료 epoch별 조건 균형 IoU">'
    body += '<polyline fill="none" stroke="#38bdf8" stroke-width="3" points="'+' '.join('{},{}'.format(*p) for p in points)+'"/>'
    for r,(x,y) in zip(history,points):
        body += '<circle cx="{}" cy="{}" r="5" fill="#38bdf8"/><text x="{}" y="285">E{} {}</text>'.format(x,y,x-25,r['epoch'],percent(r['validation']['six_condition_macro_mean_head_iou']))
    body += '</svg><p>이 추이는 학습 중 AMP 평가입니다. 아래 재평가는 두 모델 모두 같은 GPU FP32 평가기로 수행합니다.</p>'
    for split in ('val', 'test'):
        rows=[]
        for name, report in metrics.items():
            if not report:
                continue
            result=report['results'][split]
            da=result['metrics']['drivable']['all']['class_iou']['drivable']
            marks=result['metrics']['road_marking']['all']['class_iou']
            rows.append((name,result['sample_count'],percent(result['six_condition_macro_mean_head_iou']),
                         percent(da),*[percent(marks.get(c)) for c in ('white_lane','yellow_lane','stopline')]))
        body += '<h2>'+split+' · 같은 GPU FP32</h2>'+table(
            ['모델','이미지','6조건 균형 두 head','drivable pooled','흰차선 pooled','황색차선 pooled','정지선 pooled'],rows)
        rows=[]
        for group in ('front','left','right','SUNNY:11','SUNNY:13','SUNNY:15','FOGGY:11','FOGGY:13','FOGGY:15','SUNNY:17','FOGGY:17'):
            for name,report in metrics.items():
                if not report:
                    continue
                result=report['results'][split]
                marks=result['metrics']['road_marking'].get(group)
                if marks:
                    rows.append((group,name,*[percent(marks['class_iou'].get(c)) for c in ('white_lane','yellow_lane','stopline')]))
        body += '<details><summary>뷰·날씨·시간별 클래스 IoU</summary>'+table(['조건','모델','흰차선','황색차선','정지선'],rows)+'</details>'
    rows = []
    for name, folder in [('Medium v6', 'medium-hour17'), ('Large E8', 'large-hour17')]:
        report = read(root/folder/'metrics.json')
        if not report:
            continue
        result = report['results']['test']
        for group in ('all', 'SUNNY:17', 'FOGGY:17'):
            da = result['metrics']['drivable'].get(group)
            marks = result['metrics']['road_marking'].get(group)
            if da and marks:
                rows.append((name,group,percent(da['class_iou']['drivable']),
                             *[percent(marks['class_iou'].get(c)) for c in ('white_lane','yellow_lane','stopline')]))
    if rows:
        body += '<h2>학습에 없던 17시 · 별도 진단</h2>'+table(
            ['모델','조건','drivable','흰차선','황색차선','정지선'],rows)
    live = read(root/'live-runtime/index.json')
    if live:
        acceptance = read(root/'live-acceptance.json') or {}
        decisions = {row['name']: row for row in acceptance.get('trials', [])}
        rows = []
        for item in live:
            job = item['job']
            report = read(root/'live-runtime'/job['name']/'report.json') or {}
            streams = report.get('streams', {})
            lane = [s for name, s in streams.items() if '/road_marking/model_grid/class_id:' in name]
            family = report.get('lane_family', {})
            valid = family.get('valid_supply', {})
            rows.append((job['name'], job['seconds'],
                min((min(s.get('five_second_hz') or [0]) for s in lane), default=0),
                round(max((s.get('wall_ms', {}).get('p95', 0) for s in lane), default=0), 2),
                min(valid.get('five_second_hz') or [0]),
                round(1000*valid.get('max_gap_sec', 0), 2),
                round(100*report.get('resources', {}).get('vram_peak_fraction', 0), 2),
                '; '.join(decisions.get(job['name'], {}).get('failures', []))
                or ('PASS' if decisions.get(job['name'], {}).get('pass_') else '미집계')))
        body += '<h2>MORAI · 전체 GUI·Main/LiDAR 부하</h2>'+table(
            ['실행','초','TwinLite 최소5초 Hz','TwinLite p95 ms','유효 family 최소5초 Hz',
             '유효 family 최대공백 ms','전체 VRAM %','판정'],rows)
        body += '<p>모든 실패 실행도 포함합니다. 정차한 음영 위치의 공급 시험이며 주행 안전 수락이나 새로운 GT 정확도 평가가 아닙니다.</p>'
        body += '<p><a href="live-runtime/index.json">실행 목록·원본 판정</a></p>'
        body += '<p><a href="live-acceptance.json">수치·ACTIVE readiness를 포함한 배포 수락 판정</a>을 정본으로 사용합니다. 원본 보고서의 failures=[]는 실행 오류가 없다는 뜻이며 성능 수락과 다릅니다.</p>'
    if (root/'live-samples/provenance.json').is_file():
        body += '<h2>실시간 FOGGY 13시 · Large E8</h2><p>이미 발행 중인 debug 영상에서 한 장씩 저장했습니다. 각 패널은 별도 시각이며 동기 GT 비교본으로 사용하지 않습니다.</p>'
        for name, label in [('semantic', '3뷰 원본·drivable·노면 표시'),
                            ('family', '음영 family'), ('objects', 'YOLO 기존 E8')]:
            body += '<details><summary>'+label+'</summary><img loading="lazy" style="max-width:100%" src="live-samples/'+name+'.jpg"></details>'
    body += '<h2>검증 상태</h2><pre>'+html.escape(json.dumps(status,ensure_ascii=False,indent=2))+'</pre>'
    body += '<p>선정 원본 SHA: <code>'+html.escape(selection['checkpoint_sha256'])+'</code></p>'
    for folder,label in [('family-medium-large-cpu','같은 RGB·pose의 Medium/Large family 비교'),
                         ('quality-trt','Large PyTorch/TensorRT 검증'),
                         ('family-large-trt','Large PyTorch/TensorRT family 비교')]:
        if (root/folder/'index.html').is_file():
            body += '<p><a href="'+folder+'/index.html">'+label+'</a></p>'
    for name in ('selection.json','status.json','large-e8-evaluation/metrics.json','medium-evaluation/metrics.json'):
        if (root/name).is_file():
            body += '<p><a href="'+name+'">'+name+'</a></p>'
    page='<!doctype html><html lang="ko"><meta charset="utf-8"><title>TwinLite Large E8 배포 검증</title><style>body{background:#111827;color:#e5e7eb;font:16px system-ui;max-width:1400px;margin:24px auto;padding:20px;line-height:1.7}a{color:#7dd3fc}table{border-collapse:collapse;width:100%}th,td{padding:9px;border:1px solid #475569;text-align:right}.scroll{overflow-x:auto}svg{width:100%;max-width:1000px}text{fill:#cbd5e1;font-size:12px}pre,code{white-space:pre-wrap;overflow-wrap:anywhere}</style>'+body+'</html>'
    (root/'index.html').write_text(page)


if __name__ == '__main__':
    main()
