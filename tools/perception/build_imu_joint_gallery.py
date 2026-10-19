#!/usr/bin/env python3
"""Publish retained joint-fit evidence to one local experiment folder."""
import argparse
import html
import json
from pathlib import Path
import shutil


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    source = args.output / 'source'
    source.mkdir()
    for path in (args.run_root / 'source-review-r1').iterdir():
        if path.suffix in ('.jpg', '.html') or path.name == 'report.json': shutil.copy2(path, source / path.name)
    for src, dst in [('source-review-r1/acceptance.json', 'quality.json'),
                     ('runtime-acceptance.json', 'runtime.json'),
                     ('legacy-review-r1/summary.json', 'legacy.json'),
                     ('compute-optimization.json', 'compute.json')]:
        shutil.copy2(args.run_root / src, args.output / dst)
    runtime = json.loads((args.output / 'runtime.json').read_text())
    quality = json.loads((args.output / 'quality.json').read_text())
    rows = []
    for r in runtime['trials']:
        family = r.get('family') or {}
        rates = [min(s.get('five_second_hz', []), default=0) for key, s in (r.get('streams') or {}).items()
                 if 'model_grid' in key or 'objects:' in key]
        valid = family.get('valid_supply', {})
        imu = r.get('imu') or {}
        goals = [s.get('target_95pct_pass', False) for key, s in (r.get('streams') or {}).items()
                 if 'model_grid' in key or 'objects:' in key]
        ratio = '%.2f%%' % (100 * imu.get('corrected', 0) / imu['frames']) if imu.get('frames') and r['mode'] == 'source' else 'M0'
        values = [r['trial'], '통과' if r['passed'] else ', '.join(r['failures']), '%gs' % r['seconds'],
                  '%.1fHz' % min(rates, default=0), '%.1fHz' % min(valid.get('five_second_hz', []), default=0),
                  '%.1fms' % (1000 * valid.get('max_gap_sec', 0)),
                  '%.1fms' % family.get('compute_wall_ms', {}).get('p95', 0), ratio,
                  '달성' if len(goals) == 6 and all(goals) else '최소10Hz와 구분']
        rows.append('<tr>' + ''.join('<td>' + html.escape(v) + '</td>' for v in values) + '</tr>')
    passed = quality['passed'] and runtime['all_candidate_passed'] and runtime['ten_minute_pass']
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>IMU 공동 폭·곡선 계산과 실시간 공급 검증</title>
<style>body{font:16px system-ui;background:#14202a;color:#eee;margin:28px;line-height:1.6}a{color:#8ed7ff}table{border-collapse:collapse;width:100%}td,th{padding:8px;border:1px solid #456}section{overflow:auto}pre{white-space:pre-wrap}</style>
<h1>2026-10-18 · IMU 공동 폭·곡선 계산과 실시간 공급</h1>
<p><a href="../index.html">ASMC 실험 목록</a></p>
<p>지면 법선 거리0.36m · 원본 시각 IMU · corrected prior0 · 관측 폭/공통 B-spline 동시 계산.
모델·support·100ms/35ms/350ms 제한 유지. M0 복귀는 prior0.35. 독립 GT 위치 정확도·자율주행 수락과 구분합니다.</p>'''
    page += '<p>품질·실시간 후보 판정: <b>' + ('통과' if passed else '미완료/실패 항목 확인') + '</b></p>'
    page += '<p>최종 revision 선택: <code>' + html.escape(runtime['candidate_name_filter']) + '</code>. 이전 실패 실행도 아래에 보존합니다.</p>'
    page += '<p><a href="source/index.html">동일 RGB의 M0 / 이전 M1 / 공동 계산 M1 비교</a> · <a href="quality.json">엄격 품질 판정</a> · <a href="runtime.json">반복·10분 원본 집계</a> · <a href="legacy.json">6조건 구형 bag 민감도</a> · <a href="compute.json">동일 결과의 계산 비용 대조</a></p>'
    headers = ['실행', '판정', '길이', 'YOLO/차선 최소5초 공급', '유효 family 최소5초 공급', 'family 최대 수신 공백', 'family 계산 p95', '원본 시각 IMU 적용', 'YOLO/차선15Hz 목표95%']
    page += '<section><table><thead><tr>' + ''.join('<th>' + h + '</th>' for h in headers) + '</tr></thead><tbody>' + ''.join(rows) + '</tbody></table></section>'
    page += '<p>전체 GUI·rqt1개·Main/LiDAR 부하. 지연은 수신 callback→발행 wall time이며 family 계산과 분리합니다. 원본 trace는 run 디렉토리에 보존합니다. 구형 bag은 receive clock만 있으므로 원본 시각 검증으로 세지 않습니다.</p></html>'
    (args.output / 'index.html').write_text(page, encoding='utf-8')
    print(json.dumps(dict(output=str(args.output), trials=len(runtime['trials']), passed=passed)))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    main(p.parse_args())
