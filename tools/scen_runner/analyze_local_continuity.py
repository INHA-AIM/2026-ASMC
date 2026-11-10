#!/usr/bin/env python3
"""Compare registered LOCAL trial records without a ROS master or control."""
import argparse
import html
import json
from pathlib import Path
from scen_runner.records.local_continuity import summarize


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs",nargs="+",type=Path)
    p.add_argument("--output",required=True,type=Path)
    a=p.parse_args()
    if not any((q/"registration.json").is_file() for q in a.output.parents):
        raise ValueError("output must be in a registered run")
    result=[summarize(r) for r in a.runs]
    with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    page=a.output.with_suffix('.html')
    parts=['<!doctype html><meta charset="utf-8"><title>LOCAL 연속성 비교</title><style>body{background:#10202b;color:#def;font:16px system-ui;max-width:1300px;margin:24px auto}a{color:#6df}table{border-collapse:collapse}td,th{border:1px solid #456;padding:8px}pre{white-space:pre-wrap}svg{background:#08151e;width:100%}</style><h1>LOCAL 실제 정지·계획·입력 비교</h1><p>준비 구간 제외. 속도 곡선은 SDK 평가 기록이며 제어 입력이 아니다. 유효 경로 공백과 실제 명령 공백은 다르다.</p>']
    parts.append('<table><tr><th>run</th><th>결과/거리</th><th>실제 정체</th><th>입력 무효/계획</th><th>계산 p95</th><th>유효 경로 최대 공백</th></tr>')
    for r in result:
        if not r.get('driving'):
            parts.append('<tr><td>'+html.escape(r['run'])+'</td><td colspan="5">준비 실패/실제 구간 없음</td></tr>');continue
        parts.append('<tr><td>'+html.escape(r['run'])+'</td><td>'+html.escape(r['result']['reason'])+' / %.1fm</td><td>%.1fs</td><td>%d/%d</td><td>%.1fms</td><td>%.1fms</td></tr>'%(r['result']['travel_m'],r['stationary_seconds'],r['input_invalid_plans'],r['plans'],r['processing_ms'].get('p95',0),r['valid_path_gap_ms'].get('max',0)))
    parts.append('</table>')
    for r in result:
        parts.append('<h2>'+html.escape(r['run'])+'</h2>')
        if r.get('driving'):
            xy=' '.join('%.1f,%.1f'%(20+960*t/max(1,r['seconds']),190-9*max(0,v)) for t,v in r['speed_samples'])
            parts.append('<svg viewBox="0 0 1000 210"><text x="25" y="20" fill="white">실측 km/h / 경과 초</text><polyline fill="none" stroke="#1edcce" stroke-width="2" points="'+xy+'"/></svg>')
            parts.append('<pre>'+html.escape(json.dumps({k:r[k] for k in ['plan_reasons','stop_command_reasons','actual_stationary_episodes','reference_sources','camera_age_ms','lidar_age_ms']},ensure_ascii=False,indent=2))+'</pre>')
    with page.open('x') as f:f.write(''.join(parts))
    print(page)


if __name__=='__main__':main()
