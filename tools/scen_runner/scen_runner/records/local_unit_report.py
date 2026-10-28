"""Read-only trial metrics and one registered gallery; no control or source-run edits."""
import argparse
from collections import Counter
import html
import json
from pathlib import Path
from .analyze_local import quantiles
from .local_campaign import build as campaign
from .local_route_review import build as route_review


def rows(path):
    return [json.loads(l) for l in path.open()] if path.exists() else []


def summarize(root):
    result=json.loads((root/'trial-result.json').read_text())
    events=rows(root/'events.jsonl')
    native=[e for e in events if e.get('event')=='native_sample']
    registration=json.loads((root/'registration.json').read_text())
    result['registration']=registration
    result['native_samples']=len(native)
    if not native:
        result.update(wire_full_brake_ticks=None,active_path_gap_ms=None,critical_p95_ms=None,
                      limitation='No native drive interval; preparation failure is retained, not a zero-error success.')
        return result
    begin,end=native[0]['ros_time'],native[-1]['ros_time']
    wire=[r for r in rows(root/'wire.jsonl') if begin<=r['ros_time']<=end]
    plans=[r for r in rows(root/'decisions.jsonl') if begin<=r['decision_stamp']<=end and r.get('phase')=='active']
    summary=json.loads((root/'summary.json').read_text())
    moving=next((i for i,r in enumerate(native) if r['ego']['speed_kmh']>1.),None)
    stationary_start=None;stationary=0.
    for r in native[moving:] if moving is not None else native:
        if r['ego']['speed_kmh']<=.36:
            if stationary_start is None:stationary_start=r['ros_time']
            stationary=max(stationary,r['ros_time']-stationary_start)
        else:stationary_start=None
    result.update(
        native_interval=[begin,end],wire_full_brake_ticks=sum(r['brake']>.9 for r in wire),
        active_path_gap_ms=summary.get('active_local_valid_plan_max_silence_ms'),
        critical_p95_ms=quantiles([r.get('critical_path_ms',r['processing_ms']) for r in plans]).get('p95'),
        input_lock_p95_ms=quantiles([r.get('input_lock_hold_ms',0) for r in plans]).get('p95'),
        actual_udp_contacts=[e for e in events if e.get('event')=='udp_collision'],
        stationary_after_motion_max_s=stationary,
        controlled_braking_frames=sum(r.get('braking_limit_mps2',1.5)>1.5 for r in plans),
        retained_measurement_frames=sum(any(o.get('retained_measurement') for o in r.get('scene',{}).get('obstacles',[])) for r in plans),
        active_plan_reasons=dict(Counter(r['reason'] for r in plans)),
        recovery_frames=sum(bool(r.get('recovery')) for r in plans),active_plan_frames=len(plans),
        limitation='Distance reached/recovery evidence is distinct from operational acceptance. Startup/preparation is excluded only from native-drive metrics, not from raw logs.')
    return result


def build(output, reference, roots):
    output=Path(output)
    if not (output/'registration.json').is_file():raise ValueError('new registered output required')
    results=[summarize(p) for p in roots]
    campaign(output/'campaign',[p for p,r in zip(roots,results) if r['native_samples']],title='LOCAL-2F 경로·속도·실패 대조')
    route_review(reference,output,roots)
    with (output/'review.json').open('x') as f:json.dump(results,f,indent=2)
    route=json.loads((output/'native-segment-review.json').read_text())
    metrics={r['run']:r for r in route['runs']}
    summary=dict(trials=len(results),distance_limit=sum(r['reason']=='distance_limit' for r in results),
        total_travel_m=sum(r['travel_m'] for r in results),failures=[dict(run=r['run_id'],reason=r['reason']) for r in results if r['reason']!='distance_limit'],
        contacts=[r['run_id'] for r in results if r['native_contacts'] or r.get('actual_udp_contacts')],
        qualified_candidate_runs=[r['run_id'] for r in results if int(r['run_id'].split('drive')[-1])>=16],
        all_runs_have_operational_acceptance=False)
    with (output/'review-summary.json').open('x') as f:json.dump(summary,f,indent=2)
    def number(v):return '—' if v is None else '%.2f'%v
    table=[]
    for r in results:
        run=r['run_id'];link='../'+run+'/local-gallery/index.html'
        native_metric=metrics.get(run,{})
        tail=native_metric.get('native_segment_final2s_error_m',{}).get('max')
        table.append('<tr><td><a href="%s">%s</a></td><td>%s</td><td>%.1f / %.1f</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>'%(
            html.escape(link),html.escape(run.rsplit('drive',1)[-1]),html.escape(r['reason']),r['cap_kph'],r['travel_m'],
            number(r['wire_full_brake_ticks']),number(r['active_path_gap_ms']),number(r['critical_p95_ms']),number(tail),str(r.get('controlled_braking_frames','—'))))
    page='''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LOCAL-2F · 제동·복구·MPC 검수</title><style>body{font:16px system-ui;background:#101b25;color:#def;margin:24px;line-height:1.6}a{color:#6df}table{border-collapse:collapse;width:100%}th,td{padding:8px;border:1px solid #345;text-align:left}pre{white-space:pre-wrap}.scroll{overflow:auto}</style>
<h1>LOCAL-2F · 제동·복구 경로 검수</h1>
<p>실제 MORAI 주행, 최종 wire 명령, 원본 RGB/노면/OBB와 선택 경로를 연결했습니다. 기록별 소스·profile은 runtime-provenance.json과 registration.json에서 확인합니다. 16번 이후는 controlled-brake 후보이며 static 관측 기억은 OFF입니다.</p>
<p><a href="campaign/index.html">시간축 경로·속도·제동</a> · <a href="native-review.html">실제 주행·본차로 복귀</a> · <a href="review.json">전체 수치</a> · <a href="handoff/README.md">실패·인지 인계</a> · <a href="analysis.html">인지·MPC 분석</a></p>
<p>07은 실제12초 정지, 09/10은 준비 실패입니다. 실패를 숨기지 않았습니다. 좁은 prefix와3km/h 복구는 일반 노면의 충돌 무결성 인증이 아닙니다. 일반 구간의 최고속도는 LOCAL 상한 성능에 포함하지 않습니다.</p>
<pre>__SUMMARY__</pre><div class="scroll"><table><tr><th>시험/원본 사례</th><th>종료 사유</th><th>상한km/h / 거리m</th><th>전정지 명령 tick</th><th>유효 경로 최대 공백ms</th><th>계산 p95 ms</th><th>마지막2초 기준경로 최대거리m</th><th>강화 제동 계획</th></tr>__TABLE__</table></div>
<p>기준 경로 거리에는 의도적인 회피도 포함됩니다. 복귀는 마지막 구간을 확인하고, 고정 경로/native 자체를 survey GT로 해석하지 않습니다. 순서대로 실행한 실시간 비교는 완전히 동일한 센서 입력 A/B가 아닙니다. 동일 입력의 수학·충돌 회귀는 별도 native fixture로 확인합니다.</p>'''
    with (output/'index.html').open('x') as f:f.write(page.replace('__SUMMARY__',html.escape(json.dumps(summary,ensure_ascii=False,indent=2))).replace('__TABLE__',''.join(table)))
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);p.add_argument('reference',type=Path);p.add_argument('roots',nargs='+',type=Path)
    a=p.parse_args();build(a.output,a.reference,a.roots)
