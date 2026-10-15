#!/usr/bin/env python3
"""Render retained camera visualization measurements; no inference or GT edits."""
import argparse
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import shutil


def esc(value):
    return html.escape(str(value))


def number(value, digits=1):
    return 'unknown' if value is None else str(round(value,digits))


def table(headers, rows):
    return '<div class="scroll"><table><thead><tr>'+''.join('<th>'+esc(v)+'</th>' for v in headers)+\
        '</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(v)+'</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table></div>'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--results',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    report=json.loads(a.summary.read_text())
    registry_path=a.output.parent/'gallery-identifiers.json'
    registry=json.loads(registry_path.read_text()) if registry_path.exists() else {}
    key=a.output.name+'/index.html'
    if key not in registry:
        registry[key]=dict(number=max((v['number'] for v in registry.values()),default=0)+1,
            timestamp=datetime.now(timezone.utc).isoformat(),time_basis='report_first_creation_utc')
        registry_path.write_text(json.dumps(registry,ensure_ascii=False,indent=2)+'\n')
    identifier='E%03d'%registry[key]['number']
    soak_matrix=report.get('soak_matrix','soak')
    main_trials=[r for r in report['trials'] if r.get('effective_final_trial') or r['matrix']==soak_matrix]
    primary=[r for r in main_trials if '-control-' not in r['trial'] and r['matrix']!=soak_matrix]
    overview=[]
    for group in ('baseline','candidate'):
        for weather in ('sunny','foggy'):
            subset=[r for r in primary if '-'+group+'-' in r['trial'] and '-'+weather+'-' in r['trial']]
            if not subset:continue
            resource=[r['resources'] for r in subset]
            overview.append([group,weather,len(subset),
                number(sum(r['camera_cpu_mean'] for r in resource)/len(resource)),
                number(max(r['camera_rss_peak_mib'] for r in resource)),
                number(100*max(r['vram_peak_fraction'] for r in resource)),
                number(sum(r['lane_family']['valid_supply']['hz'] for r in subset)/len(subset),2),
                number(min(min(r['lane_family']['valid_supply']['five_second_hz']) for r in subset),2),
                sum(r.get('active_status_failures',{}).get('lane',0) for r in subset)])
    details=[]; codecs=[]
    for r in main_trials:
        for stream,s in sorted(r['streams'].items()):
            details.append([r['trial'],stream.replace('/perception/',''),number(s['hz'],2),
                number(min(s['five_second_hz'],default=0),2),
                *[number(s['wall_ms']['p'+str(q)]) for q in (50,95,99)],
                number(s['max_gap_sec']*1000),s['duplicate_stamps'],s['reversed_stamps'],
                'PASS' if s.get('minimum_supply_pass') and s['timing_pass'] else 'FAIL'])
        c=r.get('codecs',{})
        codecs.append([r['trial'],r.get('object_visual_node_count',0),
            sum(v['decode']['calls'] for v in c.values()),sum(v['encode']['calls'] for v in c.values()),
            r.get('debug_outputs',{}).get('objects',{}).get('messages',0),
            r.get('view_counter_delta',{}).get('individual_encode','legacy'),
            r.get('codec_contract_pass','legacy')])
    photos=[]
    for r in primary:
        if '-r1' not in r['trial']:continue
        root=a.results/r['matrix']/r['trial']
        for role in ('objects','lane','traffic_light'):
            source=root/('sample-'+role+'.jpg')
            if not source.exists():continue
            dest=r['trial']+'-'+source.name;shutil.copyfile(source,a.output/dest)
            photos.append('<figure><figcaption>'+esc(r['trial']+' · '+role)+'</figcaption><a href="'+esc(dest)+'"><img loading="lazy" src="'+esc(dest)+'"></a></figure>')
    failures=[];failure_photos=[]
    for r in report['trials']:
        if not r['minimum_acceptance_failures']:continue
        failures.append([r['matrix']+'/'+r['trial'],'; '.join(r['minimum_acceptance_failures']),
            str(r.get('readiness_failure_reasons',{})),r.get('resource_recovery_trial','')])
        if not (r['matrix']==soak_matrix or (r.get('effective_final_trial') and '-baseline-' in r['trial'] and '-r1' in r['trial'])):continue
        root=a.results/r['matrix']/r['trial']
        for role in ('objects','lane'):
            source=root/('failure-'+role+'.jpg')
            if not source.exists():continue
            dest=r['trial']+'-'+source.name;shutil.copyfile(source,a.output/dest)
            failure_photos.append('<figure><figcaption>'+esc(r['trial']+' · '+role+' · readiness 저하 때의 최근 화면')+'</figcaption><a href="'+esc(dest)+'"><img loading="lazy" src="'+esc(dest)+'"></a></figure>')
    pooled=[]
    for group,streams in report['pooled_frame_latency'].items():
        for stream,s in sorted(streams.items()):
            pooled.append([group,stream.replace('/perception/',''),s['count'],*[number(s['p'+str(q)+'_ms']) for q in (50,95,99)]])
    for group,s in report['pooled_family_latency'].items():
        pooled.append([group,'lane family (separate)',s['count'],*[number(s['p'+str(q)+'_ms']) for q in (50,95,99)]])
    title='[%s · 2026-10-15] 객체 시각화·차선 공급 최적화'%identifier
    status='수락 통과' if report.get('acceptance_complete') else '검증 결과 확인 필요 / 기본 승격 보류'
    content='<h1>'+esc(title)+'</h1><p><a href="../index.html">ASMC 실험 목록</a> · <a href="summary.json">전체 수치 JSON</a> · <a href="trials.csv">실행별 CSV</a></p>'
    content+='<p class="status">'+esc(status)+'</p><p>'+esc(report['introduction'])+'</p>'
    if report.get('condition_correction'):
        content+='<p class="status">측정 조건 정정: '+esc(report['condition_correction']['reason'])+' 초기 수치는 보존하며 viewer를 정리한 별도 대조를 수행한다.</p>'
    content+='<p>모델 입력 callback→발행 지연과 차선 family 지연은 별도다. 백분위는 원시 프레임을 합쳐 계산했다. CPU는 camera/debug/Main preview 프로세스 합산(100%=1코어), rqt는 WSL 전체 CPU에 포함된다. GPU는 MORAI·Windows 전체다.</p>'
    content+='<h2>기존 / 신규 전체 GUI · 날씨별 3회</h2>'+table(['구성','날씨','회차','CPU % 평균','RSS peak MiB','VRAM peak %','유효 family Hz 평균','최저 5초 Hz','readiness false 상태 수'],overview)
    content+='<h2>실제 화면</h2><p>각 회차에서 수신한 디버그 JPEG이며 추론 품질 GT가 아니다. 좌/전방/우 순서와 뷰별 시각·mask SHA를 유지한다.</p><div class="photos">'+''.join(photos)+'</div>'
    content+='<h2>실패 화면 보존</h2><p>readiness가 꺼졌을 때 가장 최근에 수신한 디버그 화면이다. 정확히 동기화한 source GT가 아니며, 화면만으로 센서 또는 추론 원인을 단정하지 않는다.</p><div class="photos">'+''.join(failure_photos)+'</div>'
    content+='<h2>원시 프레임 통합 지연</h2>'+table(['구성·날씨','stream','n','p50 ms','p95 ms','p99 ms'],pooled)
    soak_rows=[]
    for r in main_trials:
        if r['matrix']!=soak_matrix:continue
        family=r['lane_family'];streams=list(r['streams'].values())
        soak_rows.append([r['trial'],number(r['end']-r['start']),number(family['valid_supply']['hz'],2),
            number(min(family['valid_supply']['five_second_hz'],default=0),2),
            *[number(family['input_to_subscriber_ms']['p'+str(q)]) for q in (50,95,99)],
            number(max(v['wall_ms']['p95'] for v in streams)),number(max(v['wall_ms']['p99'] for v in streams)),
            number(max(v['max_gap_sec'] for v in streams)*1000),number(family['valid_supply']['max_gap_sec']*1000),
            r['active_status_failures']['lane'],number(100*r['resources']['vram_peak_fraction'])])
    content+='<h2>30분 연속 시험</h2>'+table(['실행','초','유효 family Hz','최저5초 Hz','family p50','family p95','family p99','모델 최대p95','모델 최대p99','모델 최대공백 ms','family 최대공백 ms','readiness false','VRAM %'],soak_rows)
    content+='<details><summary>뷰·회차·30분 연속 시험 상세</summary>'+table(['회차','stream','Hz','최저 5초 Hz','p50 ms','p95 ms','p99 ms','최대 공백 ms','중복','역행','최소/지연'],details)+'</details>'
    content+='<details><summary>노드·JPEG 호출 수</summary><p>계측 창 경계의 비동기 호출 수는 출력 수와 최대 한 프레임 차이가 날 수 있다. 신규 내부 counter와 CPU/ROS 시험으로 중간 인코딩0·합성당1회·같은 소스 중복 디코딩0을 검사했다.</p>'+table(['회차','객체 시각화 노드','decode','encode','mosaic 수신','개별 encode','counter 계약'],codecs)+'</details>'
    content+='<details><summary>이전 실패·미완료·계측 무효 기록 보존</summary>'+table(['실행','실패','readiness 원인','자원 재측정'],failures)+'</details>'
    content+='<h2>출력 동등성·최종 판정</h2><pre>'+esc(json.dumps(dict(output_parity=report.get('output_parity'),final_candidate_failures=report['final_candidate_failures'],invalid_resource_trials=report['invalid_resource_trials'],acceptance_review=report.get('acceptance_review')),ensure_ascii=False,indent=2))+'</pre>'
    content+='<p>기존 클래스/NMS 품질 대조: <a href="../camera-rate-confidence-20261015-r1/index.html">E348 · confidence0.40 / NMS0.50</a>. 이번 작업은 수집·학습·LiDAR 연결·판제 정책·실차 주행 수락이나 PR 병합을 포함하지 않는다.</p>'
    style='body{font:15px system-ui;background:#111923;color:#dae5f3;margin:24px}a{color:#75caff}h2{margin-top:32px}.status{padding:16px;background:#223c4d}.scroll{overflow:auto}table{border-collapse:collapse;white-space:nowrap;font-size:13px}th,td{padding:8px;border:1px solid #344556}th{background:#223141}.photos{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:12px}figure{margin:0}img{max-width:100%}summary{cursor:pointer;padding:12px}pre{white-space:pre-wrap}'
    (a.output/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+esc(title)+'</title><style>'+style+'</style>'+content+'</html>')
    print(json.dumps(dict(experiment_id=identifier,trials=len(main_trials),page=str(a.output/'index.html'))))


if __name__=='__main__':main()
