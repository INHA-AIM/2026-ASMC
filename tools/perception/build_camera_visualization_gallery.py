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
    if report.get('schema')=='shadow_family_runtime_v1':
        return family_supply_gallery(a,report,identifier)
    soak_matrix=report.get('soak_matrix','soak')
    main_trials=[r for r in report['trials'] if r.get('effective_final_trial') or r['matrix']==soak_matrix]
    primary=[r for r in main_trials if '-control-' not in r['trial'] and r['matrix']!=soak_matrix]
    overview=[]
    for group in report.get('comparison_groups',('baseline','candidate')):
        for weather in ('sunny','foggy'):
            subset=[r for r in primary if r.get('comparison_group','baseline' if '-baseline-' in r['trial'] else 'candidate')==group and '-'+weather+'-' in r['trial']]
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
    title='[%s · 2026-10-15] %s'%(identifier,report.get('gallery_title','객체 시각화·차선 공급 최적화'))
    status='수락 통과' if report.get('acceptance_complete') else '검증 결과 확인 필요 / 기본 승격 보류'
    content='<h1>'+esc(title)+'</h1><p><a href="../index.html">ASMC 실험 목록</a> · <a href="summary.json">전체 수치 JSON</a> · <a href="trials.csv">실행별 CSV</a></p>'
    content+='<p class="status">'+esc(status)+'</p><p>'+esc(report['introduction'])+'</p>'
    if report.get('condition_correction'):
        content+='<p class="status">측정 조건 정정: '+esc(report['condition_correction']['reason'])+' 초기 수치는 보존하며 viewer를 정리한 별도 대조를 수행한다.</p>'
    content+='<p>모델 입력 callback→발행 지연과 차선 family 지연은 별도다. 백분위는 원시 프레임을 합쳐 계산했다. CPU는 camera/debug/Main preview 프로세스 합산(100%=1코어), rqt는 WSL 전체 CPU에 포함된다. GPU는 MORAI·Windows 전체다.</p>'
    for extra in report.get('additional_tables',[]):
        content+='<h2>'+esc(extra['title'])+'</h2>'+table(extra['headers'],extra['rows'])
    if report.get('quality_gallery'):
        content+='<h2>같은 RGB · Native GT / 기존384 / 후보480</h2><p>6개 조건 run의 고정54장 중18장 표시. 연속 관측이며 독립 장면54개로 해석하지 않는다.</p><div class="photos">'+''.join(
            '<figure><figcaption>'+esc(r['label'])+'</figcaption><a href="'+esc(r['file'])+'"><img loading="lazy" src="'+esc(r['file'])+'"></a></figure>' for r in report['quality_gallery'])+'</div>'
    content+='<h2>'+esc(report.get('comparison_heading','기존 / 신규 전체 GUI · 날씨별 3회'))+'</h2>'+table(['구성','날씨','회차','CPU % 평균','RSS peak MiB','VRAM peak %','유효 family Hz 평균','최저 5초 Hz','readiness false 상태 수'],overview)
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


def family_supply_gallery(args, report, identifier):
    """Render the same retained producer/consumer measurements, without pooling medians."""
    title=identifier+' · TensorRT 기본·family 공급 최적화 — 2026-10-15–16'
    content='<h1>'+esc(title)+'</h1><p><a href="../index.html">ASMC 실험 목록</a> · <a href="summary.json">전체 JSON</a> · <a href="trials.csv">뷰·회차 CSV</a> · <a href="parity/index.html">동일 TensorRT 입력 전후 비교</a></p>'
    content+='<p>TensorRT 기본 설정과 전체 공급 수락은 별도입니다. PR #85는 사용자 주행 확인 전 Draft입니다. 기존·신규 모두 같은 engine·입력·유효성 기준을 사용합니다. 반복 시험은 SUNNY/FOGGY13시 각3회60초이며 마지막 FOGGY 전체 GUI 연속 시험은10분입니다. 실제 rqt 한 개·Main preview·LiDAR 부하를 포함합니다.</p>'
    content+='<p>모델 callback→발행, family 계산, callback→family 수신, family 발행→수신을 분리합니다. Source stamp는 촬영 시각 보장이 아닙니다. 분위수는 실제 raw 표본에서 계산하며 실행별 중앙값을 평균내지 않습니다. Family producer/consumer gap은 각각250ms 기준입니다.</p>'
    primary=set(report.get('primary_trial_reports',[]))
    if primary:
        content+='<p><strong>'+('자동 수락 통과 · 사용자 주행 확인 대기' if report.get('automatic_acceptance_passed') else '자동 수락 미완료 · Draft 유지')+'</strong>. 본문 비교표는 정규 측정13회이며, 이전 결함/중단 실행은 실패 표와 JSON에 보존합니다.</p>'
    overview=[];details=[];photos=[];failures=[]
    for trial in report['trials']:
        if trial['failures']:failures.append([str(Path(trial['report']).parent),'; '.join(trial['failures'])])
        if primary and trial['report'] not in primary:continue
        if 'lane_family' not in trial:
            continue
        family=trial['lane_family'];valid=family['valid_supply'];producer=family.get('producer_supply',{})
        overview.append([trial['name'],number(valid['hz'],2),number(min(valid['five_second_hz'],default=0),2),
            number(1000*producer.get('max_gap_sec',0)),number(1000*valid['max_gap_sec']),
            *[number(family.get('input_to_subscriber_ms',{}).get('p'+str(q))) for q in (50,95,99)],
            number(family.get('compute_wall_ms',{}).get('p95')),
            trial['active_status_failures'].get('lane'),number(100*trial['resources']['vram_peak_fraction']),
            'PASS' if not trial['failures'] else 'FAIL'])
        for key,stream in trial['streams'].items():
            details.append([trial['name'],key.replace('/perception/camera/',''),number(stream['hz'],2),
                number(min(stream['five_second_hz'],default=0),2),
                *[number(stream['wall_ms']['p'+str(q)]) for q in (50,95,99)],
                number(1000*stream['max_gap_sec']),stream['target_95pct_pass']])
        root=args.results/Path(trial['report']).parent
        if '-r1' in trial['name'] or 'soak' in trial['name']:
            for pattern in ('sample-*.jpg','failure-*.jpg'):
                for source in sorted(root.glob(pattern)):
                    dest=trial['name']+'-'+source.name;shutil.copyfile(source,args.output/dest)
                    photos.append('<figure><figcaption>'+esc(dest)+'</figcaption><a href="'+esc(dest)+'"><img loading="lazy" src="'+esc(dest)+'"></a></figure>')
    content+='<h2>회차별 공급·공백</h2>'+table(['회차','유효 family Hz','최저5초 Hz','발행 공백ms','수신 공백ms','전체 p50','p95','p99','계산 p95','readiness false','VRAM%','수락'],overview)
    pooled=[]
    for group,streams in report['pooled_latency'].items():
        if primary and ('condition-invalid' in group or group.startswith('smoke-')):continue
        for key,value in streams.items():
            pooled.append([group,key,value['count'],*[number(value['p'+str(q)+'_ms']) for q in (50,95,99)]])
    content+='<h2>원시 프레임 통합 분위수</h2>'+table(['구성·날씨','항목','n','p50 ms','p95 ms','p99 ms'],pooled)
    content+='<h2>실제 화면·실패 보존</h2><p>수신한 debug JPEG입니다. 실패 화면은 readiness 저하 시 최근 화면이며 동기 GT라고 설명하지 않습니다.</p><div class="photos">'+''.join(photos)+'</div>'
    content+='<details><summary>뷰별 모든 회차</summary>'+table(['회차','stream','Hz','최저5초 Hz','p50','p95','p99','공백ms','목표95%'],details)+'</details>'
    content+='<h2>실패·계측 오류</h2>'+table(['회차','사유'],failures)
    content+='<p>GT·모델 품질과 실행 구조를 구분합니다. 이전 PyTorch/TensorRT 곡선 차이·365.9ms 수신 공백은 <a href="../camera-shadow-family-20261015-r1/index.html">이전 E356</a>에 보존합니다. 이번 결과는 주행 수락·LiDAR 연결·PR 병합이 아닙니다.</p>'
    style='body{font:15px system-ui;background:#111923;color:#dae5f3;margin:24px}a{color:#75caff}h2{margin-top:32px}.scroll{overflow:auto}table{border-collapse:collapse;white-space:nowrap;font-size:13px}td,th{padding:8px;border:1px solid #344556}.photos{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:12px}img{max-width:100%}summary{cursor:pointer;padding:12px}'
    (args.output/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+esc(title)+'</title><style>'+style+'</style>'+content+'</html>')
    print(json.dumps(dict(experiment_id=identifier,trials=len(report['trials']),page=str(args.output/'index.html'))))


if __name__=='__main__':main()
