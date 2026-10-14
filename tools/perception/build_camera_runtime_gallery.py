#!/usr/bin/env python3
"""Local camera runtime report and frozen-image NMS gallery. No model execution."""
import argparse
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--quality',type=Path,required=True)
    p.add_argument('--confirmation',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--confidence-report',type=Path,help='Follow-up 0.40/0.50/0.60 at fixed NMS0.50')
    a=p.parse_args()
    import cv2
    import numpy as np
    sys.path[:0]=[str(Path(__file__).resolve().parents[2]/'src/asmc_camera/src')]
    from asmc_camera.detection.output_nms import suppress
    from asmc_camera.detection.hood_mask import HoodMask
    a.output.mkdir(parents=True,exist_ok=True)
    registry_path=a.output.parent/'gallery-identifiers.json'
    registry=json.loads(registry_path.read_text()) if registry_path.exists() else {}
    key=a.output.name+'/index.html'
    if key not in registry:
        registry[key]=dict(number=max((v['number'] for v in registry.values()),default=0)+1,
            timestamp=datetime.now(timezone.utc).isoformat(),time_basis='report_first_creation_utc')
        registry_path.write_text(json.dumps(registry,ensure_ascii=False,indent=2)+'\n')
    identifier='E%03d'%registry[key]['number']
    report=json.loads(a.summary.read_text());quality=json.loads(a.quality.read_text());confirmation=json.loads(a.confirmation.read_text())
    candidate=next(r for r in quality['grid'] if r['confidence']==.4 and r['nms_iou']==.5 and r['exclusive_iou'] is None)
    confidence_report=json.loads(a.confidence_report.read_text()) if a.confidence_report else None
    def qrow(name,r):
        return '<tr><td>'+name+'</td>'+''.join('<td>'+v+'</td>' for v in [
            '%.2f%%'%(100*r['groups']['macro_f1_fixed3']),'%.2f%%'%(100*r['labels']['macro_f1_fixed16']),
            str(r['duplicate_fp']),str(r['wrong_name_fp']),str(r['background_or_geometry_fp'])])+'</tr>'
    quality_rows=qrow('selection · .40/.70',quality['baseline'])+qrow('selection · .40/.50',candidate)+qrow(
        'heldout · .40/.70',confirmation['baseline'])+qrow('heldout · .40/.50',confirmation['confirmation'])
    if confidence_report:
        quality_rows=''.join(qrow('selection · %.2f/.50'%r['confidence'],r) for r in confidence_report['rows'])
    performance=[]
    for r in report['trials']:
        for key,s in sorted(r['streams'].items()):
            accepted = (s.get('minimum_supply_pass',False) if confidence_report else s['target_95pct_pass']) and s['timing_pass']
            performance.append('<tr><td>'+html.escape(r['trial'])+'</td><td>'+html.escape(key.replace('/perception/camera/',''))+'</td>'+''.join(
                '<td>'+str(v)+'</td>' for v in [round(s['hz'],2),round(min(s['five_second_hz'],default=0),2),
                *[round(s['wall_ms']['p'+str(q)],1) for q in [50,95,99]],round(s['max_gap_sec']*1000,1),
                'PASS' if accepted else 'FAIL'])+'</tr>')
    package=Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera'
    masks={v:HoodMask.load(True,package/f) for v,f in [('front','yolo_hood_mask_v1.yaml'),
           ('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
    gt=[json.loads(s) for s in a.manifest.read_text().splitlines() if s]
    predictions={r['id']:r['predictions'] for r in [json.loads(s) for s in a.predictions.read_text().splitlines() if s]}
    (a.output/'images').mkdir(exist_ok=True);cards=[]
    for index,row in enumerate(gt):
        image=cv2.imread(row['image'])
        if image is None:raise ValueError('Missing gallery input: '+row['image'])
        if row.get('view') in masks:image=masks[row['view']].apply(image)
        tiles=[]
        for confidence,threshold in ([(.4,.5),(.5,.5),(.6,.5)] if confidence_report else [(.4,.7),(.4,.5)]):
            tile=image.copy()
            for ann in row['annotations']:
                if ann.get('supervision')=='excluded':continue
                color=(230,0,230) if ann.get('supervision')=='ignore' else (255,210,0)
                x1,y1,x2,y2=map(round,ann['xyxy']);cv2.rectangle(tile,(x1,y1),(x2,y2),color,1)
            boxes,_=suppress(predictions[row['id']],confidence,threshold)
            for box in boxes:
                x1,y1,x2,y2=map(round,box['xyxy']);cv2.rectangle(tile,(x1,y1),(x2,y2),(0,220,255),2)
                cv2.putText(tile,box['class_name']+' %.2f'%box['confidence'],(x1,max(y1-4,18)),0,.45,(0,220,255),1)
            tile=cv2.resize(tile,(640,360));cv2.putText(tile,'confidence %.2f / NMS %.2f'%(confidence,threshold),(12,27),0,.65,(255,255,255),2);tiles.append(tile)
        cv2.imwrite(str(a.output/'images'/f'{index:03d}.jpg'),np.hstack(tiles))
        source=html.escape(row['source']);view=html.escape(row.get('view','unknown'))
        cards.append(f'<article data-source="{source}"><p>{index+1:03d} · {source} · {view} · {html.escape(row["id"][:16])}</p><img loading="lazy" src="images/{index:03d}.jpg" alt="동일 영상의 GT와 후처리 설정 비교"></article>')
    title='카메라 실행 구조·후처리 검증 — 2026-10-14–15'
    if confidence_report:title=report['title']
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'''+title+'''</title>
<style>body{background:#121c2a;color:#edf4fa;font:15px/1.6 system-ui;margin:28px}main{max-width:1400px;margin:auto}a{color:#8bc9ff}td,th{padding:8px;border-bottom:1px solid #345;text-align:left}table{border-collapse:collapse}img{max-width:100%;height:auto}article,section{margin:30px 0}input{padding:10px;width:360px;background:#223045;color:#fff;border:1px solid #567}.scroll{overflow:auto;max-height:650px}summary{cursor:pointer}code{color:#9fe3cd}</style><main>
<a href="../index.html">ASMC 실험 목록</a><h1>'''+identifier+' · '+title+'''</h1>
<p>기본은 R1 독립 프로세스 · YOLO15Hz/TwinLite10Hz/ResNet10Hz · E8 · confidence0.40/NMS0.70입니다.
최종 기본 반복의 두 회차는 상류 입력 공백으로 전체 공급 수락에 실패했습니다. 자동 면제하지 않으며 sensor/bridge 대조와 후속 확인을 구분합니다. 공유 R2는 지연 악화로 채택하지 않았습니다. TwinLite15Hz는 SUNNY 한 5초 창14.2Hz로 목표14.25Hz에 미달하여 기본 승격하지 않았습니다.
NMS0.50은 E8 대조 설정이며 교차 클래스 억제는 제외했습니다. 이 결과는 판제/asmc_lidar 연결이나 공동 자율주행 수락이 아닙니다.</p>
<p><a href="summary.json">전체 근거 JSON</a> · <a href="trials.csv">뷰·회차별 CSV</a> · <a href="runtime-postprocessing-final.json">104개 후처리 조합</a> · <a href="runtime-heldout-final.json">고정 설정 확인</a> · <a href="nms-gt-retention-r3.json">GT 검출 유지 확인</a></p>
<h2>품질 — 기존 개발 평가본</h2><p>16종·3그룹 모두 일대일 IoU≥0.5. 정상 GT 우선, ignore·partial 구분. selection205는 반복 사용한 개발본이고 heldout116도 새 미공개 시험으로 설명하지 않습니다.
입력 packing·FP16·mask·clipping 전 tensor NMS를 실제 실행과 일치시켰습니다. 뷰 미확인 영상에는 mask를 추정 적용하지 않습니다.
이전 PyTorch 동적 입력 평가와 특히 unknown 뷰의 tensor shape가 달라 수치를 직접 비교하지 않습니다. 희소 클래스·소스 편중의 한계는 JSON에 보존합니다.</p>
<table><tr><th>자료·confidence/NMS</th><th>3그룹 F1</th><th>16종 F1</th><th>중복 FP</th><th>이름 FP</th><th>배경·위치 FP</th></tr>'''+quality_rows+'''</table>
<h2>처리율·지연</h2><p>실제 모델/broker 입력 callback 진입→ROS 발행의 monotonic wall 시간입니다. 촬영 시각→결과 지연이 아닙니다.
p50/p95/p99는 해당 회차의 원시 프레임 분포입니다. 반복 통합 값도 원시 프레임을 합쳐 계산하며 중앙값을 평균내지 않습니다.
완전한 5초 창만 사용합니다. FAIL은 목표95%·지연·공백 중 하나 이상의 미달입니다. 전체 VRAM과 Linux CPU/RSS는 JSON에서 확인합니다.</p>
<div class="scroll"><table><tr><th>회차</th><th>모델·뷰</th><th>Hz</th><th>최소5s Hz</th><th>p50 ms</th><th>p95 ms</th><th>p99 ms</th><th>최대공백 ms</th><th>판정</th></tr>'''+''.join(performance)+'''</table></div>
<h2>동일 영상 · 기본0.70 / 대조0.50</h2><p>청록은 양성 GT, 자홍은 ignore, 노랑은 예측입니다. GT는 수정하지 않습니다. 한 객체의 여러 박스를 여러 TP로 세지 않습니다.</p>
<input id="search" placeholder="출처 검색"><div id="cards">'''+''.join(cards)+'''</div>
<script>document.getElementById('search').oninput=e=>document.querySelectorAll('article').forEach(a=>a.hidden=!a.textContent.toLowerCase().includes(e.target.value.toLowerCase()));</script></main></html>'''
    (a.output/'index.html').write_text(page)
    if confidence_report:
        begin=page.index('<p>기본은 R1')
        end=page.index('</p>',begin)+4
        page=page[:begin]+'<p>'+html.escape(report['introduction'])+'</p><p><a href="confidence-comparison.json">confidence별 recall·TP/FP</a></p>'+page[end:]
        page=page.replace('FAIL은 목표95%·지연·공백 중 하나 이상의 미달입니다.',
            '이번 PASS는 YOLO/TwinLite 최소10Hz·ResNet 최소9.5Hz와 기존 지연·공백 기준입니다. 목표15Hz의95% 달성 여부는 CSV/JSON에 별도 보존합니다.')
        page=page.replace('동일 영상 · 기본0.70 / 대조0.50','동일 영상 · confidence0.40 / 0.50 / 0.60 · NMS0.50')
        latest_rows=''.join('<tr><td>'+html.escape(r['trial'])+'</td>'+''.join('<td>'+str(v)+'</td>' for v in [
            round(r.get('lane_family',{}).get('hz',0),2),
            min(r.get('lane_family',{}).get('five_second_hz',[]),default=0),
            round(100*r.get('resources',{}).get('vram_peak_fraction',0),1),
            'PASS' if not r['minimum_acceptance_failures'] else 'FAIL'])+'</tr>'
            for r in report['trials'] if r['trial'].startswith(('live-latest-','extra-')))
        page=page.replace('<h2>품질', '<h2>최신 실제 MORAI 실행</h2><p>모델 공급·지연과 차선 readiness를 함께 검사한 전체 회차 판정입니다. 마지막 두 객체 시각화 회차는 세 모델과 기존 Main preview가 모두 ACTIVE이며 view.launch models:=objects만 켰습니다.</p><table><tr><th>회차</th><th>차선 family Hz</th><th>최소5초 Hz</th><th>전체 VRAM %</th><th>전체 판정</th></tr>'+latest_rows+'</table><h2>품질',1)
        family_rows=''.join('<tr><td>'+html.escape(r['trial'])+'</td>'+''.join('<td>'+str(v)+'</td>' for v in [
            round(r.get('lane_family',{}).get('hz',0),2),
            *[round(r['lane_family']['input_to_subscriber_ms']['p'+str(q)],1)
              if r.get('lane_family',{}).get('input_to_subscriber_ms',{}).get('p'+str(q)) is not None else '—' for q in (50,95,99)],
            html.escape(str(r.get('model_grid_counter_delta',{})))])+'</tr>' for r in report['trials'])
        extra='<h2>차선 family·BEV 모델 격자 공급</h2><p>family 지연은 같은 source stamp의 TwinLite callback→family 구독 수신이며 모델 발행 지연과 별도입니다. 격자 counter는 측정 중 두 diagnostics 사이 차이입니다.</p><table><tr><th>회차</th><th>family Hz</th><th>p50</th><th>p95</th><th>p99</th><th>격자 counter</th></tr>'+family_rows+'</table>'
        issues=''.join('<li>'+html.escape(r['trial'])+': '+html.escape('; '.join(r['minimum_acceptance_failures']))+'</li>'
            for r in report['trials'] if r['minimum_acceptance_failures'])
        extra+='<h2>실패·중단 기록</h2><p>모델별 표의 PASS와 차선 readiness·전체 회차 판정은 별도입니다. 이전 실패와 중단 실행은 숨기지 않습니다.</p><ul>'+issues+'</ul>'
        page=page.replace('<h2>동일 영상',extra+'<h2>동일 영상',1)
        (a.output/'index.html').write_text(page)
    print(json.dumps(dict(experiment_id=identifier,images=len(gt),page=str(a.output/'index.html'))))


if __name__=='__main__':main()
