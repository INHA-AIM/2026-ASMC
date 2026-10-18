#!/usr/bin/env python3
"""Render retained shared-IMU BEV trials without scanning the artifact tree."""
import argparse
import html
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rows=[]
    for name,label in [('baseline','기존 고정 BEV'),('candidate','보정 공유 첫 구현'),
                       ('early-projection','피팅 전 보정 공유')]:
        src=a.run_root/'live'/name;dst=a.output/name;dst.mkdir()
        report=json.loads((src/'report.json').read_text())
        for path in src.iterdir():
            if path.suffix in ('.json','.jpg'):shutil.copy2(path,dst/path.name)
        stream=report['streams']['comparison']
        cpu=sum(report['cpu_mean'].get(n,0) for n in ('/camera_bev','/camera_bev_fusion','/camera_bev_visualizer'))
        # First candidate started during warmup; its PID inventory is incomplete.
        cpu_text='초기 PID 수집 불완전' if name=='candidate' else '%.2f%%'%cpu
        values=[label,'%.2f'%stream['hz'],'%.1f'%stream['minimum_5s_hz'],
                '%.1f'%stream['source_age_ms']['p95'],'%.1f'%stream['gap_max_ms'],cpu_text]
        rows.append('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+'</tr>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>Family·comparison BEV의 IMU 보정 재사용</title>
<style>body{font:16px system-ui;background:#14202a;color:#eee;margin:24px;line-height:1.6}a{color:#8ed7ff}td,th{border:1px solid #567;padding:8px}table{border-collapse:collapse}img{max-width:100%;height:auto}.pair{display:grid;grid-template-columns:1fr 1fr;gap:16px}@media(max-width:900px){.pair{grid-template-columns:1fr}}</style>
<h1>2026-10-18 · 두 시각화의 IMU 보정 재사용</h1><a href="../index.html">ASMC 실험 목록</a>
<p>IMU 조회·M1/M0 선택은 family worker에서 한 번. 작은 metadata를 피팅 전에 발행하고,
comparison은 같은 원 RGB·semantic을 뷰별 한 geometry로 투영합니다. 추론·피팅을 다시 실행하지 않습니다.</p>
<p>차량·날씨·제어를 변경하지 않은 각60초 읽기 전용 참고 측정입니다. 경로·자세를 통제한 품질 대조나
음영 ODD 주행 수락이 아닙니다. 기존 실행부터 family 무효가 많았으며 최종843개 중780개도 무효입니다.
이 결과를 readiness 통과로 사용하지 않습니다. rqt 창 대신 실제 영상 토픽 구독으로 렌더링 부하를 만들었습니다.</p>
<table><tr><th>구성</th><th>평균 Hz</th><th>최소5초 Hz</th><th>bridge header→표시 p95 ms</th><th>최대 공백 ms</th><th>BEV 체인 CPU, core100% 기준</th></tr>'''
    page+=''.join(rows)+'</table><p>첫 공유 구현의 대기를 줄인 뒤 최종 comparison p95 약238ms입니다. 기존 약217ms와 비교해 보정 raster 비용이 남습니다. 카메라 촬영→광자 표시 지연으로 해석하지 않습니다. CPU 수치는 서로 다른 참고 구간이며 엄밀한 동일 pose의 인과 효과가 아닙니다.</p>'
    page+='<div class="pair">'
    for name,label in [('baseline','기존 고정 지면'),('early-projection','같은 프레임의 IMU 공유')]:
        page+='<section><h2>'+label+'</h2><a href="'+name+'/comparison.jpg"><img src="'+name+'/comparison.jpg"></a><a href="'+name+'/report.json">원본 측정</a></section>'
    page+='</div><p>그림은 각 실행의 실제 수신 예시로 같은 장면 비교가 아닙니다. M0 복귀는 화면에 사유를 표시합니다. 차량 이동/yaw에 따른 Ego 기준 도로 이동과 roll/pitch 흔들림을 구분합니다.</p></html>'
    (a.output/'index.html').write_text(page,encoding='utf-8')
    print(a.output/'index.html')


if __name__=='__main__':main()
