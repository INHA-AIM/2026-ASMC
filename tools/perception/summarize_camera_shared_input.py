#!/usr/bin/env python3
"""Combine retained input trials/GT/model provenance using the existing reporter."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model-manifest',type=Path,required=True)
    a=p.parse_args(); tools=Path(__file__).resolve().parent
    subprocess.run([sys.executable,str(tools/'summarize_camera_visualization.py'),
        '--results',str(a.runtime_root/'results'),'--output',str(a.output),
        '--matrix','worker-streams-r1','--structure-matrices','worker-streams-r1,cpu-cache-r1,final-repeats-r1',
        '--soak-matrix','soak-r1'],check=True)
    report=json.loads((a.output/'summary.json').read_text())
    quality=json.loads((a.runtime_root/'quality-streams-r1/report.json').read_text())
    benchmark=json.loads((a.runtime_root/'input-benchmark-exclusive.json').read_text())
    forward=json.loads((a.runtime_root/'package-forward.json').read_text())
    manifest=json.loads(a.model_manifest.read_text())
    selected=[r for r in report['trials'] if r.get('effective_final_trial') and r['comparison_group']=='separate']
    soak=[r for r in report['trials'] if r['matrix']=='soak-r1']
    failures={r['matrix']+'/'+r['trial']:r['minimum_acceptance_failures'] for r in selected+soak if r['minimum_acceptance_failures']}
    checks=dict(three_per_weather=all(sum('-'+w+'-' in r['trial'] for r in selected)==3 for w in ('sunny','foggy')),
                completed_soak=len(soak)==1 and soak[0].get('complete') and soak[0]['end']-soak[0]['start']>=1800,
                selected_runtime_gates=not failures,quality=quality['direct_pass'] and quality['gpu_detection_pass'] and quality['twin_gpu_pass'],
                package_forward=forward['all_outputs_equal'],
                viewer_conditions=all(r.get('qt_inventory_before')==[] and r.get('qt_inventory_after')==[] and
                    len(r.get('qt_inventory_during',[]))==1 and r['qt_inventory_during'][0]['owned_trial'] for r in selected+soak))
    tables=[dict(title='패키지 배포 파일',headers=['파일','bytes','source SHA','deployment SHA'],
        rows=[[r['path'],r['bytes'],r['source_sha256'],r['sha256']] for r in manifest['files']]),
        dict(title='같은 Native GT의 측방384 / 480',headers=['뷰','drivable384','drivable480','white384','white480','yellow384','yellow480','stop384','stop480'],rows=[]),
        dict(title='같은 프레임 직렬 A/B/A · 대형 영상 복사 계측',headers=['구성','decode','H2D calls','H2D bytes','p50 ms','p95 ms','p99 ms'],rows=[])]
    for view in ('front','left','right'):
        old,new=quality['gt']['side384:'+view],quality['gt']['side480:'+view]
        values=[view,round(old['drivable_iou']*100,2),round(new['drivable_iou']*100,2)]
        for i in (1,2,3):
            values.extend([round(r['marking_iou'][i]*100,2) if r['target_pixels'][i+1] else 'GT 없음' for r in (old,new)])
        tables[1]['rows'].append(values)
    for name,row in benchmark['results'].items():
        tables[2]['rows'].append([name,row['decode_calls'],row['image_h2d_calls'],row['image_h2d_bytes'],
                                  *[round(row['wall_ms']['p'+str(q)],2) for q in (50,95,99)]])
    gallery=[]
    for file in sorted((a.runtime_root/'quality-streams-r1').glob('*.jpg')):
        shutil.copyfile(file,a.output/file.name)
        gallery.append(dict(file=file.name,label=file.stem))
    report.update(title='B1/B2 입력 통일·YOLO 배포 모델·공유 경로 검증 — 2026-10-15',
        gallery_title='B1/B2 입력 통일·YOLO 배포 모델·공유 경로 비교',
        comparison_heading='독립 worker 날씨별3회 / 공유 후보 각각1회 · 전체 GUI',
        introduction='전방544×960·측방480×640 후보. 공유 GPU/디코딩은 지연 기준에 미달해 기본은 독립 worker다. GT54장은6개 기존 run의 연속 관측이다. A/B/A는 직렬 복사 진단이며 live 수락을 대체하지 않는다. 원본 checkpoint와 모든 실패를 보존한다.',
        comparison_groups=['separate','shared_gpu','shared_decode'],additional_tables=tables,quality_gallery=gallery,
        quality_summary={k:v for k,v in quality.items() if k.endswith('_pass') or k=='gt'},
        package_forward=forward,selected_runtime='separate',shared_gpu_default=False,
        selected_runtime_completion_checks=checks,selected_runtime_failures=failures,
        runtime_acceptance_pass=all(checks.values()),acceptance_complete=False,
        acceptance_review='PR Draft 유지. 런타임 수락과 CI를 별도로 판정하며 이 분석기는 운영 승격이나 병합을 승인하지 않는다.')
    (a.output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    shutil.copyfile(a.model_manifest,a.output/'model-manifest.json')
    for source,name in [(a.runtime_root/'quality-streams-r1/report.json','quality.json'),
                        (a.runtime_root/'input-benchmark-exclusive.json','input-benchmark.json')]:
        shutil.copyfile(source,a.output/name)
    subprocess.run([sys.executable,str(tools/'build_camera_visualization_gallery.py'),
        '--summary',str(a.output/'summary.json'),'--results',str(a.runtime_root/'results'),'--output',str(a.output)],check=True)
    print(json.dumps(dict(checks=checks,failures=failures)))


if __name__=='__main__':main()
