#!/usr/bin/env python3
"""Add TwinLite conversion evidence to the existing camera runtime reporter."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import shutil
import subprocess
import sys
from summarize_camera_visualization import viewer_condition_verified
from lifecycle_measurements import percentile


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    tools = Path(__file__).resolve().parent
    # Only completed runtime reports go to the existing runtime analyzer.
    # Conversion/quality reports have a different schema and stay separate.
    staged = a.runtime_root / 'analysis-input'
    for matrix in sorted(a.runtime_root.glob('*matrix-*')):
        if not matrix.is_dir():
            continue
        for report in matrix.glob('*/report.json'):
            dest = staged / matrix.name / report.parent.name
            dest.mkdir(parents=True, exist_ok=True)
            for source in report.parent.iterdir():
                if source.is_file() and not (dest / source.name).exists():
                    (dest / source.name).symlink_to(source.resolve())
            jobs = matrix / 'jobs.json'
            if jobs.exists() and not (dest.parent / jobs.name).exists():
                (dest.parent / jobs.name).symlink_to(jobs.resolve())
    subprocess.run([sys.executable, str(tools / 'summarize_camera_visualization.py'),
        '--results', str(staged), '--output', str(a.output),
        '--matrix', 'live-final-matrix-r1', '--soak-matrix', 'soak-matrix-r1',
        '--structure-matrices', 'live-final-matrix-r1',
        '--group-prefixes', json.dumps({'live-t0-': 'pytorch', 'live-t2-': 'tensorrt'})], check=True)
    summary = json.loads((a.output / 'summary.json').read_text())
    quality = {}
    table = dict(title='동일 native GT 1,404장 · 변환/전처리 검증',
                 headers=['구성', '장수', '최대 영상별 확률 p99 차이', '최저 class 일치율 %',
                          '최대 GT IoU 하락 %p', '수치/GT 통과'], rows=[])
    for label, folder in [('T1 logits', 'quality-logits-full-r1'),
                          ('T2 GPU', 'quality-reduced-gpu-full-r1'),
                          ('T2 CPU', 'quality-reduced-cpu-full-r1')]:
        source = a.runtime_root / folder
        if not (source / 'report.json').exists():
            continue
        q = json.loads((source / 'report.json').read_text())
        quality[label] = dict(images=q['images'], checkpoint_sha256=q['checkpoint_sha256'],
            validation_sha256=q['validation_sha256'], numerical_pass=q['numerical_pass'],
            quality_pass=q['quality_pass'], side_batch1=q.get('side_batch1'), batch1_pass=q.get('batch1_pass',False),
            maximum_p99=max(max(s['probability_abs_p99']) for s in q['samples']),
            minimum_class_agreement=1-max(s['class_mismatch'] for s in q['samples']),
            maximum_iou_drop=max(max(s['iou_drop']) for s in q['quality']), per_group=q['quality'])
        v = quality[label]
        table['rows'].append([label, v['images'], round(v['maximum_p99'], 6),
            round(100*v['minimum_class_agreement'], 4), round(100*v['maximum_iou_drop'], 4),
            v['numerical_pass'] and v['quality_pass']])
        shutil.copytree(source, a.output / folder, dirs_exist_ok=True)
    offline = dict(title='저장 영상 · 세 모델 동시 실행 · MORAI/GUI 제외',
        headers=['구성', 'stream', 'Hz', '최저5초Hz', 'p50', 'p95', 'p99', '최대공백ms', 'CPU%', 'VRAM MiB'], rows=[])
    for t in summary['trials']:
        if t['matrix'] == 'live-final-matrix-r1':
            t['comparison_group'] = 'pytorch' if 't0-' in t['trial'] else 'tensorrt'
        if t.get('saved_input'):
            for key, s in t['streams'].items():
                if 'model_grid' not in key and '/objects:' not in key and '/traffic_light/' not in key:
                    continue
                offline['rows'].append([t['trial'], key.replace('/perception/', ''), round(s['hz'], 2),
                    min(s['five_second_hz'], default=0), *[round(s['wall_ms']['p'+str(q)], 2) for q in (50, 95, 99)],
                    round(1000*s['max_gap_sec'], 2), round(t['resources']['camera_cpu_mean'], 1),
                    t['resources']['vram_peak_mib']])
    candidates = [t for t in summary['trials'] if t['matrix'] == 'live-final-matrix-r1' and 't2-' in t['trial']]
    baselines = [t for t in summary['trials'] if t['matrix'] == 'live-final-matrix-r1' and 't0-' in t['trial']]
    soaks = [t for t in summary['trials'] if t['matrix'] == 'soak-matrix-r1']
    stages = dict(title='TensorRT CUDA stage · 5Hz diagnostic snapshots의 p50 (ms), 프레임 전체 분위수 아님',
        headers=['실험', 'batch 뷰', '표본', '입력 버퍼 copy', 'engine (+ reduction)', 'engine 밖 reduction', 'D2H', 'backend wall'], rows=[])
    for t in summary['trials']:
        values = defaultdict(lambda: defaultdict(list))
        path = staged / t['matrix'] / t['trial'] / 'diagnostics.json'
        if not path.exists() or t.get('start') is None:
            continue
        for event in json.loads(path.read_text()):
            if not t['start'] <= event['at'] < t['end']:
                continue
            for status in event['statuses']:
                for group, timings in json.loads(status['values'].get('latest_backend_stage_ms', '{}')).items():
                    for key, value in timings.items():
                        if key.endswith('_ms'):
                            values[group][key].append(value)
        for group, timings in values.items():
            keys=('input_copy_gpu_ms','engine_gpu_ms','reduction_gpu_ms','d2h_gpu_ms','backend_wall_ms')
            stages['rows'].append([t['trial'],group,len(timings['engine_gpu_ms'])] +
                [round(percentile(timings[k],50),3) if timings[k] else None for k in keys])
    checks = dict(quality=len(quality) == 3 and all(q['numerical_pass'] and q['quality_pass'] for q in quality.values()),
        side_batch1=any(q['batch1_pass'] and len(q.get('side_batch1') or [])==2 for q in quality.values()),
        six_live_repeats=len(candidates) == 6 and all(sum('-'+w+'-' in t['trial'] for t in candidates)==3 for w in ('sunny','foggy')),
        six_baseline_repeats=len(baselines) == 6 and all(sum('-'+w+'-' in t['trial'] for t in baselines)==3 for w in ('sunny','foggy')),
        viewer_conditions_verified=all(viewer_condition_verified(t) for t in baselines+candidates+soaks),
        fixed_candidate_source=len({t['source_sha256'] for t in candidates+soaks})==1,
        live_repeats_pass=bool(candidates) and all(not t['minimum_acceptance_failures'] for t in candidates),
        continuous_30min=any(t.get('complete') and t['end']-t['start'] >= 1800 for t in soaks),
        continuous_pass=bool(soaks) and all(not t['minimum_acceptance_failures'] for t in soaks))
    summary.update(title='TwinLite TensorRT 전환·전후처리 검증 — 2026-10-15',
        introduction='Medium v6 EMA·전방544×960 B1·좌우480×640 B2. 동일 GT 품질과 통합 부하를 별도 판정한다. T0/T1/T2/T3 및 실패 원본을 보존하며, 단순 변환 성공은 운영 수락이 아니다.',
        gallery_title='TwinLite TensorRT · PyTorch/ONNX/FP16·GPU 입력',
        comparison_groups=['pytorch', 'tensorrt'], comparison_heading='PyTorch / TensorRT · MORAI·전체 GUI',
        additional_tables=[table, offline, stages], quality_summary=quality,
        runtime_completion_checks=checks, selected_runtime_completion_checks=checks, runtime_acceptance_pass=all(checks.values()),
        final_candidate_failures={t['trial']: t['minimum_acceptance_failures'] for t in candidates+soaks if t['minimum_acceptance_failures']},
        acceptance_review='기본 전환은 전체 수락 후 판단. 이번 단계에서 PR 병합·차량 제어·재학습 없음.')
    (a.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    subprocess.run([sys.executable, str(tools / 'build_camera_visualization_gallery.py'),
        '--summary', str(a.output / 'summary.json'), '--results', str(staged), '--output', str(a.output)], check=True)
    links = '<h2>Native GT / PyTorch / TensorRT 비교</h2>'
    for folder, label in (('quality-logits-full-r1', 'T1 · 기존 전후처리와 TensorRT'),
                          ('quality-reduced-gpu-full-r1', 'T2 · GPU 입력과 축약 출력'),
                          ('quality-reduced-cpu-full-r1', '대조 · CPU 입력과 측방 batch 1')):
        if (a.output / folder / 'index.html').exists():
            links += '<p><a href="{0}/index.html">{1} — 1,404장 검증·대표 영상</a></p>'.format(folder,label)
    page = a.output / 'index.html'
    page.write_text(page.read_text().replace('</html>', links + '</html>'))
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
