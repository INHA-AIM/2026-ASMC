#!/usr/bin/env python3
"""Publish compact provenance and a local review index; never publish images to Git."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import platform
import subprocess
from urllib.parse import quote

from camera_data_audit import read_jsonl, save, sha


def report(root, curated, dataset, gallery):
    downloads = json.loads((root/'download-report.json').read_text())
    snapshots = json.loads((root/'snapshot-report.json').read_text()) if (root/'snapshot-report.json').exists() else {'projects':[]}
    source_rows = {r['project']:r for r in downloads['projects']}
    for row in snapshots['projects']:
        if row['status']=='downloaded_snapshot':
            source_rows[row['project']]=row
    summary = json.loads((curated/'audit-summary.json').read_text())
    exported = json.loads((dataset/'summary.json').read_text()) if dataset and (dataset/'summary.json').exists() else None
    rows = list(read_jsonl(curated/'curated.jsonl'))
    sources=[]
    for project, row in sorted(source_rows.items()):
        metadata=row.get('metadata',{})
        stats=summary['sources'].get('roboflow:'+project,{})
        subset=[r for r in rows if r['source']=='roboflow:'+project]
        selected=metadata.get('selected_version',{})
        sources.append(dict(project=project,url='https://universe.roboflow.com/'+project,
            status=row['status'],version=row.get('version'),task=metadata.get('type','unknown'),
            license=metadata.get('license','unknown'),publisher_images=metadata.get('images'),
            exported_images=stats.get('images',0),accepted=stats.get('accepted',0),
            quarantined=stats.get('images',0)-stats.get('accepted',0),
            original_labels=stats.get('original_labels',{}),accepted_classes=stats.get('accepted_classes',{}),
            missing_native_annotations=sum('native_annotation_missing' in r['issues'] for r in subset),
            native_instance_annotations=sum(bool(a.get('native_segmentation')) for r in subset for a in r['annotations']),
            preprocessing=selected.get('preprocessing','unknown'),augmentation=selected.get('augmentation','unknown'),
            archive_sha256=row.get('archive_sha256'),native_annotation_sha256=row.get('native_annotation_sha256'),
            metadata_sha256=sha(Path(row['raw_path'])/'metadata.json') if row.get('raw_path') else None,
            version_policy_exception=metadata.get('version_policy_exception')))
    mappings=defaultdict(Counter)
    conditions={name:Counter() for name in ('weather','tunnel','view')}
    sizes=Counter();review_basis=Counter();unresolved=0;structure_failures=0;classes=Counter();splits=Counter()
    classes_by_split=defaultdict(Counter);negatives=Counter()
    exact_roles=defaultdict(set)
    for row in rows:
        for ann in row['annotations']:
            status=ann['canonical'] if ann['canonical'] else ann['disposition']
            mappings[(row['source'],ann['original_label'])][status]+=1
        if not row['eligible']:
            continue
        review_basis[row['review_basis']]+=1;splits[row['split']]+=1
        negatives[row['split']]+=not any(a['canonical'] for a in row['annotations'])
        exact_roles[row['pixel_sha256']].add(row['split'])
        structure_failures+=bool(row['issues'] or any(a['issues'] for a in row['annotations']))
        for field in conditions:conditions[field][str(row.get(field,'unknown'))]+=1
        for ann in row['annotations']:
            unresolved+=ann['disposition']=='ambiguous'
            if ann['canonical']:
                classes[ann['canonical']]+=1
                classes_by_split[row['split']][ann['canonical']]+=1
                x,y,x2,y2=ann['xyxy'];area=(x2-x)*(y2-y)
                sizes['small' if area<32**2 else ('medium' if area<96**2 else 'large')]+=1
    exact_leaks=sum(len(roles)>1 for roles in exact_roles.values())
    acquired=sum(r['status'] in ('downloaded','downloaded_snapshot') for r in source_rows.values())
    tools=Path(__file__).resolve().parent
    code={p.name:sha(p) for p in [tools/'download_roboflow_sources.py',tools/'snapshot_roboflow_projects.py',tools/'camera_data_audit.py',Path(__file__)]}
    try:
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=tools,text=True).strip()
    except subprocess.CalledProcessError:
        commit='unknown'
    result=dict(schema_version=1,captured_at=datetime.now(timezone.utc).isoformat(),
        branch='feat/camera-training-data-audit',parent_code_commit=commit,tool_sha256=code,
        scope='data acquisition, audit, reviewed bbox subset; no training or runtime changes',
        python=platform.python_version(),sdk='roboflow==1.7.0',sources=sources,
        source_acquisition=dict(expected=22,acquired=acquired,
            public_versions=sum(r['status']=='downloaded' for r in source_rows.values()),
            unversioned_snapshots=sum(r['status']=='downloaded_snapshot' for r in source_rows.values()),
            complete=acquired==22,all_latest_public_versions_available=downloads['complete']),
        audit_summary=summary,export=exported,
        accepted=dict(images=sum(splits.values()),splits=dict(splits),objects=dict(classes),
            classes_by_split={k:dict(v) for k,v in classes_by_split.items()},negatives=dict(negatives),
            classes_in_all_splits=[c for c in classes if all(classes_by_split[s][c]>0 for s in ('train','selection','heldout'))],
            review_basis=dict(review_basis),unresolved_labels=unresolved,structure_failures=structure_failures,
            exact_pixel_leaks=exact_leaks,conditions={k:dict(v) for k,v in conditions.items()},
            bbox_area_pixels=dict(sizes),bbox_size_definition='small <32^2; medium <96^2; large >=96^2, at stored resolution'),
        class_mapping=[dict(source=s,original_label=n,dispositions=dict(v)) for (s,n),v in sorted(mappings.items())],
        remaining=['Quarantined ambiguous/missing/composite labels require individual review.',
            'pHash candidates are unconfirmed, preserved, and cross-split candidates excluded.',
            'Absent classes are not trained; missing_target_classes lists coverage gaps.',
            'Weather, distance and tunnel status remain unknown without independent evidence.'],
        training_performed=False,operating_model_changed=False)
    result['all_sources_inventoried']=all(s['exported_images']>0 for s in sources) and len(sources)==22
    result['reviewed_subset_ready']=bool(acquired==22 and result['all_sources_inventoried'] and exported and exported['split_ready']
        and not unresolved and not structure_failures and not exact_leaks)
    pilot_targets=['person','dog','bicycle','car','motorcycle','bus','truck','cargo_box','wooden_box']
    result['bbox_pilot_missing_evaluation_classes']=[c for c in pilot_targets if c not in result['accepted']['classes_in_all_splits']]
    result['bbox_pilot_ready']=bool(result['reviewed_subset_ready'] and not result['bbox_pilot_missing_evaluation_classes'])
    samples_path=gallery/'review/review-samples.json'
    samples=json.loads(samples_path.read_text()) if samples_path.exists() else {}
    reviewed={r['id'] for r in rows if r.get('review_record')}
    sample_ids={i for ids in samples.values() for i in ids}
    result['visual_review_coverage']=dict(reviewed_images=len(reviewed),
        planned_sample_buckets=len(samples),planned_unique_sample_images=len(sample_ids),
        reviewed_sample_images=len(sample_ids & reviewed),
        fully_reviewed_sample_buckets=sum(set(ids)<=reviewed for ids in samples.values()),
        full_visual_review_complete=bool(samples and sample_ids<=reviewed),
        note='Generated samples are not completed reviews. Quarantined unresolved images are not training data.')
    archive_proof=root/'archive-verification-v1.json'
    if archive_proof.exists():
        result['archive_verification']=dict(sha256=sha(archive_proof),
            **{k:v for k,v in json.loads(archive_proof.read_text()).items() if k!='archives'})
    gallery.mkdir(parents=True,exist_ok=True)
    save(gallery/'summary.json',result)
    esc=lambda s:html.escape(str(s))
    table=''.join('<tr><td><a href="'+esc(s['url'])+'">'+esc(s['project'])+'</a></td><td>'+esc(s['version'])+'</td><td>'+esc(s['status'])+'</td><td>'+str(s['exported_images'])+'</td><td>'+str(s['accepted'])+'</td><td>'+str(s['quarantined'])+'</td><td>'+esc(s['license'])+'</td></tr>' for s in sources)
    mapping=''.join('<tr><td>'+esc(m['source'])+'</td><td>'+esc(m['original_label'])+'</td><td>'+esc(m['dispositions'])+'</td></tr>' for m in result['class_mapping'])
    page='''<!doctype html><meta charset="utf-8"><title>학습 데이터 감사 · 2026-10-09</title>
<style>body{font:16px system-ui;max-width:1300px;margin:30px auto;padding:15px}table{border-collapse:collapse;width:100%}th,td{border:1px solid #ccc;padding:8px;text-align:left}pre{white-space:pre-wrap}a{color:#1565c0}</style>
<h1>학습 데이터 감사 · 2026-10-09</h1>
<p>원본 확보와 학습용 채택은 별도입니다. 버전 없는 스냅샷은 작성자의 공개 버전으로 표시하지 않습니다. 미검수·복합 주석은 격리하며 학습·운영 교체는 수행하지 않았습니다.</p>
<p><a href="review/index.html">검수 갤러리</a> · <a href="review/index.html?q=box">상자</a> · <a href="review/index.html?q=bicycle%20man">복합 주석</a> · <a href="review/index.html?q=사슴">사슴 라벨 수정</a> · <a href="review/index.html?q=주석되지">누락 주석 반례</a> · <a href="summary.json">전체 요약·SHA</a></p>
<h2>채택 부분집합</h2><pre>__ACCEPTED__</pre><p>부분집합 구조 검사: __READY__. 미지원 클래스: __MISSING__. 이것은 전체 대회 객체의 학습 준비 완료를 뜻하지 않습니다.</p>
<p>기존 7종 + CargoBox/WoodBox 비교 학습 평가 준비: __PILOT_READY__. train/selection/heldout 중 자료가 부족한 클래스: __PILOT_MISSING__.</p>
<h2>수동 검수 범위</h2><pre>__REVIEW_COVERAGE__</pre><p>표본을 생성한 수와 실제 검수한 수는 다릅니다. 아직 검수하지 않은 자료는 학습에 넣지 않았습니다.</p>
<h2>22개 프로젝트</h2><table><tr><th>출처</th><th>버전</th><th>확보 상태</th><th>확보 이미지·증강 포함</th><th>채택</th><th>격리</th><th>라이선스</th></tr>__SOURCES__</table>
<h2>원본 이름 → 현재 판정</h2><p>ambiguous는 미확정, excluded는 대상 제외입니다. 개별 수정과 자동 이름 대응을 구분하는 근거는 manifest의 review_record와 원본 주석에 남습니다.</p><table><tr><th>출처</th><th>원본 라벨</th><th>정규 라벨·상태와 객체 수</th></tr>__MAPPINGS__</table>
<h2>해석 제한</h2><p>날씨·터널·거리는 근거 없이 파일명으로 확정하지 않았습니다. 유사 이미지 수는 누출 수가 아니며, 분할을 가로지르는 미확정 후보는 채택하지 않았습니다. Capture 고정 run 분할과 원본 instance mask는 유지합니다.</p>'''
    page=page.replace('__ACCEPTED__',esc(json.dumps(result['accepted'],ensure_ascii=False,indent=2)))
    page=page.replace('__READY__',esc(result['reviewed_subset_ready'])).replace('__MISSING__',esc(exported.get('missing_target_classes',[]) if exported else 'export 대기'))
    page=page.replace('__PILOT_READY__',esc(result['bbox_pilot_ready'])).replace('__PILOT_MISSING__',esc(result['bbox_pilot_missing_evaluation_classes']))
    page=page.replace('__REVIEW_COVERAGE__',esc(json.dumps(result['visual_review_coverage'],ensure_ascii=False,indent=2)))
    (gallery/'index.html').write_text(page.replace('__SOURCES__',table).replace('__MAPPINGS__',mapping))
    if dataset and dataset.exists():
        attribution=['# 원본 출처와 라이선스','', 'Capture 원본의 출처는 manifest를 참조합니다. Roboflow 원본과 파생 주석의 출처는 다음과 같습니다.','']
        attribution += ['- '+s['url']+' — '+s['license']+'; version '+str(s['version']) for s in sources]
        attribution += ['', '원본 이미지·주석은 보존하고 정규 이름·bbox 수정은 manifest에 별도 기록합니다.']
        (dataset/'ATTRIBUTION.md').write_text('\n'.join(attribution)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--curated',type=Path,required=True)
    p.add_argument('--dataset',type=Path);p.add_argument('--gallery',type=Path,required=True)
    p.add_argument('--baseline',type=Path)
    args=p.parse_args();result=report(args.root,args.curated,args.dataset,args.gallery)
    if args.baseline:
        import yaml
        args.baseline.parent.mkdir(parents=True,exist_ok=True)
        args.baseline.write_text(yaml.safe_dump(result,allow_unicode=True,sort_keys=False))
    print(json.dumps(dict(acquisition=result['source_acquisition'],accepted=result['accepted']['images'],
        reviewed_subset_ready=result['reviewed_subset_ready'])))


if __name__=='__main__':main()
