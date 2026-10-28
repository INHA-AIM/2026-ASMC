#!/usr/bin/env python3
"""Build one local catalogue of existing experiment pages, without changing them."""
import argparse
from datetime import datetime, timezone, timedelta
import html
from itertools import chain
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
from urllib.parse import quote, unquote, urljoin, urlsplit


def catalogue(root, output, scan_roots=None):
    pages = []
    for directory, directories, filenames in chain.from_iterable(
            os.walk(p, followlinks=False) for p in (scan_roots or [root])):
        # Reproduction sources are not rendered experiments (keep them on disk).
        directories[:] = [d for d in directories if d not in ("source-snapshot", "source-untracked")]
        for name in filenames:
            if not name.lower().endswith('.html'):
                continue
            path = Path(directory)/name
            if path == output or path.is_symlink():
                continue
            try:
                path.resolve().relative_to(root)
            except ValueError:
                continue
            relative = path.relative_to(root)
            if 'code' in relative.parts and 'templates' in relative.parts:
                continue  # Frozen source templates are not rendered experiment pages.
            with path.open(encoding='utf-8', errors='replace') as stream:
                start = stream.read(32768)
            title = re.search(r'<(?:h1|title)\b[^>]*>(.*?)</(?:h1|title)>', start, re.S | re.I)
            title = html.unescape(re.sub('<[^>]+>', '', title.group(1))) if title else relative.stem
            pages.append(dict(group=relative.parts[0], path=relative.as_posix(),
                              href=quote(relative.as_posix(), safe='/'), title=title.strip()[:180]))
    return sorted(pages, key=lambda row: (row['group'], row['path']))


def collect_pages(root, output, groups=()):
    """Explicit partial refresh preserves unselected registered experiment pages."""
    if not groups:
        return catalogue(root, output)
    for group in groups:
        if Path(group).name != group or group in (".", "..") or not (root/group).is_dir():
            raise ValueError("group must be an existing direct child: " + group)
        (root/group).resolve().relative_to(root.resolve())
    previous_path = root/"gallery-catalogue.json"
    if not previous_path.is_file():
        raise ValueError("partial refresh requires an existing catalogue")
    previous = json.loads(previous_path.read_text())["pages"]
    kept = []
    for p in previous:
        if p["group"] not in groups:
            kept.append(dict(p, title=p.get("original_title", p["title"])))
    return kept + catalogue(root, output, [root/g for g in groups])


def number_pages(root, pages):
    """Keep identifiers and first registered file times stable across rebuilds.

    Historic experiments have no uniformly recorded start time. File times are
    explicitly labelled as gallery times, never represented as sensor/run times.
    Removed entries remain reserved so an old bookmarked ID is never reused.
    """
    registry_path = root/'gallery-identifiers.json'
    registry = json.loads(registry_path.read_text()) if registry_path.exists() else {}
    used = [v['number'] for v in registry.values()]
    if len(used) != len(set(used)):
        raise ValueError('duplicate persistent gallery identifier')
    number = max(used, default=0)
    new = sorted((p for p in pages if p['path'] not in registry),
                 key=lambda p: ((root/p['path']).stat().st_mtime, p['path']))
    for row in new:
        number += 1
        timestamp = datetime.fromtimestamp((root/row['path']).stat().st_mtime, timezone.utc)
        registry[row['path']] = dict(number=number, timestamp=timestamp.isoformat(),
            time_basis='gallery_file_mtime_at_first_registration')
    for row in pages:
        metadata = registry[row['path']]
        timestamp = datetime.fromisoformat(metadata['timestamp']).astimezone(timezone(timedelta(hours=9), 'KST'))
        row.update(metadata)
        row['experiment_id'] = 'E%03d' % metadata['number']
        row['original_title'] = row['title']
        row['title'] = '[%s · %s KST] %s' % (row['experiment_id'], timestamp.strftime('%Y-%m-%d %H:%M'), row['title'])
    registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2)+'\n')
    return sorted(pages, key=lambda p: (p['timestamp'], p['number']), reverse=True)


def check_local_links(root, pages):
    missing, count = [], 0

    class Reader(HTMLParser):
        def handle_starttag(self, tag, attrs):
            nonlocal count
            if tag == 'base':
                self.base = urljoin(self.base, dict(attrs).get('href', ''))
                return
            for key, value in attrs:
                if key not in ('href', 'src', 'poster') or not value:
                    continue
                parsed = urlsplit(urljoin(self.base, value))
                if parsed.netloc != 'artifact.local':
                    continue
                count += 1
                target = root/unquote(parsed.path).lstrip('/')
                if not target.exists():
                    missing.append(dict(page=self.page, link=value))

    for row in pages:
        reader = Reader()
        reader.page = row['path']
        reader.base = 'http://artifact.local/'+row['href']
        reader.feed((root/row['path']).read_text(encoding='utf-8', errors='replace'))
    return dict(pages=len(pages), local_static_links=count, missing=missing,
                scope='Static href/src/poster; dynamic gallery data and HTTP availability checked separately')


CATEGORIES = [
    ('training', '학습·성능 평가'), ('data', '데이터·라벨 검수'),
    ('models', '모델 비교·선정'), ('collection', 'Capture·자동 수집'),
    ('fusion', '센서·퓨전'), ('planning', '로컬 경로·주행·실패 검수'), ('other', '기타 실험')]


def category(row):
    path=row['path'].lower()
    if path.startswith(('local1-', 'local2a-', 'local2b-', 'local2c-', 'local2d-', 'local2e-', 'local2f-', 'shadow-')):return 'planning'
    if path.startswith('lidar') or 'fusion' in path:return 'fusion'
    if any(v in path for v in ('training-data','data-expansion','teacher','dataset','supervision','background-full','validation16','capture-size','tiny-gt','training-audit','source-coverage','capture-restoration','v11-preflight')):return 'data'
    if any(v in path for v in ('bbox-main','bbox-pilot','bbox-regression','vehicle-diagnostic','epoch8-decision')):return 'training'
    if path.startswith(('capture-', 'resume', 'camera_objects_bbox_capture')):return 'collection'
    if any(v in path for v in ('coco','yoloe','rfdetr','camera-seg')):return 'models'
    return 'other'


def friendly_group(group):
    if group.startswith('local2f-'):return 'LOCAL-2F · 제동·복구 경로 검수 · '+group[len('local2f-'):]
    if group.startswith('local2e-'):return 'LOCAL-2E · 전환·장애물 회피 개선 · '+group[len('local2e-'):]
    if group.startswith('local2d-'):return 'LOCAL-2D · 공급·전환·주행 검수 · '+group[len('local2d-'):]
    if group.startswith('local2c-'):return 'LOCAL-2C · 속도 연속성·지면 OBB · '+group[len('local2c-'):]
    if group.startswith('local2b-'):return 'LOCAL-2B · 경로 길이·안정화 · '+group[len('local2b-'):]
    if group.startswith('local2a-'):return 'LOCAL-2A · 입력 정렬·벽 간격 · '+group[len('local2a-'):]
    if group.startswith('local1-'):return 'LOCAL-1 · 초기 주행·실패 · '+group[len('local1-'):]
    known={'camera-epoch8-decision-20261013-r1':'8 epoch 평가·이어 학습 판단',
           'camera-dataset-v10-review-20261013-r1':'v10 데이터 재분할·16종 검수',
           'camera-supervision-policy-review-20261013-r2':'극소 GT·ignore·배경 정책 검토',
           'camera-coco7-live-v1':'COCO 7종 · 3카메라 모델 선정',
           'camera-yoloe-selection-v1':'YOLOE · 해상도·프롬프트 선정',
           'camera-rfdetr-selection-v1':'RF-DETR · 품질·통합 부하 비교',
           'camera-teacher-v6':'Teacher 비교·v6 자료 보강',
           'camera-data-expansion-v7':'v7 자료 채택 확대',
           'camera-bbox-pilot-v1':'YOLO·YOLOE bbox 파일럿 학습',
           'camera-bbox-regression-v1':'사람·이륜차 회귀 진단',
           'lidar-pr72':'LiDAR · PR #72 검증'}
    if group in known:return known[group]
    if group.startswith('camera-bbox-main-'):return 'YOLO26l 본학습 · '+group[len('camera-bbox-main-'):]
    if group.startswith('camera-training-data-audit-'):return '학습 자료 검수 · '+group[len('camera-training-data-audit-'):]
    if group.startswith('camera-small-supervision-analysis-'):return '작은 GT·오검출 분석 · '+group.split('-')[-1]
    return group


def enrich(pages):
    for row in pages:
        row['category']=category(row)
        row['group_title']=friendly_group(row['group'])
        title=row.get('original_title', row['title'])
        row['display_title']=re.sub(r'^\d{4}-\d{2}-\d{2}(?:T[^ ]+)?\s*[·—-]\s*', '', title)
    return pages


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts',type=Path,required=True)
    parser.add_argument('--group',action='append',default=[],help='Refresh only this direct child; preserve other catalogue entries')
    args=parser.parse_args();root=args.artifacts.resolve()
    if not root.is_dir():parser.error('artifacts directory does not exist')
    output=root/'index.html';pages=enrich(number_pages(root,collect_pages(root,output,args.group)))
    # Shortcut choices are explicit roles. File existence is never model acceptance.
    live=next((r for r in pages if 'camera-bbox-main-' in r['path'] and r['path'].endswith('/live.html')),None)
    local2c=next((r for r in pages if r['path'].startswith('local2c-') and r['path'].endswith('/review-v2/index.html')),None)
    local=next((r for r in pages if r['path'].startswith('local2b-') and r['path'].endswith('/campaign/index.html')),None)
    prepared_review='camera-v11-preflight-20261013-r1/index.html'
    local2d=next((r for r in pages if r['path'].startswith('local2d-') and r['path'].endswith('/index.html') and r['path'].count('/') == 1),None)
    local2e=next((r for r in pages if r['path'].startswith('local2e-') and r['path'].endswith('/index.html') and r['path'].count('/') == 1),None)
    local2f=next((r for r in pages if r['path'].startswith('local2f-') and r['path'].endswith('/index.html') and r['path'].count('/') == 1),None)
    wishes=[('LOCAL-2F 제동·복구 경로 검수',local2f['path'] if local2f else None),
            ('LOCAL-2E 전환·장애물 회피 검수',local2e['path'] if local2e else None),
            ('LOCAL-2D 공급·전환·주행 검수',local2d['path'] if local2d else None),
            ('LOCAL-2C 속도·노면 OBB 검수',local2c['path'] if local2c else None),
            ('LOCAL-2B 주행·실패 검수',local['path'] if local else None),
            ('최근 본학습',live['path'] if live else None),
            ('현재 데이터 검수',prepared_review if (root/prepared_review).is_file() else 'camera-dataset-v10-review-20261013-r1/index.html'),
            ('이전 학습 평가','camera-epoch8-decision-20261013-r1/index.html'),
            ('운영 모델 선정 근거','camera-coco7-live-v1/broad-model-summary-v1/index.html')]
    by_path={r['path']:r for r in pages}
    featured=[dict(label=label,**by_path[path]) for label,path in wishes if path in by_path]
    updated=datetime.now(timezone.utc).astimezone(timezone(timedelta(hours=9))).isoformat(timespec='seconds')
    payload=dict(updated=updated,pages=pages,categories=CATEGORIES,featured=featured,
                 refresh_scope=args.group or "all",
                 live_status_path=(str(Path(live['path']).parent/'supervisor.json') if live else None))
    template=Path(__file__).with_name('templates')/'experiment_catalogue.html'
    output.write_text(template.read_text().replace('__CATALOGUE_DATA__',json.dumps(payload,ensure_ascii=False).replace('<','\\u003c')),encoding='utf-8')
    (root/'gallery-catalogue.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    links=check_local_links(root,pages)
    (root/'gallery-link-report.json').write_text(json.dumps(links,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if links['missing']:raise SystemExit('Missing local gallery links; see gallery-link-report.json')
    print(json.dumps(dict(output=str(output),pages=len(pages),groups=len({p['group'] for p in pages}))))


if __name__=='__main__':
    main()
