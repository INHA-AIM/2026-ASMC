#!/usr/bin/env python3
"""Build one local catalogue of existing experiment pages, without changing them."""
import argparse
from datetime import datetime, timezone, timedelta
import html
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
from urllib.parse import quote, unquote, urljoin, urlsplit


def catalogue(root, output):
    pages = []
    for directory, _, filenames in os.walk(root, followlinks=False):
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
            with path.open(encoding='utf-8', errors='replace') as stream:
                start = stream.read(32768)
            title = re.search(r'<(?:h1|title)\b[^>]*>(.*?)</(?:h1|title)>', start, re.S | re.I)
            title = html.unescape(re.sub('<[^>]+>', '', title.group(1))) if title else relative.stem
            pages.append(dict(group=relative.parts[0], path=relative.as_posix(),
                              href=quote(relative.as_posix(), safe='/'), title=title.strip()[:180]))
    return sorted(pages, key=lambda row: (row['group'], row['path']))


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    if not root.is_dir():
        parser.error('artifacts directory does not exist')
    output = root/'index.html'
    pages = number_pages(root, catalogue(root, output))
    data = json.dumps(pages, ensure_ascii=False).replace('<', '\\u003c')
    featured = [('YOLOE 6조합·최종 반복·fusion', 'camera-yoloe-selection-v1/final-analysis-v2/index.html'),
                ('YOLOE 선정 프롬프트 동일 RGB', 'camera-yoloe-selection-v1/selected-spawn-v1/comparison.html'),
                ('confidence 0.30/0.40/0.50 오검출 비교', 'camera-coco7-live-v1/confidence-followup-v3/index.html'),
                ('8개 조합 최종 선정', 'camera-coco7-live-v1/broad-model-summary-v1/index.html'),
                ('8개 모델 동일 RGB', 'camera-coco7-live-v1/broad-model-spawn-inspection50-v1/comparison.html'),
                ('선정 모델 차체 mask 대조', 'camera-coco7-live-v1/broad-model-selected-mask-controls50-v1/index.html'),
                ('기존 COCO7 실험', 'camera-coco7-live-v1/index.html')]
    featured_html = ' · '.join('<a href="'+quote(path, safe='/')+'" target="_blank" rel="noopener">'+
        html.escape(next((p['title'] for p in pages if p['path']==path),label))+'</a>' for label, path in featured if (root/path).is_file())
    updated = datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')
    document = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ASMC 실험 갤러리 목차</title>
<style>
body{font:16px system-ui,sans-serif;background:#f5f7fa;color:#172536;margin:0}
main{max-width:1100px;margin:auto;padding:28px}h1{font-size:30px;margin-bottom:10px}
p{line-height:1.65}input,select{font:inherit;padding:10px;border:1px solid #abb8c6;border-radius:7px}
input{flex:1;min-width:210px}.filters{display:flex;gap:12px;flex-wrap:wrap;position:sticky;top:0;background:#f5f7fa;padding:12px 0}
article{background:white;border:1px solid #dbe2e9;border-radius:9px;padding:17px;margin:12px 0}
article a{font-weight:650;color:#075ca8;text-decoration:none}article a:hover{text-decoration:underline}
code{display:block;font-size:13px;margin-top:10px;overflow-wrap:anywhere;color:#526274}
h2{font-size:22px;margin-top:30px}.tag{font-size:13px;color:#506277;margin-left:12px}
.latest{background:#e7f1ff;padding:15px;border-radius:9px}.latest a{color:#075ca8}
small{color:#506277}#empty{padding:20px}</style>
<main><h1>ASMC 실험 갤러리 목차</h1>
<p>카메라 모델·거리·mask·학습 자료와 LiDAR 비교 갤러리를 한곳에서 찾습니다.
E 번호는 고정된 갤러리 식별자이며 최신 시각순으로 정렬합니다. 표기 시각은 최초 등록 당시 HTML 파일의 생성·수정 시각(KST)으로, 실험 시작 시각은 아닙니다. 모든 링크는 새 탭으로 열립니다. 과거 실패와 이전 후보도 포함하며, 갤러리의 존재가 운영 수락을 뜻하지 않습니다.</p>
<div class="latest"><b>최근 모델 비교</b> · FEATURED
</div><div class="filters"><input id="query" type="search" placeholder="모델·거리·mask·실험 경로 검색" aria-label="갤러리 검색">
<select id="group" aria-label="실험 종류"><option value="">모든 실험</option></select></div>
<p id="count" aria-live="polite"></p><div id="pages"></div><p id="empty" hidden>검색 결과가 없습니다.</p>
<small>생성 시각 UPDATED · 원본 HTML은 변경하지 않았습니다. 새 실험 후 목차를 다시 생성합니다.</small></main>
<script>const rows=DATA;const byId=id=>document.getElementById(id);
const groups=[...new Set(rows.map(r=>r.group))];
for(const group of groups){const option=document.createElement('option');option.value=group;option.textContent=group;byId('group').append(option);}
function draw(){const query=byId('query').value.trim().toLowerCase(),group=byId('group').value;
 const visible=rows.filter(r=>(!group||r.group===group)&&(!query||(r.title+' '+r.path).toLowerCase().includes(query)));
 const container=byId('pages');container.replaceChildren();
 for(const row of visible){
  const article=document.createElement('article'),link=document.createElement('a'),path=document.createElement('code');
  link.href=row.href;link.target='_blank';link.rel='noopener';link.textContent=row.title||row.path;
  path.textContent=row.group+' · '+row.path;article.append(link,path);container.append(article);}
 byId('count').textContent=visible.length+' / '+rows.length+'개 페이지';byId('empty').hidden=visible.length>0;}
byId('query').addEventListener('input',draw);byId('group').addEventListener('change',draw);draw();</script></html>'''
    output.write_text(document.replace('UPDATED', updated).replace('FEATURED', featured_html).replace('DATA', data), encoding='utf-8')
    (root/'gallery-catalogue.json').write_text(json.dumps(dict(updated=updated, pages=pages),
        ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    links = check_local_links(root, pages)
    (root/'gallery-link-report.json').write_text(json.dumps(links, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if links['missing']:
        raise SystemExit('Missing local gallery links; see gallery-link-report.json')
    print(json.dumps(dict(output=str(output), pages=len(pages), groups=len({p['group'] for p in pages}))))


if __name__ == '__main__':
    main()
