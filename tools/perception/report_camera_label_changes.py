#!/usr/bin/env python3
"""Render saved individual relabel decisions against the resulting curated dataset."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import html
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from camera_data_audit import read_jsonl, save, sha


def report(previous, curated, decisions, output):
    before = {r['id']: r for r in read_jsonl(previous)}
    after = {r['id']: r for r in read_jsonl(curated)}
    reviews = json.loads(decisions.read_text())
    output.mkdir(parents=True, exist_ok=False)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 13)
    except OSError:
        font = ImageFont.load_default()
    rows, changes = [], Counter()
    for ident, review in sorted(reviews.items()):
        old, new = before[ident], after[ident]
        if sha(old['image']) != review['image_sha256'] or old['image_sha256'] != new['image_sha256']:
            raise ValueError('review/source image SHA mismatch')
        if old['split'] != new['split']:
            raise ValueError('review moved an existing split')
        if [(a['id'], a['xyxy']) for a in old['annotations']] != [(a['id'], a['xyxy']) for a in new['annotations']]:
            raise ValueError('this report requires unchanged original geometry')
        panels = []
        status = 'ADMITTED' if new['eligible'] else 'PENDING / NOT EXPORTED'
        for source, title in [(old, 'ORIGINAL LABEL'), (new, status)]:
            with Image.open(source['image']) as image:
                rgb = image.convert('RGB')
            rgb.thumbnail((640, 640))
            scale = rgb.width/source['width']
            panel = Image.new('RGB', (640, rgb.height+42), 'white')
            panel.paste(rgb, (0, 42))
            draw = ImageDraw.Draw(panel)
            draw.text((4, 3), title+' | '+source['split'], fill='black', font=font)
            draw.text((4, 23), ident[:16]+' | '+source['source'][9:], fill='black', font=font)
            for ann in source['annotations']:
                label = str(ann['original_label']) if source is old else ann.get('training_class') or 'excluded'
                xy = ann['xyxy']
                rect = [xy[0]*scale, xy[1]*scale+42, xy[2]*scale, xy[3]*scale+42]
                color = '#ff00ff' if source is old else '#00ff66' if new['eligible'] else '#ffae00'
                draw.rectangle(rect, outline=color, width=2)
                draw.text((rect[0], max(42, rect[1]-14)), label+':'+ann['id'], fill=color, font=font)
            panels.append(panel)
        sheet = Image.new('RGB', (1280, max(p.height for p in panels)), 'white')
        for i, panel in enumerate(panels):
            sheet.paste(panel, (640*i, 0))
        name = ident+'.jpg'
        sheet.save(output/name, quality=93)
        overrides = review.get('overrides', [])
        if new['eligible']:
            for override in overrides:
                changes[override.get('canonical') or 'excluded'] += 1
        rows.append(dict(id=ident, admitted=new['eligible'], decision=review['decision'],
            split=new['split'], source=new['source'], image=name, image_sha256=old['image_sha256'],
            rendered_sha256=sha(output/name), overrides=overrides, notes=review.get('notes'),
            reasons=new['reasons'], original_annotation_ids=[a['id'] for a in old['annotations']]))
    save(output/'changes.json', dict(schema_version=1, previous_sha256=sha(previous),
        curated_sha256=sha(curated), decisions_sha256=sha(decisions), rows=rows,
        admitted_override_counts=dict(changes), automatic_approval=False))
    sections=[]
    for index, row in enumerate(rows):
        status='채택' if row['admitted'] else '보완 대기'
        details=json.dumps(row['overrides'], ensure_ascii=False)
        sections.append(f'<details><summary>{index:03d} · {status} · {html.escape(row["source"])} · {row["split"]} · {row["id"][:12]}</summary>'
            f'<a href="{row["image"]}"><img loading="lazy" width="100%" src="{row["image"]}"></a>'
            f'<p>{html.escape(row["notes"] or "")}</p><pre>{html.escape(details)}</pre></details>')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px system-ui;margin:24px;max-width:1400px}details{padding:12px;border:1px solid #ccc}pre{white-space:pre-wrap}</style>'
        '<h1>혼합 라벨 수정 전후 · '+datetime.now(timezone.utc).date().isoformat()+'</h1>'
        '<p>왼쪽: 원본 이름 / 오른쪽: 실제 export 판정. 주황색은 아직 미채택이며 제안이 정답으로 반영된 상태가 아닙니다. 원본 좌표·split 유지, teacher 자동 확정 없음.</p>'
        '<a href="changes.json">개별 ID·SHA·수정 기록</a>'+''.join(sections))
    return dict(reviewed=len(rows), admitted=sum(r['admitted'] for r in rows), override_counts=dict(changes))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('previous', 'curated', 'decisions', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a=p.parse_args()
    print(json.dumps(report(a.previous, a.curated, a.decisions, a.output)))
