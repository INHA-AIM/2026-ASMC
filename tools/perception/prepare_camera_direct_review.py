#!/usr/bin/env python3
"""Render fixed source/class samples without generating approval decisions."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from camera_data_audit import read_jsonl, save, sha


def prepare(curated, buckets, output):
    progress = json.loads((curated / 'review-progress.json').read_text())
    states = {b['bucket']: b for b in progress['buckets']}
    unknown = set(buckets) - states.keys()
    if unknown:
        raise ValueError('Unknown buckets: ' + str(sorted(unknown)))
    ids = list(dict.fromkeys(i for b in buckets for i in states[b]['sample']['sample_ids']))
    wanted = set(ids)
    rows = {r['id']: r for r in read_jsonl(curated / 'curated.jsonl') if r['id'] in wanted}
    output.mkdir(parents=True, exist_ok=False)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    entries = []
    links = []
    for start in range(0, len(ids), 8):
        page = ids[start:start + 8]
        panels = []
        for index, ident in enumerate(page, start):
            row = rows[ident]
            if sha(row['image']) != row['image_sha256']:
                raise ValueError('Image SHA changed: ' + ident)
            im = Image.open(row['image']).convert('RGB')
            im.thumbnail((640, 640))
            scale = im.width / row['width']
            panel = Image.new('RGB', (640, im.height + 64), '#fff')
            panel.paste(im, (0, 64))
            d = ImageDraw.Draw(panel)
            d.text((4, 2), f"{index:04d} {ident[:10]} {row['source'][9:]}", fill='black', font=font)
            labels = ', '.join(sorted({str(a['original_label']) for a in row['annotations']}))
            d.text((4, 22), labels[:82] or '[empty annotation]', fill='black', font=font)
            d.text((4, 42), f"{row['width']}x{row['height']} | boxes={len(row['annotations'])}", fill='black', font=font)
            for ann in row['annotations']:
                xy = ann['xyxy']
                box = [xy[0] * scale, xy[1] * scale + 64, xy[2] * scale, xy[3] * scale + 64]
                color = '#ff00ff' if ann['disposition'] == 'ambiguous' else '#aaa' if ann['disposition'] == 'excluded' else '#00ff00'
                if box[2] >= box[0] and box[3] >= box[1]:
                    d.rectangle(box, outline=color, width=2)
                    d.text((box[0], max(64, box[1] - 15)), str(ann['original_label']), fill=color, font=font)
            panels.append(panel)
            entries.append(dict(index=index, id=ident, image_sha256=row['image_sha256'],
                source=row['source'], image=row['image'],
                buckets=[b for b in buckets if ident in states[b]['sample']['sample_ids']],
                sheet=f'sheet-{start // 8:03d}.jpg', annotations=row['annotations']))
        height = max(p.height for p in panels)
        sheet = Image.new('RGB', (2560, height * 2), '#ddd')
        for offset, panel in enumerate(panels):
            sheet.paste(panel, ((offset % 4) * 640, (offset // 4) * height))
        name = f'sheet-{start // 8:03d}.jpg'
        sheet.save(output / name, quality=95)
        links.append(f'<p>{start}–{start+len(page)-1} <a href="{name}">{name}</a></p>')
    manifest = dict(schema_version=1, input_sha256=sha(curated/'curated.jsonl'),
        buckets=buckets, samples=entries, sheet_sha256={p.name:sha(p) for p in output.glob('sheet-*.jpg')},
        decisions_generated=False, note='Every panel needs actual RGB inspection. Generated sheets are not approvals.')
    save(output/'samples.json', manifest)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>직접 대응 자료 고정 표본 RGB 검수</h1><p>No automatic approval. Green: target; magenta: mixed; gray: excluded source class.</p><a href="samples.json">SHA and panel manifest</a>'+''.join(links))
    return dict(samples=len(entries), sheets=len(links), buckets=len(buckets))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--curated', type=Path, required=True)
    p.add_argument('--bucket', action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(prepare(a.curated, a.bucket, a.output)))
