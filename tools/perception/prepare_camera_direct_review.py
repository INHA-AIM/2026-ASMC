#!/usr/bin/env python3
"""Render source/class samples or all matching images; never generate approvals."""
import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from camera_data_audit import read_jsonl, save, sha


def annotation_crops(rows, labels, output):
    """Preserve annotation IDs and native pixel sizes in nearest-neighbour crops."""
    entries = []
    for index, row in enumerate(rows):
        for ann in row['annotations']:
            if str(ann['original_label']) not in labels:
                continue
            x1, y1, x2, y2 = ann['xyxy']
            if not (0 <= x1 < x2 <= row['width'] and 0 <= y1 < y2 <= row['height']):
                continue  # Invalid geometry remains visible in the full-frame manifest.
            entries.append(dict(index=index, id=row['id'], image_sha256=row['image_sha256'],
                                annotation=ann, sheet=f'annotation-crops-{len(entries)//32:03d}.jpg'))
    for start in range(0, len(entries), 32):
        sheet = Image.new('RGB', (1920, 960), 'white')
        d = ImageDraw.Draw(sheet)
        for k, entry in enumerate(entries[start:start + 32]):
            row = rows[entry['index']]
            if sha(row['image']) != entry['image_sha256']:
                raise ValueError('Image SHA changed: ' + row['id'])
            ann = entry['annotation']
            x1, y1, x2, y2 = ann['xyxy']
            bounds = [max(0, math.floor(x1)-8), max(0, math.floor(y1)-8),
                      min(row['width'], math.ceil(x2)+8), min(row['height'], math.ceil(y2)+8)]
            with Image.open(row['image']) as source:
                crop = source.convert('RGB').crop(bounds)
            scale = min(232/crop.width, 190/crop.height)
            crop = crop.resize((max(1, int(crop.width*scale)), max(1, int(crop.height*scale))),
                               Image.Resampling.NEAREST)
            ImageDraw.Draw(crop).rectangle([(x1-bounds[0])*scale, (y1-bounds[1])*scale,
                                           (x2-bounds[0])*scale, (y2-bounds[1])*scale], outline='#ff00ff')
            x, y = (k % 8)*240, (k//8)*240
            sheet.paste(crop, (x, y+38))
            d.text((x+2, y+2), f"{entry['index']}:{ann['id']}", fill='black')
            d.text((x+2, y+20), f"native {x2-x1:.0f}x{y2-y1:.0f}px", fill='black')
        sheet.save(output/f'annotation-crops-{start//32:03d}.jpg', quality=95)
    return entries


def prepare(curated, buckets, output, all_images=False, crop_labels=()):
    progress = json.loads((curated / 'review-progress.json').read_text())
    states = {b['bucket']: b for b in progress['buckets']}
    unknown = set(buckets) - states.keys()
    if unknown:
        raise ValueError('Unknown buckets: ' + str(sorted(unknown)))
    if all_images:
        rows = {r['id']: r for r in read_jsonl(curated / 'curated.jsonl')
                if any(r['source'] + '|' + str(a['original_label']) in buckets
                       for a in r['annotations'])}
        ids = sorted(rows)
    else:
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
                    label = str(ann['original_label'])
                    if all_images and ann['disposition'] == 'ambiguous':
                        label += ':' + ann['id']
                    d.text((box[0], max(64, box[1] - 15)), label, fill=color, font=font)
            panels.append(panel)
            entries.append(dict(index=index, id=ident, image_sha256=row['image_sha256'],
                source=row['source'], image=row['image'],
                buckets=[b for b in buckets if (
                    any(row['source']+'|'+str(a['original_label']) == b for a in row['annotations'])
                    if all_images else ident in states[b]['sample']['sample_ids'])],
                sheet=f'sheet-{start // 8:03d}.jpg', annotations=row['annotations']))
        height = max(p.height for p in panels)
        sheet = Image.new('RGB', (2560, height * 2), '#ddd')
        for offset, panel in enumerate(panels):
            sheet.paste(panel, ((offset % 4) * 640, (offset // 4) * height))
        name = f'sheet-{start // 8:03d}.jpg'
        sheet.save(output / name, quality=95)
        links.append(f'<p>{start}–{start+len(page)-1} <a href="{name}">{name}</a></p>')
    crops = annotation_crops([rows[i] for i in ids], set(crop_labels), output) if crop_labels else []
    if crop_labels:
        save(output/'annotation-crops.json', crops)
        links.extend(f'<p><a href="{p.name}">{p.name}</a> — context crop, not an approval</p>'
                     for p in sorted(output.glob('annotation-crops-*.jpg')))
    manifest = dict(schema_version=1, selection='all_matching_images' if all_images else 'fixed_samples',
        input_sha256=sha(curated/'curated.jsonl'),
        buckets=buckets, samples=entries, sheet_sha256={p.name:sha(p) for p in output.glob('sheet-*.jpg')},
        crop_labels=list(crop_labels), crop_annotations=len(crops),
        crop_sheet_sha256={p.name:sha(p) for p in output.glob('annotation-crops-*.jpg')},
        decisions_generated=False, note='Every panel needs actual RGB inspection. Generated sheets are not approvals.')
    save(output/'samples.json', manifest)
    title = '혼합 라벨 전체 영상 RGB 검수' if all_images else '직접 대응 자료 고정 표본 RGB 검수'
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>'+title+'</h1><p>No automatic approval. Green: target; magenta: mixed; gray: excluded source class.</p><a href="samples.json">SHA and panel manifest</a>'+''.join(links))
    return dict(samples=len(entries), sheets=len(links), buckets=len(buckets))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--curated', type=Path, required=True)
    p.add_argument('--bucket', action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--all-images', action='store_true', help='Render every matching image for individual relabel review, not bucket approval.')
    p.add_argument('--crop-label', action='append', default=[], help='Also render ID-linked native-size annotation crops for this original label.')
    a = p.parse_args()
    print(json.dumps(prepare(a.curated, a.bucket, a.output, a.all_images, a.crop_label)))
