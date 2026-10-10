#!/usr/bin/env python3
"""Review Capture actor intervals and extend only the frozen training partition.

RGB evidence, never teacher output, resolves broad Obstacle/Vehicle categories.
Original boxes/masks and previous evaluation membership/labels remain immutable.
"""
import argparse
import copy
from collections import Counter, defaultdict
import json
from pathlib import Path

from camera_data_audit import (export_dataset, load_label_policy, read_jsonl,
    save, sha, valid_box, write_jsonl)
from prepare_capture_teacher_gt import prepare


def intervals(rows, gap=40):
    groups = defaultdict(list)
    for row in rows:
        if not row['source'].startswith('capture:') or row['split'] != 'train':
            continue
        for ann in row['annotations']:
            if ann['disposition'] == 'ambiguous':
                groups[(row['source'], ann['original_label'], str(ann['actor_id']))].append((row, ann))
    result = []
    for key, values in sorted(groups.items()):
        parts = []
        for item in sorted(values, key=lambda x: (frame(x[0]), x[0]['source_key'])):
            if not parts or frame(item[0]) - frame(parts[-1][-1][0]) > gap:
                parts.append([])
            parts[-1].append(item)
        for part in parts:
            # Three independent visible examples distributed over the interval.
            # No minimum size is applied to training GT: this threshold only
            # decides whether RGB subtype review is practical.
            best = max(part, key=lambda x: area(x[1]))
            windows = [part[:max(1, len(part)//3)], part[-max(1, len(part)//3):]]
            selected = [best] + [max(w, key=lambda x: area(x[1])) for w in windows]
            result.append(dict(index=len(result), source=key[0], original_label=key[1],
                actor_id=key[2], first_frame=frame(part[0][0]), last_frame=frame(part[-1][0]),
                observations=len(part), max_area=area(best[1]), evidence=[dict(
                    row_id=r['id'], annotation_id=a['id'], image=r['image'],
                    image_sha256=r['image_sha256'], source_key=r['source_key'],
                    xyxy=a['xyxy']) for r,a in selected]))
    return result


def frame(row):
    return int(row['source_key'].rsplit('/', 1)[1])


def area(ann):
    x,y,x2,y2 = ann['xyxy']
    return (x2-x)*(y2-y)


def render(rows, output):
    from PIL import Image, ImageDraw, ImageFont, ImageOps
    output.mkdir(parents=True, exist_ok=False)
    entries = intervals(rows)
    save(output/'intervals.json', entries)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15)
    pages = []
    practical = [e for e in entries if e['max_area'] >= 250]
    for offset in range(0, len(practical), 5):
        sheet = Image.new('RGB', (1080, 5*260), 'white')
        draw = ImageDraw.Draw(sheet)
        for pos,e in enumerate(practical[offset:offset+5]):
            y = pos*260
            run = e['source'].split(':',1)[1]
            text = f"#{e['index']} {run[:15]} {e['original_label']} actor={e['actor_id']} f={e['first_frame']}-{e['last_frame']} n={e['observations']}"
            draw.text((5,y+3), text, fill='black', font=font)
            for col,item in enumerate(e['evidence']):
                with Image.open(item['image']) as im:
                    im = im.convert('RGB')
                    x1,y1,x2,y2 = item['xyxy']
                    padding = max(12, int(max(x2-x1,y2-y1)*.18))
                    box = (max(0,x1-padding), max(0,y1-padding), min(im.width,x2+padding), min(im.height,y2+padding))
                    crop = im.crop(box)
                    ImageDraw.Draw(crop).rectangle((x1-box[0],y1-box[1],x2-box[0]-1,y2-box[1]-1), outline='#00d5ff', width=2)
                    tile = ImageOps.contain(crop, (350,205))
                    sheet.paste(tile, (col*360+(350-tile.width)//2,y+30))
                draw.text((col*360+4,y+237), item['source_key'], fill='black', font=font)
        name=f"actors-{offset//5+1:02d}.jpg"
        sheet.save(output/name, quality=90)
        pages.append(name)
    save(output/'review-summary.json', dict(intervals=len(entries), practical_intervals=len(practical),
        tiny_unresolved_intervals=len(entries)-len(practical), sheets=pages,
        training_size_filter=False, teacher_truth_used=False))
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Capture actor RGB review — 2026-10-11</title>'
        '<h1>Capture actor RGB review — 2026-10-11</h1><a href="../index.html">목차</a>'
        '<p>run·원본 category·actor·연속 frame 구간별 가장 선명한 영상과 앞/뒤 구간입니다. '
        '40프레임 초과 재등장은 별도로 검토합니다. cyan은 원본 GT, teacher는 사용하지 않습니다.</p>'
        + ''.join(f'<p>{name}</p><img style="max-width:100%" src="{name}">' for name in pages))
    return entries


def validate_reviews(rows, reviews):
    evidence = {(r['id'], str(a['id'])): (r,a) for r in rows for a in r['annotations']}
    for review in reviews:
        if review.get('teacher_derived') or review.get('reviewer') != 'Codex independent Capture RGB inspection':
            raise ValueError('independent actor RGB review required')
        if sha(review['evidence_image']) != review['evidence_image_sha256']:
            raise ValueError('actor evidence image SHA changed')
        for item in review['evidence']:
            row,ann = evidence[(item['row_id'], str(item['annotation_id']))]
            if (row['source'] != review['source'] or ann['original_label'] != review['original_label']
                    or str(ann['actor_id']) != str(review['actor_id'])
                    or not review['first_frame'] <= frame(row) <= review['last_frame']
                    or item['xyxy'] != ann['xyxy'] or item['image'] != row['image']
                    or item['image_sha256'] != row['image_sha256']
                    or sha(item['image']) != item['image_sha256']):
                raise ValueError('actor evidence identity mismatch')


def bind_reviews(rows, entries, decisions):
    """Bind explicitly inspected interval decisions to immutable source evidence."""
    result=[]
    for decision in decisions:
        entry=entries[decision['index']]
        first=decision.get('first_frame',entry['first_frame'])
        last=decision.get('last_frame',entry['last_frame'])
        if not entry['first_frame'] <= first <= last <= entry['last_frame']:
            raise ValueError('review must not extend an inspected interval')
        evidence=[item for item in entry['evidence']
                  if first <= int(item['source_key'].rsplit('/',1)[1]) <= last]
        for key in decision.get('additional_evidence_keys',[]):
            row=next(r for r in rows if r['source']==entry['source'] and r['source_key']==key)
            ann=next(a for a in row['annotations'] if a['original_label']==entry['original_label']
                     and str(a['actor_id'])==entry['actor_id'])
            evidence.append(dict(row_id=row['id'],annotation_id=ann['id'],image=row['image'],
                image_sha256=row['image_sha256'],source_key=key,xyxy=ann['xyxy']))
        if not evidence:raise ValueError('empty independent evidence')
        result.append(dict(source=entry['source'],original_label=entry['original_label'],
            actor_id=entry['actor_id'],first_frame=first,last_frame=last,canonical=decision['canonical'],
            evidence=evidence,evidence_image=evidence[0]['image'],
            evidence_image_sha256=evidence[0]['image_sha256'],
            reviewer='Codex independent Capture RGB inspection',teacher_derived=False,
            human_confirmed=False,notes=decision['notes'],review_interval_index=entry['index']))
    validate_reviews(rows,result)
    return result


def expand_capture(rows, policy, reviews):
    validate_reviews(rows, reviews)
    result = copy.deepcopy(rows)
    # prepare() also calls complete-image review machinery. Here it only resolves
    # actor types; completeness stays explicitly unasserted under v5 breadth.
    capture = [r for r in rows if r['source'].startswith('capture:') and r['split']=='train' and not r['eligible']]
    updated = {r['id']:r for r in prepare(capture, policy, reviews, {})}
    candidates = set()
    rejected = Counter()
    for i,old in enumerate(result):
        if old['id'] not in updated:
            continue
        row = updated[old['id']]
        if any('split' in reason or 'duplicate' in reason for reason in old['reasons']):
            rejected['split_or_duplicate_review'] += 1
            continue
        if any(reason in old['reasons'] for reason in ('no_visible_instance_pixels','review_rejected',
                'source_image_changed_since_gt','image_dimensions_disagree','missing_or_broken_image')):
            rejected['existing_geometry_or_individual_review_issue'] += 1
            continue
        if not row['annotations']:
            rejected['empty_or_missing_gt_not_negative'] += 1
            continue
        if any(not valid_box(a['xyxy'],row['width'],row['height']) for a in row['annotations']):
            rejected['invalid_geometry'] += 1
            continue
        if any(a['disposition']=='ambiguous' for a in row['annotations']):
            rejected['unresolved_actor_in_image'] += 1
            continue
        if not any(a['canonical'] for a in row['annotations']):
            rejected['excluded_only_not_negative'] += 1
            continue
        if not row['original_annotation'].get('bbox_file_present'):
            rejected['missing_bbox_source'] += 1
            continue
        row.update(eligible=True, reasons=[], previous_admission_reasons=old['reasons'],
            admission_tier='capture_actor_rgb_breadth_training',
            review_basis='independent_actor_rgb_and_original_capture_gt',
            breadth_quality=dict(complete_target_annotation_asserted=False,
                actor_types_independently_reviewed=True, teacher_truth_used=False,
                original_bbox_and_mask_preserved=True))
        result[i] = row
        candidates.add(row['id'])
    all_roles=defaultdict(set);pixels=defaultdict(list)
    for row in result:
        all_roles[row['pixel_sha256']].add(row['split'])
        if row['eligible']:pixels[row['pixel_sha256']].append(row)
    for pixel,group in pixels.items():
        for pos,row in enumerate(sorted(group,key=lambda r:(r['id'] in candidates,r['source'],r['id']))):
            if row['id'] in candidates and (len(all_roles[pixel])>1 or pos>0):
                row['eligible']=False
                row['reasons']=['capture_exact_pixel_split_or_duplicate']
                rejected['exact_pixel_split_or_duplicate']+=1
    added=[r for r in result if r['id'] in candidates and r['eligible']]
    return result,dict(new_images=len(added),new_objects=sum(bool(a['canonical']) for r in added for a in r['annotations']),
        new_classes=dict(Counter(a['training_class'] for r in added for a in r['annotations'] if a['canonical'])),
        new_by_run=dict(Counter(r['source'] for r in added)),new_by_view=dict(Counter(r['view'] for r in added)),
        new_actor_resolved_objects=sum('actor_review' in a for r in added for a in r['annotations']),
        rejected_capture_candidates=dict(rejected))


def run(args):
    if args.output.exists():
        raise FileExistsError('preserve previous Capture artifacts; choose a new output')
    rows=list(read_jsonl(args.previous))
    if args.command=='render':
        render(rows,args.output)
        return
    if args.command=='bind':
        entries=json.loads(args.intervals.read_text())
        decisions=json.loads(args.decisions.read_text())
        if decisions['intervals_sha256'] != sha(args.intervals):
            raise ValueError('inspected interval manifest changed')
        save(args.output,bind_reviews(rows,entries,decisions['decisions']))
        return
    policy=load_label_policy(args.labels)
    reviews=json.loads(args.reviews.read_text())
    updated,stats=expand_capture(rows,policy,reviews)
    after={r['id']:r for r in updated}
    for old in rows:
        new=after[old['id']]
        if old['eligible'] or old['split']!='train':
            assert new==old, 'previous data or evaluation changed'
        assert old['split']==new['split'] and old['frozen_split']==new['frozen_split']
        assert [(a['id'],a['xyxy']) for a in old['annotations']]==[(a['id'],a['xyxy']) for a in new['annotations']]
    args.output.mkdir(parents=True,exist_ok=False)
    write_jsonl(args.output/'curated.jsonl',updated)
    export_dataset(updated,policy,args.export,fixed_class_order=True)
    accepted=[r for r in updated if r['eligible']]
    summary=dict(previous_curated_sha256=sha(args.previous),actor_reviews_sha256=sha(args.reviews),
        label_policy_sha256=sha(args.labels),images=len(accepted),
        roboflow_images=sum(r['source'].startswith('roboflow:') for r in accepted),
        capture_images=sum(r['source'].startswith('capture:') for r in accepted),
        objects=sum(bool(a['canonical']) for r in accepted for a in r['annotations']),
        splits=dict(Counter(r['split'] for r in accepted)),
        objects_by_class=dict(Counter(a['training_class'] for r in accepted for a in r['annotations'] if a['canonical'])),
        quality_tiers=dict(Counter(r.get('admission_tier','previous_reviewed_subset') for r in accepted)),
        actor_intervals_reviewed=len(reviews),evaluation_membership_and_labels_unchanged=True,
        original_geometry_changes=0,previous_accepted_lost=0,training_size_filter=False,
        complete_target_annotation_asserted=False,teacher_truth_used=False,
        training_performed=False,sampling_changed=False,runtime_changes=False,**stats)
    save(args.output/'capture-summary.json',summary)
    save(args.output/'actor-reviews.json',reviews)
    exported=json.loads((args.export/'summary.json').read_text())
    exported.update(quality_tiers=summary['quality_tiers'],complete_target_annotation_asserted=False,
        strict_quality_equivalent=False,admission_policy='capture_actor_rgb_breadth_training_v1')
    save(args.export/'summary.json',exported)
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['render','bind','export'])
    for name in ('previous','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    for name in ('labels','reviews','export','intervals','decisions'):
        parser.add_argument('--'+name,type=Path)
    run(parser.parse_args())
