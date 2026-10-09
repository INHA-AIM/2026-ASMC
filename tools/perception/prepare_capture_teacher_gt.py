#!/usr/bin/env python3
"""Link independently reviewed Capture actor subtypes without inventing full GT.

Actor IDs are scoped by run AND original category AND inclusive frame interval:
static Obstacle 15 and Vehicle 15 can coexist. Actor evidence does not attest all
objects in an image. Complete-image reviews use the same v2 scope as export.
"""
import argparse
import copy
import json
from pathlib import Path
from camera_data_audit import (apply_review, label_metadata, load_label_policy, read_jsonl,
    save, semantic_names, sha, write_jsonl)


def prepare(rows, policy, actor_reviews, image_reviews):
    result=[]
    for source in rows:
        row=copy.deepcopy(source)
        row['split']=row['frozen_split']
        row['annotation_complete']=False
        for ann in row['annotations']:
            frame=int(row['source_key'].rsplit('/',1)[1])
            matched=[r for r in actor_reviews if r['source']==row['source']
                     and r['original_label']==ann['original_label'] and str(r['actor_id'])==str(ann.get('actor_id'))
                     and r['first_frame'] <= frame <= r['last_frame']]
            if len(matched)>1:raise ValueError('overlapping actor review intervals')
            if matched:
                review=matched[0]
                if review.get('teacher_derived') or not review.get('evidence_image_sha256'):
                    raise ValueError('actor GT must have independent image evidence')
                target=review['canonical']
                if target is not None and target not in semantic_names(policy):raise ValueError('unknown reviewed actor class')
                ann.update(canonical=target,disposition='candidate' if target else 'excluded',
                    actor_review=review,issues=[i for i in ann['issues'] if i!='capture_subtype_unknown'])
            label_metadata(ann,policy)
        row,reviewed=apply_review(row,image_reviews.get(row['id']),policy)
        if reviewed:row['review_scope']=policy['review_scope']
        result.append(row)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inventory',type=Path,required=True)
    p.add_argument('--labels',type=Path,required=True);p.add_argument('--actor-reviews',type=Path)
    p.add_argument('--image-reviews',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('preserve previous GT; output exists')
    reviews=json.loads(args.actor_reviews.read_text()) if args.actor_reviews else []
    # Verify cited independent RGB evidence before trusting run/actor propagation.
    for r in reviews:
        if sha(r['evidence_image']) != r['evidence_image_sha256']:raise ValueError('actor evidence image SHA changed')
    image_reviews=json.loads(args.image_reviews.read_text()) if args.image_reviews else {}
    rows=[r for r in read_jsonl(args.inventory) if r['frozen_split'] in ('selection','heldout')]
    result=prepare(rows,load_label_policy(args.labels),reviews,image_reviews)
    args.output.mkdir(parents=True)
    write_jsonl(args.output/'frames.jsonl',result)
    from camera_teacher_audit import complete_gt
    policy=load_label_policy(args.labels)
    save(args.output/'summary.json',dict(frames=len(result),complete_frames=sum(complete_gt(r,policy) for r in result),
        inventory_sha256=sha(args.inventory),actor_review_sha256=sha(args.actor_reviews) if args.actor_reviews else None,
        image_review_sha256=sha(args.image_reviews) if args.image_reviews else None,teacher_derived=False,
        interpretation='Actor subtype evidence is not image completeness. Unknown actors remain unresolved, never empty negatives.'))


if __name__=='__main__':main()
