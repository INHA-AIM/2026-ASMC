"""Explicit, bounded adjacent-lane review; never infer neighboring road scope.

Source paint is retained as continuous geometry. Physical dash segments are
created by the shared renderer, with phase aligned at the reviewed join.
"""
from __future__ import annotations

import hashlib
import json
import numpy as np

from .geometry import resample_polyline
from .schema import make_candidate_feature
from .review_bundle import _load_legacy_references


def _clip(points, origin, direction, sign):
    result = []
    for a, b in zip(points, points[1:]):
        da = sign * np.dot(a[:2] - origin[:2], direction)
        db = sign * np.dot(b[:2] - origin[:2], direction)
        if da >= 0 and (not result or np.linalg.norm(result[-1]-a) > 1e-8):
            result.append(a)
        if (da >= 0) != (db >= 0):
            result.append(a + (b-a) * da/(da-db))
        if db >= 0:
            result.append(b)
    if len(result) < 2:
        raise ValueError('reviewed adjacent marking has no points in its declared interval')
    p=np.asarray(result);keep=np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-8]
    return p[keep]


def adjacent_markings(config, review, features, repo_root):
    groups=config.get('reviewed_adjacent_corridors', [])
    if not groups:
        return []
    source=repo_root/'src/asmc_planner/config/behavior/link_set.json'
    raw=source.read_bytes();link_sha=hashlib.sha256(raw).hexdigest()
    links={r['idx']:r for r in json.loads(raw)}
    refs={r['id']:r for r in review['legacy_references']}
    refs.update({r['id']:r for r in _load_legacy_references(review['config'],repo_root)})
    output=[]
    for group in groups:
        declared=set(group['link_ids'])
        reference=np.asarray(links[group['axis_link_id']]['points'],float)
        direction=reference[-1,:2]-reference[0,:2];direction/=np.linalg.norm(direction)
        for marking in group['markings']:
            targets=marking['target_current_link_ids']
            if not set(targets).issubset(declared):
                raise ValueError('adjacent marking target is outside reviewed scope')
            anchor=None
            if marking.get('join_feature_suffix'):
                matches=[f for f in features if f['id'].endswith(marking['join_feature_suffix'])]
                if len(matches)!=1:
                    raise ValueError('adjacent join feature must be unique')
                anchor=np.asarray(matches[0]['geometry']['coordinates'][0],float)
            if marking.get('legacy_source_ids'):
                pieces=[]
                for source_id in marking['legacy_source_ids']:
                    p=np.asarray(refs[source_id]['coordinates'],float)
                    if np.dot(p[-1,:2]-p[0,:2],direction)<0:p=p[::-1]
                    if pieces and np.linalg.norm(pieces[-1][-1,:2]-p[0,:2])>0.5:
                        raise ValueError('adjacent legacy pieces have an unreviewed gap')
                    pieces.append(p)
                points=np.vstack(pieces);source_kind='legacy_katri'
                source_id='+'.join(marking['legacy_source_ids'])
                source_sha=review['config']['comparison']['legacy_lane_set_sha256']
                transform='declared_origin_translation_reviewed_adjacent_crop'
            else:
                p=np.asarray(links[marking['current_link_id']]['points'],float)
                points,tangent=resample_polyline(p,float(config['geometry']['output_step_m']))
                offset=float(marking['left_offset_m'])
                points[:,:2]+=offset*np.column_stack([-tangent[:,1],tangent[:,0]])
                source_kind='current_mgeo';source_id=marking['current_link_id'];source_sha=link_sha
                transform='reviewed_missing_left_marking_nominal_width_offset'
            points=_clip(points,reference[0],direction,1)
            if anchor is not None:
                points=_clip(points,anchor,direction,-1)
                if np.linalg.norm(points[-1,:2]-anchor[:2])>0.5:
                    raise ValueError('adjacent endpoint differs from reviewed final marking by >0.5m')
                points[-1]=anchor
            pattern=marking['pattern'];length=np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum()
            phase=float((-length)%6.0) if pattern=='broken' and anchor is not None else 0.0
            f=make_candidate_feature(
                feature_type='lane_marking',canonical_id=config['map_id']+':lane_marking:adjacent:'+marking['id'],
                points=points,attributes=dict(color='white',pattern=pattern,line_group='single',
                    line_width_m=0.15,dash_length_m=3.0 if pattern=='broken' else 0.0,
                    dash_gap_m=3.0 if pattern=='broken' else 0.0,dash_phase_m=phase,
                    functional_role='lane_boundary_paint',curation_segment_id=group['curation_segment_id'],
                    reviewed_adjacent_group_id=group['id'],source_display='reviewed_adjacent_corridor'),
                source_id=source_id,source_sha256=source_sha,target_link_ids=targets,
                validation=dict(human_reviewed=True,evidence_ids=group['evidence'],
                    inferred_missing_paint=not bool(marking.get('legacy_source_ids')),
                    join_feature_suffix=marking.get('join_feature_suffix'),
                    dash_phase_aligned_to_join=pattern=='broken' and anchor is not None))
            f['provenance'].update(source_kind=source_kind,transform=transform)
            f['status']='reviewed';output.append(f)
    return output
