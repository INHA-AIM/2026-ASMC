#!/usr/bin/env python3
"""Per-run track continuity diagnostics, separate from frame sync matched counts.

Track IDs are not independent physical-object GT. New IDs and reacquisition are
reported as events, never counted as improved recall. Cross-ID fragmentation
requires independent actor identity and cannot be inferred from this report.
"""
import argparse
import json
from pathlib import Path

def summarize(rows):
    stamps=[r['stamp_ns'] for r in rows]
    if stamps!=sorted(set(stamps)):raise ValueError('duplicate or reversed LiDAR stamps')
    observations={}
    for index,row in enumerate(rows):
        ids=[o[0] for o in row['objects']]
        if len(ids)!=len(set(ids)):raise ValueError('duplicate track IDs in one frame')
        for obj in row['objects']:observations.setdefault(obj[0],[]).append((index,row['stamp_ns'],obj[2]))
    tracks=[]
    for track,values in sorted(observations.items()):
        gaps=[(b[1]-a[1])/1e6 for a,b in zip(values,values[1:])]
        missing=sum(b[0]-a[0]-1 for a,b in zip(values,values[1:]))
        losses=sum(a[2] not in ('unknown','') and b[2] in ('unknown','') for a,b in zip(values,values[1:]))
        known=[v for v in values if v[2] not in ('unknown','')]
        tracks.append(dict(track_id=track,observations=len(values),internal_missing_output_frames=missing,
            presence_reacquisitions=sum(b[0]>a[0]+1 for a,b in zip(values,values[1:])),
            classification_loss_transitions=losses,classified_observations=len(known),
            max_observation_gap_ms=max(gaps,default=0),first_stamp_ns=values[0][1],last_stamp_ns=values[-1][1]))
    return dict(output_frames=len(rows),track_ids=len(tracks),tracks=tracks,
        classification_loss_transitions=sum(t['classification_loss_transitions'] for t in tracks),
        internal_missing_output_frames=sum(t['internal_missing_output_frames'] for t in tracks),
        presence_reacquisitions=sum(t['presence_reacquisitions'] for t in tracks),
        physical_object_fragmentation='not_measured_without_independent_actor_identity')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);args=p.parse_args()
    result={f.stem:summarize(json.loads(f.read_text())['tracks']) for f in sorted(args.directory.glob('*.frames.json'))}
    (args.directory/'track-continuity.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({name:{k:v for k,v in r.items() if k!='tracks'} for name,r in result.items()},indent=2))
if __name__=='__main__':main()
