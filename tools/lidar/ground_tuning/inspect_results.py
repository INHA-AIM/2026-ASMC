#!/usr/bin/env python3
"""Compact, reproducible tables from computed summaries (no manual score transcription)."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('summary',type=Path)
a=p.parse_args();s=json.loads(a.summary.read_text())
percent=lambda n,d:100*n/d if d else None
print('frames',s['frames'])
for name in sorted(s['objects']):
    cls=s['classes'][name];o=s['objects'][name];g=s.get('gates',{}).get(name,{})
    data={'profile':name,'object_pass':g.get('object_pass'),'eligible':g.get('eligible_on_this_recording'),
          'violations':g.get('violations',[]),'extra_complete_loss':s['regressions'].get(name,{}).get('additional_complete_loss',0)}
    data['actors']={c:{'retain_pct':percent(o[c+'/observed10plus']['retained_points'],o[c+'/observed10plus']['input_points']),
                      'lower_pct':percent(o[c+'/observed10plus']['lower_retained'],o[c+'/observed10plus']['lower_input']),
                      'lost':o[c+'/observed10plus']['complete_loss']} for c in ('vehicle','pedestrian','obstacle')}
    data['background']={c:{'remaining_pct':percent(cls[c]['retained_points'],cls[c]['input_points']),
        'split_04':s['topology'][name][c]['0.4']['added_split_parts'],
        'isolated_04':s['topology'][name][c]['0.4']['newly_isolated_points']} for c in ('asphalt','sidewalk')}
    print(json.dumps(data,ensure_ascii=False))
