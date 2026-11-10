#!/usr/bin/env python3
"""Native-only normal-GPS interval audit for main/localization A/B trials."""
import argparse,csv,hashlib,json,math,subprocess
from pathlib import Path
from localization_assessment import quantile


def assess(root):
    from scen_runner.morai.signal_trial_coordinates import Coordinates
    import rosbag
    rows=list(map(json.loads,(root/'events.jsonl').read_text().splitlines()))
    native=[r for r in rows if r['event']=='native']
    if not native:raise RuntimeError('normal interval has no measured motion')
    result=json.loads((root/'result.json').read_text())
    coordinates=Coordinates([302595.,4124145.])
    raw=root/'normal-native.csv';ref=root/'normal-reference.csv'
    with raw.open('x') as f:
        writer=csv.writer(f);writer.writerow(['index','stamp','x','y','yaw'])
        for i,n in enumerate(native):
            p=n['ego']['pose'];writer.writerow([i,n['ros_time'],p['x'],p['y'],coordinates.grid_yaw(p['x'],p['y'],p['yaw'])])
    auditor=Path('/devel/lib/asmc_planner/localization_evaluate')
    parameters=Path('/root/ws/src/asmc_planner/config/frenet/params.yaml')
    subprocess.run([str(auditor),str(parameters),str(raw),str(ref)],check=True,stdout=subprocess.DEVNULL)
    reference=list(csv.DictReader(ref.open()))
    commands=[r for r in rows if r.get('topic')=='/ctrl_cmd' and
              native[0]['monotonic']<=r['monotonic']<=native[-1]['monotonic']]
    gaps=[b['monotonic']-a['monotonic'] for a,b in zip(commands,commands[1:])]
    if commands:gaps.extend([commands[0]['monotonic']-native[0]['monotonic'],native[-1]['monotonic']-commands[-1]['monotonic']])
    errors=[abs(n['ego']['speed_kmh']-result['cap_kph']) for n in native if n['monotonic']-native[0]['monotonic']>=5]
    modes=[r['value'] for r in rows if r.get('topic')=='/planning/mode']
    bag_count=0
    for p in root.glob('*.bag'):
        with rosbag.Bag(str(p)) as bag:
            bag_count+=sum(native[0]['ros_time']<=t.to_sec()<=native[-1]['ros_time']
                           for _,_,t in bag.read_messages(topics=['/ctrl_cmd']))
    metrics=dict(planner=result.get('planner'),native_samples=len(native),
        lateral_p95_m=quantile([abs(float(r['lateral_m'])) for r in reference]),
        heading_p95_deg=quantile([abs(float(r['heading_deg'])) for r in reference]),
        command_max_gap_s=max(gaps) if gaps else None,speed_max_kph=max(n['ego']['speed_kmh'] for n in native),
        steady_speed_error_p95_kph=quantile(errors),active_receipts=len(commands),active_bag_commands=bag_count)
    le=lambda value,bound:value is not None and math.isfinite(value) and value<=bound
    gates=dict(completed=result['reason']=='completed',no_contact=not result['contacts'],restored=not result['cleanup_errors'],
        normal_only=bool(modes) and all(m==0 for m in modes),
        native_reference_complete=len(reference)==len(native) and all(int(r['projected']) for r in reference),
        physical_map_inside=all(int(r['map_inside']) for r in reference),
        tracking=le(metrics['lateral_p95_m'],.25) and le(metrics['heading_p95_deg'],3),
        control_gap=le(metrics['command_max_gap_s'],.1),overspeed=metrics['speed_max_kph']<=result['cap_kph']+1,
        steady_speed=le(metrics['steady_speed_error_p95_kph'],.5),
        owned_inference_ready=any(r['event']=='perception_ready' for r in rows),
        recording=not list(root.glob('*.active')) and abs(len(commands)-bag_count)<=2)
    summary=dict(metrics=metrics,gates=gates,accepted=all(gates.values()),
                 scope='normal GPS interval only; same private validation speed fixture for both launch options',
                 audit_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                    (raw,auditor,parameters,Path(__file__).resolve(),
                     Path(__file__).resolve().with_name('localization_assessment.py'))})
    (root/'normal-assessment.json').open('x').write(json.dumps(summary,indent=2))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    print(json.dumps(assess(parser.parse_args().root),indent=2))
