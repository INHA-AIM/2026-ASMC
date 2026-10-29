#!/usr/bin/env python3
"""Independent full-timeline native reference audit and compact trial plots."""
import argparse
import csv
import json
import math
import hashlib
from pathlib import Path
import subprocess
from collections import Counter
from localization_assessment import completed_gps_recovery,maximum_stopped_interval,shadow_timeline_coverage,phase_speed_metrics


def finite_le(value,bound):
    return value is not None and math.isfinite(value) and value<=bound


def assess(root):
    from scen_runner.morai.signal_trial_coordinates import Coordinates
    rows=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    native=[r for r in rows if r['event']=='native']
    if not native: raise RuntimeError('trial contains no native motion samples')
    topics={}
    for r in rows:
        if r['event']=='topic': topics.setdefault(r['topic'],[]).append(r)
    coordinate=Coordinates([302595.,4124145.])
    with (root/'native.csv').open('x') as f:
        writer=csv.writer(f);writer.writerow(['index','stamp','x','y','yaw'])
        for i,r in enumerate(native):
            p=r['ego']['pose']
            writer.writerow([i,r['ros_time'],p['x'],p['y'],coordinate.grid_yaw(p['x'],p['y'],p['yaw'])])
    subprocess.run(['/devel/lib/asmc_planner/localization_evaluate',
                    '/root/ws/src/asmc_planner/config/frenet/params.yaml',
                    str(root/'native.csv'),str(root/'reference.csv')],check=True,stdout=subprocess.DEVNULL)
    reference=list(csv.DictReader((root/'reference.csv').open()))
    result=json.loads((root/'result.json').read_text())
    def percentile(values,q=.95):
        values=sorted(values)
        return values[round((len(values)-1)*q)] if values else None
    def data(topic): return topics.get(topic,[])
    decisions=data('/planning/localization/trace')
    coverage=shadow_timeline_coverage(native,data('/planning/mode'),decisions)
    cmds=[r for r in data('/ctrl_cmd') if native[0]['monotonic']<=r['monotonic']<=native[-1]['monotonic']]
    gaps=[b['monotonic']-a['monotonic'] for a,b in zip(cmds,cmds[1:])]
    if cmds:gaps.extend([cmds[0]['monotonic']-native[0]['monotonic'],native[-1]['monotonic']-cmds[-1]['monotonic']])
    lat=[abs(float(r['lateral_m'])) for r in reference if int(r['projected'])]
    heading=[abs(float(r['heading_deg'])) for r in reference if int(r['projected'])]
    sources=Counter(r['value']['source'] for r in decisions)
    handoffs=[]
    for r in data('/planning/localization/handoff_trace'):
        near=[n for n in native if abs(n['ros_time']-r['ros_time'])<=.5]
        commands=[c for c in data('/ctrl_cmd') if abs(c['ros_time']-r['ros_time'])<=.15]
        handoffs.append(dict(**r['value'], stamp=r['ros_time'],
            minimum_speed_kph=min((n['ego']['speed_kmh'] for n in near),default=None),
            maximum_brake=max((c['value']['brake'] for c in commands),default=None)))
    import rosbag
    bag_counts=Counter();bag_controls=[]
    for bag_path in root.glob('*.bag'):
        with rosbag.Bag(str(bag_path)) as bag:
            bag_counts.update({t:v.message_count for t,v in bag.get_type_and_topic_info()[1].items()})
            bag_controls.extend(stamp.to_sec() for _,_,stamp in bag.read_messages(topics=['/ctrl_cmd'])
                                if native[0]['ros_time']<=stamp.to_sec()<=native[-1]['ros_time'])
    metrics=dict(native_samples=len(native), lateral_p95_m=percentile(lat),lateral_max_m=max(lat),
        heading_p95_deg=percentile(heading), command_max_gap_s=max(gaps) if gaps else None,
        speed_max_kph=max(n['ego']['speed_kmh'] for n in native),
        maximum_midrun_stopped_interval_s=maximum_stopped_interval(native),
        speed_error_after_5s_p95_kph=percentile([abs(n['ego']['speed_kmh']-result['cap_kph']) for n in native
            if n['monotonic']-native[0]['monotonic']>=5]),
        processing_p95_ms=percentile([r['value']['processing_ms'] for r in decisions]),
        pose_sources=dict(sources),handoffs=handoffs,bag_counts=dict(bag_counts),
        active_receipts=len(cmds),active_bag_commands=len(bag_controls),
        reasons=dict(Counter(r['value']['reason'] for r in decisions)))
    metrics.update(coverage)
    normal_ceiling=result.get('normal_speed_ceiling_kph',result['cap_kph'])
    speed_phases=phase_speed_metrics(native,data('/planning/mode'),result['cap_kph'],normal_ceiling)
    metrics.update(speed_phases)
    mixed_speed=normal_ceiling!=result['cap_kph']
    metrics['steady_speed_scope']='JAMMING only; NORMAL has separate native ceiling' if mixed_speed else 'whole trial'
    gates=dict(completed=result['reason']=='completed',no_contact=not result['contacts'],
        restored=not result['cleanup_errors'],reference_complete=len(lat)==len(native),
        physical_map_inside=all(int(r['map_inside']) for r in reference),
        lateral_p95=metrics['lateral_p95_m']<=.25,lateral_max=metrics['lateral_max_m']<=.5,
        heading_p95=metrics['heading_p95_deg']<=3,
        mode_coverage=metrics['mode_coverage_ratio']==1.,
        pairing=metrics['shadow_pairing_ratio']>=.95,valid_moving=metrics['shadow_valid_moving_ratio']>=.95,
        control_gap=metrics['command_max_gap_s'] is not None and metrics['command_max_gap_s']<=.25,
        overspeed=speed_phases['speed_violation_samples']==0 and speed_phases['speed_unknown_mode_samples']==0,
        steady_speed=finite_le(speed_phases['shadow_speed_error_after_5s_p95_kph'] if mixed_speed else metrics['speed_error_after_5s_p95_kph'],.5),
        no_midrun_stop=finite_le(metrics['maximum_midrun_stopped_interval_s'],.15),
        processing=metrics['processing_p95_ms']<=50,
        gps_dr_recovery=completed_gps_recovery([r['value'] for r in decisions]) is not None,
        both_handoffs={h['target'] for h in handoffs}=={0,1},
        handoff_no_stop=bool(handoffs) and all(h['minimum_speed_kph'] is not None and h['minimum_speed_kph']>.5 and
                                            h['maximum_brake'] is not None and h['maximum_brake']<.9 for h in handoffs),
        recording=not list(root.glob('*.active')) and abs(len(cmds)-len(bag_controls))<=2)
    fixture=result.get('fixture','none')
    temporal_fixture=fixture in ('parked_vehicle','parked_truck','angled_vehicle','slow_lead','crossing_left','crossing_right','standing_person')
    udp_events=[r for r in data('/CollisionData') if any(r['value'].get(k) for k in ('collision_object','collision_objecta'))]
    metrics['udp_collision_nonempty_messages']=len(udp_events)
    if '/CollisionData' in topics:
        from localization_trial_policy import udp_contact_classification
        from localization_assessment import nearest
        nt=[n['ros_time'] for n in native];verdicts=Counter()
        for row in udp_events:
            if not native[0]['ros_time']<=row['ros_time']<=native[-1]['ros_time']:continue
            n=nearest(native,nt,row['ros_time'],.15)
            verdicts[udp_contact_classification(row['value'],n['ego'] if n else None,row['ros_time'])]+=1
        metrics['active_udp_verdicts']=dict(verdicts)
        gates['udp_collision_no_unreviewed_events']=not any(v for k,v in verdicts.items() if k not in ('empty','position_matched_self_status'))
    if temporal_fixture:
        from localization_assessment import moving_fixture_evidence
        evidence=moving_fixture_evidence(native,[r for r in rows if r['event']=='fixture_motion'],coordinate.grid_yaw)
        metrics.update(evidence)
        for key in ('lateral_p95','lateral_max','heading_p95','steady_speed','no_midrun_stop'):gates.pop(key)
        displacement=list(evidence['fixture_max_displacement_m'].values())
        moving=fixture in ('slow_lead','crossing_left','crossing_right')
        gates.update(owned_actor_timeline=evidence['fixture_pairing_ratio']>=.95,
            native_actor_clearance=evidence['native_obb_clearance_min_m'] is not None and evidence['native_obb_clearance_min_m']>.05,
            fixture_motion_confirmed=bool(displacement) and (max(displacement)>2. if moving else max(displacement)<.5),
            bounded_stop_and_restart=finite_le(metrics['maximum_midrun_stopped_interval_s'],8. if moving else 3.))
    if result.get('static_avoidance') and fixture in ('center','left','right','staggered'):
        from localization_assessment import native_static_clearance,native_fixture_polygon,native_ego_polygon,confirm_static_return
        fixtures=[actor for row in rows if row['event']=='fixture_native' for actor in row['actors']]
        fixture_polygons=[native_fixture_polygon(a,coordinate.grid_yaw(a['pose']['x'],a['pose']['y'],a['pose']['yaw'])) for a in fixtures]
        clearances=[];sphere_bounds=[]
        for n in native:
            p=n['ego']['pose'];yaw=coordinate.grid_yaw(p['x'],p['y'],p['yaw'])
            ego=native_ego_polygon(p['x'],p['y'],yaw)
            clearances.extend(ego.distance(fixture) for fixture in fixture_polygons)
            sphere_bounds.extend(native_static_clearance(p['x'],p['y'],yaw,actor) for actor in fixtures)
        end_station=native[-1]['station']
        returned=[r for n,r in zip(native,reference) if n['station']>=end_station-8 and int(r['projected'])]
        phases=Counter(r['value']['phase'] for r in decisions)
        # ShadowZone stations use the 2D input polyline. Avoid mixing those
        # stations with the scenario runner's 3D arc or native spline station.
        import numpy as np
        import xml.etree.ElementTree as ET
        from scen_runner.config import load_config,repo_path
        launch=Path(__file__).resolve().parents[2]/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch'
        end_s=float(next(p.get('value') for p in ET.parse(launch).iter('param')
                         if p.get('name')=='shadow_zone/station_end_m'))
        points=np.loadtxt(repo_path(load_config()['map']['global_path']))[:,:2]
        delta=points[1:]-points[:-1];length=np.linalg.norm(delta,axis=1)
        stations=np.r_[0.,np.cumsum(length)]
        index=min(int(np.nonzero(stations<=end_s)[0][-1]),len(points)-2)
        direction=delta[index]/length[index]
        exit_xy=points[index]+(end_s-stations[index])*direction
        confirmation=confirm_static_return(native,reference,fixtures,exit_xy,direction)
        metrics.update(static_phases=dict(phases),fixture_native_count=len(fixtures),
            native_obb_clearance_min_m=min(clearances) if clearances else None,
            native_sphere_lower_bound_min_m=min(sphere_bounds) if sphere_bounds else None,
            returned_lateral_p95_m=percentile([abs(float(r['lateral_m'])) for r in returned]),
            returned_heading_p95_deg=percentile([abs(float(r['heading_deg'])) for r in returned]))
        metrics.update(return_before_exit_confirmation=confirmation,exit_plane_2d_station_m=end_s)
        # Static bypass intentionally leaves the fixed centerline and may slow
        # for verified stopping space. Keep these as reported metrics; evaluate
        # actual physical clearance and return instead of M2's tracking gates.
        for key in ('lateral_p95','lateral_max','heading_p95','steady_speed','no_midrun_stop'):gates.pop(key)
        gates.update(owned_fixture_snapshot=bool(fixtures),
            no_prolonged_static_stop=finite_le(metrics['maximum_midrun_stopped_interval_s'],3.),
            native_static_clearance=bool(clearances) and min(clearances)>.05,
            bypass_and_return=phases['PASS']>0 and phases['RETURN']>0,
            return_before_tunnel_exit=confirmation is not None,
            returned_to_reference=bool(returned) and metrics['returned_lateral_p95_m']<=.25 and
                                  metrics['returned_heading_p95_deg']<=3)
    summary=dict(evaluation_schema=2,metrics=metrics,gates=gates,accepted=all(gates.values()),result=result,
        audit_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            root/'native.csv',Path('/devel/lib/asmc_planner/localization_evaluate'),
            Path('/root/ws/src/asmc_planner/config/frenet/params.yaml'),
            Path(__file__).resolve(),Path(__file__).resolve().with_name('localization_assessment.py'))},
        evaluation='every native pose independently projected to fixed Frenet reference; native truth is evaluation only; GPS injection explicitly recorded')
    (root/'assessment.json').write_text(json.dumps(summary,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin=native[0]['ros_time'];times=[n['ros_time']-origin for n in native]
    figure,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    axes[0].plot(times,[n['ego']['speed_kmh'] for n in native]);axes[0].axhline(result['cap_kph'],color='grey',linestyle='--');axes[0].set_ylabel('Speed km/h')
    axes[1].plot(times,[abs(float(r['lateral_m'])) for r in reference]);axes[1].axhline(.25,color='grey',linestyle='--');axes[1].set_ylabel('Lateral error m')
    for field in ('steering','accel','brake'):
        axes[2].plot([r['ros_time']-origin for r in data('/ctrl_cmd')], [r['value'][field] for r in data('/ctrl_cmd')],label=field)
    axes[2].legend();axes[2].set_ylabel('Applied command')
    ordered=['unavailable','gps_anchored','dead_reckoning','recovery_verify','recovery_blend']
    axes[3].plot([r['ros_time']-origin for r in decisions],[ordered.index(r['value']['source']) if r['value']['source'] in ordered else -1 for r in decisions]);axes[3].set_yticks(range(5));axes[3].set_yticklabels(ordered);axes[3].set_xlabel('Seconds from departure')
    for ax in axes:
        ax.grid(True,alpha=.3)
        for h in handoffs:ax.axvline(h['stamp']-origin,color='orange',alpha=.5)
    figure.suptitle(root.name+' | '+result['reason']+' | '+('ACCEPTED' if summary['accepted'] else 'GATES OPEN'))
    figure.tight_layout();figure.savefig(root/'timeline.png',dpi=130);plt.close(figure)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    print(json.dumps(assess(parser.parse_args().root),indent=2))
