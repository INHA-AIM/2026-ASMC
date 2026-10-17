#!/usr/bin/env python3
"""Offline M0/M1/M2 lane-family analysis. Never publishes or selects defaults.

M1 changes geometry only: M0 pixel selection/quality are deliberately frozen.
M2 uses development Ego poses as an upper-bound reference, not a runtime input.
Run with PYTHONPATH=src/asmc_camera/src and OPENBLAS_NUM_THREADS=1.
"""
import argparse
from collections import Counter, defaultdict
from dataclasses import replace
import hashlib
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict, GroundPlane
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.model_grid_projection import MetricEvidence, ModelGridGroundProjector
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.boundary_candidates import select_shadow_lane_tracks_with_audit
from asmc_camera.bev.shared_lane_family import (
    fit_shared_lane_family_with_audit, contiguous_sample_state_segments,
)
from asmc_camera.bev.visualization import draw_metric_grid, draw_stateful_lane_family
from asmc_camera.bev.planning_usability import measure_planning_usability, LongitudinalZone

VIEWS = ('front', 'left', 'right')
VARIANTS = ('M0','M1_absolute','M1_rest','M2_align_only','M2_align','M2_history')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path):
    return list(map(json.loads, Path(path).read_text().splitlines()))


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return dict(count=len(values), **({'min': float(values.min()), 'max': float(values.max()),
                **{'p'+str(q): float(np.percentile(values,q)) for q in (50,95,99)}} if len(values) else {}))


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid quaternion')
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def rpy(q):
    R = rotation(q)
    return np.array([np.arctan2(R[2,1],R[2,2]),
                     np.arcsin(np.clip(-R[2,0],-1,1)),np.arctan2(R[1,0],R[0,0])])


def interpolate(samples, stamp, kind, max_span_ns=100_000_000):
    """Bracketed source-time interpolation, no extrapolation/clock mixing."""
    stamps = np.asarray([s['source_ns'] for s in samples], dtype=np.int64)
    at = int(np.searchsorted(stamps, stamp))
    if at < len(samples) and stamps[at] == stamp:
        value=samples[at]
        if kind=='imu':
            try:rotation(value['q_xyzw'])
            except ValueError:return None
        elif not np.isfinite(value['position_xyz']+[value['heading_deg']]).all():return None
        return value
    if at == 0 or at == len(samples):
        return None
    a,b = samples[at-1:at+1]
    span = b['source_ns'] - a['source_ns']
    if span <= 0 or span > max_span_ns:
        return None
    f = (stamp-a['source_ns'])/span
    out = dict(source_ns=stamp, bracket_span_ms=span/1e6)
    if kind == 'imu':
        try:
            rotation(a['q_xyzw']);rotation(b['q_xyzw'])
        except ValueError:return None
        x,y = (np.asarray(s['q_xyzw'],dtype=float) for s in (a,b))
        x /= np.linalg.norm(x); y /= np.linalg.norm(y)
        if x @ y < 0: y = -y
        cosine = np.clip(x @ y,-1,1)
        angle = np.arccos(cosine)
        q = ((1-f)*x+f*y if angle < 1e-5 else
             (np.sin((1-f)*angle)*x + np.sin(f*angle)*y)/np.sin(angle))
        out['q_xyzw'] = (q/np.linalg.norm(q)).tolist()
    else:
        if not all(np.isfinite(s['position_xyz']+[s['heading_deg']]).all() for s in (a,b)):return None
        out['position_xyz'] = ((1-f)*np.asarray(a['position_xyz'])+f*np.asarray(b['position_xyz'])).tolist()
        delta = (b['heading_deg']-a['heading_deg']+180)%360-180
        out['heading_deg'] = a['heading_deg']+f*delta
    return out


def attitude_plane(camera, quaternion, reference=None):
    """Flat-world plane in current body; mount is never rotated a second time.

    Constant normal distance is retained; IMU alone cannot infer body heave.
    A rest reference is an explicit sensitivity variant, not calibrated truth.
    """
    R = rotation(quaternion)
    normal = R.T @ np.array([0.,0.,1.])
    if reference is not None:
        # Remove reference roll/pitch only; a rest yaw is not a road slope.
        roll,pitch,_ = rpy(quaternion)-rpy(reference)
        normal = np.array([-np.sin(pitch),np.cos(pitch)*np.sin(roll),
                           np.cos(pitch)*np.cos(roll)])
    if normal[2] < .8:
        raise ValueError('attitude outside planar candidate domain')
    plane = GroundPlane('base_link', camera.ground_plane.z_at_origin_m/normal[2],
                        -normal[0]/normal[2], -normal[1]/normal[2], 'offline_imu_flat_plane')
    return replace(camera, ground_plane=plane)


def move_xy(xy, source_pose, target_pose):
    """Static ground points from old base_link to new base_link (SE(2))."""
    points = np.asarray(xy,dtype=float).reshape(-1,2)
    a,b = np.deg2rad([source_pose['heading_deg'],target_pose['heading_deg']])
    def R(t): return np.array([[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]])
    return points @ R(a-b).T + R(-b) @ (
        np.asarray(source_pose['position_xyz'][:2])-np.asarray(target_pose['position_xyz'][:2]))


def geometry_evidence(projector, classes, confidence, camera):
    """Sparse ray/plane intersection; same M0 selected pixels and quality."""
    base = projector.extract(classes,confidence,{1:.5,2:.5})
    if not len(base.xy): return base
    native, _ = projector.camera.project_ground(base.xy)
    h = np.column_stack((native,np.ones(len(native)))) @ np.linalg.inv(camera.ground_to_image_homography).T
    xy = h[:,:2]/h[:,2:]
    finite = np.isfinite(xy).all(axis=1)
    depth = camera.ground_to_optical(camera.ground_plane.xyz(xy))[:,2]
    valid = (finite & (depth>.1) & (np.linalg.norm(xy,axis=1)<=22) &
             (xy[:,0]>=-5)&(xy[:,0]<=20)&(np.abs(xy[:,1])<=8))
    return MetricEvidence(np.ascontiguousarray(xy[valid],dtype=np.float32),
                          base.class_id[valid],base.confidence[valid],
                          base.geometric_quality[valid],base.source_mask[valid])


def warped_evidence(value, source_pose, target_pose):
    return replace(value,xy=np.ascontiguousarray(move_xy(value.xy,source_pose,target_pose),dtype=np.float32))


def add_history(current, past, age_ns, same_context=True):
    """Offline-only gap evidence. Current 25cm cells always win; no restamping."""
    if not same_context or age_ns <= 0 or age_ns > 200_000_000:
        return current, 0
    def keys(e):
        cells = np.floor(e.xy/.25).astype(np.int64)
        return list(zip(e.class_id.tolist(),cells[:,0].tolist(),cells[:,1].tolist()))
    present = set(keys(current))
    valid = np.array([key not in present for key in keys(past)])
    valid &= (past.xy[:,0]>=-5)&(past.xy[:,0]<=20)&(np.abs(past.xy[:,1])<=8)
    selected = MetricEvidence(past.xy[valid],past.class_id[valid],past.confidence[valid],
                              past.geometric_quality[valid]*.25,past.source_mask[valid])
    return MetricEvidence.concatenate([current,selected]),int(valid.sum())


class Fitter:
    def __init__(self, config):
        self.config = config
    def fit(self, evidence):
        c = self.config; timing = {}
        def stage(name, call):
            start = time.perf_counter(); result = call()
            timing[name] = 1000*(time.perf_counter()-start)
            return result
        nodes = stage('node_reduction',lambda:reduce_evidence_to_nodes(evidence,c['x_bin_m'],
                      c['lateral_cluster_gap_m'],c['evidence_x_min_m']))
        tracks,_ = stage('association',lambda:associate_lane_nodes_with_audit(nodes,**{
            k:c[k] for k in ('max_track_gap_m','max_lateral_residual_m','max_heading_change_deg',
                             'min_track_nodes','min_observed_span_m')}))
        tracks,_ = stage('prefit',lambda:select_shadow_lane_tracks_with_audit(tracks,c['shadow_boundary']))
        family,audit = stage('shared_fit',lambda:fit_shared_lane_family_with_audit(tracks,c['shared_lane_family']))
        return family,audit,timing


def family_dict(family, audit):
    usage=measure_planning_usability(family,zones=tuple(
        LongitudinalZone('%d-%d'%(a,b),a,b) for a,b in ((0,5),(5,10),(10,15),(15,20))))
    usage={k:v['both_strips'] for k,v in usage['zones'].items()}
    if family is None: return dict(valid=False,reason=audit.get('reason'),boundaries=[],usability=usage)
    return dict(valid=True,reason=audit.get('reason'),widths_m=[family.left_width_m,family.right_width_m],
                residual_p95_m=family.inlier_residual_p95_m,usability=usage,
                boundaries=[dict(role=b.role,class_id=b.class_id,xy=b.sample_points.tolist(),
                                 support=b.sample_state.tolist()) for b in family.boundaries])


def temporal_delta(old, new, old_pose, new_pose):
    """Same-world-location lateral change, shared usable segments only, 0–15m."""
    result = []
    for b in new['boundaries']:
        prior = next((a for a in old['boundaries'] if a['role']==b['role']),None)
        if prior is None: continue
        target = np.asarray(b['xy']); support = np.asarray(b['support'])
        keep = (support!=0)&(target[:,0]>=0)&(target[:,0]<=15)
        query = target[keep]
        for state, segment in contiguous_sample_state_segments(np.asarray(prior['xy']),prior['support']):
            if state=='invalid': continue
            moved = move_xy(segment,old_pose,new_pose)
            moved = moved[np.argsort(moved[:,0])]
            usable = (query[:,0]>=moved[0,0])&(query[:,0]<=moved[-1,0])
            points = query[usable]
            if len(points): result.extend(np.abs(points[:,1]-np.interp(points[:,0],moved[:,0],moved[:,1])).tolist())
    return stats(result)


def reprojection(family, cameras, arrays):
    """Internal semantic consistency, NOT independent image/metric GT error."""
    distances = []
    for view,camera in cameras.items():
        classes = arrays[view+'_class']
        distances_by_class = {i:cv2.distanceTransform((classes!=i).astype(np.uint8),cv2.DIST_L2,5)
                              for i in (1,2) if np.any(classes==i)}
        scale = np.array([classes.shape[1]/camera.width, classes.shape[1]/camera.width])
        # Front has 2px vertical letterbox padding; sides are native.
        pad = np.array([0,(classes.shape[0]-camera.height*scale[1])/2])
        for b in family['boundaries']:
            if b['class_id'] not in distances_by_class: continue
            xy = np.asarray(b['xy']); state = np.asarray(b['support'])
            xy = xy[(state==1)&(xy[:,0]>=0)&(xy[:,0]<=15)]
            uv,depth = camera.project_ground(xy)
            uv = np.rint(uv*scale+pad).astype(int)
            valid = (depth>.1)&(uv[:,0]>=0)&(uv[:,0]<classes.shape[1])&(uv[:,1]>=0)&(uv[:,1]<classes.shape[0])
            for u,v in uv[valid]: distances.append(float(distances_by_class[b['class_id']][v,u]))
    return stats(distances)


def render_family(family, grid, background=None):
    canvas = np.zeros((*grid.shape,3),np.uint8) if background is None else background.copy()
    if background is None: draw_metric_grid(canvas,grid)
    segments = [(b['role'],state,b['class_id'],points) for b in family['boundaries']
                for state,points in contiguous_sample_state_segments(b['xy'],b['support'])]
    draw_stateful_lane_family(canvas,segments,grid,family['reason'] or '')
    return canvas


def overlay(image, family, camera):
    canvas = image.copy()
    for b in family['boundaries']:
        for state,segment in contiguous_sample_state_segments(b['xy'],b['support']):
            if state=='invalid': continue
            pixels,depth = camera.project_ground(segment)
            valid = (depth>.1)&np.isfinite(pixels).all(axis=1)&(pixels[:,0]>=0)&(pixels[:,0]<camera.width)&(pixels[:,1]>=0)&(pixels[:,1]<camera.height)
            # Do not connect across an off-image/behind-camera sample.
            for i in range(len(pixels)-1):
                if valid[i] and valid[i+1]:
                    color = (0,255,255) if b['class_id']==2 else (0,255,0)
                    if state!='observed': color=(0,165,255)
                    cv2.line(canvas,tuple(np.rint(pixels[i]).astype(int)),
                             tuple(np.rint(pixels[i+1]).astype(int)),color,2)
    return canvas


def view_benchmark(families,grid):
    background=np.zeros((*grid.shape,3),np.uint8);draw_metric_grid(background,grid)
    costs=defaultdict(list);parity=True
    # Interleaved pairs avoid systematic warmup/order advantage.
    for i in range(300):
        family=families[i%len(families)];images={}
        for name in (('baseline','cached') if i%2==0 else ('cached','baseline')):
            start=time.perf_counter();canvas=render_family(family,grid,background if name=='cached' else None)
            drawn=time.perf_counter();ok,jpeg=cv2.imencode('.jpg',canvas,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:raise RuntimeError('JPEG failure')
            ended=time.perf_counter();images[name]=canvas
            costs[name+'_render_ms'].append((drawn-start)*1000)
            costs[name+'_encode_ms'].append((ended-drawn)*1000)
            costs[name+'_critical_work_ms'].append((ended-start)*1000)
        parity &= np.array_equal(images['baseline'],images['cached'])
    input_times=np.arange(0,10,1/15);last=-1.;accepted=[]
    for t in input_times:
        if t-last>=.1:last=t;accepted.append(t)
    return dict(stages={k:stats(v) for k,v in costs.items()},pixel_parity=bool(parity),
        callback_throttle_15hz_to_10hz=len(accepted)/10,
        latest_frame_timer_10hz=10,
        note='Offline draw/encode microbenchmark and ideal-clock scheduling calculation. No ROS transport/rqt presentation measurement.')


def run(args):
    tool_sha=sha(Path(__file__))
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True,exist_ok=False)
    grid_pin=json.loads((args.grids.parent/'report.json').read_text())
    continuous_pin=json.loads((args.continuous/'summary.json').read_text()) if args.continuous.exists() else None
    if grid_pin['fixture_sha256']!=sha(args.representatives):raise ValueError('representative fixture SHA mismatch')
    if continuous_pin and continuous_pin['checkpoint_sha256']!=grid_pin['checkpoint_sha256']:
        raise ValueError('continuous and representative checkpoint mismatch')
    config_root=args.config
    config=yaml.safe_load((config_root/'bev/lane_fitting.yaml').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((config_root/'cameras_v3.yaml').read_text()))
    grid=BevGridSpec.from_mapping(yaml.safe_load((config_root/'bev/bev_grid.yaml').read_text()))
    projectors={v:ModelGridGroundProjector(cameras[v],model_hw=(544,960) if v=='front' else (480,640),
                  source_mask=1<<i,max_ground_range_m=22) for i,v in enumerate(VIEWS)}
    fitter=Fitter(config)
    signals=read_rows(args.inputs/'signals.jsonl'); groups=defaultdict(lambda:defaultdict(list))
    for row in signals: groups[row['run_id']][row['topic']].append(row)
    regressions={run:{topic:sum(b['source_ns']<=a['source_ns'] for a,b in zip(samples,samples[1:]))
                      for topic,samples in topics.items()} for run,topics in groups.items()}
    for topics in groups.values():
        for samples in topics.values():samples.sort(key=lambda x:x['source_ns'])
    representatives=read_rows(args.representatives)
    rep_index=Counter();frames=[]
    for row in representatives:
        index=rep_index[row['run_id']];rep_index[row['run_id']]+=1
        frames.append(dict(row,split='representative',index=index,
                           grid=str(args.grids/(row['run_id']+'-%03d.npz'%index))))
    if args.continuous.exists():
        for row in read_rows(args.continuous/'manifest.jsonl'):
            path=args.continuous/row['grid']
            if sha(path)!=row['grid_sha256']:raise ValueError('continuous grid SHA mismatch')
            frames.append(dict(row,split='continuous',grid=str(path)))
    references={};signal_report={}
    for run,topics in sorted(groups.items()):
        im=topics['/imu'];eg=topics['/Ego_topic'];gps=topics['/gps']
        angles=np.rad2deg(np.array([rpy(s['q_xyzw']) for s in im]))
        yaw_errors=[]
        for e in eg:
            q=interpolate(im,e['source_ns'],'imu')
            if q:yaw_errors.append(abs((np.rad2deg(rpy(q['q_xyzw'])[2])-e['heading_deg']+180)%360-180))
        stationary=[]
        for s in im:
            e=min(eg,key=lambda e:abs(e['source_ns']-s['source_ns']))
            if abs(e['source_ns']-s['source_ns'])<=50_000_000 and np.linalg.norm(e['velocity_xyz'][:2])<.2:
                stationary.append(s['q_xyzw'])
        # Explicit per-run rest sensitivity reference; no hidden calibration fit.
        references[run]=stationary[0] if len(stationary)>=5 else None
        signal_report[run]=dict(roll_deg=stats(angles[:,0]),pitch_deg=stats(angles[:,1]),
            yaw_ego_imu_abs_delta_deg=stats(yaw_errors),rest_samples=len(stationary),
            rest_reference_q=references[run],
            gps_zero_samples=sum(s['latitude']==0 or s['longitude']==0 for s in gps),gps_samples=len(gps),
            source_minus_bag_ms={t:stats([(s['source_ns']-s['bag_ns'])/1e6 for s in rows])
                                 for t,rows in topics.items()},
            available_topics=list(topics),
            source_nonincreasing_counts=regressions[run],
            production_motion_input='unverified: Ego is development-only; GPS-invalid bridge VehicleState is invalid')
    rgb_index={(r['run_id'],r['front_ns'],r['view']):r for r in read_rows(args.inputs/'rgb-index.jsonl')}
    records=[];history={};previous={};gallery=[];render_samples=[]
    for number,frame in enumerate(frames):
        run=frame['run_id'];stamp=frame['views']['front']['source_ns'];topics=groups[run]
        arrays=dict(np.load(frame['grid']));poses={v:interpolate(topics['/Ego_topic'],frame['views'][v]['source_ns'],'pose') for v in VIEWS}
        quats={v:interpolate(topics['/imu'],frame['views'][v]['source_ns'],'imu') for v in VIEWS}
        target=max(frame['views'][v]['source_ns'] for v in VIEWS)
        target_pose=interpolate(topics['/Ego_topic'],target,'pose')
        variants={};base_values={};result_families={}
        for variant in VARIANTS:
            if variant not in ('M0','M2_align_only') and regressions[run].get('/imu',0):continue
            if variant.startswith('M2') and regressions[run].get('/Ego_topic',0):continue
            if variant not in ('M0','M2_align_only') and not all(quats.values()):continue
            if variant=='M1_rest' and references[run] is None:continue
            if variant.startswith('M2') and (not all(poses.values()) or target_pose is None):continue
            begin=time.perf_counter();cpu_start=time.process_time();view_e=[];used_cameras={}
            for v in VIEWS:
                cam=cameras[v]
                if variant not in ('M0','M2_align_only'):cam=attitude_plane(cam,quats[v]['q_xyzw'],references[run] if variant=='M1_rest' else None)
                used_cameras[v]=cam
                if variant in ('M0','M2_align_only'):
                    evidence=projectors[v].extract(arrays[v+'_class'],arrays[v+'_confidence'],{1:.5,2:.5})
                    base_values[v]=evidence
                else:evidence=geometry_evidence(projectors[v],arrays[v+'_class'],arrays[v+'_confidence'],cam)
                if variant.startswith('M2'):evidence=warped_evidence(evidence,poses[v],target_pose)
                view_e.append(evidence)
            current=MetricEvidence.concatenate(view_e);current_count=len(current.xy);past_count=0;past_age_ms=None
            key=(run,frame['split'],frame.get('sequence','representative'))
            if variant=='M2_history' and frame['split']=='continuous' and key in history:
                last_stamp,last_pose,last_e=history[key]
                warped=warped_evidence(last_e,last_pose,target_pose)
                current,past_count=add_history(current,warped,target-last_stamp)
                if past_count:past_age_ms=(target-last_stamp)/1e6
            projected=time.perf_counter()
            family,audit,timings=fitter.fit(current)
            timings.update(projection_ms=(projected-begin)*1000,total_ms=(time.perf_counter()-begin)*1000,
                           cpu_ms=(time.process_time()-cpu_start)*1000)
            result=family_dict(family,audit)
            result['plane_coefficients']=used_cameras['front'].ground_plane.coefficients
            result.update(timings_ms=timings,history_pixels=past_count,
                          history_age_ms=past_age_ms,current_pixels=current_count,
                          support_authority=('offline historical-fit labels, not fresh observed evidence'
                                             if variant=='M2_history' else 'same-frame-only'),
                          semantic_consistency_px=reprojection(result,used_cameras,arrays) if not variant.startswith('M2') else None)
            if variant=='M2_align': aligned_current=current
            if variant=='M2_history' and frame['split']=='continuous':history[key]=(target,target_pose,aligned_current)
            if target_pose and frame['split']=='continuous':
                previous_key=(key,variant)
                if previous_key in previous:
                    old_stamp,old_pose,old_result=previous[previous_key]
                    if 0<target-old_stamp<=200_000_000:
                        result['same_world_temporal_delta_m']=temporal_delta(old_result,result,old_pose,target_pose)
                previous[previous_key]=(target,target_pose,result)
            variants[variant]=result;result_families[variant]=(family,used_cameras)
        row=dict(run_id=run,phase=frame['phase'],split=frame['split'],source_ns=stamp,
                 target_ns=target,skew_ms=(target-min(s['source_ns'] for s in frame['views'].values()))/1e6,
                 grid_sha256=sha(frame['grid']),variants=variants)
        records.append(row);render_samples.append(variants['M0'])
        # One midpoint per condition/phase, selected independently of result.
        if frame['split']=='representative' and frame['index']%8==4:
            panels=[]
            for v in ('left','front','right'):
                src=rgb_index[run,stamp,v];blob=(args.inputs/src['path']).read_bytes()
                if hashlib.sha256(blob).hexdigest()!=src['sha256']:raise ValueError('RGB SHA mismatch')
                image=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
                for variant in ('M0','M1_absolute'):
                    if variant not in variants:continue
                    panel=cv2.resize(overlay(image,variants[variant],result_families[variant][1][v]),(480,270))
                    cv2.putText(panel,v+' '+variant,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,100,255),2)
                    panels.append(panel)
            image=np.vstack([np.hstack(panels[i:i+2]) for i in range(0,len(panels),2)])
            bevs=[]
            for variant in ('M0','M1_absolute'):
                canvas=render_family(variants[variant],grid)
                origin=tuple(np.rint(grid.ego_pixel).astype(int))
                cv2.arrowedLine(canvas,origin,(origin[0],origin[1]-30),(0,255,0),2)
                cv2.putText(canvas,'LEFT',(3,95),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,180,255),1)
                cv2.putText(canvas,'RIGHT',(110,95),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,180,255),1)
                cv2.putText(canvas,'EGO',origin,cv2.FONT_HERSHEY_SIMPLEX,.3,(0,255,0),1)
                bevs.append(cv2.resize(canvas,(480,750),interpolation=cv2.INTER_NEAREST))
            image=np.vstack((image,np.hstack(bevs)))
            filename=run+'-'+frame['phase']+'.jpg';cv2.imwrite(str(args.output/filename),image)
            gallery.append(dict(path=filename,run_id=run,phase=frame['phase']))
        if number%24==23:print('family comparison',number+1,'/',len(frames),flush=True)
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r,sort_keys=True,allow_nan=False)+'\n' for r in records))
    summaries={}
    for split in ('representative','continuous'):
        selected=[r for r in records if r['split']==split]
        summaries[split]={}
        for variant in VARIANTS:
            rows=[r['variants'][variant] for r in selected if variant in r['variants']]
            summaries[split][variant]=dict(frames=len(rows),valid=sum(r['valid'] for r in rows),
                skipped_frames=len(selected)-len(rows),
                newly_invalid_vs_M0=sum(r['variants']['M0']['valid'] and not r['variants'][variant]['valid']
                                        for r in selected if variant in r['variants']),
                reasons=dict(Counter(r['reason'] for r in rows)),
                stages_ms={k:stats([r['timings_ms'][k] for r in rows]) for k in rows[0]['timings_ms']} if rows else {},
                temporal_pair_p95_m=stats([r['same_world_temporal_delta_m']['p95'] for r in rows
                                          if r.get('same_world_temporal_delta_m',{}).get('count')]),
                semantic_frame_p95_px=stats([r['semantic_consistency_px']['p95'] for r in rows
                                            if r.get('semantic_consistency_px',{} ) and r['semantic_consistency_px']['count']]),
                history_frames=sum(r['history_pixels']>0 for r in rows))
            summaries[split][variant]['support_zones']={z:{k:stats([r['usability'][z][k] for r in rows])
                for k in ('observed_length_m','degraded_length_m','available_length_m',
                          'longest_internal_unusable_gap_m','trailing_unusable_gap_m')}
                for z in ('0-5','5-10','10-15','15-20')}
    orientation=dict(convention='base_link +x forward/up; +y vehicle-left/image-left',
        asymmetric_points_xy=[[10,3],[5,-2],[0,0]],
        bev_pixels=grid.metric_to_pixel([[10,3],[5,-2],[0,0]]).tolist(),
        front_pixels=cameras['front'].project_ground(np.array([[10,3],[5,-2]]))[0].tolist(),
        left_mount_yaw_deg=cameras['left'].rotation_deg[2],right_mount_yaw_deg=cameras['right'].rotation_deg[2],
        finding='No algebraic mirror in canonical projection/rendering. Real same-stamp RGB overlays require visual review; no live view node was running at inspection.')
    benchmark=view_benchmark(render_samples,grid)
    orientation_canvas=np.zeros((*grid.shape,3),np.uint8);draw_metric_grid(orientation_canvas,grid)
    for point,label,color in (([10,3],'LEFT +y',(0,255,0)),([5,-2],'RIGHT -y',(0,100,255)),([0,0],'EGO',(255,255,255))):
        pixel=tuple(np.rint(grid.metric_to_pixel([point])[0]).astype(int))
        cv2.circle(orientation_canvas,pixel,3,color,-1)
        cv2.putText(orientation_canvas,label,(max(0,pixel[0]-35),pixel[1]-8),cv2.FONT_HERSHEY_SIMPLEX,.3,color,1)
    cv2.imwrite(str(args.output/'orientation.jpg'),cv2.resize(orientation_canvas,(480,750),interpolation=cv2.INTER_NEAREST))
    report=dict(schema='lane_motion_review_v1',summary=summaries,signals=signal_report,orientation=orientation,
                visualization=benchmark,frames=len(records),gallery=gallery,
                inputs_sha256=sha(args.inputs/'summary.json'),representatives_sha256=sha(args.representatives),
                config_sha256={str(p.relative_to(config_root)):sha(p) for p in
                               (config_root/'cameras_v3.yaml',config_root/'bev/lane_fitting.yaml',config_root/'camera/lane_input.yaml')},
                tool_sha256=tool_sha,checkpoint_sha256=grid_pin['checkpoint_sha256'],
                automatic_promotion=False,live_acceptance=False,
                limitations=['M1 freezes baseline pixel selection/geometric weights: geometry sensitivity, not complete dynamic projector.',
                             'M1 absolute assumes IMU world z equals road normal; rest variant is sensitivity only.',
                             'M2 uses development Ego pose; production tunnel speed/odometry remains unverified.',
                             'Past support is recorded separately and never serialized as a fresh observed ROS family.',
                             'Temporal change/semantic consistency are not independent geometric GT accuracy.',
                             'Temporal curve comparison and M2 motion are SE(2); full SE(3) body/height alignment is not validated.',
                             'Continuous grids are CPU FP32, not newly measured TensorRT outputs.'])
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    table='<table border="1"><tr><th>Input</th><th>Variant</th><th>Valid / tested</th><th>New invalid</th><th>Compute p95 ms</th><th>Temporal pair p95 median m</th></tr>'+''.join(
        '<tr><td>'+split+'</td><td>'+variant+'</td><td>'+str(v['valid'])+'/'+str(v['frames'])+'</td><td>'+str(v['newly_invalid_vs_M0'])+'</td><td>'+str(round(v['stages_ms'].get('total_ms',{}).get('p95',0),2))+'</td><td>'+str(v['temporal_pair_p95_m'].get('p50','-'))+'</td></tr>'
        for split,variants in summaries.items() for variant,v in variants.items())+'</table>'
    (args.output/'index.html').write_text('<meta charset="utf-8"><title>Lane motion / orientation review</title><style>body{font:16px sans-serif;max-width:1100px;margin:24px auto;background:#101720;color:#ddd}img{max-width:100%}a{color:#7ccfff}pre{white-space:pre-wrap}td,th{padding:8px}</style><h1>Lane motion / orientation review</h1><p>Offline analysis only. No runtime defaults changed. Past evidence / Ego reference is not a fresh observed ROS output.</p><p><a href="report.json">Full report</a> · <a href="frames.jsonl">Per-frame evidence</a></p>'+table+'<h2>Orientation fixture</h2><img src="orientation.jpg">'+''.join('<h2>'+html.escape(g['run_id']+' '+g['phase'])+'</h2><img loading="lazy" src="'+g['path']+'">' for g in gallery))
    print(json.dumps(summaries,indent=2),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','representatives','grids','continuous','config','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    run(parser.parse_args())


if __name__=='__main__':main()
