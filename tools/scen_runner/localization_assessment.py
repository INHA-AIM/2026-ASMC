"""Full-timeline M2 gates. Input reference rows come from the native C++ auditor.

No plan validity filter is applied before measuring native tracking error.
Receive-clock pairing is bounded, reported, and is not source-clock calibration.
"""
import bisect
import math
from collections import Counter


def phase_speed_metrics(native,modes,shadow_cap,normal_ceiling,grace=5.):
    """Full-timeline mode-specific ceilings; never hide shadow overspeed in NORMAL."""
    modes=sorted(modes,key=lambda r:r['ros_time']);mt=[r['ros_time'] for r in modes]
    shadow=[];errors=[];violations=unknown=0
    for n in native:
        i=bisect.bisect_right(mt,n['ros_time'])-1
        mode=modes[i]['value'] if i>=0 else None;speed=n['ego']['speed_kmh']
        if mode not in (0,1):unknown+=1;continue
        ceiling=shadow_cap if mode==1 else normal_ceiling
        violations+=not math.isfinite(speed) or speed>ceiling+1.
        if mode==1:
            shadow.append(speed)
            if n['monotonic']-native[0]['monotonic']>=grace:errors.append(abs(speed-shadow_cap))
    errors.sort()
    return dict(speed_violation_samples=violations,speed_unknown_mode_samples=unknown,
        shadow_speed_max_kph=max(shadow) if shadow else None,
        shadow_speed_error_after_5s_p95_kph=errors[round(.95*(len(errors)-1))] if errors else None)


def shadow_timeline_coverage(native, modes, decisions, maximum_trace_age=.15):
    """Carry change-only mode forward; pair every shadow pose causally.

    A mode has no heartbeat/age contract. A trajectory trace does: future or
    stale decisions cannot repair a missing/invalid interval retroactively.
    Stopped poses stay in coverage and invalid counts, with motion reported
    separately so intentional bounded obstacle stops remain visible.
    """
    modes=sorted(modes,key=lambda r:r['ros_time'])
    decisions=sorted(decisions,key=lambda r:r['ros_time'])
    mt=[r['ros_time'] for r in modes];dt=[r['ros_time'] for r in decisions]
    known=shadow=moving=paired=valid_moving=invalid=0
    for n in native:
        t=n['ros_time'];mi=bisect.bisect_right(mt,t)-1
        mode=modes[mi]['value'] if mi>=0 else None
        known+=mode in (0,1)
        if mode!=1:continue
        shadow+=1;is_moving=n['ego']['speed_kmh']>.5;moving+=is_moving
        di=bisect.bisect_right(dt,t)-1
        trace=decisions[di]['value'] if di>=0 and 0<=t-dt[di]<=maximum_trace_age else None
        paired+=trace is not None
        valid=bool(isinstance(trace,dict) and trace.get('valid'));valid_moving+=is_moving and valid
        invalid+=not valid
    return dict(mode_coverage_ratio=known/max(1,len(native)),shadow_native_samples=shadow,
        shadow_moving_native_samples=moving,shadow_paired_samples=paired,
        invalid_or_missing_shadow_samples=invalid,
        shadow_pairing_ratio=paired/max(1,shadow),
        shadow_valid_moving_ratio=valid_moving/max(1,moving))


def maximum_stopped_interval(native,grace_seconds=5.,threshold_kph=.5):
    """Measured mid-run dwell; initial departure waiting is reported separately."""
    if not native:return None
    start=None;maximum=0.;origin=native[0]['monotonic']
    for row in native:
        t=row['monotonic']
        if t-origin<grace_seconds:continue
        if row['ego']['speed_kmh']<=threshold_kph:
            if start is None:start=t
            maximum=max(maximum,t-start)
        elif start is not None:
            maximum=max(maximum,t-start);start=None
    return maximum


def completed_gps_recovery(decisions):
    """A continuous measured recovery, excluding a stopped reanchor/restart.

    Seeing the right source names somewhere in a run is insufficient: an
    unavailable epoch between blend and GPS must never count as completion.
    """
    expected=('gps_anchored','dead_reckoning','recovery_verify','recovery_blend','gps_anchored')
    stages=[];last=None;epoch=None
    for row in decisions:
        stamp=row['stamp'];source=row.get('source')
        current=tuple(row.get(k) for k in ('motion_epoch','anchor_epoch'))
        if last is not None and (stamp<last or stamp-last>.35 or current!=epoch):stages=[]
        last=stamp;epoch=current
        if not stages:
            if source==expected[0]:stages=[stamp]
            continue
        if source==expected[len(stages)-1]:continue
        if len(stages)==1 and source in ('recovery_verify','recovery_blend'):
            continue # tracking-noise verification still retains a validated anchor
        if len(stages) in (3,4) and source=='dead_reckoning':
            stages=[stages[0],stamp];continue # loss during validation must validate again
        if source==expected[len(stages)]:
            stages.append(stamp)
            if len(stages)==len(expected):
                if stages[3]-stages[2]>=.8 and stages[4]-stages[3]>=.8:
                    return dict(sources=list(expected),stamps=stages)
                stages=[stamp]
        else:stages=[stamp] if source==expected[0] else []
    return None


def native_static_clearance(x,y,yaw,actor):
    """Conservative native-only audit: projected bounding sphere contains OBB.

    Ego uses the same 2.160835 x 4.635 rear-axle footprint as the map auditor.
    This lower bound never enters planning and does not assume obstacle yaw.
    """
    dx,dy=actor['box_center'][0]-x,actor['box_center'][1]-y
    c,s=math.cos(yaw),math.sin(yaw)
    longitudinal=c*dx+s*dy;lateral=-s*dx+c*dy
    dx=max(-.79-longitudinal,0.,longitudinal-(4.635-.79))
    dy=max(abs(lateral)-2.160835/2,0.)
    return math.hypot(dx,dy)-math.sqrt(sum(v*v for v in actor['box_extents']))


def native_fixture_polygon(actor,grid_yaw):
    """Independent projection of all eight SDK box corners, evaluation only.

    SDK extent.y is body-forward half-length and extent.x half-width, as in
    the scenario runner's measured obstacle catalogue. Include height/tilt.
    """
    from shapely.geometry import MultiPoint
    roll=math.radians(actor['pose']['roll']);pitch=math.radians(actor['pose']['pitch'])
    cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    c,s=math.cos(grid_yaw),math.sin(grid_yaw);cx,cy=actor['box_center'][:2]
    width,length,height=actor['box_extents'];points=[]
    for x in (-length,length):
        for y in (-width,width):
            for z in (-height,height):
                bx=cp*x+sp*(sr*y+cr*z);by=cr*y-sr*z
                points.append((cx+c*bx-s*by,cy+s*bx+c*by))
    return MultiPoint(points).convex_hull


def native_ego_polygon(x,y,yaw):
    from shapely.geometry import Polygon
    c,s=math.cos(yaw),math.sin(yaw)
    return Polygon([(x+c*a-s*b,y+s*a+c*b) for a,b in
                    ((-.79,-2.160835/2),(4.635-.79,-2.160835/2),
                     (4.635-.79,2.160835/2),(-.79,2.160835/2))])


def moving_fixture_evidence(native,frames,grid_yaw,tolerance=.15):
    """Time-paired SDK boxes are independent evaluation, never planner input.

    Missing actor frames cannot become an empty, collision-free world. OBB
    separation is reported separately from native/UDP contact evidence.
    """
    frames=sorted(frames,key=lambda f:f['ros_time']);stamps=[f['ros_time'] for f in frames]
    clearances=[];paired=0;positions={};speeds={}
    for row in frames:
        for a in row['actors']:
            positions.setdefault(a['id'],[]).append((a['pose']['x'],a['pose']['y']))
            speeds.setdefault(a['id'],[]).append(a['speed_kmh'])
    for n in native:
        row=nearest(frames,stamps,n['ros_time'],tolerance)
        if not row or not row['actors']:continue
        paired+=1;p=n['ego']['pose'];ego=native_ego_polygon(p['x'],p['y'],grid_yaw(p['x'],p['y'],p['yaw']))
        for a in row['actors']:
            p=a['pose'];box=native_fixture_polygon(a,grid_yaw(p['x'],p['y'],p['yaw']))
            clearances.append(ego.distance(box))
    return dict(fixture_pairing_ratio=paired/max(1,len(native)),
        native_obb_clearance_min_m=min(clearances) if clearances else None,
        fixture_max_displacement_m={k:max(math.dist(p,ps[0]) for p in ps) for k,ps in positions.items()},
        fixture_speed_max_kph={k:max(v) for k,v in speeds.items()},
        fixture_truth_scope='time-paired native OBB evaluation only; not mesh contact certification')


def confirm_static_return(native,reference,fixtures,exit_xy,exit_direction):
    """Require 0.5s measured return after every fixture, before the exit plane."""
    if not fixtures:return None
    start=None
    for n,r in zip(native,reference):
        p=n['ego']['pose'];x,y=p['x'],p['y']
        before=(x-exit_xy[0])*exit_direction[0]+(y-exit_xy[1])*exit_direction[1]
        passed=all((x-a['box_center'][0])*exit_direction[0]+(y-a['box_center'][1])*exit_direction[1]>10.
                   for a in fixtures)
        valid=(int(r['projected']) and passed and before<=-(4.635-.79+.5) and
               abs(float(r['lateral_m']))<=.25 and abs(float(r['heading_deg']))<=3)
        if not valid:start=None;continue
        if start is None:start=n['monotonic']
        if n['monotonic']-start>=.5:
            return dict(monotonic=n['monotonic'],distance_before_exit_m=-before,
                        native_x=x,native_y=y)
    return None


def quantile(values, q=.95):
    values=sorted(v for v in values if v is not None and math.isfinite(v))
    return values[min(len(values)-1, round((len(values)-1)*q))] if values else None


def nearest(rows, stamps, t, tolerance=.15):
    i=bisect.bisect_left(stamps,t)
    candidates=[j for j in (i-1,i) if 0<=j<len(stamps)]
    if not candidates:return None
    j=min(candidates,key=lambda j:abs(stamps[j]-t))
    return rows[j] if abs(stamps[j]-t)<=tolerance else None


def assess(events, decisions, reference, receipts, result, recording):
    decisions=sorted(decisions,key=lambda r:r['stamp'])
    stamps=[r['stamp'] for r in decisions]
    sources=[r.get('source') for r in decisions]
    matched=valid_moving=moving=0
    paired=[]; position_errors=[]
    for e in events:
        r=nearest(decisions,stamps,e['ros_time']);paired.append(r)
        matched+=r is not None
        if e['ego']['speed_kmh']>.5:
            moving+=1;valid_moving+=bool(r and r.get('valid'))
        if r and r.get('anchor_ready',r.get('source') in ('gps_anchored','dead_reckoning')):
            p=e['ego']['pose'];position_errors.append(math.hypot(r['map_x']-p['x'],r['map_y']-p['y']))
    # Missing/stopped/invalid periods remain in this denominator. Native errors
    # are evaluated at every independently captured pose, including invalid plans.
    coverage=matched/len(events) if events else 0
    moving_valid=valid_moving/moving if moving else 0
    projected=[r for r in reference if int(r['projected'])]
    lateral=[abs(float(r['lateral_m'])) for r in projected]
    heading=[abs(float(r['heading_deg'])) for r in projected]
    speed=[e['ego']['speed_kmh'] for e in events]
    cap=result.get('cap_kph',0)
    speed_error=[abs(e['ego']['speed_kmh']-cap) for e in events
                 if e['monotonic']-events[0]['monotonic']>=5]
    times=sorted(r['monotonic'] for r in receipts if events and
                 events[0]['monotonic']<=r['monotonic']<=events[-1]['monotonic'])
    gaps=[b-a for a,b in zip(times,times[1:])]
    if times:gaps += [times[0]-events[0]['monotonic'],events[-1]['monotonic']-times[-1]]
    transition=0
    for source in sources:
        expected=('gps_anchored','dead_reckoning','gps_anchored')[min(transition,2)]
        if transition<3 and source==expected:transition+=1
    costs=[r.get('processing_ms',float('nan')) for r in decisions]
    metrics=dict(native_samples=len(events),reference_samples=len(projected),
        time_pairing_ratio=coverage,valid_moving_ratio=moving_valid,
        invalid_or_missing_native_samples=sum(not r or not r.get('valid') for r in paired),
        missing_native_samples=len(events)-matched,
        stopped_native_samples=len(events)-moving,
        lateral_p95_m=quantile(lateral),lateral_max_m=quantile(lateral,1),
        heading_p95_deg=quantile(heading),position_pair_error_p95_m=quantile(position_errors),
        speed_overshoot_kph=max(speed,default=0)-cap,speed_error_after_5s_p95_kph=quantile(speed_error),
        command_max_gap_s=max(gaps) if gaps else None,reference_p95_ms=quantile(costs),
        sources=dict(Counter(sources)),reasons=dict(Counter(r.get('reason') for r in decisions)),
        gps_loss_dr_recovery=transition==3,
        recovery_policy_trace_present=all('recovery_state' in r for r in decisions) and bool(decisions))
    def le(value, bound):return value is not None and math.isfinite(value) and value<=bound
    gates=dict(measured_motion=moving>=20,completed_distance=result.get('reason')=='distance_limit',
        no_contact='native_contacts' in result and not result['native_contacts'],
        restored='cleanup_errors' in result and not result['cleanup_errors'],
        full_native_reference=len(projected)==len(events) and len(events)>0,
        footprint_inside_map=bool(projected) and all(int(r['map_inside']) for r in projected),
        lateral_p95=le(metrics['lateral_p95_m'],.25),lateral_max=le(metrics['lateral_max_m'],.5),
        heading_p95=le(metrics['heading_p95_deg'],3),pairing=coverage>=.95,
        valid_moving=moving_valid>=.95,gps_transition=transition==3,
        validated_recovery=metrics['recovery_policy_trace_present'] and completed_gps_recovery(decisions) is not None,
        speed_overshoot=le(metrics['speed_overshoot_kph'],1),steady_speed=le(metrics['speed_error_after_5s_p95_kph'],.5),
        control_gap=le(metrics['command_max_gap_s'],.25),reference_cost=le(metrics['reference_p95_ms'],50),
        recording=all(recording.get(k) is True for k in
                      ('counts_match','no_topic_recorded_by_both_groups','no_active_bag_files')))
    return dict(metrics=metrics,gates=gates,stage_passed=all(gates.values()),
                evaluation_basis='native poses independently projected on immutable Frenet reference; all native samples; bounded receive-clock pairing')
