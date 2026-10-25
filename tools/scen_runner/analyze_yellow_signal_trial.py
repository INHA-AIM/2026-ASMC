#!/usr/bin/env python3
"""Read recorded trials; separate physical driving acceptance and failure response.

Does not control MORAI, change a result.json, or use simulator truth as policy
input. Response delays are notification upper bounds for source-state detection,
not a claim that constant deceleration was delivered throughout braking.
"""
import argparse
import bisect
import json
import math
import hashlib
from pathlib import Path


def records(path):
    if path.exists():
        with path.open() as stream:
            for line in stream:
                if line.strip():yield json.loads(line)


def failure_response(evaluation,events,statuses):
    crossings=evaluation.get('signal_violations_or_uncertain',[])
    if not crossings:return dict(accepted=False,reason='no independent unavoidable crossing evidence')
    crossing=min(crossings,key=lambda x:x['begin'])
    before=[s for s in statuses if s['monotonic']<=crossing['begin'] and s.get('brake',0)>=.99]
    kinds={e['kind'] for e in events}
    # Target failure and either confirmed crossing or explicit uncertainty
    # must survive. Ground truth must not turn an uncertain policy observation
    # into a fabricated confirmed physical-line event.
    retained=1 in kinds and bool(kinds & {2,3})
    first_failure=min((e['stamp'] for e in events if e['kind'] in (1,2,3)),default=math.inf)
    reclassified=any(e['kind']==0 and e['stamp']>=first_failure for e in events)
    clean=evaluation.get('road_departure_samples',0)==0 and not evaluation.get('collisions') and not evaluation.get('observation_gaps')
    return dict(accepted=bool(before) and retained and not reclassified and clean,
        maximum_brake_before_crossing=bool(before),overshoot_records_retained=retained,
        confirmed_crossing_recorded=2 in kinds,uncertainty_recorded=3 in kinds,
        normal_reclassification=reclassified,other_physical_failures=not clean,
        violation_free_driving=False)


def physical_response(request, oracle):
    """Independent actual-state evidence, separate from Controller reports.

    Acceleration.x is longitudinal body acceleration in this MORAI build.
    RPC completion bounds detection from above, including sampling/transport;
    it does not identify the exact physical response onset.
    """
    decision=request['monotonic']-(request['ros_time']-request['brake_decision_ros'])
    before=[r for r in oracle if r.get('state_query_interval', [math.inf,math.inf])[1]<=decision]
    if not before:return dict(response_upper_s=None,reason='no pre-decision actual state')
    baseline=before[-1];interval=baseline['state_query_interval']
    acceleration=baseline.get('reported_acceleration',{}).get('x')
    if decision-interval[1]>.3 or acceleration is None:
        return dict(response_upper_s=None,reason='stale or incomplete actual baseline')
    for row in oracle:
        query=row.get('state_query_interval')
        actual_a=row.get('reported_acceleration',{}).get('x')
        if not query or query[0]<decision or query[1]-decision>2 or actual_a is None:continue
        maximum_applied=request.get('brake_level')!=2 or row.get('actual_actuation',{}).get('brake',0)>=.99
        ordinary=request.get('brake_level')==1
        threshold=-.2 if ordinary else min(-.2,acceleration-.2,request.get('brake_acceleration',acceleration)-.2)
        if maximum_applied and row['speed']<=min(baseline['speed'],request.get('brake_speed',baseline['speed']))-.02 and \
           actual_a<=threshold:
            return dict(response_upper_s=query[1]-decision,baseline_age_s=decision-interval[1],
                response_kind='continued_ordinary_deceleration' if ordinary and acceleration<=-.2 else
                    'ordinary_onset' if ordinary else 'additional_maximum_deceleration',
                actual_baseline_speed_mps=baseline['speed'],actual_baseline_acceleration_mps2=acceleration,
                response_speed_mps=row['speed'],response_acceleration_mps2=actual_a,
                response_query_interval_s=query[1]-query[0])
    return dict(response_upper_s=None,reason='no qualifying actual response in two seconds')


def held_without_permission(ros,oracle):
    """A deadline alone is insufficient; require a continuous actual held tail."""
    if not oracle:return False
    end=oracle[-1]['monotonic'];begin=end-2.
    actual=[r for r in oracle if begin<=r['monotonic']<=end]
    status=[r for r in ros if r.get('topic')=='status' and begin<=r['monotonic']<=end]
    policy=[r for r in ros if r.get('topic')=='policy' and begin<=r['monotonic']<=end]
    def covered(rows):
        return len(rows)>=2 and rows[-1]['monotonic']-rows[0]['monotonic']>=1.8 and \
            all(0<b['monotonic']-a['monotonic']<=.3 for a,b in zip(rows,rows[1:]))
    if not all(covered(rows) for rows in (actual,status,policy)):return False
    return all(r['speed']<=.15 and (r.get('actual_actuation',{}).get('brake') or 0)>=.99 for r in actual) and \
        all(r.get('brake',0)>=.99 and r.get('accel',1)<=0 for r in status) and \
        all(r['data'].get('motion')=='RECOVERY_BRAKE' and r['data'].get('green_count',3)<3 and
            r['data'].get('severity',0)>=1 for r in policy)


def red_exit_validation(ros,oracle,signals,binding):
    from scen_runner.runtime.signal_trial_oracle import red_recovery_exit_proof
    exits=[r for r in ros if r.get('topic')=='policy' and r['data'].get('motion')=='EXITED' and
           r['data'].get('stop_line')==binding['stop_node_id']]
    milestones=[s for s in signals if s.get('event')=='recovery_red_exit_confirmed']
    greens=[s for s in signals if s.get('event')=='signal_set_verified' and
            s.get('light_id')==binding['traffic_light_id'] and s.get('color')==binding['green_color'] and
            oracle and s.get('requested_monotonic',0)>oracle[0]['monotonic']]
    if milestones:
        for event in milestones:
            policy=next((r['data'] for r in exits if
                r['data'].get('route_generation')==event.get('policy_route_generation') and
                abs(r['data'].get('decision_ros',0)-event.get('policy_decision_ros',-1))<1e-6),None)
            actual=next((r for r in reversed(oracle) if r['monotonic']<=event['monotonic']),None)
            proof=red_recovery_exit_proof(binding,actual,policy) if actual and policy else None
            if proof and greens and min(s['requested_monotonic'] for s in greens)>=event['monotonic'] and \
               abs(proof['actual_rear_body_exit_clearance_m']-event['actual_rear_body_exit_clearance_m'])<1e-6:
                return dict(verified=True,mode='verified_exit_then_post_exit_green',**proof)
        return dict(verified=False,reason='milestone/body/policy/phase order does not match')
    # Earlier passing harness retained the stronger red stimulus all the way
    # to the +15m endpoint. Replay its original rows; never rewrite its result.
    observed=[r['aspects'][binding['traffic_light_id']] for r in oracle if binding['traffic_light_id'] in r.get('aspects',{})]
    if greens or not observed or any(color!=1 for color in observed):
        return dict(verified=False,reason='red was not retained until a verified exit')
    for policy in exits:
        for actual in oracle:
            if actual['monotonic']<policy['monotonic']:continue
            proof=red_recovery_exit_proof(binding,actual,policy['data'])
            if proof:return dict(verified=True,mode='red_retained_through_endpoint',**proof)
    return dict(verified=False,reason='missing full-body exit or policy EXITED proof')


def analyze(directory):
    from scen_runner.runtime.signal_trial_oracle import box_projection,footprint
    result=json.loads((directory/'result.json').read_text())
    ros=list(records(directory/'ros.jsonl'));oracle=list(records(directory/'oracle.jsonl'))
    requests={};statuses=[];states=[];events=[];motions=[];causes=set();failures=set()
    for row in ros:
        topic=row.get('topic')
        if topic=='request' and row.get('brake_level'):
            key=(tuple(row['token'][:2]),row['brake_generation'],row['brake_level'])
            requests.setdefault(key,row)
        elif topic=='status':
            statuses.append(row)
            if row.get('failure_mask'):failures.add(row['failure_mask'])
        elif topic=='state' and row.get('valid'):states.append(row)
        elif topic=='policy':
            policy=row['data'];events.extend(policy.get('new_events',[]))
            if policy.get('motion') and (not motions or policy['motion']!=motions[-1]):motions.append(policy['motion'])
            for name,level in policy.get('causes',{}).items():
                if level:causes.add(name)
    delays=[]
    for (epochs,generation,level),request in requests.items():
        if request['brake_speed']<=.15 and request['brake_acceleration']<=.2:continue
        observed=[s for s in statuses if tuple(s['token'][:2])==epochs and
            s.get('brake_generation')==generation and s.get('brake_level')==level and s.get('response_observed')]
        delays.append(dict(generation=generation,level=level,decision_ros=request['brake_decision_ros'],
            initial_speed=request['brake_speed'],budget=request['brake_budget'],
            response_notification_upper_s=min((s['ros_time']-request['brake_decision_ros'] for s in observed),default=None),
            independent_actual=physical_response(request,oracle)))
    active_times=sorted(r['ros_time'] for r in ros if r.get('topic')=='policy' and
        r['data'].get('motion') and r['data'].get('actual_control_allowed'))
    states.sort(key=lambda s:s['stamp']);stamps=[s['stamp'] for s in states];position=[];heading=[];body_error=[];active_body_error=[];wheel_error=[];qualified_wheel_error=[]
    for actual in oracle:
        interval=actual.get('state_query_interval')
        if not interval or interval[1]-interval[0]>.04:continue
        # emit time follows separate signal readbacks. Pair against the ACTOR
        # RPC interval instead; those later queries are not a new pose sample.
        captured=actual['ros_time']-(actual['monotonic']-sum(interval)/2)
        i=bisect.bisect_left(stamps,captured)
        if i==0 or i==len(states):continue
        a,b=states[i-1],states[i];dt=b['stamp']-a['stamp']
        if not 0<dt<=.1:continue
        q=(captured-a['stamp'])/dt
        px=a['x']+q*(b['x']-a['x']);py=a['y']+q*(b['y']-a['y'])
        yaw=a['yaw']+q*math.remainder(b['yaw']-a['yaw'],2*math.pi)
        position.append(math.hypot(px-actual['x'],py-actual['y']))
        heading.append(abs(math.remainder(yaw-actual['yaw'],2*math.pi)))
        box=actual['bounding_box']
        shape=box_projection(box['center'],box['half_length'],box['half_width'],box['half_height'],box['roll'],box['pitch'])
        body_error.append(footprint(px,py,yaw,3.845,.79,1.0805).hausdorff_distance(shape))
        j=bisect.bisect_left(active_times,captured)
        # Include both sides of a scope boundary conservatively. Diagnostics
        # identify actual policy activity; this is not an entry authorization.
        if any(abs(captured-t)<=.2 for t in active_times[max(0,j-1):j+1]):
            active_body_error.append(body_error[-1])
        wheel=max(math.hypot(px+3*math.cos(yaw)-side*.814*math.sin(yaw)-
            (actual['x']+3*math.cos(actual['yaw'])-side*.814*math.sin(actual['yaw'])),
            py+3*math.sin(yaw)+side*.814*math.cos(yaw)-
            (actual['y']+3*math.sin(actual['yaw'])+side*.814*math.cos(actual['yaw']))) for side in (-1,1))
        wheel_error.append(wheel)
        v=a['speed']+q*(b['speed']-a['speed']);acceleration=a['acceleration']+q*(b['acceleration']-a['acceleration'])
        if v+max(0,acceleration)*.1<=7.:qualified_wheel_error.append(wheel)
    maximum_samples=[s for s in states if any(0<=s['stamp']-r['brake_decision_ros']<=2
        for (_,_,level),r in requests.items() if level==2) and s['speed']>.5]
    stopped_clearance=[]
    configuration=json.loads((directory/'trial-config.json').read_text())
    light=configuration['arguments']['light']
    binding=next(b for b in configuration['reviewed_bindings'] if b['traffic_light_id']==light)
    for actual in oracle:
        if actual['speed']>.15 or actual.get('aspects',{}).get(light) not in (1,4):continue
        box=actual['bounding_box'];shape=box_projection(box['center'],box['half_length'],box['half_width'],
            box['half_height'],box['roll'],box['pitch'])
        lx,ly=binding['line_point'];nx,ny=binding['approach_direction']
        stopped_clearance.append(min((lx-x)*nx+(ly-y)*ny for x,y in shape.exterior.coords))
    summary=dict(trial=directory.name,analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        recorded_result=result,motions=motions,event_kinds=sorted({e['kind'] for e in events}),
        events=events,active_causes=sorted(causes),execution_failure_masks=sorted(failures),response=delays,
        maximum_recorded_speed_mps=max((s['speed'] for s in states),default=None),
        independent_actual_samples=len(oracle),
        maximum_actual_distance_to_sampled_global_route_m=max((s['route_error'] for s in oracle if 'route_error' in s),default=None),
        actual_stopped_front_clearance_m=min(stopped_clearance,default=None),
        position_pairing=dict(samples=len(position),max_error_m=max(position,default=None),
            max_yaw_error_rad=max(heading,default=None),maximum_rpc_interval_s=.04,
            max_body_footprint_discrepancy_m=max(body_error,default=None),
            active_signal_max_body_discrepancy_m=max(active_body_error,default=None),
            max_front_wheel_discrepancy_m=max(wheel_error,default=None),
            qualified_front_wheel_max_discrepancy_m=max(qualified_wheel_error,default=None),
            qualified_front_wheel_samples=len(qualified_wheel_error),
            wheel_qualification_speed_max_mps=7.,wheel_positive_acceleration_horizon_s=.1,
            maximum_source_interpolation_gap_s=.1,method='RPC midpoint bracket + original-state interpolation'),
        maximum_brake_observed_acceleration_mps2=min((s['acceleration'] for s in maximum_samples),default=None))
    if result.get('case')=='yellow_close':
        summary['failure_response']=failure_response(result.get('independent_evaluation',{}),events,statuses)
        held=held_without_permission(ros,oracle)
        summary['failure_response']['held_without_entry_permission']=held
        summary['failure_response']['post_crossing_outcome']='endpoint_completed' if result.get('completed') else \
            'safe_hold_without_entry_permission' if held else 'unqualified_end'
        summary['failure_response']['run_qualified']=summary['failure_response']['accepted'] and \
            (result.get('completed',False) or held)
    if result.get('case')=='recovery':
        summary['red_recovery_exit_validation']=red_exit_validation(ros,oracle,
            list(records(directory/'signals.jsonl')),binding)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials',nargs='+',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=[analyze(p) for p in args.trials]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False))
    for row in report:
        notices=[r['response_notification_upper_s'] for r in row['response'] if r['response_notification_upper_s'] is not None]
        actual=[r['independent_actual']['response_upper_s'] for r in row['response'] if r['independent_actual']['response_upper_s'] is not None]
        print(json.dumps(dict(trial=row['trial'],passed=row['recorded_result'].get('passed'),
            events=row['event_kinds'],failure_response=row.get('failure_response'),response_count=len(row['response']),
            response_notification_max_s=max(notices,default=None),actual_response_max_s=max(actual,default=None))))


if __name__=='__main__':main()
