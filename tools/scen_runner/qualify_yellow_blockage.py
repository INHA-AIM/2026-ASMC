#!/usr/bin/env python3
"""Reversible real-prop blockage, using the ordinary owned yellow trial.

Qualifies a blocked exit while Ego is held during readiness, then removes the
prop and requires actual red recovery. Does not inject ROS hazard flags,
simulator truth, or a brake-release command into the policy.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import yaml
from scen_runner.config import REPO_ROOT
from scen_runner.morai.clock_contract import ClockGuard
from scen_runner.morai.signal_trial_coordinates import Coordinates
from scen_runner.orchestration.process import OwnedProcess
from scen_runner.runtime.signal_trial_fixture import StaticFixture
from scen_runner.runtime.signal_trial_oracle import Oracle
from scen_runner.runtime.validation_episode import EpisodeLink,EpisodeRoute


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True);parser.add_argument('--port',type=int,default=7789)
    parser.add_argument('--clock-profile',type=Path,required=True)
    parser.add_argument('--physical-road',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['red_recovery_blocked'],default='red_recovery_blocked')
    parser.add_argument('--static-id',required=True,help='explicit existing stationary prop; never another vehicle/pedestrian')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('GRPC_POLL_STRATEGY','epoll1')
    clock=ClockGuard();clock.check()
    result=dict(case=args.case,passed=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    cfg={'morai':dict(host=args.host,port=args.port,client_key='asmc_yellow_blockage',ego_id='Ego',grpc_src='tools/grpc_inha_univ/src')}
    link=EpisodeLink(cfg);link.connect();child=None;stream=None;log=None;route=None
    events=(args.output/'blockage.jsonl').open('w',buffering=1)
    def emit(row):events.write(json.dumps(dict(monotonic=time.monotonic(),**row),allow_nan=False)+'\n')
    fixture=StaticFixture(link,emit)
    try:
        link.hold();route=EpisodeRoute()
        bindings=yaml.safe_load((REPO_ROOT/'src/asmc_planner/config/behavior/signal_policy_geometry_kcity2025.yaml').read_text())['reviewed_bindings']
        target=next(b for b in bindings if b['traffic_light_id']=='C1256W000091')
        sline,_=route.curve.project(*target['line_point'])
        original=fixture.state(args.static_id)
        # Same physical prop, placed safely ahead of the held inside pose.
        # Its full native collision geometry must block the exit planner.
        x,y,_,_=route.curve.pose(sline+24.)
        fixture.park(args.static_id,x-original['pose']['x'],y-original['pose']['y'])
        oracle=Oracle(json.loads(args.physical_road.read_text()),bindings,Coordinates([302595.,4124145.]),maximum_gap_s=.3)
        trial=args.output/'trial';log=(args.output/'driver.log').open('w')
        command=[sys.executable,str(REPO_ROOT/'tools/scen_runner/run_yellow_signal_trial.py'),
            '--host',args.host,'--port',str(args.port),'--clock-profile',str(args.clock_profile.resolve()),
            '--physical-road',str(args.physical_road.resolve()),'--output',str(trial.resolve()),
            '--light',target['traffic_light_id'],'--case','recovery','--start-distance','40']
        child=OwnedProcess(command,stdout=log,stderr=log,cwd=str(REPO_ROOT),env=dict(os.environ))
        latest={};stable=None;removed=False;held_samples=0;deadline=time.monotonic()+140.
        while child.observe() is None and time.monotonic()<deadline:
            clock.check()
            if stream is None and (trial/'ros.jsonl').exists():stream=(trial/'ros.jsonl').open()
            if stream:
                while True:
                    offset=stream.tell();line=stream.readline()
                    if not line:break
                    if not line.endswith('\n'):stream.seek(offset);break
                    row=json.loads(line);latest[row.get('topic')]=row
            if not removed:
                now=time.monotonic();policy=latest.get('policy',{}).get('data',{})
                status=latest.get('status',{});plan=latest.get('plan',{});features=latest.get('features',{})
                state=latest.get('state',{});vehicle=policy.get('vehicle',{})
                known=policy.get('stop_line')==target['stop_node_id'] and policy.get('causes',{}).get('INPUT_INVALID',1)==0
                fresh=all(0<=now-latest.get(k,{}).get('monotonic',-math.inf)<=.3
                          for k in ('policy','status','plan','features','state','request'))
                token=latest.get('request',{}).get('token')
                current=token is not None and token==status.get('token')==plan.get('token')
                blocked=known and fresh and current and vehicle.get('speed',math.inf)<=.15 and state.get('speed',math.inf)<=.15 and \
                    status.get('brake',0)>=.29 and status.get('accel',1)==0 and \
                    not plan.get('exit_path_valid',True) and \
                    (features.get('static_obstacle_ahead',False) or policy.get('causes',{}).get('EXECUTION_FAILURE',0)>0)
                blocked=blocked and policy.get('green_count',3)<3 and policy.get('motion')=='RECOVERY_BRAKE'
                if blocked:
                    actual=link.ego();station,_=route.curve.project(actual.transform.location.x,actual.transform.location.y)
                    if abs(station-(sline+8.))>.75:raise RuntimeError('unexpected held recovery pose')
                    # The child's oracle starts after readiness. Independently
                    # check the held red preflight too; do not hide a spawn contact.
                    row=oracle.observe(actual,time.monotonic(),{target['traffic_light_id']:link.traffic_light_color(target['traffic_light_id'])})
                    held_samples+=1
                    emit(dict(event='held_preflight_oracle',data=row))
                    if not row['road_ok'] or row['collisions'] or not row['observation_complete']:
                        raise RuntimeError('held preflight physical evaluation failed')
                    stable=now if stable is None else stable
                    if now-stable>=2.:
                        proof={k:latest.get(k) for k in ('policy','status','plan','features','state')}
                        (args.output/'blocked-proof.json').write_text(json.dumps(proof,indent=2))
                        # Reuse the same off-route parking pose as isolated
                        # trials, including its original height/orientation.
                        parked=dict(pose=dict(original['pose']));parked['pose']['x']+=30.
                        verified=fixture.place(args.static_id,parked)
                        emit(dict(event='blockage_removed',actor=args.static_id,actual=verified))
                        removed=True;result['blocked_hold_verified']=True
                else:stable=None
            time.sleep(.03)
        if child.observe() is None:raise RuntimeError('blockage qualification deadline')
        recorded=json.loads((trial/'result.json').read_text());result['trial']=recorded
        result['passed']=removed and recorded.get('passed',False)
        result['held_preflight_evaluation']=oracle.result();result['held_preflight_samples']=held_samples
        result['passed']=result['passed'] and held_samples>=2
    except Exception as error:result['error']=str(error)
    finally:
        try:
            link.hold()
            if child and child.observe() is None:child.stop()
            # A failed held-inside trial may still be near the prop's original
            # pose. Move only the held Ego to the official start before restore.
            if route and fixture.original:
                link.reset(dict(route.scene_start))
                if not link.held_at(route.scene_start):raise RuntimeError('cleanup start hold not verified')
                result['cleanup_start_hold_verified']=True
        except Exception as error:result['shutdown_error']=str(error);result['passed']=False
        try:fixture.restore()
        except Exception as error:result['fixture_restore_error']=str(error);result['passed']=False
        link.disconnect()
        if stream:stream.close()
        if log:log.close()
        events.close()
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
