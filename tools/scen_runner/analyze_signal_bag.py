#!/usr/bin/env python3
"""Read closed trial bags; join actual Controller output to explicit C–T IDs.

Arrival timestamps are not proof of command ownership. This does not publish,
change a raw bag, or promote incomplete evidence to driving acceptance.
"""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path

from scen_runner.records.control_binding import decode


def semantic_digest(row):
    command=copy.deepcopy(row['command'])
    command.header.seq=0
    command.header.stamp.secs=command.header.stamp.nsecs=0
    command.valid_until.secs=command.valid_until.nsecs=0
    command.reason=''
    stream=io.BytesIO();command.serialize(stream)
    values={key:row[key] for key in (
        'signal','recovery_release','release_authorized','brake_generation','brake_level',
        'brake_decision_ros','brake_state_stamp','brake_speed','brake_acceleration',
        'brake_budget','execution_recovery_ack')}
    values['command']=stream.getvalue().hex()
    return hashlib.sha256(json.dumps(values,sort_keys=True,allow_nan=False).encode()).hexdigest()


def check_binding_evidence(requests,plans,statuses):
    failures=[];request_index={};plan_index={};used=set();linked=0;actuated=0
    for row in requests:
        token=tuple(row['token'])
        previous=request_index.setdefault(token,row)
        if previous['semantic']!=row['semantic']:
            failures.append(dict(reason='one token has different command semantics',token=token))
    for row in plans:
        key=(tuple(row['token']),row['planner_epoch'],row['sequence'])
        previous=plan_index.setdefault(key,row)
        if previous!=row:failures.append(dict(reason='one plan identity has different content',key=key))
    for row in statuses:
        token=tuple(row['token'])
        if row.get('actuated'):
            actuated+=1
            if not all(math.isfinite(row[name]) for name in ('brake','steering','accel')):
                failures.append(dict(reason='non-finite Controller output',token=token))
            if row['accel']>1e-6 and (not row['linked'] or row['brake']>1e-6):
                failures.append(dict(reason='accelerating without link or while braking',token=token))
        if not row['linked']:continue
        linked+=1
        key=(token,row['planner_epoch'],row['plan_sequence'])
        request=request_index.get(token);plan=plan_index.get(key)
        if request is None or plan is None:
            failures.append(dict(reason='linked output has no recorded exact request and plan',key=key));continue
        used.add(key)
        if plan['semantic']!=request['semantic'] or not plan['accepted_valid']:
            failures.append(dict(reason='linked output uses another command or invalid planning result',key=key))
        if row.get('actuated') and row['accel']>1e-6 and not request['release_authorized']:
            failures.append(dict(reason='acceleration without current release authorization',key=key))
    complete=bool(requests and plans and statuses and linked and actuated)
    return dict(passed=complete and not failures,evidence_complete=complete,
        request_tokens=len(request_index),plan_identities=len(plan_index),
        linked_samples=linked,actuated_samples=actuated,used_plan_identities=len(used),failures=failures)


def check_header_order(headers, arrival_reorders, sensor_failures,
                       identity, source_stamp, sequence, bag_stamp):
    """Reject source-time regressions; label a bounded recorder arrival swap.

    ROS transport can deliver two consecutive messages from one publisher to
    rosbag in the opposite order. Their publisher-assigned header sequences
    still establish the original source order. Never advance the source high
    watermark from the delayed older message.
    """
    old=headers.get(identity)
    current=(source_stamp,sequence,bag_stamp)
    if old is None:
        headers[identity]=current
        return
    old_stamp,old_sequence,old_bag_stamp=old
    if source_stamp<old_stamp:
        bounded_swap=(sequence<old_sequence and old_sequence-sequence<=2 and
                      source_stamp>=old_stamp-.02 and
                      0<=bag_stamp-old_bag_stamp<=.02)
        row=dict(topic=identity[0],frame=identity[1],previous=old_stamp,
                 current=source_stamp,previous_sequence=old_sequence,
                 current_sequence=sequence)
        if bounded_swap:
            arrival_reorders.append(row)
            return
        sensor_failures.append(dict(row,reason='original header moved backward'))
        return
    if sequence<old_sequence:
        sensor_failures.append(dict(topic=identity[0],frame=identity[1],
            reason='publisher sequence moved backward while source time advanced',
            previous_sequence=old_sequence,current_sequence=sequence))
        return
    headers[identity]=current


def object_frame(message):
    """Use the atomic observation header, with historical bag schema support."""
    if hasattr(message,'observation'):
        return message.observation.objects
    return getattr(message,'tracked',message)


def analyze(directory):
    import rosbag
    requests=[];plans=[];statuses=[];topics={};headers={};sensor_failures=[]
    arrival_reorders=[]
    nonempty_objects={};maximum_track_id={};behavior_counts={}
    paths=sorted(directory.glob('drive*.bag'))
    if not paths or list(directory.glob('drive*.bag.active')):
        raise ValueError('closed drive bags required; raw evidence is never repaired in place')
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            selected=('/behavior/control_request','/planning/control_plan','/control/binding_status',
                '/planning/vehicle/state','/planning/objects','/perception/traffic_light/detection',
                '/morai_sensor_bridge/frame_timing','/image_jpeg/compressed',
                '/cam_extra/image_jpeg/compressed','/gps','/imu','/velodyne_points','/ctrl_cmd',
                '/Ego_topic','/planning/vehicle/gps_imu_state','/planning/vehicle/speed_diagnostics')
            selected+=('/planning/mode','/planning/control_selector/diagnostics')
            selected+=('/lidar/motion_debug','/lidar/tracked_objects_map','/last_lidar',
                       '/planning/input/tracked_objects','/behavior/feature_debug',
                       '/perception/lidar/tracks_local','/perception/lidar/tracks_map',
                       '/perception/lidar/diagnostics')
            for topic,msg,stamp in bag.read_messages(topics=list(selected)):
                topics[topic]=topics.get(topic,0)+1
                frame=object_frame(msg)
                if hasattr(frame,'objects'):
                    nonempty_objects[topic]=nonempty_objects.get(topic,0)+bool(frame.objects)
                    for obj in frame.objects:
                        maximum_track_id[topic]=max(maximum_track_id.get(topic,-1),getattr(obj,'track_id',getattr(obj,'id',-1)))
                if topic=='/behavior/feature_debug':
                    key=str(msg.selected_behavior)
                    behavior_counts[key]=behavior_counts.get(key,0)+1
                if topic in selected[:3]:
                    row=decode(msg)
                    if row['kind']==1:
                        requests.append(dict(token=row['token'],semantic=semantic_digest(row),
                            release_authorized=row['release_authorized']))
                    elif row['kind']==2:
                        feedback=row['feedback'];trajectory=row['trajectory'];command=row['command']
                        hold=(row['stationary_hold'] and command.maneuver_type==command.STOP and
                            feedback.execution_state==feedback.COMPLETED and
                            all(math.isfinite(p.velocity) and abs(p.velocity)<=.01 for p in trajectory.points))
                        waiting=(row.get('merge_waiting_path_valid',False) and
                            command.maneuver_type==command.MERGE and command.merge_type==command.MERGE_ROUNDABOUT and
                            command.execution_request==command.PREPARE and not row['signal']['enabled'] and
                            not row['recovery_release'] and feedback.execution_state in (feedback.PREPARING,feedback.WAITING) and
                            feedback.trajectory_source==feedback.TRAJECTORY_NEW and bool(trajectory.points) and
                            abs(trajectory.points[-1].velocity)<=.01 and
                            all(math.isfinite(p.velocity) and p.velocity>=-.01 for p in trajectory.points))
                        avoid_waiting=(command.maneuver_type==command.AVOID and
                            command.execution_request in (command.PREPARE,command.EXECUTE) and
                            not row['signal']['enabled'] and not row['recovery_release'] and row['brake_level']==0 and
                            feedback.execution_state in (feedback.PREPARING,feedback.WAITING) and
                            feedback.result_code==feedback.RESULT_NONE and
                            feedback.trajectory_source==feedback.TRAJECTORY_NEW and len(trajectory.points)>=2 and
                            abs(trajectory.points[-1].velocity)<=.01 and
                            all(math.isfinite(p.velocity) and p.velocity>=0 for p in trajectory.points))
                        valid=(feedback.command_accepted and feedback.command_id==command.command_id and
                            feedback.active_maneuver==command.maneuver_type and (feedback.plan_valid or hold or waiting or avoid_waiting) and
                            trajectory.valid and trajectory.failure_reason==0 and bool(trajectory.points) and
                            feedback.trajectory_source in (feedback.TRAJECTORY_NEW,feedback.TRAJECTORY_LOCKED))
                        plans.append(dict(token=row['token'],planner_epoch=row['planner_epoch'],
                            sequence=row['sequence'],semantic=semantic_digest(row),accepted_valid=bool(valid)))
                    else:statuses.append(row)
                elif hasattr(frame,'header') and topic not in ('/perception/traffic_light/detection','/perception/lidar/diagnostics'):
                    now=frame.header.stamp.to_sec()
                    # A frame-timing topic multiplexes cameras. Each original
                    # source must be checked independently, not interleaved.
                    identity=(topic,frame.header.frame_id)
                    if hasattr(msg,'observation'):
                        identity+=(msg.observation.producer_session_id,msg.observation.generation)
                    check_header_order(headers,arrival_reorders,sensor_failures,
                                       identity,now,getattr(getattr(msg,'observation',None),'scan_sequence',frame.header.seq),stamp.to_sec())
    binding=check_binding_evidence(requests,plans,statuses)
    required=('/planning/vehicle/state','/planning/objects','/perception/traffic_light/detection',
        '/cam_extra/image_jpeg/compressed','/morai_sensor_bridge/frame_timing','/ctrl_cmd')
    speed_sources=('/Ego_topic','/planning/vehicle/gps_imu_state','/planning/vehicle/speed_diagnostics')
    manifest_path=directory/'result.json'
    if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('recording',{}).get('schema_version',1)>=2:
        required+=speed_sources
    if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('recording',{}).get('schema_version',1)>=3:
        required+=('/planning/mode','/planning/control_selector/diagnostics')
    if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('recording',{}).get('schema_version',1)==4:
        required+=('/lidar/motion_debug','/lidar/tracked_objects_map',
                   '/planning/input/tracked_objects','/behavior/feature_debug')
    if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('recording',{}).get('schema_version',1)>=5:
        required+=('/perception/lidar/tracks_local','/perception/lidar/tracks_map',
                   '/perception/lidar/diagnostics','/behavior/feature_debug')
    missing=[name for name in required if not topics.get(name)]
    result=dict(passed=binding['passed'] and not sensor_failures and not missing,
        scope='recorded binding/input integrity only; independent physical driving verdict is separate',
        binding=binding,topic_counts=topics,header_failures=sensor_failures,
        arrival_reorders=arrival_reorders,missing_topics=missing,
        speed_assembly_sources_recorded=all(topics.get(name) for name in speed_sources),
        perception_observations=dict(nonempty_object_frames=nonempty_objects,
            maximum_track_ids=maximum_track_id,selected_behavior_frames=behavior_counts,
            note='nonempty input and selected behavior evidence; not an obstacle detection recall verdict'))
    (directory/'bag-analysis.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);args=parser.parse_args()
    result=analyze(args.directory);print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
