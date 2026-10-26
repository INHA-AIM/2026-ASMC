"""Read common tracking observations for evaluation, without ROS/Marker/JSON joins."""
import math


def observation_samples(message, minimum_age=0):
    observation=message.observation
    array=observation.objects
    if array.header.frame_id!='map' or array.status not in (1,2):
        return [], 'map_observation_unavailable'
    if not observation.producer_session_id or not observation.generation or not observation.scan_sequence:
        return [], 'missing_identity'
    if len(array.objects)!=len(message.tracking):
        return [], 'tracking_alignment'
    samples=[];ids=set();invalid=0
    for obj,state in zip(array.objects,message.tracking):
        if obj.track_id!=state.track_id or obj.track_id in ids:
            return [], 'tracking_alignment'
        ids.add(obj.track_id)
        if obj.observation_state!=1 or obj.miss_count or obj.age<minimum_age:
            continue
        velocity=obj.absolute_velocity_mps
        values=(obj.center.x,obj.center.y,obj.center.z,velocity.x,velocity.y,velocity.z)
        if not obj.absolute_velocity_valid or not all(map(math.isfinite,values)):
            invalid+=1;continue
        samples.append(dict(id=int(obj.track_id),classification=str(obj.classification),
            x=float(obj.center.x),y=float(obj.center.y),z=float(obj.center.z),
            vx=float(velocity.x),vy=float(velocity.y),vz=float(velocity.z),
            speed=math.hypot(velocity.x,velocity.y),age=int(obj.age),
            motion_state=int(state.motion_state),dynamic_valid=bool(obj.dynamic_valid),
            producer_session_id=observation.producer_session_id,generation=observation.generation,
            scan_sequence=observation.scan_sequence))
    return samples, 'invalid_velocity_samples:'+str(invalid) if invalid else 'observed_empty' if not array.objects else 'observed'
