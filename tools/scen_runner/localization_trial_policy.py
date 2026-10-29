"""Small, ROS-free policy functions for the bounded empty-scene runner."""
import math


def native_speed_limit(mode,shadow_cap,normal_ceiling):
    """Independent native guard; unknown mode never borrows a faster ceiling."""
    if mode not in (0,1):return None
    return (shadow_cap if mode==1 else normal_ceiling)+1.


class ClearContactWindow:
    """Fresh native queries must be clear before a physical episode departs."""
    def __init__(self, duration=.5):
        self.duration=duration;self.started=None;self.last=None;self.samples=0

    def observe(self, now, contacts, ego_id):
        if not math.isfinite(now):raise ValueError('nonfinite contact query time')
        if self.last is not None and now<=self.last:
            self.started=None;self.samples=0;self.last=now;return False
        self.last=now
        if any(c and c!=ego_id for c in contacts):
            self.started=None;self.samples=0;return False
        if self.started is None:self.started=now
        self.samples+=1
        return self.samples>=3 and now-self.started>=self.duration


def ready(now, plan_valid, plan_received, trace, trace_received):
    return bool(plan_valid and 0<=now-plan_received<=.2 and 0<=now-trace_received<=.2
                and trace.get('anchor_ready') and trace.get('source')=='gps_anchored'
                and trace.get('recovery_state')=='gps_anchored'
                and trace.get('anchor_samples',0)>=10)


def offset_start(pose, lateral_m, heading_deg):
    if not all(math.isfinite(v) for v in (lateral_m,heading_deg)) or abs(lateral_m)>.2 or abs(heading_deg)>2:
        raise ValueError('M2 start offsets limited to 0.2m and 2deg')
    result=dict(pose)
    # Actor-state pose yaw is ENU degrees. Convert to grid for lateral translation.
    from scen_runner.morai.signal_trial_coordinates import Coordinates
    yaw=Coordinates([302595.,4124145.]).grid_yaw(pose['x'],pose['y'],pose['yaw'])
    result['x']-=math.sin(yaw)*lateral_m;result['y']+=math.cos(yaw)*lateral_m
    result['yaw']+=heading_deg
    return result
def udp_contact_classification(message,ego,stamp):
    """Identify only a time/position-matched SDK self-status tuple.

    MolitComp03 repeatedly transmits type=-1/id=0 at the native ego origin
    while native contacts are empty. Never discard other negative IDs or a
    self-shaped tuple elsewhere. Raw messages remain in the trial recording.
    This is evidence classification, not collision/mesh certification.
    """
    import math
    objects=message.get('collision_object',[])+message.get('collision_objecta',[])
    if not objects:return 'empty'
    if not ego or abs(stamp-message['stamp'])>.2:return 'unresolved'
    p=ego['pose']
    if any(x for x in ego['native_contacts'] if x!='Ego'):return 'native_contact_present'
    if all(o['unique_id']==0 and o['type']==-1 and
           math.dist(o['position'],[p['x'],p['y'],p['z']])<=.75 for o in objects):
        return 'position_matched_self_status'
    return 'unresolved'
