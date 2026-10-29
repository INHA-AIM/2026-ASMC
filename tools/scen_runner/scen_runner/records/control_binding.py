"""Read-only decoder of docs/contracts/control-binding-v1.md for trial records.

This tool never issues a control request or publishes a Controller proof.
"""
import io
import struct


class Reader:
    def __init__(self,data):
        self.data=bytes(data);self.at=0
        if len(self.data)>4*1024*1024:raise ValueError('oversized control packet')

    def value(self,fmt):
        width=struct.calcsize('<'+fmt)
        if self.at+width>len(self.data):raise ValueError('truncated control packet')
        result=struct.unpack_from('<'+fmt,self.data,self.at)[0];self.at+=width;return result

    def string(self):
        width=self.value('I')
        if width>len(self.data)-self.at:raise ValueError('truncated control string')
        result=self.data[self.at:self.at+width].decode('utf-8');self.at+=width;return result

    def message(self,cls,legacy_trajectory=False):
        raw=self.data[self.at:]
        if legacy_trajectory:
            # v1/v2 lacked execution_policy after Header, valid, failure_reason.
            # Upgrade this read-only view without consuming an extra wire byte.
            if len(raw)<16:raise ValueError('truncated trajectory header')
            frame_width=struct.unpack_from('<I',raw,12)[0]
            offset=16+frame_width+2
            if offset>len(raw):raise ValueError('truncated trajectory header')
            raw=raw[:offset]+b'\0'+raw[offset:]
        result=cls();result.deserialize(raw)
        encoded=io.BytesIO();result.serialize(encoded);self.at+=len(encoded.getvalue())-int(legacy_trajectory)
        return result

    def fields(self,names,fmt):return {name:self.value(fmt) for name in names.split()}
    def token(self):return [self.string(),self.string(),self.value('Q')]


def request(reader):
    from asmc_msgs.msg import BehaviorCommand
    r={'token':reader.token(),'command':reader.message(BehaviorCommand)}
    s=reader.fields('enabled','?');s.update(reader.fields('route_generation target_revision','Q'))
    s['stop_line']=reader.string();s['movement']=reader.string()
    s.update(reader.fields('line_x line_y normal_x normal_y clearance position_error','d'))
    s.update(reader.fields('braking_only','?'));r.update(reader.fields('recovery_release release_authorized','?'))
    s.update(reader.fields('recovery_geometry','?'));s.update(reader.fields('exit_x exit_y exit_normal_x exit_normal_y','d'))
    r['signal']=s;r.update(reader.fields('brake_generation','Q'));r.update(reader.fields('brake_level','B'))
    r.update(reader.fields('brake_decision_ros brake_state_stamp brake_speed brake_acceleration brake_budget','d'))
    r.update(reader.fields('execution_recovery_ack','?'));return r


def decode(message):
    from asmc_msgs.msg import FrenetFeedback,Trajectory
    if message.layout.dim or message.layout.data_offset:raise ValueError('invalid control packet layout')
    r=Reader(message.data)
    if r.value('I')!=0x41534d43:raise ValueError('unsupported control schema')
    version=r.value('B')
    if version not in (1,2,3):raise ValueError('unsupported control schema')
    policy_aware='execution_policy' in Trajectory.__slots__
    if version==3 and not policy_aware:raise ValueError('control schema v3 requires rebuilt asmc_msgs')
    kind=r.value('B');out={'kind':kind,'schema_version':version}
    if kind in (1,2):out.update(request(r))
    elif kind==3:out['token']=r.token()
    else:raise ValueError('unsupported control packet kind')
    if kind==2:
        out['planner_epoch']=r.string();out['sequence']=r.value('Q')
        out['feedback']=r.message(FrenetFeedback);out['trajectory']=r.message(Trajectory,legacy_trajectory=version<3 and policy_aware)
        out.update(r.fields('goal_satisfied exit_path_valid stationary_hold execution_recovery_safe','?'))
        out['merge_waiting_path_valid']=r.value('?') if version>=2 else False
    elif kind==3:
        out['planner_epoch']=r.string();out.update(r.fields('plan_sequence output_sequence','Q'))
        out.update(r.fields('linked planning_checked stop_path_valid goal_satisfied exit_path_valid actuated execution_failed','?'))
        out.update(r.fields('failure_mask','B'));out.update(r.fields('execution_recovery_safe','?'))
        out.update(r.fields('brake_generation','Q'));out.update(r.fields('brake_level','B'))
        out.update(r.fields('response_observed','?'));out.update(r.fields('response_remaining brake steering accel','d'))
    if r.at!=len(r.data):raise ValueError('trailing control packet data')
    return out
