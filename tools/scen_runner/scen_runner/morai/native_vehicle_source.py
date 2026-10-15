"""One native spawn-point ticket per NPC, retaining its real Custom target."""
import math
import uuid


class SourcePlacementDeferred(RuntimeError):
    """MORAI rejected point placement; no successful point ID was returned."""


def create_ticket(link,lane,model,target):
    from proto.morai.scenario.spawn_point_pb2 import CreateVehicleSpawnPointParam
    from proto.morai.common.object_info_pb2 import ObjectInfo
    r=CreateVehicleSpawnPointParam()
    scope=link.client_key+'_source_'+uuid.uuid4().hex
    info=r.spawn_point_info.object_info;info.id.value='';info.client_key=scope;info.object_type=5
    r.spawn_point_info.pause=True;r.spawn_point_info.model_name=model
    pose=lane.plan.cfg['spawn_pose']
    r.spawn_point_info.location.x=pose['x'];r.spawn_point_info.location.y=pose['y'];r.spawn_point_info.location.z=pose['z']
    c=r.config;c.list_vehicle_length_type.append(2);c.parameter_type=1;c.maximum_spawn_vehicle=1
    c.min_spawn_period=c.max_spawn_period=lane.plan.period
    c.spawn_velocity_type=2;c.desired_velocity_type=2
    if 'initial_speed_kmh' in lane.plan.cfg:
        c.min_spawn_velocity_custom=c.max_spawn_velocity_custom=lane.plan.cfg['initial_speed_kmh']
    c.min_desired_velocity_custom=c.max_desired_velocity_custom=target
    c.destination_obj_unique_id=-1;c.lateral_bias_mode=1
    c.start_link_info.id.value=lane.plan.cfg['npc_links'][0];c.start_link_info.waypoint_idx=lane.plan.cfg['start_point_index']
    c.end_link_info.id.value=lane.plan.cfg['npc_links'][-1];c.end_link_info.waypoint_idx=lane.plan.cfg['end_point_index']
    reply=link._require()._scenario_stub.CreateVehicleSpawnPoint(r,timeout=5)
    if reply.status != 1 and str(reply.description).strip().lower() == 'false':
        raise SourcePlacementDeferred('MORAI가 생성점 배치를 거부했습니다 (False)')
    link.check(reply,'고주로 원생성점 생성 실패')
    aid=str(reply.custom_message).strip()
    if not aid.isdigit() or int(aid)<=0:raise RuntimeError('원생성점 ID 미확정: 추정 삭제 중단')
    created=ObjectInfo(object_type=5,client_key=scope);created.id.value=aid
    name=link._require()._actor_stub.GetOptionName(created,timeout=3).value
    if name not in ('SpawnPoint','VehicleSpawnPoint'):raise RuntimeError('원생성점 반환 유형 불일치: 추정 삭제 중단')
    return dict(info=created,lane=lane,model=model,target=target,actor=None,missing_since=None)


def enable_ticket(link,ticket,active):
    from proto.morai.scenario.spawn_point_pb2 import EnableSpawnPointParam
    request=EnableSpawnPointParam(is_active=active);request.object_info.CopyFrom(ticket['info'])
    link.check(link._require()._scenario_stub.EnableSpawnPoint(request,timeout=3),'원생성점 활성화 변경 실패')


def destroy_ticket(link,ticket):
    link.check(link._require()._actor_stub.DestroyActor(ticket['info'],timeout=5),'원생성점 삭제 실패')


def ticket_candidates(ticket,states,known_ids):
    lane=ticket['lane'];pose=lane.plan.cfg['spawn_pose'];start=lane.plan.cfg['npc_links'][0]
    return [state for state in states if state.actor_info.id.value not in known_ids and
            state.actor_info.id.value not in ticket['baseline'] and
            state.actor_info.object_type==1 and state.vehicle_state.current_link_info.id.value==start and
            math.hypot(state.transform.location.x-pose['x'],state.transform.location.y-pose['y'])<=8.]


def waiting_details(ticket,states,known_ids):
    """Explain our matching filters, without inventing a native TTC verdict."""
    pose=ticket['lane'].plan.cfg['spawn_pose'];start=ticket['lane'].plan.cfg['npc_links'][0]
    nearby=[];new=[]
    for state in states:
        if state.actor_info.object_type!=1:continue
        aid=state.actor_info.id.value
        distance=math.hypot(state.transform.location.x-pose['x'],state.transform.location.y-pose['y'])
        reasons=[]
        if aid in known_ids:reasons.append('기존/관리 ID')
        if aid in ticket['baseline']:reasons.append('요청 전 ID')
        current=state.vehicle_state.current_link_info.id.value
        if current!=start:reasons.append('시작 링크 불일치')
        if distance>8.:reasons.append('8m 범위 밖')
        text=f'ID={aid} 거리={distance:.1f}m 링크={current} 제외={"/".join(reasons) or "없음"}'
        if distance<=15.:nearby.append(text)
        if aid not in known_ids and aid not in ticket['baseline']:new.append(text)
    return f'주변 차량: {"; ".join(nearby) or "없음"} · 신규 후보: {"; ".join(new) or "없음"}'
