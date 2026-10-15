"""RPC client for the independent episode runtime. No section runner imports."""
import math
import uuid
import time
from scen_runner.morai.morai_link import MoraiLink

class SceneLink(MoraiLink):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.actor_scopes = {}

    def info(self, aid):
        from proto.morai.common.object_info_pb2 import ObjectInfo
        info = ObjectInfo(object_type=1, client_key=self.actor_scopes.get(aid,self.client_key))
        info.id.value = aid
        return info

    def destroy(self, actor_id, object_type=1, client_key=None):
        scope = self.actor_scopes.get(actor_id,self.client_key) if client_key is None else client_key
        removed = super().destroy(actor_id,object_type=object_type,client_key=scope)
        if removed:
            self.actor_scopes.pop(actor_id,None)
        return removed

    @staticmethod
    def check(result, operation):
        if result.status != 1:
            raise RuntimeError(f'{operation}: {result.status} {result.description}')

    def _query_signal_state(self, method, param, label):
        import grpc
        for attempt in range(3):
            try:
                result = method(param, timeout=3)
            except grpc.RpcError as error:
                # Observed in MORAI's infrastructure read handlers while the
                # simulator is running. Retry only this specific query failure;
                # no write retries, cached colors, or success assumed on error.
                if (attempt == 2 or error.code() != grpc.StatusCode.UNKNOWN or
                        error.details() != 'Exception was thrown by handler.'):
                    raise
                time.sleep(.05)
            else:
                if attempt:
                    print(f'[신호 상태 재조회] {label} · MORAI 조회 예외 후 {attempt}회 재조회 성공', flush=True)
                return result

    def traffic_light_color(self, aid):
        from proto.morai.infrastructure.traffic_light_pb2 import GetTrafficLightInfoParam
        param = GetTrafficLightInfoParam(type=2, value=aid)  # GET_TL_INFO_BY_TL_ID
        result = self._query_signal_state(self._require()._infrastructure_stub.GetTrafficLightInfo, param, aid)
        if result.id.value != aid or result.color < 0:
            raise RuntimeError('신호등 상태 미확인: '+aid)
        return int(result.color)

    def intersection_state(self, aid):
        from proto.morai.common.object_identifier_pb2 import ObjectIdentifier
        result = self._query_signal_state(self._require()._infrastructure_stub.GetIntersectionTLInfo,
                                         ObjectIdentifier(value=aid), aid)
        colors = {item.id.value: int(item.color) for item in result.traffic_lights}
        if result.phase < 0 or not colors:
            raise RuntimeError('교차로 상태 미확인: '+aid)
        return dict(phase=int(result.phase), colors=colors)

    def set_intersection_phase(self, aid, phase):
        from proto.morai.infrastructure.intersection_pb2 import IntersectionPhase
        param = IntersectionPhase(phase=phase)
        param.id.value = aid
        self.check(self._require()._infrastructure_stub.SetIntersectionPhase(param, timeout=3),
                   '교차로 단계 변경 실패: '+aid)

    def set_traffic_light_color(self, aid, color, impulse=False):
        from proto.morai.infrastructure.traffic_light_pb2 import TrafficLightStateParam
        param = TrafficLightStateParam(is_impulse=impulse, set_sibling=False)
        param.info.id.value = aid
        param.info.color = color
        self.check(self._require()._infrastructure_stub.SetTrafficLightState(param, timeout=3),
                   '신호등 변경 실패: '+aid)

    def verify_traffic_light_colors(self, colors):
        for aid, color in colors.items():
            for attempt in range(6):
                actual = self.traffic_light_color(aid)
                if actual == color:
                    break
                if attempt == 5:
                    raise RuntimeError(f'신호등 적용 불일치: {aid} · 요청={color}, 실제={actual}')
                time.sleep(.05)

    def states(self):
        from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter
        result = self._require()._actor_stub.GetAllActorsState(
            GetAllActorsFilter(vehicle=True, pedestrian=True, obstacle=True), timeout=3)
        return list(result.states)

    def scoped_statics(self):
        from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter
        return list(self._require()._actor_stub.GetAllActorsState(
            GetAllActorsFilter(obstacle=True, client_key=self.client_key), timeout=3).states)

    def static_native_id(self, alias):
        scoped = next((s for s in self.scoped_statics() if s.actor_info.id.value == alias), None)
        if scoped is None:
            return None
        p = scoped.transform.location
        matches = [s.actor_info.id.value for s in self.states()
                   if s.actor_info.object_type == 3 and math.dist((p.x,p.y,p.z),
                   (s.transform.location.x,s.transform.location.y,s.transform.location.z)) < .05]
        return matches[0] if len(matches) == 1 else None

    @staticmethod
    def crossing_waypoints(waypoints,plan):
        # Native first-entry removal: duplicate the start, then one endpoint.
        for fraction in (0.,0.,1.):
            point=waypoints.waypoints.add()
            point.speed=plan.cfg['walk_speed_kmh']
            point.waypoint.x=plan.start[0]+fraction*(plan.end[0]-plan.start[0])
            point.waypoint.y=plan.start[1]+fraction*(plan.end[1]-plan.start[1])
            point.waypoint.z=plan.start[2]+fraction*(plan.end[2]-plan.start[2])

    def spawn_crossing(self, plan,with_route=True):
        from proto.morai.actor.actor_spawn_pb2 import PedestrianSpawnParam
        from proto.morai.common.object_info_pb2 import ObjectInfo
        group, model = plan.models[0]
        r = PedestrianSpawnParam(velocity=plan.cfg['walk_speed_kmh'],
            active_dist=plan.cfg['activation_distance_m'], move_dist=plan.move_distance,
            start_action=getattr(plan,'replay_started',False))
        if getattr(plan,'replay_finished',False):r.velocity=0.
        info = ObjectInfo(object_type=2, client_key=self.client_key)
        info.id.value = ''
        r.spawn_info.actor_info.CopyFrom(info)
        r.spawn_info.model_name = model
        r.spawn_info.label = f'episode_{group}_{model}'
        r.spawn_info.transform.location.x, r.spawn_info.transform.location.y, r.spawn_info.transform.location.z = plan.start
        r.spawn_info.transform.location.z += plan.cfg['spawn_height_offset_m']
        r.spawn_info.transform.rotation.z = plan.heading_deg
        if with_route:self.crossing_waypoints(r.waypoints,plan)
        reply = self._require()._actor_stub.SpawnPedestrian(r, timeout=5)
        self.check(reply, '횡단 객체 생성 실패')
        aid = str(reply.custom_message).strip()
        if not aid.isdigit() or int(aid) <= 0:
            raise RuntimeError('횡단 객체의 MORAI ID 미확정')
        return aid

    def register_crossing(self,aid,plan):
        from proto.morai.actor.actor_set_pb2 import PedestrianWaypointInfo
        from proto.morai.common.object_info_pb2 import ObjectInfo
        request=PedestrianWaypointInfo(is_create=True)
        info=ObjectInfo(object_type=2,client_key=self.client_key);info.id.value=aid
        request.actor_info.CopyFrom(info)
        self.crossing_waypoints(request.waypoints,plan)
        self.check(self._require()._actor_stub.SetPedestrianWaypoint(request,timeout=3),
                   f'횡단 경로 등록 실패 ID={aid}')

    def pause_actor(self, aid, kind, paused):
        from proto.morai.actor.actor_set_pb2 import EnableActor
        from proto.morai.common.object_info_pb2 import ObjectInfo
        request = EnableActor(enable=paused)
        info = ObjectInfo(object_type=kind, client_key=self.actor_scopes.get(aid,self.client_key))
        info.id.value = aid
        request.actor_info.CopyFrom(info)
        self.check(self._require()._actor_stub.SetPause(request,timeout=3),
                   f'객체 일시 정지 설정 실패 ID={aid}')

    def set_environment(self, weather, hour, instantly=True):
        from proto.morai.environment.weather_pb2 import Weather
        from proto.morai.environment.time_pb2 import SimulationTime
        from proto.morai.environment import environment_enum_pb2 as enum
        from proto.morai.common.type_pb2 import Empty
        types={'Sunny':enum.WEATHER_TYPE_SUNNY,'Foggy':enum.WEATHER_TYPE_FOGGY}
        if weather not in types or hour not in (11,13,15):
            raise ValueError('날씨/시간은 Sunny/Foggy와 11/13/15시 중 선택해야 합니다.')
        stub=self._require()._environment_stub
        self.check(stub.SetWeather(Weather(type=types[weather],instantly=instantly),timeout=3),'날씨 변경 실패')
        self.check(stub.SetTime(SimulationTime(hour=hour,data_only=False,instantly=instantly),timeout=3),'시간 변경 실패')
        # MORAI acknowledges SetTime before the render/environment frame has
        # committed it. Poll readback without resending or choosing a new pair.
        deadline=time.monotonic()+3.
        while True:
            actual_weather=stub.GetWeather(Empty(),timeout=3)
            actual_time=stub.GetTime(Empty(),timeout=3)
            if actual_weather.type==types[weather] and actual_time.hour==hour:return
            if time.monotonic()>=deadline:
                raise RuntimeError(f'날씨/시간 확인 불일치: 요청 {weather} {hour}:00 · '
                                   f'조회 type={actual_weather.type} {actual_time.hour}:00')
            time.sleep(.05)

    def ego_contacts(self):
        from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter
        result = self._require()._actor_stub.GetAllActorsState(
            GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=.5)
        self.contact_states = list(result.states)
        self.contact_native_timestamp = getattr(result,'timestamp',None)
        self.contact_tick_count = getattr(result,'sync_mode_tick_count',None)
        state = next((s for s in result.states if s.actor_info.id.value==self.ego_id),None)
        if state is None:
            raise RuntimeError('ego 접촉 상태를 수신하지 못했습니다.')
        return tuple(str(c) for c in state.vehicle_state.collision_objects if str(c)!=self.ego_id)

    def set_speed_limit(self, aid, speed_kmh):
        from proto.morai.actor.actor_set_pb2 import VehicleDynamicsSpeedLimit
        if not math.isfinite(speed_kmh) or speed_kmh < 0:
            raise ValueError('속도 상한은 유한한 0 이상 값이어야 합니다.')
        request = VehicleDynamicsSpeedLimit(speed_limiter=True, speed_limit=speed_kmh/3.6, reset=False)
        request.actor_info.CopyFrom(self.info(aid))
        self.check(self._require()._actor_stub.SetVehicleDynamicsSpeedLimit(request, timeout=3),
                   '차량 속도 상한 적용 실패')

    def spawn(self, model, speed, plan, on_created=None, paused=False):
        from proto.morai.actor.actor_spawn_pb2 import VehicleSpawnParam
        from proto.morai.actor.actor_set_pb2 import VehicleRoute, VehicleDestination, EnableActor
        stub = self._require()._actor_stub
        # Hold only during route setup; departure has no ego/signal gate.
        request = VehicleSpawnParam(velocity=speed, pause=True, multi_ego=False)
        request.spawn_info.actor_info.CopyFrom(self.info(''))
        # Native destination deletion can leave an ID registration behind. A
        # fresh registration scope prevents reused native IDs colliding with it.
        spawn_scope = self.client_key + '_spawn_' + uuid.uuid4().hex
        request.spawn_info.actor_info.client_key = spawn_scope
        request.spawn_info.model_name = model
        request.spawn_info.label = f'roundabout_{model}_{speed}'
        pose = plan.cfg['spawn_pose']
        loc = request.spawn_info.transform.location
        loc.x, loc.y, loc.z = pose['x'], pose['y'], pose['z']
        request.spawn_info.transform.rotation.z = pose['yaw_deg']
        response = stub.SpawnVehicle(request, timeout=5)
        aid = str(getattr(response,'custom_message','')).strip()
        if response.status != 1:
            raise RuntimeError(f'차량 생성 실패: {response.status} {response.description} · 모델 {model} · '
                               f'등록 범위 {spawn_scope} · 생성 ID 미확인: 추가 생성 중단')
        if not aid.isdigit() or int(aid)<=0:
            raise RuntimeError('생성된 차량의 MORAI ID를 받지 못했습니다. 설정을 중단했습니다.')
        self.actor_scopes[aid] = spawn_scope
        if on_created is not None:
            on_created(aid)
        route = VehicleRoute(decision_range=plan.cfg['decision_range_m'])
        route.actor_info.CopyFrom(self.info(aid))
        for i, lid in enumerate(plan.cfg['npc_links']):
            point = route.links.add()
            point.id.value = lid
            point.waypoint_idx = (plan.cfg['start_point_index'] if i == 0 else
                                  plan.cfg['end_point_index'] if i == len(plan.cfg['npc_links'])-1 else 0)
        self.check(stub.SetVehicleRoute(route, timeout=5), '차량 경로 등록 실패')
        destination = VehicleDestination(decision_range=plan.cfg['decision_range_m'])
        destination.actor_info.CopyFrom(self.info(aid))
        destination.position.x, destination.position.y, destination.position.z = plan.destination
        # This simulator's SetVehicleDestination handler selects DESTROY (1).
        self.check(stub.SetVehicleDestination(destination, timeout=5), '목적지 등록 실패')
        # Spawn velocity alone does not select/guarantee a per-NPC cruise target.
        # Same per-actor dynamics limiter used by scenario_runner's bridge.
        self.set_speed_limit(aid, speed)
        resume = EnableActor(enable=paused)
        resume.actor_info.CopyFrom(self.info(aid))
        self.check(stub.SetPause(resume, timeout=3), '차량 출발 실패')
        return aid

    def restore_actor(self, aid, kind, state):
        """Restore a recorded pose/speed while the replay preparation barrier holds."""
        from proto.morai.actor.actor_set_pb2 import SetTransformParam, SetVelocityParam
        from proto.morai.common.object_info_pb2 import ObjectInfo
        info=ObjectInfo(object_type=kind,client_key=self.actor_scopes.get(aid,self.client_key));info.id.value=aid
        pose=state['pose'];r=SetTransformParam();r.actor_info.CopyFrom(info)
        r.transform.location.x,r.transform.location.y,r.transform.location.z=pose['x'],pose['y'],pose['z']
        r.transform.rotation.x,r.transform.rotation.y,r.transform.rotation.z=pose['roll'],pose['pitch'],pose['yaw']
        self.check(self._require()._actor_stub.SetTransform(r,timeout=3),'재현 객체 위치 복원 실패')
        if kind==1:
            v=SetVelocityParam(velocity=state['speed_kmh']);v.actor_info.CopyFrom(info)
            self.check(self._require()._actor_stub.SetVelocity(v,timeout=3),'재현 차량 속도 복원 실패')
