"""MORAI gRPC 연결: 차·장애물 읽기, 장애물 스폰·삭제, 모델 크기 측정.

SimulationWorld 는 생성할 때 액터를 지우므로 쓰지 않고 SimAdapter 만 직접 쓴다.
"""
from __future__ import annotations

import math
import sys
import time
import uuid
from dataclasses import dataclass

from scen_runner.config import repo_path, require_morai_endpoint
from scen_runner.planning.vehicle import Obstacle

OBJECT_TYPE_NAMES = {1: "차량", 2: "보행자", 3: "장애물"}


@dataclass
class EgoState:
    x: float
    y: float
    z: float
    yaw: float
    speed_kmh: float
    link: str
    tl_id: str
    tl_color: int
    collisions: list[str]


class MoraiLink:
    def __init__(self, cfg: dict):
        mcfg = cfg["morai"]
        self.host = mcfg["host"]
        self.port = mcfg["port"]
        self.client_key = mcfg["client_key"]
        self.ego_id = mcfg.get("ego_id", "Ego")
        grpc_src = str(repo_path(mcfg["grpc_src"]))
        for p in (grpc_src, f"{grpc_src}/api", f"{grpc_src}/proto"):
            if p not in sys.path:
                sys.path.append(p)
        self._sa = None

    # ------------------------------------------------------------ connection
    def connect(self, timeout: float = 3.0) -> str:
        self.host, self.port = require_morai_endpoint(dict(host=self.host, port=self.port))
        import grpc
        from proto.sim_adapter import SimAdapter

        self.disconnect()
        sa = SimAdapter()
        sa.connect(self.host, self.port)
        try:
            grpc.channel_ready_future(sa._channel).result(timeout=timeout)
        except grpc.FutureTimeoutError as exc:
            sa.disconnect()
            raise ConnectionError(f"MORAI gRPC {self.host}:{self.port} 에 연결할 수 없습니다.") from exc
        self._sa = sa
        return f"{self.host}:{self.port}"

    def disconnect(self) -> None:
        adapter = self._sa
        self._sa = None
        if adapter is not None:
            adapter.disconnect()

    @property
    def connected(self) -> bool:
        return self._sa is not None

    def _require(self):
        if self._sa is None:
            raise ConnectionError("MORAI 에 연결되어 있지 않습니다.")
        return self._sa

    # ------------------------------------------------------------ read
    def available_obstacle_models(self) -> list[str]:
        from proto.morai.simulator.category_obstacles_pb2 import CategoryObstacles

        param = CategoryObstacles()
        param.obstacle = True
        objs = self._require()._simulator_stub.GetAvailableObject(param, timeout=5.0)
        return list(objs.obstacle)

    def read_actors(self) -> tuple[EgoState | None, list[Obstacle]]:
        """시뮬레이터의 모든 액터. 장애물은 BoundingBox 로 크기를 채운다."""
        from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter

        flt = GetAllActorsFilter()
        # 이 client의 객체는 요청 ID로 받아 개별 조회/삭제 ID와 일치시킨다.
        # 키를 생략하면 MORAI 내부 숫자 ID로 바뀌며 client_key도 비어 있다.
        flt.client_key = self.client_key
        flt.vehicle = flt.pedestrian = flt.obstacle = True
        states = self._require()._actor_stub.GetAllActorsState(flt, timeout=3.0)
        ego, obstacles = None, []
        for st in states.states:
            aid = st.actor_info.id.value
            if aid == self.ego_id:
                ego = self._ego_from_state(st)
                continue
            obstacles.append(self._obstacle_from_state(st))
        return ego, obstacles

    def _ego_from_state(self, st) -> EgoState:
        v = st.velocity
        vs = st.vehicle_state
        return EgoState(
            x=st.transform.location.x, y=st.transform.location.y, z=st.transform.location.z,
            yaw=math.radians(st.transform.rotation.z),
            speed_kmh=math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
            link=vs.current_link_info.id.value, tl_id=vs.tl_id.value,
            tl_color=int(vs.tl_color),
            collisions=[c for c in vs.collision_objects if c != self.ego_id])

    @staticmethod
    def _obstacle_from_state(st) -> Obstacle:
        bb = st.bounding_box
        tf = st.transform
        has_box = bb.extent.x > 0 and bb.extent.y > 0
        cx, cy = (bb.location.x, bb.location.y) if has_box else (tf.location.x, tf.location.y)
        info = st.actor_info
        ob = Obstacle(x=cx, y=cy, yaw=math.radians(tf.rotation.z),
                      length=2.0 * bb.extent.y if has_box else 1.0,
                      width=2.0 * bb.extent.x if has_box else 1.0,
                      height=2.0 * bb.extent.z if has_box else 1.0,
                      model=OBJECT_TYPE_NAMES.get(int(info.object_type), "액터"),
                      z=tf.location.z, source="morai", actor_id=info.id.value)
        ob.extra = {"client_key": info.client_key, "object_type": int(info.object_type),
                    "spawn_x": tf.location.x, "spawn_y": tf.location.y}
        return ob

    # ------------------------------------------------------------ write
    def spawn_obstacle(self, model: str, x: float, y: float, z: float, yaw_deg: float,
                       label: str = "scen_runner") -> str:
        from proto.morai.actor.actor_spawn_pb2 import ObstacleSpawnParam
        from proto.morai.common.enum_pb2 import OBJECT_TYPE_OBSTACLE, STATUS_CODE_SUCCESS

        param = ObstacleSpawnParam()
        info = param.spawn_info
        # 현재 MORAI 빌드는 빈 요청 ID로 스폰하면 Unknown failure를 반환한다.
        info.actor_info.id.value = f"{self.client_key}_{uuid.uuid4().hex[:12]}"
        info.actor_info.object_type = OBJECT_TYPE_OBSTACLE
        info.actor_info.client_key = self.client_key
        info.transform.location.x = float(x)
        info.transform.location.y = float(y)
        info.transform.location.z = float(z)
        info.transform.rotation.z = float(yaw_deg)
        info.model_name = model
        info.label = label
        param.scale.x = param.scale.y = param.scale.z = 1.0
        res = self._require()._actor_stub.SpawnObstacle(param, timeout=5.0)
        if res.status != STATUS_CODE_SUCCESS:
            raise RuntimeError(f"'{model}' 스폰 실패: status={res.status} {res.description}")
        return res.custom_message

    def destroy(self, actor_id: str, object_type: int = 3, client_key: str | None = None) -> bool:
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS
        from proto.morai.common.object_info_pb2 import ObjectInfo

        info = ObjectInfo()
        info.id.value = actor_id
        info.object_type = object_type
        info.client_key = self.client_key if client_key is None else client_key
        res = self._require()._actor_stub.DestroyActor(info, timeout=5.0)
        return res.status == STATUS_CODE_SUCCESS

    def destroy_mine(self) -> bool:
        """이 도구(client_key)로 스폰한 액터만 지운다."""
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS
        from proto.morai.common.type_pb2 import StringValue

        param = StringValue()
        param.value = self.client_key
        res = self._require()._actor_stub.DestroyAllActors(param, timeout=5.0)
        return res.status == STATUS_CODE_SUCCESS

    def read_obstacle(self, actor_id: str) -> Obstacle:
        from proto.morai.common.object_info_pb2 import ObjectInfo

        info = ObjectInfo()
        info.id.value = actor_id
        info.object_type = 3
        info.client_key = self.client_key
        st = self._require()._actor_stub.GetActorState(info, timeout=5.0)
        return self._obstacle_from_state(st)

    def measure_model(self, model: str, x: float, y: float, z: float,
                      settle_sec: float = 0.3) -> dict:
        """모델을 잠깐 스폰해서 BoundingBox 를 읽고 지운다. yaw=0 으로 스폰한다."""
        from proto.morai.common.object_info_pb2 import ObjectInfo

        actor_id = self.spawn_obstacle(model, x, y, z, 0.0, label=f"measure_{model}")
        try:
            time.sleep(settle_sec)
            info = ObjectInfo()
            info.id.value = actor_id
            info.object_type = 3
            info.client_key = self.client_key
            st = self._require()._actor_stub.GetActorState(info, timeout=5.0)
            bb, tf = st.bounding_box, st.transform
            yaw = math.radians(tf.rotation.z)
            dx, dy = bb.location.x - tf.location.x, bb.location.y - tf.location.y
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            return {"morai_model": model, "length": round(2.0 * bb.extent.y, 3),
                    "width": round(2.0 * bb.extent.x, 3), "height": round(2.0 * bb.extent.z, 3),
                    "center_offset": [round(fwd, 3), round(left, 3)], "measured": True}
        finally:
            if not self.destroy(actor_id):
                raise RuntimeError(f"측정용 객체 삭제 실패: {actor_id}")
