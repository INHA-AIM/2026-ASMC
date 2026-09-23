#!/usr/bin/env python3
"""구운 맵을 MORAI 실주행과 대조 검증한다.

MORAI 의 Ego 상태를 받아 (1) ego 가 구운 주행가능영역 위에 있는지, (2) MORAI 가
직접 알려주는 좌/우 차선까지 거리가 합성 차선과 맞는지 확인하고, ego 중심 BEV 를
띄운다. 맵 판본이 어긋나면 ego 가 즉시 도로 밖으로 나가므로 바로 드러난다.

    # MORAI Ego 를 UDP 로 받는 경우 (ROS 빌드 불필요, 권장)
    python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py

    # 이미 /Ego_topic 이 뜬 ROS 환경
    python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source ros

    # MORAI 없이 뷰어만 확인
    python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source demo

MORAI 설정: Network Settings > Ego Vehicle Status 를 이 호스트 IP / 포트 9011 로.
WSL 이면 호스트 IP 는 `hostname -I` 의 첫 값을 쓴다.
"""
from __future__ import annotations

import argparse
import json
import math
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import h5py
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mapbake.raster import RasterGrid

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_H5 = Path(__file__).resolve().parent / 'out' / 'link_node_map.h5'

# --- MORAI Ego UDP 패킷 (src/asmc_bridge/scripts/morai_udp_bridge.py 와 동일) ---
EGO_HDR = b'#MoraiInfo$'
EGO_PRE = len(EGO_HDR) + 4 + 12   # 27
TAIL = 2
EGO_COMMON_FMT = '<25f'           # body offset 10~109
# 전체 길이 -> 확장 필드 (offset, 개수). 245 판본만 차선 거리 3개를 포함한다.
EGO_VARIANTS = {181: (152, 0), 229: (152, 12), 245: (156, 15)}
# morai_udp_bridge.launch 의 ego_port 와 같은 값. MORAI Network Settings 의
# 'Ego Vehicle Status' Destination PORT 다.
EGO_PORT_DEFAULT = 909

# MORAI Object Info UDP (morai_udp_bridge.py 의 OBJ_* 와 동일)
OBJ_HDR = b'#MoraiObjInfo$'
OBJ_PRE = len(OBJ_HDR) + 4 + 12      # 30
OBJ_ONE_FMT = '<hh' + 'fff' + 'f' + 'fff' + 'fff' + 'fff' + 'fff' + '38s'
OBJ_ONE = 106
OBJ_N = 20
OBJECT_PORT_DEFAULT = 7505
OBJ_TYPE_NAME = {0: 'ped', 1: 'npc'}

EXT_NAMES = [
    'tire_lateral_force_fl', 'tire_lateral_force_fr',
    'tire_lateral_force_rl', 'tire_lateral_force_rr',
    'side_slip_angle_fl', 'side_slip_angle_fr',
    'side_slip_angle_rl', 'side_slip_angle_rr',
    'tire_cornering_stiffness_fl', 'tire_cornering_stiffness_fr',
    'tire_cornering_stiffness_rl', 'tire_cornering_stiffness_rr',
    'distance_left_lane_boundary', 'distance_right_lane_boundary',
    'cross_track_error',
]


def host_ips() -> list:
    """MORAI(Windows) 가 목적지로 지정해야 하는 이 호스트의 IP 후보."""
    ips = []
    try:
        s_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s_.connect(('8.8.8.8', 80))
        ips.append(s_.getsockname()[0])
        s_.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith('127.'):
                ips.append(ip)
    except OSError:
        pass
    return ips


@dataclass
class ObjectState:
    """MORAI Object Info 한 건. 위치는 맵 로컬 좌표(m)."""
    idx: int
    type_id: int
    x: float
    y: float
    heading_deg: float
    length_m: float
    width_m: float

    @property
    def type_name(self) -> str:
        return OBJ_TYPE_NAME.get(self.type_id, f'obs{self.type_id}')


def object_dimensions(obj: ObjectState) -> Tuple[float, float]:
    """MORAI가 보낸 실제 평면 크기(L, W)를 반환한다.

    양수인 size는 작더라도 절대 확대하지 않는다. 패킷이 0/NaN을 보낸 경우에만
    화면과 영역 판정이 망가지지 않도록 타입별 대표 크기를 사용한다.
    """
    fallbacks = {
        0: (0.8, 0.8),   # pedestrian
        1: (4.6, 1.9),   # npc vehicle
        2: (0.5, 0.5),   # static obstacle
    }
    fallback = fallbacks.get(obj.type_id, (0.5, 0.5))
    length = float(obj.length_m)
    width = float(obj.width_m)
    if not np.isfinite(length) or length <= 0.0:
        length = fallback[0]
    if not np.isfinite(width) or width <= 0.0:
        width = fallback[1]
    return length, width


def parse_object_packet(data: bytes) -> Optional[list]:
    """#MoraiObjInfo$ 패킷 -> ObjectState 목록."""
    if not data.startswith(OBJ_HDR) or len(data) <= OBJ_PRE + TAIL:
        return None
    body = data[OBJ_PRE:-TAIL]
    if len(body) < 8:
        return None
    out = []
    off = 8
    for _ in range(OBJ_N):
        if off + OBJ_ONE > len(body):
            break
        v = struct.unpack(OBJ_ONE_FMT, body[off:off + OBJ_ONE])
        off += OBJ_ONE
        obj_id, obj_type = v[0], v[1]
        # 빈 슬롯은 id 0 에 좌표도 0 이다
        if obj_id == 0 and abs(v[2]) < 1e-6 and abs(v[3]) < 1e-6:
            continue
        out.append(ObjectState(idx=int(obj_id), type_id=int(obj_type),
                               x=float(v[2]), y=float(v[3]),
                               heading_deg=float(v[5]),
                               length_m=float(v[6]), width_m=float(v[7])))
    return out


@dataclass(frozen=True)
class FieldLayout:
    """본문 float 배열에서 각 값이 몇 번째인지.

    기본값은 morai_udp_bridge.py 가 쓰는 배치다. 시뮬 판본이 다르면
    `--probe --analyze` 로 실제 인덱스를 찾아 --pos-index 등으로 넘긴다.
    """
    pos: int = 10        # x = f[pos], y = f[pos+1]
    heading: int = 15
    velocity: int = 0


DEFAULT_LAYOUT = FieldLayout()


@dataclass
class EgoState:
    x: float
    y: float
    heading_deg: float
    velocity_kmh: float = 0.0
    dist_left_lane: Optional[float] = None
    dist_right_lane: Optional[float] = None


def parse_ego_packet(
    data: bytes, layout: FieldLayout = DEFAULT_LAYOUT
) -> Optional[EgoState]:
    if not data.startswith(EGO_HDR):
        return None
    var = EGO_VARIANTS.get(len(data))
    if var is None:
        return None
    ext_off, ext_n = var
    body = data[EGO_PRE:-TAIL]
    if len(body) < 110:
        return None
    f = struct.unpack(EGO_COMMON_FMT, body[10:110])
    if layout.pos + 1 >= len(f) or layout.heading >= len(f):
        return None
    state = EgoState(x=f[layout.pos], y=f[layout.pos + 1],
                     heading_deg=f[layout.heading],
                     velocity_kmh=f[layout.velocity])
    if ext_n >= 15:
        ext = struct.unpack('<%df' % ext_n, body[ext_off:ext_off + ext_n * 4])
        d = dict(zip(EXT_NAMES, ext))
        state.dist_left_lane = d.get('distance_left_lane_boundary')
        state.dist_right_lane = d.get('distance_right_lane_boundary')
    return state


def bind_udp(port: int, timeout_s: Optional[float] = 1.0) -> socket.socket:
    """UDP 소켓 바인드. 특권 포트 실패를 사람이 읽을 수 있는 안내로 바꾼다."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', port))
    except PermissionError:
        sock.close()
        raise SystemExit(
            f'udp/{port} 바인드에 권한이 필요합니다 (1024 미만 특권 포트).\n'
            f'  방법 1) sudo -E python3 {sys.argv[0]} ...\n'
            f'          (-E 를 붙여야 DISPLAY 가 유지되어 창이 뜹니다)\n'
            f'  방법 2) MORAI 의 Ego Destination PORT 를 1024 이상으로 바꾸고\n'
            f'          --port 로 같은 값을 넘긴다 (예: --port 9909)\n'
            f'  참고) docker 브릿지가 909 를 쓰는 건 컨테이너가 root 로 돌기 때문입니다.'
        )
    except OSError as e:
        sock.close()
        raise SystemExit(f'udp/{port} 바인드 실패: {e} (이미 쓰고 있는 프로세스가 있는지 확인)')
    sock.settimeout(timeout_s)   # 0.0 이면 논블로킹, None 이면 무한 블로킹
    return sock


class UdpEgoSource:
    """MORAI Ego UDP 를 직접 받는다. ROS 도 morai_msgs 도 필요 없다."""

    def __init__(self, port: int, timeout_s: float = 1.0,
                 layout: FieldLayout = DEFAULT_LAYOUT) -> None:
        self.sock = bind_udp(port, timeout_s)
        self.port = port
        self.layout = layout
        self.bad_len = set()

    def read(self) -> Optional[EgoState]:
        try:
            data, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        state = parse_ego_packet(data, self.layout)
        if state is None and len(data) not in self.bad_len:
            self.bad_len.add(len(data))
            print(f'[warn] 해석 못 한 Ego 패킷 {len(data)}B '
                  f'(지원: {sorted(EGO_VARIANTS)}) head={data[:16]!r}')
        return state

    def describe(self) -> str:
        return f'UDP 0.0.0.0:{self.port}'

    def close(self) -> None:
        self.sock.close()


def _nmea_deg(val: str, hemi: str) -> float:
    """NMEA ddmm.mmmm -> 십진 도."""
    if not val:
        return 0.0
    v = float(val)
    deg = int(v / 100.0)
    out = deg + (v - deg * 100.0) / 60.0
    return -out if hemi in ('S', 'W') else out


class GpsEgoSource:
    """위치는 GPS(NMEA), 방향은 Ego 패킷에서 가져온다.

    beta(181B) Ego 패킷은 position 이 0 으로 채워져 오는 경우가 있다. 그럴 때
    위치만 GPS 센서로 대체한다. GPS 의 NMEA GGA 에는 맵 원점 오프셋이 없으므로
    위경도를 UTM 으로 옮긴 뒤 global_info 의 local_origin_in_global 을 빼서
    맵 로컬 좌표로 만든다.
    """

    def __init__(self, gps_port: int, ego_port: int, origin_xy,
                 utm_epsg: int = 32652, layout: 'FieldLayout' = None) -> None:
        import pyproj
        # 논블로킹으로 둔다. 타임아웃 방식으로 '버퍼가 빌 때까지' 읽으면,
        # 패킷이 타임아웃보다 자주 오는 순간(MORAI 는 ~50 Hz) 루프가 영원히
        # 빠져나오지 못해 위치가 갱신되지 않는다.
        self.gps_sock = bind_udp(gps_port, 0.0)
        self.ego_sock = bind_udp(ego_port, 0.0)
        self.gps_port, self.ego_port = gps_port, ego_port
        self.origin = np.asarray(origin_xy, dtype=np.float64)[:2]
        self.tf = pyproj.Transformer.from_crs('EPSG:4326', f'EPSG:{utm_epsg}',
                                              always_xy=True)
        self.layout = layout or DEFAULT_LAYOUT
        self._xy = None
        self._heading = 0.0
        self._vel = 0.0
        self._got_gps = False

    def _drain_gps(self) -> None:
        """지금 버퍼에 있는 것만 비우고, 가장 최근 위치만 남긴다."""
        while True:
            try:
                raw, _ = self.gps_sock.recvfrom(8196)
            except (socket.timeout, BlockingIOError, OSError):
                return
            f = raw.decode('ascii', 'ignore').split('\r\n')[0].split(',')
            if len(f) < 10 or 'GGA' not in f[0]:
                continue
            try:
                lat = _nmea_deg(f[2], f[3])
                lon = _nmea_deg(f[4], f[5])
            except (ValueError, IndexError):
                continue
            if lat == 0.0 and lon == 0.0:
                continue
            east, north = self.tf.transform(lon, lat)
            self._xy = (east - self.origin[0], north - self.origin[1])
            self._got_gps = True

    def _drain_ego(self) -> None:
        while True:
            try:
                data, _ = self.ego_sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError, OSError):
                return
            st = parse_ego_packet(data, self.layout)
            if st is not None:
                self._heading = st.heading_deg
                self._vel = st.velocity_kmh

    def read(self) -> Optional[EgoState]:
        # 두 소켓 중 하나에 데이터가 올 때까지만 잠깐 기다린다 (무한 배수 방지).
        select.select([self.gps_sock, self.ego_sock], [], [], 0.1)
        self._drain_gps()
        self._drain_ego()
        if self._xy is None:
            return None
        return EgoState(x=self._xy[0], y=self._xy[1],
                        heading_deg=self._heading, velocity_kmh=self._vel)

    def describe(self) -> str:
        return f'GPS udp/{self.gps_port} (위치) + Ego udp/{self.ego_port} (방향)'

    def close(self) -> None:
        self.gps_sock.close()
        self.ego_sock.close()


class RosEgoSource:
    """이미 /Ego_topic 이 뜬 환경용.

    위치는 **/gps 를 우선 쓴다.** beta(181B) Ego 패킷은 position 이 0 으로 채워져
    오고, 브릿지가 그 값을 그대로 /Ego_topic 에 넣기 때문이다 (실측: position
    (0,0) 인데 heading 은 정상). /gps 는 위경도와 함께 맵 원점 offset
    (eastOffset / northOffset) 을 메시지에 담아 주므로 그것만으로 맵 로컬 좌표가
    나온다 — H5 의 local_origin 을 쓰지 않아도 된다.

    Ego position 이 0 이 아니면 그쪽을 그대로 쓴다.
    """

    def __init__(self, topic: str = '/Ego_topic', gps_topic: str = '/gps',
                 utm_epsg: int = 32652, fallback_origin=None) -> None:
        import rospy
        from morai_msgs.msg import EgoVehicleStatus
        self._rospy = rospy
        rospy.init_node('link_node_live_check', anonymous=True, disable_signals=True)
        self.topic = topic
        self.gps_topic = gps_topic
        self._heading = None          # deg
        self._vel_kmh = 0.0
        self._ego_xy = None           # Ego 패킷이 준 위치 (보통 0)
        self._gps_xy = None           # /gps 에서 변환한 위치
        self._announced = False
        self._fallback_origin = (np.asarray(fallback_origin, dtype=np.float64)[:2]
                                 if fallback_origin is not None else None)
        self._tf = None
        self._utm_epsg = utm_epsg
        rospy.Subscriber(topic, EgoVehicleStatus, self._ego_cb, queue_size=1)
        try:
            from morai_msgs.msg import GPSMessage
            rospy.Subscriber(gps_topic, GPSMessage, self._gps_cb, queue_size=1)
            self._gps_ok = True
        except ImportError:
            self._gps_ok = False

    def _ego_cb(self, msg) -> None:
        self._heading = float(msg.heading)
        self._vel_kmh = float(getattr(msg.velocity, 'x', 0.0)) * 3.6
        x, y = float(msg.position.x), float(msg.position.y)
        self._ego_xy = (x, y) if (abs(x) > 1e-6 or abs(y) > 1e-6) else None

    def _gps_cb(self, msg) -> None:
        lat, lon = float(msg.latitude), float(msg.longitude)
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return
        if self._tf is None:
            import pyproj
            self._tf = pyproj.Transformer.from_crs(
                'EPSG:4326', f'EPSG:{self._utm_epsg}', always_xy=True)
        east, north = self._tf.transform(lon, lat)
        # 메시지가 맵 원점 offset 을 직접 준다. 0 이면 H5 의 local_origin 으로 대체.
        off = np.array([float(getattr(msg, 'eastOffset', 0.0)),
                        float(getattr(msg, 'northOffset', 0.0))], dtype=np.float64)
        if not off.any() and self._fallback_origin is not None:
            off = self._fallback_origin
        self._gps_xy = (east - off[0], north - off[1])

    def read(self) -> Optional[EgoState]:
        time.sleep(0.02)
        if self._heading is None:
            return None
        xy = self._ego_xy or self._gps_xy
        if xy is None:
            return None
        if not self._announced:
            src = '/Ego_topic' if self._ego_xy else self.gps_topic
            print(f'[src] 위치는 {src}, 방향은 {self.topic}')
            self._announced = True
        return EgoState(x=xy[0], y=xy[1], heading_deg=self._heading,
                        velocity_kmh=self._vel_kmh)

    def describe(self) -> str:
        return f'ROS {self.topic}' + (f' + {self.gps_topic}' if self._gps_ok else '')

    def close(self) -> None:
        pass


class DemoEgoSource:
    """MORAI 없이 뷰어만 확인. 맵 중심 근처를 원운동한다."""

    def __init__(self, grid: RasterGrid, road: np.ndarray) -> None:
        rows, cols = np.nonzero(road[::17, ::17])
        pick = len(rows) // 2
        self.center = grid.to_world(
            np.array([[cols[pick] * 17, rows[pick] * 17]], dtype=np.float64))[0]
        self.t = 0.0

    def read(self) -> Optional[EgoState]:
        self.t += 0.08
        r = 15.0
        return EgoState(
            x=float(self.center[0] + r * math.cos(self.t)),
            y=float(self.center[1] + r * math.sin(self.t)),
            heading_deg=math.degrees(self.t + math.pi / 2) % 360.0,
            velocity_kmh=20.0,
        )

    def describe(self) -> str:
        return 'demo (가상 ego)'

    def close(self) -> None:
        pass


@dataclass
class BakedMap:
    road: np.ndarray
    lane: np.ndarray
    lane_white: np.ndarray
    lane_yellow: np.ndarray
    lane_segment_a: np.ndarray
    lane_segment_b: np.ndarray
    lane_segment_colors: list
    lane_paint_width_m: float
    stopline: np.ndarray
    grid: RasterGrid
    map_id: str
    local_origin: np.ndarray
    stop_pts: np.ndarray          # (N, 2) 정지선 위치
    stop_ids: list                # (N,) 노드 id
    stop_tls: list                # (N,) 신호등 id ('' 이면 없음)
    stop_headings: np.ndarray     # (N, 2) 정지선을 통과하는 주행 방향
    link_pts: np.ndarray          # (M, 2) 모든 링크 중심선을 이어붙인 점
    link_owner: np.ndarray        # (M,) 각 점이 속한 링크 인덱스
    link_ids: list                # (N,) 링크 id
    link_in_road: list            # (N,) 주행가능영역에 쓰였는지
    link_in_lane: list            # (N,) 차선에 쓰였는지

    @classmethod
    def load(cls, path: Path) -> 'BakedMap':
        with h5py.File(path, 'r') as hf:
            grid = RasterGrid(
                world_offset=np.asarray(hf.attrs['world_offset_in_meters'], dtype=np.float32),
                pixels_per_meter=float(hf.attrs['pixels_per_meter']),
                width_px=int(hf.attrs['width_in_pixels']),
                height_px=int(hf.attrs['height_in_pixels']),
            )
            pts = (np.asarray(hf['stopline_points'], dtype=np.float32)
                   if 'stopline_points' in hf else np.zeros((0, 2), dtype=np.float32))
            heads = (np.asarray(hf['stopline_headings'], dtype=np.float32)
                     if 'stopline_headings' in hf else np.zeros((0, 2), dtype=np.float32))
            ids = json.loads(hf.attrs.get('stopline_ids', '[]'))
            tls = json.loads(hf.attrs.get('stopline_traffic_lights', '[]'))
            lids = json.loads(hf.attrs.get('link_ids', '[]'))
            lroad = json.loads(hf.attrs.get('link_in_road', '[]'))
            llane = json.loads(hf.attrs.get('link_in_lane', '[]'))
            if 'link_points' in hf:
                lpts = np.asarray(hf['link_points'], dtype=np.float32)
                offs = np.asarray(hf['link_offsets'], dtype=np.int64)
                lowner = np.zeros(lpts.shape[0], dtype=np.int32)
                for k in range(len(offs) - 1):
                    lowner[offs[k]:offs[k + 1]] = k
            else:
                lpts = np.zeros((0, 2), dtype=np.float32)
                lowner = np.zeros(0, dtype=np.int32)
            lane_a, lane_b, lane_colors = [], [], []
            if 'observed_lane_points' in hf and 'observed_lane_offsets' in hf:
                observed_pts = np.asarray(hf['observed_lane_points'], dtype=np.float64)
                observed_offs = np.asarray(hf['observed_lane_offsets'], dtype=np.int64)
                feature_colors = json.loads(hf.attrs.get('observed_lane_colors', '[]'))
                for k in range(len(observed_offs) - 1):
                    p_ = observed_pts[observed_offs[k]:observed_offs[k + 1]]
                    if len(p_) < 2:
                        continue
                    lane_a.append(p_[:-1])
                    lane_b.append(p_[1:])
                    lane_colors.extend([feature_colors[k] if k < len(feature_colors) else '?']
                                       * (len(p_) - 1))
            layer_params = json.loads(hf.attrs.get('layer_params', '{}'))
            paint_width = float((layer_params.get('observed_lane') or {}).get(
                'line_width_m', 0.15))
            return cls(
                road=hf['road'][:], lane=hf['lane'][:],
                lane_white=(hf['lane_observed_white'][:]
                            if 'lane_observed_white' in hf else np.zeros_like(hf['lane'][:])),
                lane_yellow=(hf['lane_observed_yellow'][:]
                             if 'lane_observed_yellow' in hf else np.zeros_like(hf['lane'][:])),
                lane_segment_a=(np.vstack(lane_a) if lane_a else np.zeros((0, 2))),
                lane_segment_b=(np.vstack(lane_b) if lane_b else np.zeros((0, 2))),
                lane_segment_colors=lane_colors,
                lane_paint_width_m=paint_width,
                stopline=hf['stopline'][:],
                grid=grid,
                map_id=str(hf.attrs.get('map_id', '?')),
                local_origin=np.asarray(hf.attrs.get('local_origin_in_global', []),
                                        dtype=np.float64),
                stop_pts=pts, stop_ids=list(ids), stop_tls=list(tls),
                stop_headings=heads,
                link_pts=lpts, link_owner=lowner, link_ids=lids,
                link_in_road=lroad, link_in_lane=llane,
            )

    def sample(self, layer: np.ndarray, x: float, y: float) -> Optional[int]:
        c, r = self.grid.to_pixel(np.array([[x, y]], dtype=np.float64))[0]
        if 0 <= r < layer.shape[0] and 0 <= c < layer.shape[1]:
            return int(layer[r, c])
        return None


def _view_basis(ego: EgoState, window_m: float, out_px: int):
    """뷰 좌표계 기저. world <-> view 변환을 한 곳에서 만든다."""
    yaw = math.radians(ego.heading_deg)
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([fwd[1], -fwd[0]])
    mpp = window_m / out_px
    origin = np.array([ego.x, ego.y])
    return fwd, right, mpp, origin


def view_to_world(c, r, ego: EgoState, window_m: float, out_px: int) -> np.ndarray:
    fwd, right, mpp, origin = _view_basis(ego, window_m, out_px)
    return origin + (out_px / 2 - r) * mpp * fwd + (c - out_px / 2) * mpp * right


def world_to_view(
    points_xy: np.ndarray, ego: EgoState, window_m: float, out_px: int
) -> np.ndarray:
    """월드 (N,2) -> 뷰 픽셀 (N,2) float (col, row)."""
    fwd, right, mpp, origin = _view_basis(ego, window_m, out_px)
    d = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2) - origin
    col = out_px / 2 + d.dot(right) / mpp
    row = out_px / 2 - d.dot(fwd) / mpp
    return np.stack([col, row], axis=1)


def draw_world_grid(
    canvas: np.ndarray, ego: EgoState, window_m: float, out_px: int, spacing_m: float
) -> None:
    """월드 축에 고정된 격자.

    도로와 차선은 진행 방향으로 균일해서, 직선 구간을 달리면 화면이 멈춘 것처럼
    보인다. 월드에 못박힌 기준선이 있어야 전진이 눈에 보인다.
    """
    if spacing_m <= 0:
        return
    reach = window_m * 0.75  # 회전까지 감안한 여유
    for axis in (0, 1):
        lo = math.floor(((ego.y if axis else ego.x) - reach) / spacing_m)
        hi = math.ceil(((ego.y if axis else ego.x) + reach) / spacing_m)
        for k in range(lo, hi + 1):
            v = k * spacing_m
            ends = np.array(
                [[ego.x - reach, v], [ego.x + reach, v]] if axis else
                [[v, ego.y - reach], [v, ego.y + reach]], dtype=np.float64)
            pv = world_to_view(ends, ego, window_m, out_px)
            major = (k % 5 == 0)
            cv2.line(canvas, tuple(np.round(pv[0]).astype(int)),
                     tuple(np.round(pv[1]).astype(int)),
                     (70, 55, 40) if major else (45, 38, 30), 1, cv2.LINE_AA)


def draw_north(canvas: np.ndarray, ego: EgoState, out_px: int) -> None:
    """월드 +y(북) 방향 표시. 조향에 따라 도는지 바로 확인된다."""
    yaw = math.radians(ego.heading_deg)
    cx, cy, r = out_px - 34, 34, 18
    nx, ny = math.sin(yaw), -math.cos(yaw)  # 월드 +y 를 뷰 좌표로
    cv2.circle(canvas, (cx, cy), r + 4, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.arrowedLine(canvas, (cx, cy), (int(cx + r * nx), int(cy + r * ny)),
                    (200, 200, 200), 2, tipLength=0.35)
    cv2.putText(canvas, 'N', (int(cx + (r + 9) * nx) - 4, int(cy + (r + 9) * ny) + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)


def nearest_stop_lines(m: BakedMap, ego: EgoState, count: int = 1) -> list:
    """ego 기준 가까운 정지선들. 종/횡 거리를 분리해 앞뒤를 구분한다.

    유클리드 거리만으로는 '지나친 정지선'과 '앞의 정지선'이 구분되지 않는다.
    """
    if m.stop_pts.shape[0] == 0:
        return []
    yaw = math.radians(ego.heading_deg)
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([fwd[1], -fwd[0]])
    d = m.stop_pts.astype(np.float64) - np.array([ego.x, ego.y])
    ahead = d.dot(fwd)
    lat = d.dot(right)
    dist = np.hypot(ahead, lat)
    order = np.argsort(dist)[:max(1, count)]
    out = []
    for i in order:
        i = int(i)
        face = float('nan')
        if i < m.stop_headings.shape[0]:
            h = m.stop_headings[i]
            face = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(h, fwd))))))
        out.append({
            'facing_deg': face,
            'idx': m.stop_ids[i] if i < len(m.stop_ids) else '?',
            'traffic_light_id': m.stop_tls[i] if i < len(m.stop_tls) else '',
            'distance_m': float(dist[i]),
            'ahead_m': float(ahead[i]),
            'lateral_m': float(lat[i]),
            'x': float(m.stop_pts[i][0]),
            'y': float(m.stop_pts[i][1]),
        })
    return out


def links_around(m: BakedMap, ego: EgoState, radius_m: float = 12.0) -> list:
    """ego 주변 링크를 횡방향 위치(좌 +, 우 -) 와 함께 돌려준다.

    같은 도로에 차로가 여럿이면 '어느 게 그 링크인지' 를 색이나 눈대중으로는
    맞출 수 없다. 좌/우 몇 m 인지로 짚어야 한다.
    """
    if m.link_pts.shape[0] == 0:
        return []
    yaw = math.radians(ego.heading_deg)
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([fwd[1], -fwd[0]])
    d = m.link_pts.astype(np.float64) - np.array([ego.x, ego.y])
    dist = np.linalg.norm(d, axis=1)
    near = np.nonzero(dist <= radius_m)[0]
    best = {}
    for i in near:
        k = int(m.link_owner[i])
        if k not in best or dist[i] < dist[best[k]]:
            best[k] = int(i)
    out = []
    for k, i in best.items():
        # right = fwd 를 -90도 회전한 벡터이므로 d.dot(right) 는 '오른쪽이 양수'다.
        # 여기서 부호를 뒤집었다가 좌우 표시가 반대로 나갔었다.
        lat = float(d[i].dot(right))          # 오른쪽 +, 왼쪽 -
        out.append({
            'idx': m.link_ids[k] if k < len(m.link_ids) else '?',
            'distance_m': float(dist[i]),
            'lateral_m': lat,
            'ahead_m': float(d[i].dot(fwd)),
            'in_road': bool(m.link_in_road[k]) if k < len(m.link_in_road) else False,
            'in_lane': bool(m.link_in_lane[k]) if k < len(m.link_in_lane) else False,
            'owner': k,
        })
    out.sort(key=lambda c: c['lateral_m'])     # 왼쪽(음수)부터
    return out


def draw_link_overlay(canvas: np.ndarray, m: BakedMap, ego: EgoState,
                      window_m: float, out_px: int, around: list) -> None:
    """주변 링크 중심선을 색을 달리해 그리고 id 를 붙인다."""
    palette = [(255, 120, 60), (60, 220, 255), (120, 255, 140), (255, 100, 255),
               (255, 220, 80), (180, 180, 255)]
    for n, c in enumerate(around):
        k = c['owner']
        sel = m.link_owner == k
        pts = m.link_pts[sel]
        if pts.shape[0] < 2:
            continue
        col = palette[n % len(palette)]
        v = world_to_view(pts, ego, window_m, out_px)
        ok = ((v[:, 0] > -out_px) & (v[:, 0] < 2 * out_px)
              & (v[:, 1] > -out_px) & (v[:, 1] < 2 * out_px))
        if ok.sum() < 2:
            continue
        cv2.polylines(canvas, [np.round(v[ok]).astype(np.int32).reshape(-1, 1, 2)],
                      False, col, 2, cv2.LINE_AA)
        anchor = np.round(v[ok][len(v[ok]) // 2]).astype(int)
        label = f"{c['idx'][-4:]} {c['lateral_m']:+.1f}m"
        cv2.putText(canvas, label, (int(anchor[0]) + 5, int(anchor[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (int(anchor[0]) + 5, int(anchor[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)


class ObjectSource:
    """MORAI Object Info UDP 수신기. 위치는 이미 맵 로컬 좌표라 변환이 없다."""

    def __init__(self, port: int) -> None:
        self.sock = bind_udp(port, 0.0)       # 논블로킹
        self.port = port
        self.objects: list = []
        self.bad_len = set()

    def poll(self) -> list:
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, BlockingIOError, OSError):
                break
            objs = parse_object_packet(data)
            if objs is None:
                if len(data) not in self.bad_len:
                    self.bad_len.add(len(data))
                    print(f'[warn] 해석 못 한 Object 패킷 {len(data)}B '
                          f'head={data[:16]!r}')
                continue
            self.objects = objs
        return self.objects

    def close(self) -> None:
        self.sock.close()


class RosObjectSource:
    """ROS /Object_topic 에서 객체를 받는다.

    컨테이너에서 브릿지가 이미 UDP 7505 를 잡고 있으면 ObjectSource(UDP) 로는
    바인드할 수 없다. 그럴 때 이 경로를 쓴다. 위치는 이미 맵 로컬 좌표다.
    """

    def __init__(self, topic: str = '/Object_topic') -> None:
        import rospy
        from morai_msgs.msg import ObjectStatusList
        self.topic = topic
        self.objects: list = []
        rospy.Subscriber(topic, ObjectStatusList, self._cb, queue_size=1)

    def _cb(self, msg) -> None:
        out = []
        groups = ((getattr(msg, 'pedestrian_list', []), 0),
                  (getattr(msg, 'npc_list', []), 1),
                  (getattr(msg, 'obstacle_list', []), 2))
        for items, type_id in groups:
            for o in items:
                out.append(ObjectState(
                    idx=int(getattr(o, 'unique_id', 0)),
                    type_id=int(getattr(o, 'type', type_id)),
                    x=float(o.position.x), y=float(o.position.y),
                    heading_deg=float(getattr(o, 'heading', 0.0)),
                    length_m=float(getattr(o.size, 'x', 0.0)),
                    width_m=float(getattr(o.size, 'y', 0.0))))
        self.objects = out

    def poll(self) -> list:
        return self.objects

    def close(self) -> None:
        pass


def object_on_road(m: BakedMap, obj: 'ObjectState', margin_px: int = 0) -> bool:
    """객체가 주행가능영역 안인지. margin_px 만큼 여유를 준다.

    한 점만 보면 GPS·맵 오차로 경계에서 들락거린다. 차체 네 귀퉁이와 중심을 함께
    보고, 하나라도 도로에 걸치면 안으로 친다.
    """
    yaw = math.radians(obj.heading_deg)
    f = np.array([math.cos(yaw), math.sin(yaw)])
    r = np.array([f[1], -f[0]])
    length_m, width_m = object_dimensions(obj)
    hl, hw = 0.5 * length_m, 0.5 * width_m
    c = np.array([obj.x, obj.y])
    pts = np.stack([c,
                    c + hl * f + hw * r, c + hl * f - hw * r,
                    c - hl * f + hw * r, c - hl * f - hw * r])
    px = m.grid.to_pixel(pts)
    h, w = m.road.shape
    for dc in range(-margin_px, margin_px + 1, max(1, margin_px or 1)):
        for dr in range(-margin_px, margin_px + 1, max(1, margin_px or 1)):
            cc = np.clip(px[:, 0] + dc, 0, w - 1)
            rr = np.clip(px[:, 1] + dr, 0, h - 1)
            if (m.road[rr, cc] > 0).any():
                return True
    return False


def _point_on_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    den = float(np.dot(ab, ab))
    if den < 1e-12:
        return a.copy()
    t = float(np.clip(np.dot(point - a, ab) / den, 0.0, 1.0))
    return a + t * ab


def _segment_intersection(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> Optional[np.ndarray]:
    ab, cd = b - a, d - c
    den = float(ab[0] * cd[1] - ab[1] * cd[0])
    if abs(den) < 1e-12:
        return None
    ac = c - a
    t = float((ac[0] * cd[1] - ac[1] * cd[0]) / den)
    u = float((ac[0] * ab[1] - ac[1] * ab[0]) / den)
    return a + t * ab if -1e-9 <= t <= 1.0 + 1e-9 and -1e-9 <= u <= 1.0 + 1e-9 else None


def _segment_to_object_box(
    a: np.ndarray, b: np.ndarray, obj: ObjectState
) -> Tuple[float, np.ndarray, np.ndarray]:
    """차선 중심선 segment와 회전 객체 박스 사이의 정확한 최단거리와 두 점."""
    length_m, width_m = object_dimensions(obj)
    yaw = math.radians(obj.heading_deg)
    f = np.array([math.cos(yaw), math.sin(yaw)])
    r = np.array([f[1], -f[0]])
    hl, hw = 0.5 * length_m, 0.5 * width_m
    center = np.array([obj.x, obj.y], dtype=np.float64)
    corners = np.stack([center + hl * f + hw * r, center + hl * f - hw * r,
                        center - hl * f - hw * r, center - hl * f + hw * r])
    edges = list(zip(corners, np.roll(corners, -1, axis=0)))
    for e0, e1 in edges:
        hit = _segment_intersection(a, b, e0, e1)
        if hit is not None:
            return 0.0, hit, hit

    candidates = []
    for lane_point in (a, b):
        rel = lane_point - center
        box_point = (center + np.clip(np.dot(rel, f), -hl, hl) * f
                     + np.clip(np.dot(rel, r), -hw, hw) * r)
        candidates.append((float(np.linalg.norm(lane_point - box_point)),
                           box_point, lane_point))
    for box_point in corners:
        lane_point = _point_on_segment(box_point, a, b)
        candidates.append((float(np.linalg.norm(lane_point - box_point)),
                           box_point, lane_point))
    return min(candidates, key=lambda item: item[0])


def nearest_object_lane_gap(m: BakedMap, obj: ObjectState) -> Optional[dict]:
    """객체 외곽부터 실측 차선 도색 외곽까지의 거리와 측정 선분."""
    a, b = m.lane_segment_a, m.lane_segment_b
    if len(a) == 0:
        return None
    center = np.array([obj.x, obj.y], dtype=np.float64)
    ab = b - a
    den = np.sum(ab * ab, axis=1)
    t = np.divide(np.sum((center - a) * ab, axis=1), den,
                  out=np.zeros_like(den), where=den > 1e-12)
    nearest = a + np.clip(t, 0.0, 1.0)[:, None] * ab
    center_d = np.linalg.norm(nearest - center, axis=1)
    count = min(64, len(a))
    indices = np.argpartition(center_d, count - 1)[:count]
    best = None
    best_idx = -1
    for i in indices:
        candidate = _segment_to_object_box(a[i], b[i], obj)
        if best is None or candidate[0] < best[0]:
            best, best_idx = candidate, int(i)
    if best is None:
        return None
    centerline_gap, object_point, lane_center_point = best
    paint_half = 0.5 * max(0.0, m.lane_paint_width_m)
    direction = object_point - lane_center_point
    norm = float(np.linalg.norm(direction))
    lane_edge = (lane_center_point if norm < 1e-12 else
                 lane_center_point + min(paint_half, norm) * direction / norm)
    return {
        'distance_m': max(0.0, centerline_gap - paint_half),
        'object_point': object_point,
        'lane_point': lane_edge,
        'color': (m.lane_segment_colors[best_idx]
                  if best_idx < len(m.lane_segment_colors) else '?'),
    }


def draw_objects(canvas: np.ndarray, m: BakedMap, ego: EgoState, objects: list,
                 window_m: float, out_px: int, margin_px: int = 0,
                 show_outside: bool = False,
                 show_lane_distance: bool = False) -> tuple:
    """주행가능영역 안의 객체만 그린다 (show_outside 면 밖도 흐리게)."""
    inside = outside = 0
    for o in objects:
        on = object_on_road(m, o, margin_px)
        if on:
            inside += 1
        else:
            outside += 1
            if not show_outside:
                continue
        yaw = math.radians(o.heading_deg)
        f = np.array([math.cos(yaw), math.sin(yaw)])
        r = np.array([f[1], -f[0]])
        length_m, width_m = object_dimensions(o)
        hl, hw = 0.5 * length_m, 0.5 * width_m
        c = np.array([o.x, o.y])
        box = np.stack([c + hl * f + hw * r, c + hl * f - hw * r,
                        c - hl * f - hw * r, c - hl * f + hw * r])
        v = world_to_view(box, ego, window_m, out_px)
        if not ((v[:, 0] > -out_px).any() and (v[:, 0] < 2 * out_px).any()):
            continue
        poly = np.round(v).astype(np.int32).reshape(-1, 1, 2)
        col = (0, 0, 230) if o.type_id == 0 else (230, 90, 0)   # 보행자 빨강 / 차량 파랑
        if on:
            cv2.fillPoly(canvas, [poly], col)
            cv2.polylines(canvas, [poly], True, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.polylines(canvas, [poly], True, (70, 70, 70), 1, cv2.LINE_AA)
        anchor = np.round(v.mean(axis=0)).astype(int)
        cv2.putText(canvas, f'{o.type_name}{o.idx}',
                    (int(anchor[0]) + 6, int(anchor[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (255, 255, 255) if on else (110, 110, 110), 1, cv2.LINE_AA)
        if show_lane_distance:
            measure = nearest_object_lane_gap(m, o)
            if measure is not None:
                endpoints = world_to_view(
                    np.stack([measure['object_point'], measure['lane_point']]),
                    ego, window_m, out_px)
                p0, p1 = (tuple(np.round(p).astype(int)) for p in endpoints)
                cv2.line(canvas, p0, p1, (0, 255, 255), 1, cv2.LINE_AA)
                mid = np.round(endpoints.mean(axis=0)).astype(int)
                prefix = 'W' if measure['color'] == 'WHITE' else 'Y' if measure['color'] == 'YELLOW' else 'L'
                text_ = f"{prefix} {measure['distance_m']:.2f}m"
                cv2.putText(canvas, text_, (int(mid[0]) + 3, int(mid[1]) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(canvas, text_, (int(mid[0]) + 3, int(mid[1]) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
    return inside, outside


def nearest_link(m: BakedMap, ego: EgoState) -> Optional[dict]:
    """ego 에서 가장 가까운 링크. 제외 목록을 만들 때 '지금 밟고 있는' 링크다."""
    if m.link_pts.shape[0] == 0:
        return None
    d = np.linalg.norm(m.link_pts.astype(np.float64) - np.array([ego.x, ego.y]), axis=1)
    i = int(np.argmin(d))
    k = int(m.link_owner[i])
    return {
        'idx': m.link_ids[k] if k < len(m.link_ids) else '?',
        'distance_m': float(d[i]),
        'in_road': bool(m.link_in_road[k]) if k < len(m.link_in_road) else False,
        'in_lane': bool(m.link_in_lane[k]) if k < len(m.link_in_lane) else False,
        'x': float(m.link_pts[i][0]),
        'y': float(m.link_pts[i][1]),
    }


def _warp_thin_binary(
    layer: np.ndarray, affine: np.ndarray, size: tuple, threshold: int = 64
) -> np.ndarray:
    """1 px 선을 회전할 때 최근접 보간으로 생기는 시각적 단절을 막는다.

    H5 원본 선은 연속이어도 회전·확대된 출력 격자의 픽셀 중심이 원본 선을 비껴가면
    INTER_NEAREST는 점을 건너뛴다. 선 레이어만 선형 보간 후 낮은 임계값으로 다시
    이진화하면 연결성을 유지한다. 원본 H5나 거리 계산용 벡터는 바꾸지 않는다.
    """
    warped = cv2.warpAffine(layer, affine, size, flags=cv2.INTER_LINEAR)
    return np.where(warped >= threshold, 255, 0).astype(np.uint8)


def ego_view(
    m: BakedMap, ego: EgoState, window_m: float, out_px: int, grid_m: float = 10.0
) -> np.ndarray:
    """ego 중심, 진행 방향이 위로 오는 BEV 크롭."""
    src = np.array(
        [m.grid.to_pixel_f(view_to_world(c, r, ego, window_m, out_px).reshape(1, 2))[0]
         for c, r in ((0.0, 0.0), (out_px - 1.0, 0.0), (0.0, out_px - 1.0))],
        dtype=np.float32,
    )
    dst = np.array([[0, 0], [out_px - 1, 0], [0, out_px - 1]], dtype=np.float32)
    aff = cv2.getAffineTransform(src, dst)

    size = (out_px, out_px)
    road = cv2.warpAffine(m.road, aff, size, flags=cv2.INTER_NEAREST)
    lane = _warp_thin_binary(m.lane, aff, size)
    lane_white = _warp_thin_binary(m.lane_white, aff, size)
    lane_yellow = _warp_thin_binary(m.lane_yellow, aff, size)
    stop = _warp_thin_binary(m.stopline, aff, size)

    canvas = np.zeros((out_px, out_px, 3), dtype=np.uint8)
    draw_world_grid(canvas, ego, window_m, out_px, grid_m)
    canvas[road > 0] = (90, 90, 90)
    colored = cv2.bitwise_or(lane_white, lane_yellow)
    canvas[(lane > 0) & (colored == 0)] = (40, 230, 240)
    canvas[lane_white > 0] = (235, 235, 235)
    canvas[lane_yellow > 0] = (0, 220, 255)
    canvas[stop > 0] = (60, 60, 235)

    # ego 차체 (Ioniq 5, 후륜 차축 원점) 를 뷰 좌표에 직접 그린다.
    mpp = window_m / out_px
    length, width, rear_overhang = 4.635, 1.892, 0.700
    fl, rr_ = (length - rear_overhang) / mpp, rear_overhang / mpp
    hw = 0.5 * width / mpp
    cx = cy = out_px / 2
    body = np.array([[cx - hw, cy - fl], [cx + hw, cy - fl],
                     [cx + hw, cy + rr_], [cx - hw, cy + rr_]], dtype=np.int32)
    cv2.fillPoly(canvas, [body], (255, 200, 100))
    cv2.arrowedLine(canvas, (int(cx), int(cy)), (int(cx), int(cy - fl)),
                    (255, 255, 255), 1, tipLength=0.25)
    draw_north(canvas, ego, out_px)
    return canvas


def annotate(canvas: np.ndarray, lines) -> np.ndarray:
    for i, (text, color) in enumerate(lines):
        cv2.putText(canvas, text, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
    return canvas


def lateral_lane_distance(
    m: BakedMap, ego: EgoState, side: int, max_m: float = 6.0
) -> Optional[float]:
    """ego 에서 좌(+1)/우(-1) 횡방향으로 훑어 첫 차선 픽셀까지의 거리 (m).

    MORAI 가 보내주는 distance_left/right_lane_boundary 와 맞춰보기 위한 값이다.
    """
    yaw = math.radians(ego.heading_deg)
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([fwd[1], -fwd[0]])
    direction = -side * right  # side=+1 이면 좌측
    step = 1.0 / m.grid.pixels_per_meter
    n = int(max_m / step)
    offs = np.arange(1, n + 1) * step
    pts = np.array([ego.x, ego.y]) + offs[:, None] * direction
    px = m.grid.to_pixel(pts)
    h, w = m.lane.shape
    ok = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
    if not ok.any():
        return None
    vals = np.zeros(len(px), dtype=np.uint8)
    vals[ok] = m.lane[px[ok, 1], px[ok, 0]]
    hit = np.nonzero(vals > 0)[0]
    if not hit.size:
        return None
    # 선이 두께를 가지므로 첫 픽셀을 쓰면 두께의 절반만큼 짧게 나온다.
    # 처음 만난 연속 구간의 중심을 거리로 삼는다.
    end = hit[0]
    while end + 1 < len(vals) and vals[end + 1] > 0:
        end += 1
    return float(0.5 * (offs[hit[0]] + offs[end]))


# MORAI Ioniq 5 제원. 패킷 프레임이 제대로 맞았는지 확인하는 지문으로 쓴다.
IONIQ5_SIZE = (4.635, 1.892, 2.434)          # 공통부 offset 26 (f[4..6])
IONIQ5_OVERHANG = (0.845, 3.000, 0.700)      # 공통부 offset 38 (f[7..9])


def _unpack_bodies(bodies: list):
    """본문들을 (시간(s), float 배열) 로 편다."""
    blen = len(bodies[0])
    n = (blen - 10) // 4
    ts, rows = [], []
    for b in bodies:
        sec, nsec = struct.unpack_from('<II', b, 0)
        ts.append(sec + nsec * 1e-9)
        rows.append(struct.unpack_from('<%df' % n, b, 10))
    return np.array(ts, dtype=np.float64), np.array(rows, dtype=np.float64), n


def _check_frame(arr: np.ndarray) -> bool:
    """차체 제원이 제자리에 있는지로 패킷 프레임 정렬을 검증한다.

    여기서 통과하면 오프셋 계산은 맞은 것이므로, 위치가 0 이면 파싱 실패가 아니라
    시뮬이 실제로 0 을 보내는 것이다.
    """
    med = np.median(arr, axis=0)
    ok_size = all(abs(med[4 + i] - v) < 0.05 for i, v in enumerate(IONIQ5_SIZE))
    ok_over = all(abs(med[7 + i] - v) < 0.05 for i, v in enumerate(IONIQ5_OVERHANG))
    print('[analyze] 프레임 검증 (차체 제원이 문서상 자리에 있는지):')
    print(f'    f[4..6]  size            = {med[4]:.3f}, {med[5]:.3f}, {med[6]:.3f}'
          f'   기대 {IONIQ5_SIZE}  -> {"일치" if ok_size else "불일치"}')
    print(f'    f[7..9]  overhang/wb/ro  = {med[7]:.3f}, {med[8]:.3f}, {med[9]:.3f}'
          f'   기대 {IONIQ5_OVERHANG}  -> {"일치" if ok_over else "불일치"}')
    print(f'    f[10..12] position       = {med[10]:.3f}, {med[11]:.3f}, {med[12]:.3f}')
    return ok_size and ok_over


def analyze_bodies(bodies: list, bounds: Optional[tuple] = None) -> None:
    """패킷 본문에서 위치 필드를 실측으로 찾는다.

    '변하는 값' 만으로는 accel/brake 나 360도로 감기는 각도까지 후보로 걸린다.
    그래서 속도(공통부 offset 10) 와의 정합으로 거른다: 위치라면 연속 표본의
    이동량을 시간으로 나눈 값이 보고된 속도와 맞아야 한다.
    """
    if len(bodies) < 5:
        print('[analyze] 표본이 부족합니다 (차를 움직이면서 다시 실행하세요)')
        return
    ts, arr, n = _unpack_bodies(bodies)
    lo, hi = arr.min(axis=0), arr.max(axis=0)
    span = hi - lo

    print(f'[analyze] body {len(bodies[0])}B -> float 인덱스 0..{n - 1} '
          f'(본문 오프셋 10 부터), 표본 {len(bodies)}개')
    frame_ok = _check_frame(arr)

    moving = [i for i in range(n) if np.isfinite(arr[:, i]).all() and span[i] > 1e-3]
    print('[analyze] 변하는 값:')
    for i in moving:
        print(f'    f[{i:2d}] (offset {10 + 4 * i:3d})  '
              f'min {lo[i]:12.3f}  max {hi[i]:12.3f}  변화폭 {span[i]:10.3f}')
    if not moving:
        print('    (없음 — 차가 정지해 있었을 수 있습니다)')
        return

    # 속도 기준 (공통부 offset 10 = signed_velocity, km/h)
    speed_ms = np.abs(arr[:, 0]) / 3.6
    dt = np.diff(ts)
    good = dt > 1e-3
    if good.sum() < 3 or speed_ms.max() < 0.5:
        print('[analyze] 속도 표본이 부족해 위치 후보를 검증할 수 없습니다 '
              '(차를 좀 더 달리게 하세요)')
        return
    expect = 0.5 * (speed_ms[:-1] + speed_ms[1:])[good]

    print('[analyze] 위치 후보 검증 — 연속 표본의 이동속도가 보고 속도와 맞는지:')
    ranked = []
    for i in moving:
        j = i + 1
        if j >= n:
            continue
        d = np.hypot(np.diff(arr[:, i]), np.diff(arr[:, j]))[good]
        # 각도가 360 도에서 감기면 한 스텝에 거대한 점프가 생긴다 -> 위치가 아니다
        if d.max() > 100.0:
            continue
        err = float(np.median(np.abs(d / dt[good] - expect)))
        in_map = True
        if bounds is not None:
            min_x, min_y, max_x, max_y = bounds
            in_map = (lo[i] >= min_x and hi[i] <= max_x
                      and lo[j] >= min_y and hi[j] <= max_y)
        ranked.append((err, i, j, in_map))
    ranked.sort()

    if not ranked:
        print('    후보 없음.')
    for err, i, j, in_map in ranked[:5]:
        verdict = ('위치로 보임' if err < 0.5 and in_map else
                   '맵 범위 밖' if err < 0.5 else '속도와 불일치')
        print(f'    f[{i}], f[{j}] (offset {10 + 4 * i:3d})  '
              f'속도 오차 중앙값 {err:8.3f} m/s  -> {verdict}')

    # 축에 나란한 구간을 달리면 (x,y) 와 (y,z) 가 둘 다 속도와 맞는다.
    # 최소 오차만 보면 엉뚱한 쪽이 뽑히므로, 맵 범위를 통과한 후보 중에서 고른다.
    best = next((r for r in ranked if r[0] < 0.5 and r[3]), None)
    if best:
        print(f'[analyze] => --pos-index {best[1]} 로 실행하세요. '
              f'(화면의 ego 좌표가 시뮬 위치와 맞는지 한 번 눈으로 확인할 것)')
    else:
        print('[analyze] => 이 패킷에는 쓸 수 있는 위치 필드가 없습니다.')
        if frame_ok:
            print('    프레임 검증은 통과했으므로 파싱 문제가 아니라 시뮬이 위치를 '
                  '0 으로 보내고 있는 것입니다.')
            print('    MORAI Network Settings 에서 Ego 메시지 판본을 바꾸거나 '
                  '(245B 판본은 위치를 담습니다),')
            print('    ROS 브릿지의 /Ego_topic 을 쓰세요: --source ros')


def run_probe(port: int, seconds: float, analyze: bool = False,
              bounds: Optional[tuple] = None) -> int:
    """포트에 무엇이든 들어오는지부터 확인한다.

    '수신 없음' 과 '수신은 되는데 해석 실패' 는 원인이 전혀 다르다.
    전자는 MORAI 목적지/방화벽 문제, 후자는 포트를 다른 항목에 쓴 경우다.
    """
    sock = bind_udp(port)
    print(f'[probe] udp/{port} 수신 대기 ({seconds:.0f}s) — Ctrl-C 로 중단')
    if analyze:
        print('[probe] 이 동안 시뮬에서 차를 계속 움직이세요.')
    seen = {}
    bodies = []
    started = time.time()
    try:
        while time.time() - started < seconds:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            key = (addr[0], len(data), data[:14])
            if key not in seen:
                kind = 'Ego (해석 가능)' if parse_ego_packet(data) else 'Ego 아님/미지 판본'
                print(f'[probe] {addr[0]}:{addr[1]}  {len(data)}B  '
                      f'head={data[:14]!r}  -> {kind}')
                if data.startswith(EGO_HDR):
                    print(f'[probe] prefix {EGO_PRE}B = {data[:EGO_PRE]!r}')
            seen[key] = seen.get(key, 0) + 1
            if analyze and data.startswith(EGO_HDR):
                body = data[EGO_PRE:-TAIL]
                if not bodies or len(body) == len(bodies[0]):
                    bodies.append(body)
    except KeyboardInterrupt:
        print()
    finally:
        sock.close()

    if not seen:
        print(f'[probe] udp/{port} 로 들어온 패킷이 없습니다.')
        for ip in host_ips():
            print(f'        MORAI Destination IP 후보: {ip}   PORT: {port}')
        print('        Windows 방화벽에서 이 포트의 인바운드 UDP 를 막고 있지 않은지도 확인하세요.')
        return 0

    print(f'[probe] 총 {sum(seen.values())} 패킷 / {len(seen)} 종류')
    if analyze:
        analyze_bodies(bodies, bounds)
    return 0


def load_links(path: Path) -> list:
    try:
        cfg = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError):
        return []
    ids = cfg.get('exclude_links') or []
    return [{'idx': str(i), 'x': float('nan'), 'y': float('nan'),
             'in_road': True, 'in_lane': True} for i in ids]


def format_links(links: list) -> str:
    """config 의 drivable.exclude_links / lane.exclude_links 에 붙여넣을 yaml."""
    if not links:
        return '# 수집된 링크가 없습니다.'
    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    out = [f'# live_check.py 로 수집 ({stamp}), {len(links)}개',
           '# config 의 drivable.exclude_links / lane.exclude_links 에 붙여넣으세요.',
           '# (road=O 는 주행가능영역에, lane=O 는 차선에 쓰이고 있다는 뜻)',
           'exclude_links:']
    for c in links:
        if not np.isfinite(c.get('x', float('nan'))):
            out.append(f"  - {c['idx']}")
            continue
        out.append(f"  - {c['idx']}"
                   f"    # ({c['x']:.2f}, {c['y']:.2f})  "
                   f"road={'O' if c['in_road'] else '-'} "
                   f"lane={'O' if c['in_lane'] else '-'}")
    return '\n'.join(out)


def load_collected(path: Path) -> list:
    """이전에 저장한 수집 yaml 을 읽는다 (--collect-append 용).

    주석의 좌표까지 되살리진 않는다. id 만 유지하면 include_nodes 로는 충분하다.
    """
    try:
        cfg = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError):
        return []
    ids = ((cfg.get('stopline') or {}).get('include_nodes')) or []
    return [{'idx': str(i), 'x': float('nan'), 'y': float('nan'),
             'traffic_light_id': '', 'facing_deg': float('nan')} for i in ids]


def format_collected(collected: list) -> str:
    """config 의 stopline.include_nodes 에 그대로 붙여넣을 수 있는 yaml."""
    if not collected:
        return '# 수집된 정지선이 없습니다.'
    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    lines = [f'# live_check.py 로 수집 ({stamp}), {len(collected)}개',
             '# config 의 stopline: 아래에 붙여넣으세요.',
             'stopline:',
             '  include_nodes:']
    for c in collected:
        if not np.isfinite(c.get('x', float('nan'))):
            lines.append(f"    - {c['idx']}")      # 이전 파일에서 읽어온 것
            continue
        tl = c['traffic_light_id'] or '-'
        lines.append(f"    - {c['idx']}"
                     f"    # ({c['x']:.2f}, {c['y']:.2f})  TL={tl}"
                     f"  통과각 {c['facing_deg']:.0f}deg")
    return '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--h5', type=Path, default=DEFAULT_H5)
    ap.add_argument('--source', choices=('udp', 'gps', 'ros', 'demo'),
                    default='udp',
                    help="udp=Ego 패킷만, gps=위치는 GPS·방향은 Ego, "
                         "ros=/Ego_topic, demo=시뮬 없이")
    ap.add_argument('--gps-port', type=int, default=9301,
                    help='MORAI GPS Destination PORT (기본 9301)')
    ap.add_argument('--utm-epsg', type=int, default=32652,
                    help='맵의 UTM 좌표계 (기본 32652 = zone 52N/WGS84)')
    ap.add_argument('--port', type=int, default=EGO_PORT_DEFAULT,
                help='MORAI Ego Vehicle Status Destination PORT '
                     '(기본 %d, morai_udp_bridge.launch 와 동일)' % EGO_PORT_DEFAULT)
    ap.add_argument('--window-m', type=float, default=64.0, help='BEV 한 변 (m)')
    ap.add_argument('--out-px', type=int, default=640)
    ap.add_argument('--grid-m', type=float, default=10.0,
                    help='월드 고정 격자 간격 (m). 0 이면 끔')
    ap.add_argument('--probe', action='store_true',
                    help='들어오는 UDP 패킷만 덤프한다 (송신 IP/길이/헤더)')
    ap.add_argument('--pos-index', type=int, default=DEFAULT_LAYOUT.pos,
                    help='본문 float 배열에서 x 의 인덱스 (y 는 +1). '
                         '--probe --analyze 결과를 그대로 넣는다. 기본 %d'
                         % DEFAULT_LAYOUT.pos)
    ap.add_argument('--heading-index', type=int, default=DEFAULT_LAYOUT.heading,
                    help='heading(deg) 의 인덱스. 기본 %d' % DEFAULT_LAYOUT.heading)
    ap.add_argument('--analyze', action='store_true',
                    help='--probe 와 함께. 차를 움직이는 동안 변하는 float 를 찾아\n'
                         '위치 필드가 실제로 어느 오프셋인지 알아낸다')
    ap.add_argument('--no-window', action='store_true', help='창 없이 지표만')
    ap.add_argument('--save', type=Path, default=None, help='마지막 프레임 저장 경로')
    ap.add_argument('--collect', type=float, nargs='?', const=15.0, default=None,
                    metavar='R',
                    help='지나친 정지선을 자동 수집한다 (기본 반경 15 m). '
                         '창이 떠 있으면 s 키로 수동 추가, u 로 마지막 취소')
    ap.add_argument('--objects', type=int, nargs='?', const=OBJECT_PORT_DEFAULT,
                    default=None, metavar='PORT',
                    help='MORAI Object Info UDP 를 받아 주행가능영역 안의 객체만 '
                         '표시한다 (기본 포트 %d)' % OBJECT_PORT_DEFAULT)
    ap.add_argument('--object-topic', default='/Object_topic',
                    help='--source ros 일 때 객체를 받을 토픽 (기본 /Object_topic)')
    ap.add_argument('--object-margin-m', type=float, default=0.0,
                    help='객체 판정에 줄 여유 (m). GPS·맵 오차로 경계 객체가 '
                         '들락거리면 0.5~1.0 을 준다')
    ap.add_argument('--show-outside-objects', action='store_true',
                    help='주행가능영역 밖 객체도 회색 테두리로 표시한다 (확인용)')
    ap.add_argument('--show-object-lane-distance', action='store_true',
                    help='객체 외곽과 가장 가까운 실측 차선 도색 외곽 사이 거리를 표시한다')
    ap.add_argument('--show-links', type=float, nargs='?', const=12.0, default=None,
                    metavar='R',
                    help='주변 링크 중심선을 색으로 그리고 id 와 좌우 거리를 표시한다 '
                         '(기본 반경 12 m). 어느 링크를 뺄지 고를 때')
    ap.add_argument('--collect-links', type=float, nargs='?', const=3.0, default=None,
                    metavar='R',
                    help='주행하면서 밟은 링크 id 를 모은다 (기본 반경 3 m). '
                         '주행가능영역/차선에서 제외할 링크를 고를 때. '
                         '창이 있으면 l 키로 수동 추가, k 로 마지막 취소')
    ap.add_argument('--collect-facing-deg', type=float, default=45.0,
                    help='자동 수집 시 진행방향과 이 각도 안쪽인 정지선만 담는다 '
                         '(기본 45도). 교차 도로의 정지선이 섞이는 걸 막는다. '
                         '180 이면 방향 무시')
    ap.add_argument('--collect-append', action='store_true',
                    help='--collect-out 파일이 이미 있으면 덮어쓰지 않고 합친다. '
                         '정지선마다 프로그램을 껐다 켜며 모을 때 쓴다')
    ap.add_argument('--collect-out', type=Path, default=None,
                    help='수집 결과를 이 경로에 yaml 로 저장 '
                         '(config 의 stopline.include_nodes 에 그대로 붙여넣는 형식)')
    ap.add_argument('--seconds', type=float, default=0.0, help='0 이면 무한')
    args = ap.parse_args()

    if args.probe:
        bounds = None
        if args.h5.is_file():
            bm = BakedMap.load(args.h5)
            lo = bm.grid.to_world(np.array([[0.0, 0.0]]))[0]
            hi = bm.grid.to_world(
                np.array([[bm.grid.width_px, bm.grid.height_px]], dtype=np.float64))[0]
            bounds = (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))
        return run_probe(args.port, args.seconds or 30.0,
                         analyze=args.analyze, bounds=bounds)

    if not args.h5.is_file():
        raise SystemExit(f'H5 없음: {args.h5}  (먼저 bake_map.py 를 실행하세요)')
    m = BakedMap.load(args.h5)
    print(f'[map] {args.h5.name}  id={m.map_id}  '
          f'{m.grid.width_px}x{m.grid.height_px} px @ {m.grid.pixels_per_meter} px/m')
    print(f'[map] local_origin_in_global={m.local_origin.tolist()}')

    if args.source == 'udp':
        layout = FieldLayout(pos=args.pos_index, heading=args.heading_index)
        source = UdpEgoSource(args.port, layout=layout)
        if layout != DEFAULT_LAYOUT:
            print(f'[src] 필드 레이아웃 재정의: pos=f[{layout.pos}],f[{layout.pos + 1}] '
                  f'heading=f[{layout.heading}]')
        print(f'[src] {source.describe()} 대기 중')
        print(f'      MORAI > Network Settings > Ego Vehicle Status 의 '
              f'Destination 을 아래로 맞추세요:')
        for ip in host_ips():
            print(f'        IP {ip}   PORT {args.port}')
    elif args.source == 'gps':
        if m.local_origin.size < 2:
            raise SystemExit('H5 에 local_origin_in_global 이 없어 GPS 를 쓸 수 없습니다.')
        layout = FieldLayout(pos=args.pos_index, heading=args.heading_index)
        source = GpsEgoSource(args.gps_port, args.port, m.local_origin,
                              utm_epsg=args.utm_epsg, layout=layout)
        print(f'[src] {source.describe()}')
        print(f'      UTM EPSG:{args.utm_epsg}, 원점 {m.local_origin[:2].tolist()}')
    elif args.source == 'ros':
        source = RosEgoSource(fallback_origin=m.local_origin)
        print(f'[src] {source.describe()}')
    else:
        source = DemoEgoSource(m.grid, m.road)
        print(f'[src] {source.describe()}')

    objsrc = None
    if args.objects is not None:
        margin_note = (' (여유 %.1f m)' % args.object_margin_m
                       if args.object_margin_m else '')
        if args.source == 'ros':
            # 컨테이너에서는 브릿지가 udp/7505 를 이미 잡고 있어 ROS 토픽으로 받는다.
            objsrc = RosObjectSource(args.object_topic)
            print(f'[obj] ROS {args.object_topic} 수신 — '
                  f'주행가능영역 안의 객체만 표시{margin_note}')
        else:
            objsrc = ObjectSource(args.objects)
            print(f'[obj] Object Info udp/{args.objects} 수신 — '
                  f'주행가능영역 안의 객체만 표시{margin_note}')
    obj_margin_px = int(round(args.object_margin_m * m.grid.pixels_per_meter))

    show = not args.no_window
    win = 'link_node live check'
    if show:
        try:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win, args.out_px, args.out_px)
        except cv2.error as e:
            print(f'[warn] 창을 열 수 없어 헤드리스로 전환합니다: {e}')
            show = False

    on_road = 0
    total = 0
    outside = 0
    lane_err = []
    started = time.time()
    last_report = started
    canvas = None
    ego = None
    prev_xy = None
    odom_m = 0.0
    collected = []          # 순서 유지 (지나친 순서대로)
    collected_set = set()
    nearest = []
    links = []              # 제외 후보 링크
    links_set = set()
    near_link = None
    around = []
    objects = []
    obj_in = obj_out = 0

    try:
        while True:
            if args.seconds and time.time() - started > args.seconds:
                break
            ego = source.read()
            if ego is None:
                if show:
                    cv2.waitKey(1)
                if time.time() - last_report > 5.0:
                    print(f'[wait] {source.describe()} 에서 Ego 수신 없음 '
                          f'({int(time.time() - started)}s) — '
                          f'--probe 로 패킷 도착 여부부터 확인하세요')
                    last_report = time.time()
                continue

            total += 1
            if prev_xy is not None:
                odom_m += float(np.hypot(ego.x - prev_xy[0], ego.y - prev_xy[1]))
            prev_xy = (ego.x, ego.y)
            road_v = m.sample(m.road, ego.x, ego.y)
            if road_v is None:
                outside += 1
            elif road_v > 0:
                on_road += 1

            if objsrc is not None:
                objects = objsrc.poll()
            near_link = nearest_link(m, ego)
            around = (links_around(m, ego, args.show_links)
                      if args.show_links is not None else [])
            if (args.collect_links is not None and near_link
                    and near_link['distance_m'] <= args.collect_links
                    and near_link['idx'] not in links_set):
                links_set.add(near_link['idx'])
                links.append(near_link)
                print(f"[link] {near_link['idx']}  {near_link['distance_m']:.1f} m  "
                      f"road={'O' if near_link['in_road'] else '-'} "
                      f"lane={'O' if near_link['in_lane'] else '-'}   총 {len(links)}개")

            nearest = nearest_stop_lines(m, ego, count=1)
            if args.collect is not None and nearest:
                n0 = nearest[0]
                facing_ok = (not np.isfinite(n0['facing_deg'])
                             or n0['facing_deg'] <= args.collect_facing_deg)
                if (n0['distance_m'] <= args.collect and facing_ok
                        and n0['idx'] not in collected_set):
                    collected_set.add(n0['idx'])
                    collected.append(n0)
                    print(f"[collect] {n0['idx']}  ({n0['x']:.2f}, {n0['y']:.2f})  "
                          f"{n0['distance_m']:.1f} m  "
                          f"각도차 {n0['facing_deg']:.0f}도  "
                          f"TL={n0['traffic_light_id'] or '-'}   총 {len(collected)}개")

            left = lateral_lane_distance(m, ego, +1)
            right = lateral_lane_distance(m, ego, -1)
            if ego.dist_left_lane is not None and left is not None:
                lane_err.append(abs(left - abs(ego.dist_left_lane)))
            if ego.dist_right_lane is not None and right is not None:
                lane_err.append(abs(right - abs(ego.dist_right_lane)))

            if show or args.save:
                canvas = ego_view(m, ego, args.window_m, args.out_px,
                                  grid_m=args.grid_m)
                if around:
                    draw_link_overlay(canvas, m, ego, args.window_m, args.out_px,
                                      around)
                if objects:
                    obj_in, obj_out = draw_objects(
                        canvas, m, ego, objects, args.window_m, args.out_px,
                        margin_px=obj_margin_px,
                        show_outside=args.show_outside_objects,
                        show_lane_distance=args.show_object_lane_distance)
                ratio = on_road / max(1, total)
                status = ('ON ROAD', (120, 255, 120)) if (road_v or 0) > 0 else \
                         ('OFF ROAD', (80, 80, 255))
                lines = [
                    (f'ego ({ego.x:8.2f}, {ego.y:8.2f})  hdg {ego.heading_deg:6.1f}deg  '
                     f'{ego.velocity_kmh:5.1f} km/h  odom {odom_m:7.1f} m',
                     (230, 230, 230)),
                    (f'{status[0]}   on-road {ratio * 100:5.1f}% of {total}', status[1]),
                ]
                mine = (f'{left:.2f}' if left is not None else '  - ',
                        f'{right:.2f}' if right is not None else '  - ')
                if ego.dist_left_lane is not None:
                    lines.append((
                        f'lane L/R  baked {mine[0]}/{mine[1]} m   '
                        f'morai {abs(ego.dist_left_lane):.2f}/'
                        f'{abs(ego.dist_right_lane):.2f} m', (200, 220, 255)))
                else:
                    lines.append((f'lane L/R  baked {mine[0]}/{mine[1]} m   '
                                  f'(morai lane dist: n/a in this packet)', (170, 170, 170)))
                if nearest:
                    n0 = nearest[0]
                    where = 'ahead' if n0['ahead_m'] >= 0 else 'behind'
                    lines.append((
                        f"stopline {n0['idx']}  {abs(n0['ahead_m']):5.1f} m {where}"
                        f"  lat {n0['lateral_m']:+5.1f} m"
                        f"  face {n0['facing_deg']:3.0f}deg"
                        f"  TL {n0['traffic_light_id'] or '-'}",
                        (140, 220, 255)))
                if objsrc is not None:
                    lines.append((
                        f'objects {len(objects)}   '
                        f'on-road {obj_in}   off-road {obj_out} '
                        f'{"(shown)" if args.show_outside_objects else "(hidden)"}',
                        (255, 200, 120)))
                if near_link:
                    lines.append((
                        f"link {near_link['idx']}  {near_link['distance_m']:5.2f} m"
                        f"  road {'O' if near_link['in_road'] else '-'}"
                        f"  lane {'O' if near_link['in_lane'] else '-'}",
                        (170, 255, 190)))
                for c in around[:6]:
                    side = 'L' if c['lateral_m'] < 0 else 'R'
                    lines.append((
                        f"  {c['idx']}  {side}{abs(c['lateral_m']):4.1f} m"
                        f"  road {'O' if c['in_road'] else '-'}"
                        f"  lane {'O' if c['in_lane'] else '-'}", (200, 230, 255)))
                if args.collect is not None or args.collect_links is not None:
                    lines.append((
                        f'stoplines {len(collected)}  links {len(links)}   '
                        f'[s]/[u] stopline  [l]/[k] link  [q] quit', (200, 200, 140)))
                annotate(canvas, lines)

            if show:
                cv2.imshow(win, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
                if key == ord('s') and nearest:
                    n0 = nearest[0]
                    if n0['idx'] not in collected_set:
                        collected_set.add(n0['idx'])
                        collected.append(n0)
                        print(f"[collect] {n0['idx']}  {n0['distance_m']:.1f} m   "
                              f"총 {len(collected)}개")
                elif key == ord('l') and near_link:
                    if near_link['idx'] not in links_set:
                        links_set.add(near_link['idx'])
                        links.append(near_link)
                        print(f"[link] {near_link['idx']}   총 {len(links)}개")
                elif key == ord('k') and links:
                    gone = links.pop()
                    links_set.discard(gone['idx'])
                    print(f"[link] 취소 {gone['idx']}   총 {len(links)}개")
                elif key == ord('u') and collected:
                    gone = collected.pop()
                    collected_set.discard(gone['idx'])
                    print(f"[collect] 취소 {gone['idx']}   총 {len(collected)}개")

            if time.time() - last_report > 5.0:
                print(f'[live] n={total} on-road={on_road / max(1, total) * 100:.1f}% '
                      f'off-map={outside} ego=({ego.x:.1f}, {ego.y:.1f}) '
                      f'odom={odom_m:.1f}m')
                last_report = time.time()
    except KeyboardInterrupt:
        print()
    finally:
        source.close()
        if objsrc is not None:
            objsrc.close()
        if show:
            cv2.destroyAllWindows()

    if args.save and canvas is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save), canvas)
        print(f'[write] {args.save}')

    if args.collect is not None or collected:
        text = format_collected(collected)
        print()
        print(text)
        if args.collect_out:
            merged = collected
            if args.collect_append and args.collect_out.is_file():
                before = load_collected(args.collect_out)
                seen = {c['idx'] for c in before}
                merged = before + [c for c in collected if c['idx'] not in seen]
                print(f'[collect] 기존 {len(before)}개에 {len(merged) - len(before)}개 추가')
                text = format_collected(merged)
            args.collect_out.parent.mkdir(parents=True, exist_ok=True)
            args.collect_out.write_text(text, encoding='utf-8')
            print(f'[write] {args.collect_out}  (총 {len(merged)}개)')

    if args.collect_links is not None or links:
        text = format_links(links)
        print()
        print(text)
        if args.collect_out:
            out = args.collect_out.with_name(args.collect_out.stem + '_links.yaml')
            if args.collect_append and out.is_file():
                before = load_links(out)
                seen = {c['idx'] for c in before}
                merged = before + [c for c in links if c['idx'] not in seen]
                print(f'[link] 기존 {len(before)}개에 {len(merged) - len(before)}개 추가')
                text = format_links(merged)
                links = merged
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(text, encoding='utf-8')
            print(f'[write] {out}  (총 {len(links)}개)')

    summary = {
        'samples': total,
        'on_road_ratio': round(on_road / total, 4) if total else None,
        'outside_map': outside,
        'odometer_m': round(odom_m, 1),
    }
    if lane_err:
        summary['lane_dist_vs_morai_mean_err_m'] = round(float(np.mean(lane_err)), 3)
        summary['lane_dist_vs_morai_p90_err_m'] = round(float(np.percentile(lane_err, 90)), 3)
    print(f'[summary] {json.dumps(summary, ensure_ascii=False)}')
    if args.source in ('udp',) and total > 50 and odom_m < 0.5:
        print('[!] ego 위치가 전혀 변하지 않았습니다 (odom 0 m). heading 만 살아 있다면 '
              '위치 필드 오프셋이 이 시뮬 판본과 다른 것입니다.\n'
              '    차를 움직이면서 아래를 실행해 실제 인덱스를 찾으세요:\n'
              f'      sudo -E python3 {sys.argv[0]} --probe --analyze --port {args.port}\n'
              '    찾은 값을 --pos-index 로 넘기면 코드 수정 없이 적용됩니다.\n'
              '    Ego 패킷에 위치 자체가 없다면 --source gps 로 GPS 에서 받으세요.')
    if args.source != 'demo' and total and on_road / total < 0.9:
        print('[!] ego 가 자주 도로 밖입니다. MORAI 에 로드된 맵이 '
              'behavior_planner 판본과 다를 수 있습니다 '
              '(R_KR_PG_KATRI 판본과는 약 (-135, -1509) m 어긋납니다).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
