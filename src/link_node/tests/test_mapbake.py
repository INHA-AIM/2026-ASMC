"""mapbake 단위 테스트.

    python3 -m pytest src/link_node/tests -q
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[1]
sys.path.insert(0, str(PKG_ROOT))

from mapbake.io_mgeo import load_link_node
from mapbake.layers import bake_drivable, bake_lane, bake_stopline
from mapbake.raster import RasterGrid, compute_bounds, fill_thin_gaps
from mapbake.topology import (
    LinkTopology,
    heading_at,
    normals,
    offset_polyline,
    resample_polyline,
)

CONFIG_DIR = REPO_ROOT / 'src' / 'behavior_planner' / 'src' / 'config'
LINK_JSON = CONFIG_DIR / 'link_set.json'
NODE_JSON = CONFIG_DIR / 'node_set.json'
GINFO_JSON = CONFIG_DIR / 'global_info.json'

needs_map = pytest.mark.skipif(
    not (LINK_JSON.is_file() and NODE_JSON.is_file()),
    reason='behavior_planner 맵 JSON 이 없습니다',
)


# ---------------------------------------------------------------- 기하 유틸

def test_resample_uniform_spacing():
    pts = np.array([[0, 0], [10, 0], [10, 10]], dtype=np.float32)
    out = resample_polyline(pts, 0.5)
    seg = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert seg.max() <= 0.51
    assert np.allclose(out[0], pts[0], atol=1e-3)
    assert np.allclose(out[-1], pts[-1], atol=1e-3)


def test_resample_handles_duplicate_points():
    pts = np.array([[0, 0], [0, 0], [5, 0]], dtype=np.float32)
    out = resample_polyline(pts, 1.0)
    assert out.shape[0] >= 2
    assert np.isfinite(out).all()


def test_normal_is_left_and_unit():
    pts = np.array([[0, 0], [1, 0], [2, 0]], dtype=np.float32)  # +x 진행
    n = normals(pts)
    assert np.allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-5)
    assert np.allclose(n[:, 0], 0.0, atol=1e-5)
    assert np.all(n[:, 1] > 0)  # 좌측 = +y


def test_offset_distance_matches_request():
    pts = np.array([[0, 0], [10, 0]], dtype=np.float32)
    left = offset_polyline(pts, 1.75)
    right = offset_polyline(pts, -1.75)
    assert np.allclose(left[:, 1], 1.75, atol=1e-4)
    assert np.allclose(right[:, 1], -1.75, atol=1e-4)
    # 좌/우 경계 간격이 곧 차로 폭
    assert np.allclose(np.abs(left[:, 1] - right[:, 1]), 3.5, atol=1e-4)


def test_heading_at_ends():
    pts = np.array([[0, 0], [0, 5], [3, 5]], dtype=np.float32)
    assert np.allclose(heading_at(pts, at_end=False), [0, 1], atol=1e-5)
    assert np.allclose(heading_at(pts, at_end=True), [1, 0], atol=1e-5)
    assert heading_at(np.zeros((1, 2), dtype=np.float32), at_end=True) is None


# ---------------------------------------------------------------- 래스터 규약

def test_world_pixel_roundtrip_within_one_pixel():
    grid = RasterGrid.from_bounds(-400, -800, 120, 600, pixels_per_meter=5.0, margin_m=50.0)
    probe = np.array([[0, 0], [-123.4, 456.7], [-399.9, -799.9]], dtype=np.float64)
    back = grid.to_world(grid.to_pixel(probe))
    assert np.abs(back - probe).max() <= 1.0 / grid.pixels_per_meter


def test_pixel_axis_order_is_col_row():
    """col 은 x, row 는 y. 축이 뒤바뀌면 여기서 잡힌다."""
    grid = RasterGrid.from_bounds(0, 0, 100, 200, pixels_per_meter=2.0, margin_m=0.0)
    col, row = grid.to_pixel(np.array([[10.0, 50.0]]))[0]
    assert (col, row) == (20, 100)


def test_grid_covers_requested_bounds():
    grid = RasterGrid.from_bounds(-10, -20, 30, 40, pixels_per_meter=5.0, margin_m=5.0)
    assert np.allclose(grid.world_offset, [-15, -25])
    far = grid.to_pixel(np.array([[35.0, 45.0]]))[0]
    assert far[0] <= grid.width_px and far[1] <= grid.height_px


def test_compute_bounds_ignores_empty_arrays():
    lo_x, lo_y, hi_x, hi_y = compute_bounds(
        [np.zeros((0, 2)), np.array([[1.0, 2.0], [-3.0, 4.0]])]
    )
    assert (lo_x, lo_y, hi_x, hi_y) == (-3.0, 2.0, 1.0, 4.0)


def test_fill_thin_gaps_keeps_blobs_of_similar_area():
    """면적으로 거르면 안 된다 — 교차로 코너 인도 섬(48~58 m^2)이 임계값 근처에
    몰려 있어 같은 교차로의 네 모서리가 일부만 메워진다. 폭으로 갈라야 한다."""
    ppm = 5.0
    img = np.full((400, 400), 255, dtype=np.uint8)
    # 가느다란 틈: 1 m x 40 m = 40 m^2
    img[50:55, 50:250] = 0
    # 뭉툭한 섬: 7 m x 7 m = 49 m^2. 면적은 비슷한데 모양이 다르다.
    img[300:335, 300:335] = 0
    out = fill_thin_gaps(img, pixels_per_meter=ppm, max_width_m=2.0,
                         max_area_m2=0.0)
    assert out[52, 150] == 255, '가느다란 틈은 메워야 한다'
    assert out[317, 317] == 0, '뭉툭한 섬은 남겨야 한다'


def test_fill_thin_gaps_fills_small_blobs_too():
    """링크 분기 지점의 작은 삼각형 틈은 반경이 1 m 를 살짝 넘어(1.16 m) 폭
    조건만으로는 안 걸린다. 아주 작으면 모양과 무관하게 메워야 한다."""
    ppm = 5.0
    img = np.full((400, 400), 255, dtype=np.uint8)
    # 작고 뭉툭한 틈: 3 m x 3 m = 9 m^2, 내접반경 1.5 m
    img[100:115, 100:115] = 0
    # 크고 뭉툭한 섬: 7 m x 7 m = 49 m^2
    img[300:335, 300:335] = 0
    out = fill_thin_gaps(img, pixels_per_meter=ppm, max_width_m=2.0, max_area_m2=20.0)
    assert out[107, 107] == 255, '20 m^2 미만인 작은 틈은 메워야 한다'
    assert out[317, 317] == 0, '인도 섬 크기는 남겨야 한다'


def test_fill_thin_gaps_keeps_background_touching_border():
    img = np.zeros((50, 50), dtype=np.uint8)
    img[20:30, 20:30] = 255
    out = fill_thin_gaps(img, pixels_per_meter=1.0, max_width_m=1e9,
                         max_area_m2=1e9)
    assert out[0, 0] == 0  # 바깥 배경은 경계에 닿으므로 채우지 않는다


@needs_map
def test_intersection_corner_islands_survive(data, grid):
    """교차로 네 모서리 인도가 일관되게 남아야 한다.

    면적 임계 50 m^2 를 쓰던 때는 (-69.2, -26.8) 의 48 m^2 섬만 메워지고 나머지
    셋(50.4 / 56.8 / 57.6 m^2)은 남아, 같은 교차로가 짝짝이로 나왔다.
    """
    from mapbake.topology import lane_pinch_corrections
    road = bake_drivable(data, grid,
                         centerlines=lane_pinch_corrections(data.valid_links)).image
    corners = [(-69.2, -26.8), (-54.6, -26.8), (-69.3, -11.9), (-54.8, -11.6)]
    for x, y in corners:
        c, r = grid.to_pixel(np.array([[x, y]]))[0]
        assert road[r, c] == 0, f'인도 ({x}, {y}) 가 주행가능영역으로 메워졌다'
    # 교차로 내부는 주행 가능해야 한다
    c, r = grid.to_pixel(np.array([[-62.0, -19.0]]))[0]
    assert road[r, c] > 0

    # 링크가 갈라지는 지점의 작은 틈(9.5 m^2, 반경 1.16 m)은 메워져야 한다
    import cv2
    bg = (road == 0).astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(bg, 8)
    border = set(np.unique(np.concatenate(
        [lab[0], lab[-1], lab[:, 0], lab[:, -1]])))
    target = np.array([-60.33, -152.03])
    for i in range(1, n):
        if i in border:
            continue
        wp = grid.to_world(np.array([cent[i]]))[0]
        assert float(np.linalg.norm(wp - target)) > 20.0, \
            f'분기 틈이 메워지지 않았다: {wp}'


# ---------------------------------------------------------------- 실제 맵

@needs_map
@pytest.fixture(scope='module')
def data():
    return load_link_node(LINK_JSON, NODE_JSON, GINFO_JSON)


@needs_map
@pytest.fixture(scope='module')
def grid(data):
    arrays = [l.points for l in data.valid_links]
    arrays += [n.point.reshape(1, 2) for n in data.stop_line_nodes]
    return RasterGrid.from_bounds(*compute_bounds(arrays), pixels_per_meter=5.0, margin_m=50.0)


@needs_map
def test_map_loads_expected_counts(data):
    assert len(data.valid_links) == 634
    assert len(data.stop_line_nodes) == 150
    # 이 맵의 링크는 전부 3.5 m 차로 폭
    assert {round(l.width_m, 2) for l in data.valid_links} == {3.5}


@needs_map
def test_map_identity_is_behavior_planner_version(data):
    """R_KR_PG_KATRI 판본과 섞이면 좌표가 (-135, -1509) m 어긋난다."""
    origin = data.local_origin_in_global
    assert origin[:2] == [302595.0, 4124145.0]


@needs_map
def test_every_stop_line_node_resolves_a_heading(data):
    topo = LinkTopology(data)
    unresolved = [n.idx for n in data.stop_line_nodes if topo.node_heading(n.idx) is None]
    assert unresolved == []


@needs_map
def test_stoplines_land_on_drivable_area(data, grid):
    road = bake_drivable(data, grid).image
    stop = bake_stopline(data, grid).image
    mask = stop > 0
    assert mask.any()
    # 정지선 노드는 링크 위에 놓여 있으므로 거의 전부 road 안이어야 한다.
    assert float((road[mask] > 0).mean()) > 0.95


@needs_map
def test_lane_boundaries_hug_the_road(data, grid):
    road = bake_drivable(data, grid).image
    lane = bake_lane(data, grid).image
    mask = lane > 0
    assert mask.any()
    import cv2
    band = grid.meters_to_px(0.6)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * band + 1, 2 * band + 1))
    assert float((cv2.dilate(road, k) > 0)[mask].mean()) > 0.99


@needs_map
def test_lane_skips_intersection_links(data, grid):
    res = bake_lane(data, grid)
    # link_type=1 은 '교차로 내부'가 아니라 '정지선에서 출발하는 링크'다 (256개).
    # 통째로 버리지 않고 정지선 쪽 30 m 만 잘라내므로, 교차로를 벗어나는 47개는
    # 남은 구간에 차선이 그려진다.
    assert res.stats['intersection_links_trimmed'] == 47
    assert res.stats['intersection_links_skipped'] == 209
    assert res.stats['tight_curve_links_skipped'] == 20
    assert res.stats['links_used'] == 634 - 209 - 20

    # 통째로 버리면 교차로 너머 차선까지 사라진다
    whole = bake_lane(data, grid, intersection_clear_m=0.0)
    assert whole.stats['intersection_links_skipped'] == 256
    assert whole.stats['filled_px'] < res.stats['filled_px']


@needs_map
def test_all_layers_are_binary(data, grid):
    for img in (
        bake_drivable(data, grid).image,
        bake_lane(data, grid).image,
        bake_stopline(data, grid).image,
    ):
        assert img.dtype == np.uint8
        assert set(np.unique(img)).issubset({0, 255})


# ---------------------------------------------------------------- live_check

from live_check import (  # noqa: E402
    BakedMap,
    EgoState,
    EGO_HDR,
    ego_view,
    lateral_lane_distance,
    parse_ego_packet,
)
import struct  # noqa: E402


def _fake_ego_packet(x, y, heading, total_len=245, dist_l=1.8, dist_r=1.7):
    """MORAI Ego UDP 패킷을 합성한다 (실제 시뮬 없이 파서를 검증하기 위함)."""
    ext_off, ext_n = {181: (152, 0), 229: (152, 12), 245: (156, 15)}[total_len]
    body = bytearray(total_len - len(EGO_HDR) - 16 - 2)
    struct.pack_into('<IIbb', body, 0, 1700000000, 0, 1, 4)
    f = [0.0] * 25
    f[0] = 36.0            # signed_velocity km/h
    f[10], f[11], f[12] = x, y, 0.5
    f[15] = heading
    struct.pack_into('<25f', body, 10, *f)
    struct.pack_into('<f', body, 110, 3.5)   # front_steer
    if ext_n >= 15:
        ext = [0.0] * ext_n
        ext[12], ext[13] = dist_l, dist_r
        struct.pack_into('<%df' % ext_n, body, ext_off, *ext)
    return EGO_HDR + b'\x00' * 16 + bytes(body) + b'\r\n'


def test_parse_ego_packet_reads_pose_and_lane_distance():
    pkt = _fake_ego_packet(-103.5, -24.75, 137.5)
    assert len(pkt) == 245
    st = parse_ego_packet(pkt)
    assert st is not None
    assert st.x == pytest.approx(-103.5, abs=1e-3)
    assert st.y == pytest.approx(-24.75, abs=1e-3)
    assert st.heading_deg == pytest.approx(137.5, abs=1e-3)
    assert st.velocity_kmh == pytest.approx(36.0, abs=1e-3)
    assert st.dist_left_lane == pytest.approx(1.8, abs=1e-3)
    assert st.dist_right_lane == pytest.approx(1.7, abs=1e-3)


def test_parse_ego_packet_older_variant_has_no_lane_distance():
    st = parse_ego_packet(_fake_ego_packet(1.0, 2.0, 0.0, total_len=229))
    assert st is not None and st.x == pytest.approx(1.0, abs=1e-3)
    assert st.dist_left_lane is None


def test_parse_ego_packet_rejects_foreign_data():
    assert parse_ego_packet(b'nonsense') is None
    assert parse_ego_packet(EGO_HDR + b'\x00' * 40) is None  # 미지의 길이


@needs_map
@pytest.fixture(scope='module')
def baked(tmp_path_factory, data, grid):
    from mapbake.writer import write_h5
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from bake_map import link_records
    from mapbake.topology import lane_pinch_corrections

    out = tmp_path_factory.mktemp('bake') / 'map.h5'
    fixed = lane_pinch_corrections(data.valid_links)
    road = bake_drivable(data, grid, centerlines=fixed)
    lane = bake_lane(data, grid, centerlines=fixed)
    stop = bake_stopline(data, grid)
    write_h5(
        out,
        {'road': road.image, 'lane': lane.image, 'stopline': stop.image},
        grid,
        map_id='test',
        local_origin_in_global=data.local_origin_in_global,
        stopline_features=stop.features,
        links=link_records(data, fixed, road, lane),
    )
    return BakedMap.load(out)


@needs_map
def test_baked_map_roundtrips_through_h5(baked, grid):
    assert baked.grid.width_px == grid.width_px
    assert baked.grid.height_px == grid.height_px
    assert baked.grid.pixels_per_meter == grid.pixels_per_meter
    assert np.allclose(baked.grid.world_offset, grid.world_offset)


@needs_map
def test_lane_distance_from_lane_center_is_half_width(baked, data):
    """링크는 차로 중심선이므로 양쪽 차선까지 거리가 width/2 = 1.75 m 여야 한다.

    이 값이 MORAI 의 distance_left/right_lane_boundary 와 비교되는 대상이다.
    """
    import math
    errs = []
    for link in data.valid_links:
        if link.link_type != '6' or link.points.shape[0] < 8:
            continue
        i = link.points.shape[0] // 2
        v = link.points[i + 1] - link.points[i - 1]
        ego = EgoState(
            x=float(link.points[i][0]), y=float(link.points[i][1]),
            heading_deg=math.degrees(math.atan2(float(v[1]), float(v[0]))),
        )
        for side in (+1, -1):
            d = lateral_lane_distance(baked, ego, side)
            if d is not None:
                errs.append(abs(d - 0.5 * link.width_m))
    assert len(errs) > 300
    assert float(np.median(errs)) < 0.15
    assert float(np.mean(np.array(errs) < 0.3)) > 0.8


@needs_map
def test_ego_view_is_centered_on_ego(baked, data):
    link = next(l for l in data.valid_links if l.points.shape[0] > 4)
    ego = EgoState(x=float(link.points[2][0]), y=float(link.points[2][1]),
                   heading_deg=0.0)
    view = ego_view(baked, ego, window_m=64.0, out_px=256)
    assert view.shape == (256, 256, 3)
    # 차체가 중앙에 그려진다. 중앙 열은 진행방향 화살표(흰색)가 덮으므로
    # 차체 색은 화살표를 비껴간 위치에서 확인한다.
    assert tuple(int(v) for v in view[120, 126]) == (255, 200, 100)
    assert tuple(int(v) for v in view[128, 128]) == (255, 255, 255)


@needs_map
def test_ego_view_translates_not_only_rotates(baked, data):
    """직선 구간에서도 전진이 화면에 반영되어야 한다.

    도로와 차선은 진행 방향으로 균일해서, 월드 고정 격자가 없으면 순수 전진이
    거의 같은 그림으로 보인다. 그래서 헤딩만 반영되는 것처럼 느껴진다.
    """
    import math
    link = max((l for l in data.valid_links if l.link_type == '6'),
               key=lambda l: l.points.shape[0])
    assert link.points.shape[0] > 40
    p = link.points

    def view_at(i):
        v = p[i + 1] - p[i - 1]
        ego = EgoState(x=float(p[i][0]), y=float(p[i][1]),
                       heading_deg=math.degrees(math.atan2(float(v[1]), float(v[0]))))
        return ego, ego_view(baked, ego, window_m=64.0, out_px=320)

    ego_a, view_a = view_at(5)
    ego_b, view_b = view_at(20)
    moved = math.hypot(ego_b.x - ego_a.x, ego_b.y - ego_a.y)
    assert moved > 3.0
    # 헤딩이 사실상 같은데도 화면은 달라져야 한다.
    assert abs(ego_b.heading_deg - ego_a.heading_deg) < 2.0
    changed = float((view_a != view_b).any(axis=2).mean())
    assert changed > 0.05, f'전진 {moved:.1f} m 에 화면 변화가 {changed:.1%} 뿐'


@needs_map
def test_world_grid_is_anchored_to_world_not_vehicle(baked):
    """격자는 차량이 아니라 월드에 고정되어야 한다."""
    from live_check import world_to_view
    ego = EgoState(x=0.0, y=0.0, heading_deg=0.0)
    # 정면 10 m 앞의 월드 점은 뷰에서 ego 위쪽(row 감소)에 있어야 한다.
    v = world_to_view(np.array([[10.0, 0.0]]), ego, window_m=64.0, out_px=320)[0]
    assert v[0] == pytest.approx(160.0, abs=1e-6)
    assert v[1] < 160.0
    # 같은 월드 점을 90도 돈 상태에서 보면 좌측(col 감소) 으로 간다.
    ego90 = EgoState(x=0.0, y=0.0, heading_deg=90.0)
    v90 = world_to_view(np.array([[10.0, 0.0]]), ego90, window_m=64.0, out_px=320)[0]
    assert v90[1] == pytest.approx(160.0, abs=1e-6)
    assert v90[0] > 160.0


def test_to_pixel_f_keeps_subpixel_precision():
    from mapbake.raster import RasterGrid as RG
    grid = RG.from_bounds(0, 0, 100, 100, pixels_per_meter=5.0, margin_m=0.0)
    pf = grid.to_pixel_f(np.array([[1.03, 2.07]]))[0]
    assert pf[0] == pytest.approx(5.15, abs=1e-9)
    assert pf[1] == pytest.approx(10.35, abs=1e-9)
    assert tuple(grid.to_pixel(np.array([[1.03, 2.07]]))[0]) == (5, 10)


def test_nmea_deg_converts_ddmm_to_degrees():
    from live_check import _nmea_deg
    # 3723.2475,N = 37도 23.2475분
    assert _nmea_deg('3723.2475', 'N') == pytest.approx(37 + 23.2475 / 60, abs=1e-9)
    assert _nmea_deg('12658.3210', 'E') == pytest.approx(126 + 58.3210 / 60, abs=1e-9)
    assert _nmea_deg('3723.2475', 'S') == pytest.approx(-(37 + 23.2475 / 60), abs=1e-9)
    assert _nmea_deg('', 'N') == 0.0


def test_gps_source_sockets_are_nonblocking():
    """타임아웃 방식으로 버퍼를 비우면 패킷이 타임아웃보다 자주 올 때
    drain 루프가 빠져나오지 못해 위치가 영영 갱신되지 않는다."""
    import socket as _s
    from live_check import GpsEgoSource
    src = GpsEgoSource(0, 0, (302595.0, 4124145.0))
    try:
        assert src.gps_sock.gettimeout() == 0.0
        assert src.ego_sock.gettimeout() == 0.0
        # 빈 버퍼에서 즉시 반환해야 한다 (블로킹이면 여기서 멈춘다)
        src._drain_gps()
        src._drain_ego()
        assert src.read() is None
    finally:
        src.close()


def test_gps_source_converts_utm_to_map_local():
    """NMEA -> UTM -> 맵 로컬 좌표 왕복."""
    import pyproj
    from live_check import GpsEgoSource
    origin = (302595.0, 4124145.0)
    target = (77.0, -539.0)
    fwd = pyproj.Transformer.from_crs('EPSG:32652', 'EPSG:4326', always_xy=True)
    lon, lat = fwd.transform(origin[0] + target[0], origin[1] + target[1])

    def dm(v):
        a = abs(v)
        d = int(a)
        return d * 100 + (a - d) * 60

    gga = ('$GPGGA,000000.00,%010.5f,N,%011.5f,E,1,12,0.8,30.0,M,20.0,M,,'
           % (dm(lat), dm(lon))).encode()
    src = GpsEgoSource(0, 0, origin)
    try:
        src.gps_sock.close()

        class _Fake:
            def __init__(self, payload):
                self.payload = payload

            def recvfrom(self, _n):
                if self.payload is None:
                    raise BlockingIOError
                p, self.payload = self.payload, None
                return p, ('127.0.0.1', 1)

            def close(self):
                pass

        src.gps_sock = _Fake(gga + b'\r\n')
        src._drain_gps()
        assert src._xy is not None
        assert src._xy[0] == pytest.approx(target[0], abs=0.5)
        assert src._xy[1] == pytest.approx(target[1], abs=0.5)
    finally:
        src.close()


# ------------------------------------------------ U턴 링크의 가짜 차선 (부채꼴)

def test_average_curve_radius_on_known_arc():
    from mapbake.topology import average_curve_radius
    for r in (5.0, 20.0, 200.0):
        t = np.linspace(0, np.pi, 200)
        arc = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
        assert average_curve_radius(arc) == pytest.approx(r, rel=0.02)
    line = np.stack([np.linspace(0, 50, 50), np.zeros(50)], axis=1)
    assert average_curve_radius(line) > 1e6


def test_long_sweeping_curve_is_not_treated_as_tight():
    """총 회전각으로 거르면 오벌 링크(1186 m 에 334도)가 통째로 지워진다.

    반경 기준이어야 긴 곡선 도로가 살아남는다.
    """
    from mapbake.topology import average_curve_radius
    t = np.linspace(0, np.deg2rad(334), 800)
    oval = np.stack([203.0 * np.cos(t), 203.0 * np.sin(t)], axis=1)
    assert average_curve_radius(oval) > 100.0     # 정상 도로로 남는다
    t2 = np.linspace(0, np.deg2rad(281), 200)
    uturn = np.stack([8.8 * np.cos(t2), 8.8 * np.sin(t2)], axis=1)
    assert average_curve_radius(uturn) < 10.0     # U턴은 걸러진다


@needs_map
def test_uturn_link_gets_no_lane_markings(data, grid):
    """U턴 링크에 차선을 그리면 차 앞에 부채꼴 가짜 차선이 생긴다."""
    from mapbake.topology import average_curve_radius
    uturn = next(l for l in data.valid_links if l.idx == 'A2256W001098')
    assert average_curve_radius(uturn.points) < 10.0
    res = bake_lane(data, grid)
    assert res.stats['tight_curve_links_skipped'] == 20

    lane = res.image
    road = bake_drivable(data, grid).image
    # U턴 경로 위에는 도로만 있고 차선은 없어야 한다.
    from mapbake.topology import offset_polyline, resample_polyline
    center = resample_polyline(uturn.points, 0.5)
    mid = center[len(center) // 4:3 * len(center) // 4]
    px_road = grid.to_pixel(mid)
    assert float((road[px_road[:, 1], px_road[:, 0]] > 0).mean()) > 0.9
    hits = 0
    for off in (+0.5 * uturn.width_m, -0.5 * uturn.width_m):
        px = grid.to_pixel(offset_polyline(mid, off))
        ok = ((px[:, 0] >= 0) & (px[:, 0] < lane.shape[1])
              & (px[:, 1] >= 0) & (px[:, 1] < lane.shape[0]))
        hits += int((lane[px[ok, 1], px[ok, 0]] > 0).sum())
    # 교차하는 다른 도로의 차선이 몇 픽셀 스칠 수는 있다
    assert hits < 0.1 * 2 * len(mid), f'U턴 경로에 차선 픽셀 {hits}개'


# ------------------------------------------------------- 기울어진 정지선

@needs_map
def test_adjacent_stop_lines_are_parallel(data):
    """정지선 직전에 휘는 차로 때문에 옆 차로 정지선과 어긋나면 안 된다.

    A2256W000309 는 정지선 직전 1.5 m 에서 우측으로 꺾여, 링크 접선만 쓰면
    -19.3도로 기울어진다. 옆 차로(A2256W000308)는 0.3도라 나란하지 않다.
    """
    import math
    from mapbake.topology import LinkTopology, stop_line_bar_directions
    topo = LinkTopology(data)
    resolved = {}
    for n in data.stop_line_nodes:
        hw = topo.node_heading(n.idx)
        if hw is not None:
            resolved[n.idx] = hw

    a, b = 'A1256W000593', 'A1256W000742'
    ha = math.degrees(math.atan2(float(resolved[a][0][1]), float(resolved[a][0][0])))
    hb = math.degrees(math.atan2(float(resolved[b][0][1]), float(resolved[b][0][0])))
    assert abs(ha - hb) > 15.0, '접선 기준으로는 어긋나 있어야 이 테스트가 의미가 있다'

    bars = stop_line_bar_directions(
        data.stop_line_nodes, {k: v[0] for k, v in resolved.items()})
    assert a in bars and b in bars
    assert float(np.dot(bars[a], bars[b])) == pytest.approx(1.0, abs=1e-6)


@needs_map
def test_stop_line_bars_are_perpendicular_and_collinear(data):
    """묶인 그룹은 (1) 노드가 한 직선 위에 있고 (2) 바가 주행방향에 수직해야 한다.

    거리만으로 묶으면 다른 진입부가 섞여 바가 도로를 따라 눕는다 (최악 89도).
    """
    import math
    from mapbake.topology import LinkTopology, stop_line_bar_directions
    topo = LinkTopology(data)
    heads = {}
    for n in data.stop_line_nodes:
        hw = topo.node_heading(n.idx)
        if hw is not None:
            heads[n.idx] = hw[0]
    bars = stop_line_bar_directions(data.stop_line_nodes, heads)
    assert len(bars) >= 40

    by_node = {n.idx: n for n in data.stop_line_nodes}
    worst = 0.0
    for idx, bar in bars.items():
        h = heads[idx]
        dev = abs(90.0 - math.degrees(math.acos(min(1.0, abs(float(np.dot(bar, h)))))))
        worst = max(worst, dev)
        # 같은 바를 공유하는 노드들은 그 바 위에 놓여 있어야 한다.
        # 방향만 같고 멀리 떨어진 다른 진입부는 같은 바가 아니므로 거리로 제한한다.
        here = by_node[idx].point
        same = [by_node[k].point for k, v in bars.items()
                if float(np.dot(v, bar)) > 0.999
                and float(np.linalg.norm(by_node[k].point - here)) <= 12.0]
        if len(same) >= 2:
            pts = np.stack(same)
            normal = np.array([-bar[1], bar[0]])
            resid = (pts - pts.mean(axis=0)).dot(normal)
            assert float(np.abs(resid).max()) < 0.3
    assert worst < 20.0


@needs_map
def test_single_lane_entries_fall_back_to_tangent(data, grid):
    """혼자 있는 정지선 노드는 묶을 상대가 없으니 링크 접선으로 그린다."""
    res = bake_stopline(data, grid)
    assert res.stats['drawn'] == 150
    assert res.stats['unresolved_heading'] == []
    # 전부 정렬되지는 않는다 (단일 차로 진입부가 다수)
    assert 0 < res.stats['aligned_across_lanes'] < 150


@needs_map
def test_stopline_alignment_can_be_disabled(data, grid):
    off = bake_stopline(data, grid, align_across_lanes=False)
    assert off.stats['aligned_across_lanes'] == 0
    assert off.stats['drawn'] == 150


# --------------------------------------- 노드 핀치로 생기는 대각선 차선

def _mk_link(idx, pts, from_node='n0', to_node=None, width=3.5, ltype='6'):
    from mapbake.io_mgeo import Link
    return Link(idx=idx, points=np.asarray(pts, dtype=np.float32),
                from_node=from_node, to_node=to_node, width_m=width,
                link_type=ltype, can_move_left=False, can_move_right=False,
                ego_lane=1, road_id='r')


def _chord_deviation(c):
    v = c[-1] - c[0]
    n = float(np.linalg.norm(v))
    return float(np.abs(np.cross(v / n, c - c[0])).max()) if n > 1e-6 else 0.0


def test_pinched_lane_is_straightened_and_fork_is_left_alone():
    """한 점에서 만나는 나란한 차로는 평행하게 되돌리고, 교차로 분기는 안 건드린다."""
    from mapbake.topology import lane_pinch_corrections
    s = np.linspace(0, 60, 121)
    # 실제 맵처럼 양끝 노드를 공유한다 (A2256W000172 / A2256W000176 과 같은 구조)
    straight = _mk_link('straight', np.stack([s, np.zeros_like(s)], axis=1),
                        to_node='n1')
    pinched = _mk_link('pinched',
                       np.stack([s, -3.5 * np.sin(np.pi * s / 60.0)], axis=1),
                       to_node='n1')
    fork = _mk_link('fork',
                    np.stack([s * np.cos(np.pi / 4), s * np.sin(np.pi / 4)], axis=1),
                    to_node='n2')

    fixed = lane_pinch_corrections([straight, pinched, fork])
    assert 'fork' not in fixed, '교차로 분기를 건드리면 정상 차선이 망가진다'
    assert 'pinched' in fixed
    # 되돌린 뒤에는 기준선과 나란해야 한다
    assert _chord_deviation(fixed['pinched']) < 0.5
    # 지우는 게 아니라 옮기는 것이므로 길이가 유지되어야 한다
    from mapbake.topology import resample_polyline
    assert len(fixed['pinched']) == len(resample_polyline(pinched.points, 0.5))


def test_correction_keeps_lane_parallel_at_lane_width():
    """되돌린 차로는 기준선과 한 차로 폭만큼 떨어져 나란해야 한다."""
    from mapbake.topology import lane_pinch_corrections, resample_polyline
    s = np.linspace(0, 60, 121)
    ref = _mk_link('ref', np.stack([s, np.zeros_like(s)], axis=1), to_node='n1')
    dev = _mk_link('dev', np.stack([s, -3.5 * np.sin(np.pi * s / 60.0)], axis=1),
                   to_node='n1')
    fixed = lane_pinch_corrections([ref, dev])['dev']
    base = resample_polyline(ref.points, 0.5)
    n = min(len(fixed), len(base))
    sep = np.linalg.norm(fixed[:n] - base[:n], axis=1)
    assert float(sep.min()) > 3.0
    assert float(sep.max()) < 4.0


@needs_map
def test_shared_endpoint_lanes_are_corrected(data):
    """A2256W000172 와 A2256W000176 은 시작·끝점이 모두 같다.

    끝점에서 간격을 재면 0 이라 분기로 오판된다. 가장 벌어진 지점에서 재야 한다.
    """
    from mapbake.topology import lane_pinch_corrections, resample_polyline
    links = {l.idx: l for l in data.valid_links}
    a, b = links['A2256W000172'], links['A2256W000176']
    assert float(np.linalg.norm(a.points[0] - b.points[0])) < 0.3
    assert float(np.linalg.norm(a.points[-1] - b.points[-1])) < 0.3

    before = _chord_deviation(resample_polyline(a.points, 0.5))
    assert before > 3.0, '보정 전에는 한 차로 폭만큼 휘어 있어야 한다'
    fixed = lane_pinch_corrections(data.valid_links)
    assert a.idx in fixed
    assert _chord_deviation(fixed[a.idx]) < 0.5


@needs_map
def test_three_lanes_at_one_node_all_get_straightened(data):
    """한 노드에 셋이 모이면 쌍마다 고칠 때 서로 덮어써서 2 m 넘게 남았다.

    노드마다 기준선 하나를 정하고 나머지를 그 기준으로 옮겨야 한다.
    """
    from mapbake.topology import lane_pinch_corrections, resample_polyline
    fixed = lane_pinch_corrections(data.valid_links)
    for idx in ('A2256W000172', 'A2256W000005'):
        assert idx in fixed
        assert _chord_deviation(fixed[idx]) < 0.5, idx


@needs_map
def test_correction_never_moves_a_lane_more_than_one_width(data):
    """핀치는 최대 한 차로 폭만큼 어긋나는 현상이다.

    상한이 없을 때 A2256W000423 이 7.03 m(두 차로 폭) 밀려나, ego 가 제 차로
    한가운데 있는데도 차선에 걸친 것처럼 보였다. 최대 13.97 m 까지 밀린 링크도 있었다.
    """
    from mapbake.topology import lane_pinch_corrections, resample_polyline
    base = {l.idx: resample_polyline(l.points, 0.5) for l in data.valid_links}

    loose = lane_pinch_corrections(data.valid_links, max_shift_m=1e9)
    tight = lane_pinch_corrections(data.valid_links, max_shift_m=4.0)

    def worst(corr):
        out = 0.0
        for idx, c in corr.items():
            o = base[idx]
            n = min(len(o), len(c))
            out = max(out, float(np.linalg.norm(o[:n] - c[:n], axis=1).max()))
        return out

    assert worst(tight) <= 4.0
    assert len(tight) <= len(loose)
    # ego 가 실제로 달리는 차로는 제자리에 있어야 한다
    assert 'A2256W000423' not in tight


@needs_map
def test_road_and_lane_share_corrected_centerlines(data, grid):
    """차선만 보정하면 도로는 노드 근처에서 좁은 채라 차선이 밖으로 나간다."""
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    # 양끝 노드를 모두 공유하는(=핀치 인공물) 쌍만 고친다
    assert len(fixed) == 2

    road = bake_drivable(data, grid, centerlines=fixed).image
    lane = bake_lane(data, grid, centerlines=fixed).image
    import cv2
    band = grid.meters_to_px(0.6)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * band + 1, 2 * band + 1))
    near = cv2.dilate(road, k) > 0
    assert float(near[lane > 0].mean()) == pytest.approx(1.0, abs=1e-3)

    # 보정을 도로에만 빼먹으면 차선이 밖으로 삐져나간다
    road_raw = bake_drivable(data, grid).image
    near_raw = cv2.dilate(road_raw, k) > 0
    assert float(near_raw[lane > 0].mean()) < 1.0


# ------------------------------------------------ 정지선 일부만 굽기

@needs_map
def test_stopline_include_and_exclude(data, grid):
    both = ['A1256W000593', 'A1256W000742']
    only = bake_stopline(data, grid, include_nodes=both)
    assert only.stats['drawn'] == 2
    assert only.stats['filtered_out'] == 148

    dropped = bake_stopline(data, grid, exclude_nodes=['A1256W000593'])
    assert dropped.stats['drawn'] == 149
    assert dropped.stats['filtered_out'] == 1

    empty = bake_stopline(data, grid, include_nodes=[])
    assert empty.stats['drawn'] == 0
    assert int((empty.image > 0).sum()) == 0


@needs_map
def test_stopline_only_with_traffic_light(data, grid):
    res = bake_stopline(data, grid, only_with_traffic_light=True)
    expected = sum(1 for n in data.stop_line_nodes if n.traffic_light_id)
    assert res.stats['drawn'] == expected == 69


@needs_map
def test_filtering_keeps_direction_aligned(data, grid):
    """한 줄에서 하나만 골라도 이웃과 같은 각도여야 한다.

    방향 계산은 선택 여부와 무관하게 전체 노드로 해야 한다. 선택된 것만으로
    묶으면 짝이 사라져 접선 법선으로 폴백하면서 정지선이 기울어진다.
    """
    import math

    def bar_angle(img):
        ys, xs = np.nonzero(img)
        assert xs.size > 0
        pts = np.stack([xs, ys], axis=1).astype(np.float64)
        pts -= pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts)
        return math.degrees(math.atan2(vt[0][1], vt[0][0])) % 180.0

    alone = bake_stopline(data, grid, include_nodes=['A1256W000593'])
    pair = bake_stopline(data, grid,
                         include_nodes=['A1256W000593', 'A1256W000742'])
    assert bar_angle(alone.image) == pytest.approx(bar_angle(pair.image), abs=1.0)
    # 정렬을 끄면 접선 기준이라 눈에 띄게 기운다
    raw = bake_stopline(data, grid, include_nodes=['A1256W000593'],
                        align_across_lanes=False)
    assert abs(bar_angle(raw.image) - bar_angle(alone.image)) > 10.0


@needs_map
def test_route_selection_picks_stop_lines_on_that_route(data):
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from bake_map import stop_line_nodes_on_route
    route = REPO_ROOT / 'src/behavior_planner/src/config/route_links.yaml'
    if not route.is_file():
        pytest.skip('route yaml 없음')
    ids = stop_line_nodes_on_route(data, route)
    assert 0 < len(ids) < len(data.stop_line_nodes)
    stop_ids = {n.idx for n in data.stop_line_nodes}
    assert set(ids) <= stop_ids


# ------------------------------------------------ 가장 가까운 정지선 / 수집

@needs_map
def test_h5_carries_stop_line_vectors(baked):
    """래스터만으로는 '가장 가까운 정지선까지 몇 m' 를 답할 수 없다."""
    assert baked.stop_pts.shape == (150, 2)
    assert baked.stop_headings.shape == (150, 2)
    assert len(baked.stop_ids) == 150
    assert len(baked.stop_tls) == 150
    assert sum(1 for t in baked.stop_tls if t) == 69
    norms = np.linalg.norm(baked.stop_headings, axis=1)
    assert np.allclose(norms, 1.0, atol=1e-3)


@needs_map
def test_nearest_stop_line_splits_ahead_and_behind(baked):
    """유클리드 거리만으로는 지나친 정지선과 앞의 정지선이 구분되지 않는다."""
    from live_check import nearest_stop_lines
    # 정지선은 옆 차로에 3.5 m 간격으로 붙어 있어, 아무거나 고르면 20 m 뒤에서
    # 다른 정지선이 더 가까울 수 있다. 주변 40 m 안에 다른 정지선이 없는 것을 쓴다.
    pts = baked.stop_pts.astype(np.float64)
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    isolated = np.nonzero(d.min(axis=1) > 40.0)[0]
    assert isolated.size > 0
    i = int(isolated[0])
    sx, sy = float(baked.stop_pts[i][0]), float(baked.stop_pts[i][1])
    hx, hy = float(baked.stop_headings[i][0]), float(baked.stop_headings[i][1])
    heading = math.degrees(math.atan2(hy, hx))

    # 정지선 20 m 앞(진행 방향 기준 뒤쪽)에 서서 바라본다
    before = EgoState(x=sx - 20 * hx, y=sy - 20 * hy, heading_deg=heading)
    got = nearest_stop_lines(baked, before, count=1)[0]
    assert got['idx'] == baked.stop_ids[i]
    assert got['ahead_m'] == pytest.approx(20.0, abs=1.0)
    assert abs(got['lateral_m']) < 1.0
    assert got['facing_deg'] == pytest.approx(0.0, abs=1.0)

    # 지나친 뒤에는 ahead 가 음수여야 한다
    after = EgoState(x=sx + 10 * hx, y=sy + 10 * hy, heading_deg=heading)
    assert nearest_stop_lines(baked, after, count=1)[0]['ahead_m'] < 0

    # 반대 방향으로 보면 통과각이 180도에 가깝다 (교차 도로 거르기용)
    opposite = EgoState(x=sx - 20 * hx, y=sy - 20 * hy, heading_deg=heading + 180.0)
    assert nearest_stop_lines(baked, opposite, count=1)[0]['facing_deg'] > 170.0


def test_format_collected_is_pastable_yaml():
    import yaml as _yaml
    from live_check import format_collected
    items = [
        {'idx': 'A1', 'x': 1.0, 'y': 2.0, 'traffic_light_id': 'C1', 'facing_deg': 3.0},
        {'idx': 'A2', 'x': -3.5, 'y': 4.25, 'traffic_light_id': '', 'facing_deg': 0.0},
    ]
    text = format_collected(items)
    parsed = _yaml.safe_load(text)
    assert parsed['stopline']['include_nodes'] == ['A1', 'A2']
    assert '수집된 정지선이 없습니다' in format_collected([])


@needs_map
def test_collected_ids_are_usable_as_include_nodes(data, grid, baked):
    """수집한 id 를 그대로 include_nodes 에 넣으면 그것만 구워져야 한다."""
    import yaml as _yaml
    from live_check import format_collected
    items = [{'idx': baked.stop_ids[k], 'x': 0.0, 'y': 0.0,
              'traffic_light_id': '', 'facing_deg': 0.0} for k in (0, 5, 9)]
    ids = _yaml.safe_load(format_collected(items))['stopline']['include_nodes']
    res = bake_stopline(data, grid, include_nodes=ids)
    assert res.stats['drawn'] == 3
    assert res.stats['filtered_out'] == 147


def test_collected_yaml_round_trips_for_append(tmp_path):
    """정지선마다 프로그램을 껐다 켜며 모을 때 이전 결과가 유지되어야 한다."""
    from live_check import format_collected, load_collected
    first = [{'idx': 'A1', 'x': 1.0, 'y': 2.0,
              'traffic_light_id': 'C1', 'facing_deg': 1.0}]
    f = tmp_path / 'acc.yaml'
    f.write_text(format_collected(first), encoding='utf-8')

    before = load_collected(f)
    assert [c['idx'] for c in before] == ['A1']

    # 새로 모은 것 중 이미 있는 건 빼고 합친다
    fresh = [{'idx': 'A1', 'x': 1.0, 'y': 2.0, 'traffic_light_id': 'C1',
              'facing_deg': 1.0},
             {'idx': 'A2', 'x': 3.0, 'y': 4.0, 'traffic_light_id': '',
              'facing_deg': 2.0}]
    seen = {c['idx'] for c in before}
    merged = before + [c for c in fresh if c['idx'] not in seen]
    text = format_collected(merged)
    import yaml as _yaml
    assert _yaml.safe_load(text)['stopline']['include_nodes'] == ['A1', 'A2']
    assert load_collected(tmp_path / 'missing.yaml') == []


@needs_map
def test_two_lanes_of_one_stop_line_form_a_single_bar(data, grid):
    """차로마다 노드가 따로 있다. 둘을 넣어도 정지선이 둘이 되는 게 아니라
    한 줄로 이어진다 — 하나만 넣으면 그 차로만 덮는다."""
    pair = ['A1256W000742', 'A1256W000593']
    a, b = (next(n for n in data.stop_line_nodes if n.idx == i) for i in pair)
    assert float(np.linalg.norm(a.point - b.point)) == pytest.approx(4.05, abs=0.2)
    assert a.traffic_light_id == b.traffic_light_id

    both = bake_stopline(data, grid, include_nodes=pair).image
    # 두 노드를 잇는 구간이 빈틈없이 채워져 있어야 한 줄이다
    t = np.linspace(0, 1, 60)[:, None]
    seg = a.point * (1 - t) + b.point * t
    px = grid.to_pixel(seg)
    assert float((both[px[:, 1], px[:, 0]] > 0).mean()) > 0.95

    # 하나만 넣으면 그 차로 폭(3.5 m)만 덮는다
    one = bake_stopline(data, grid, include_nodes=[pair[0]]).image
    assert int((one > 0).sum()) < 0.65 * int((both > 0).sum())


# ------------------------------------------------ 합류부에서 도로가 뚝 좁아짐

@needs_map
def test_merge_taper_extends_only_merge_ends(data):
    """차로가 줄어드는 합류부만 늘린다. 진짜 막다른 끝은 늘리면 안 된다."""
    import collections
    from mapbake.topology import merge_taper_ends
    inc, out = collections.Counter(), collections.Counter()
    for l in data.valid_links:
        if l.to_node:
            inc[l.to_node] += 1
        if l.from_node:
            out[l.from_node] += 1
    tapers = merge_taper_ends(data.valid_links)
    assert len(tapers) > 0
    by_idx = {l.idx: l for l in data.valid_links}
    for idx, (head, tail) in tapers.items():
        link = by_idx[idx]
        if tail:
            # 이어지는 링크가 있고, 들어오는 쪽이 더 많아야 한다
            assert out[link.to_node] > 0
            assert inc[link.to_node] > out[link.to_node]
        if head:
            assert inc[link.from_node] > 0
            assert out[link.from_node] > inc[link.from_node]

    # 이어지는 링크가 없는 막다른 끝은 늘리지 않는다 (없는 도로를 만들면 안 된다)
    for l in data.valid_links:
        if l.to_node and out[l.to_node] == 0:
            assert not tapers.get(l.idx, (False, False))[1]


def test_extend_polyline_adds_requested_length():
    from mapbake.topology import extend_polyline
    pts = np.stack([np.linspace(0, 10, 21), np.zeros(21)], axis=1).astype(np.float32)
    out = extend_polyline(pts, 2.0, 3.0)
    assert out[0][0] == pytest.approx(-2.0, abs=1e-4)
    assert out[-1][0] == pytest.approx(13.0, abs=1e-4)
    assert extend_polyline(pts, 0.0, 0.0).shape == pts.shape


@needs_map
def test_merge_taper_widens_road_without_moving_lanes(data, grid):
    """도로만 늘어나야 한다 — 차선은 그대로여야 하고, 차선이 도로 밖으로
    나가지 않아야 한다."""
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    off = bake_drivable(data, grid, merge_taper_m=0.0, centerlines=fixed)
    on = bake_drivable(data, grid, merge_taper_m=10.0, centerlines=fixed)
    assert on.stats['merge_tapered_links'] > 100
    assert off.stats['merge_tapered_links'] == 0
    # 넓어지되 과하지 않아야 한다
    ratio = on.stats['filled_px'] / off.stats['filled_px']
    assert 1.0 < ratio < 1.05
    # 늘린 도로는 원래 도로를 포함해야 한다 (지우지 않는다)
    assert int(((off.image > 0) & (on.image == 0)).sum()) == 0

    lane = bake_lane(data, grid, centerlines=fixed).image
    import cv2
    band = grid.meters_to_px(0.6)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * band + 1, 2 * band + 1))
    assert float((cv2.dilate(on.image, k) > 0)[lane > 0].mean()) == pytest.approx(1.0, abs=1e-3)


# ------------------------------------------------ 링크 단위 제외

@needs_map
def test_exclude_links_removes_from_road_and_lane(data, grid):
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    picked = [l.idx for l in data.valid_links if l.link_type == '6'][:20]

    base_r = bake_drivable(data, grid, centerlines=fixed)
    base_l = bake_lane(data, grid, centerlines=fixed)
    cut_r = bake_drivable(data, grid, exclude_links=picked, centerlines=fixed)
    cut_l = bake_lane(data, grid, exclude_links=picked, centerlines=fixed)

    assert cut_r.stats['links_excluded'] == 20
    assert cut_r.stats['filled_px'] < base_r.stats['filled_px']
    assert cut_l.stats['filled_px'] < base_l.stats['filled_px']
    # 제외한 링크는 features 에서도 빠져야 한다 (H5 의 소속 정보와 일관)
    assert not ({f['idx'] for f in cut_r.features} & set(picked))
    assert not ({f['idx'] for f in cut_l.features} & set(picked))


@needs_map
def test_lane_only_exclusion_keeps_road(data, grid):
    """차선만 지우고 주행가능영역은 남기는 경우."""
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    picked = [l.idx for l in data.valid_links if l.link_type == '6'][:10]
    road = bake_drivable(data, grid, centerlines=fixed)
    lane = bake_lane(data, grid, exclude_links=picked, centerlines=fixed)
    assert road.stats['links_excluded'] == 0
    assert lane.stats['links_excluded'] == 10


@needs_map
def test_h5_carries_link_vectors_and_membership(baked, data):
    assert len(baked.link_ids) == 634
    assert baked.link_pts.shape[0] > 1000
    assert baked.link_owner.shape[0] == baked.link_pts.shape[0]
    assert sum(baked.link_in_road) == 634
    assert 0 < sum(baked.link_in_lane) < 634


@needs_map
def test_nearest_link_finds_the_link_under_ego(baked, data):
    from live_check import nearest_link
    link = next(l for l in data.valid_links if l.points.shape[0] > 20)
    p = link.points[len(link.points) // 2]
    got = nearest_link(baked, EgoState(x=float(p[0]), y=float(p[1]), heading_deg=0.0))
    assert got is not None
    assert got['distance_m'] < 1.0
    assert got['idx'] == link.idx


def test_links_yaml_round_trips():
    import yaml as _yaml
    from live_check import format_links, load_links
    items = [{'idx': 'A1', 'x': 1.0, 'y': 2.0, 'in_road': True, 'in_lane': False}]
    text = format_links(items)
    assert _yaml.safe_load(text)['exclude_links'] == ['A1']
    assert '수집된 링크가 없습니다' in format_links([])


@needs_map
def test_collected_links_are_usable_as_exclude_links(data, grid, baked, tmp_path):
    import yaml as _yaml
    from live_check import format_links
    from mapbake.topology import lane_pinch_corrections
    picked = [{'idx': baked.link_ids[k], 'x': 0.0, 'y': 0.0,
               'in_road': True, 'in_lane': True} for k in (0, 3, 7)]
    ids = _yaml.safe_load(format_links(picked))['exclude_links']
    fixed = lane_pinch_corrections(data.valid_links)
    res = bake_drivable(data, grid, exclude_links=ids, centerlines=fixed)
    assert res.stats['links_excluded'] == 3


@needs_map
def test_real_merge_is_not_treated_as_pinch(data):
    """실제로 합류하는 차로를 '핀치'로 오판해 벌려놓으면 안 된다.

    A2256W000422 / A2256W000445 / A2256W000423 은 (75, -218) 로 모이는 진짜 합류다.
    이걸 고치려 들면 A2256W000422 가 74.4 -> 77.6 m 로 3.2 m 밀려나,
    사용자가 제외한 차로(A2256W000425, x=78.9) 자리에 도로가 되살아난다.
    """
    from mapbake.topology import lane_pinch_corrections, resample_polyline
    links = {l.idx: l for l in data.valid_links}
    a, b = links['A2256W000422'], links['A2256W000423']
    # 끝점은 같지만 시작점은 다르다 -> 인공물이 아니라 합류
    assert float(np.linalg.norm(a.points[-1] - b.points[-1])) < 0.5
    assert float(np.linalg.norm(a.points[0] - b.points[0])) > 2.0

    fixed = lane_pinch_corrections(data.valid_links)
    assert 'A2256W000422' not in fixed
    assert 'A2256W000423' not in fixed


@needs_map
def test_excluded_lane_stays_empty(data, grid):
    """제외한 차로 자리에 다른 링크가 밀려와 도로를 되살리면 안 된다."""
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    road = bake_drivable(data, grid, exclude_links=['A2256W000425'],
                         centerlines=fixed).image
    # A2256W000425 (x=78.9) 한가운데. 옆 차로는 3.5 m 떨어져 있으니 비어야 한다.
    for y in (-206.0, -210.0, -214.0):
        c, r = grid.to_pixel(np.array([[78.9, y]]))[0]
        assert road[r, c] == 0, f'제외한 차로 (78.9, {y}) 에 도로가 남아 있다'


# ------------------------------------------------ 객체 게이팅

def _road_edge_probe(baked, data):
    """도로 위 한 점, 그 법선, heading, 그리고 '확실히 도로 밖'인 거리를 찾는다.

    다차로 도로에서는 중심선에서 40 m 밖도 여전히 도로일 수 있고, 제외된 링크를
    고르면 바로 옆이 도로 밖이면서 조금 더 가면 다른 도로가 나온다.
    그래서 실제로 object_on_road 가 False 인 지점을 확인해서 돌려준다.
    """
    from live_check import ObjectState, object_on_road
    for link in data.valid_links:
        if link.link_type != '6' or link.points.shape[0] < 40:
            continue
        i = link.points.shape[0] // 2
        c = link.points[i].astype(np.float64)
        v = (link.points[i + 1] - link.points[i - 1]).astype(np.float64)
        n = np.array([-v[1], v[0]])
        nrm = float(np.linalg.norm(n))
        if nrm < 1e-6:
            continue
        n /= nrm
        heading = math.degrees(math.atan2(float(v[1]), float(v[0])))
        # 중심선 자체가 도로 위여야 한다 (제외된 링크는 건너뛴다)
        if not object_on_road(baked, ObjectState(0, 1, float(c[0]), float(c[1]),
                                                 heading, 4.6, 1.9)):
            continue
        edge = None
        for d in np.arange(0.5, 60.0, 0.25):
            p = c + d * n
            o = ObjectState(0, 1, float(p[0]), float(p[1]), heading, 4.6, 1.9)
            if not object_on_road(baked, o):
                if edge is None:
                    edge = float(d)
            elif edge is not None:
                edge = None            # 다시 도로가 나왔다 -> 더 밖을 본다
            if edge is not None and d - edge > 8.0:
                return c, n, heading, edge
    pytest.skip('도로 경계를 찾지 못했다')


def _fake_object_packet(objs):
    """MORAI Object Info UDP 패킷 합성 (id, type, x, y, heading, L, W)."""
    from live_check import OBJ_HDR, OBJ_ONE_FMT
    body = bytearray(8 + 106 * 20)
    struct.pack_into('<II', body, 0, 1700000000, 0)
    off = 8
    for oid, typ, x, y, hd, L, W in objs:
        struct.pack_into(OBJ_ONE_FMT, body, off, oid, typ, x, y, 0.0, hd,
                         L, W, 1.5, 0., 0., 0., 0., 0., 0., 0., 0., 0., b'o')
        off += 106
    return OBJ_HDR + b'\x00' * 16 + bytes(body) + b'\r\n'


def test_parse_object_packet_reads_positions():
    from live_check import parse_object_packet
    pkt = _fake_object_packet([(7, 1, 12.5, -34.25, 90.0, 4.6, 1.9),
                               (8, 0, -1.0, 2.0, 0.0, 0.8, 0.8)])
    objs = parse_object_packet(pkt)
    assert objs is not None and len(objs) == 2
    assert objs[0].idx == 7 and objs[0].type_id == 1
    assert objs[0].x == pytest.approx(12.5, abs=1e-3)
    assert objs[0].y == pytest.approx(-34.25, abs=1e-3)
    assert objs[0].type_name == 'npc'
    assert objs[1].type_name == 'ped'
    # 빈 슬롯(id 0, 좌표 0)은 걸러진다
    assert len(parse_object_packet(_fake_object_packet([]))) == 0
    assert parse_object_packet(b'nonsense') is None


@needs_map
def test_object_gating_matches_road_mask(baked, data):
    """주행가능영역 안의 객체만 통과시켜야 한다."""
    from live_check import ObjectState, object_on_road
    on_pt, normal, heading, edge = _road_edge_probe(baked, data)
    off_pt = on_pt + (edge + 4.0) * normal       # 확인된 도로 밖 구간 안쪽

    on = ObjectState(idx=1, type_id=1, x=float(on_pt[0]), y=float(on_pt[1]),
                     heading_deg=heading, length_m=4.6, width_m=1.9)
    off = ObjectState(idx=2, type_id=1, x=float(off_pt[0]), y=float(off_pt[1]),
                      heading_deg=heading, length_m=4.6, width_m=1.9)
    assert object_on_road(baked, on) is True
    assert object_on_road(baked, off) is False


@needs_map
def test_object_margin_rescues_edge_cases(baked, data):
    """경계에 걸친 객체는 여유를 주면 살아남아야 한다.

    측위 오차로 경계 객체가 들락거리는 걸 막는 장치다.
    """
    from live_check import ObjectState, object_on_road
    on_pt, normal, heading, edge = _road_edge_probe(baked, data)

    # 도로 경계 바로 바깥에서 '여유 없이는 탈락, 여유 주면 통과' 지점을 찾는다
    found = False
    for d in np.arange(edge, edge + 3.0, 0.2):
        p = on_pt + d * normal
        o = ObjectState(idx=1, type_id=0, x=float(p[0]), y=float(p[1]),
                        heading_deg=heading, length_m=0.8, width_m=0.8)
        if not object_on_road(baked, o) and object_on_road(baked, o, margin_px=10):
            found = True
            break
    assert found, '여유를 줬을 때만 통과하는 경계 지점이 있어야 한다'


@needs_map
def test_draw_objects_hides_off_road_ones(baked, data):
    from live_check import EgoState as ES, ObjectState, draw_objects, ego_view
    p, normal, _heading, edge = _road_edge_probe(baked, data)
    ego = ES(x=float(p[0]), y=float(p[1]), heading_deg=0.0)
    off = p + (edge + 4.0) * normal
    objs = [ObjectState(1, 1, float(p[0]), float(p[1]), 0.0, 4.6, 1.9),
            ObjectState(2, 1, float(off[0]), float(off[1]), 0.0, 4.6, 1.9)]

    base = ego_view(baked, ego, 80.0, 320)
    hid = base.copy()
    n_in, n_out = draw_objects(hid, baked, ego, objs, 80.0, 320,
                               show_outside=False)
    assert (n_in, n_out) == (1, 1)
    shown = base.copy()
    draw_objects(shown, baked, ego, objs, 80.0, 320, show_outside=True)
    # 밖 객체를 그린 쪽이 더 많이 칠해져야 한다
    assert int((shown != hid).any(axis=2).sum()) > 0


# ------------------------------------------------ 최외곽 깎기 + 차선 클립

def test_shrink_outer_only_touches_outer_boundary():
    from mapbake.raster import shrink_outer
    img = np.zeros((200, 200), dtype=np.uint8)
    img[50:150, 50:150] = 255
    out = shrink_outer(img, pixels_per_meter=5.0, trim_m=0.2)   # 1 px
    assert out[50, 100] == 0 and out[149, 100] == 0     # 바깥 경계는 깎인다
    assert out[100, 100] == 255                          # 안쪽은 그대로
    assert shrink_outer(img, 5.0, 0.0) is img            # 0 이면 원본 그대로


def test_shrink_outer_resolves_half_pixel_steps():
    """홀수 커널만 쓰면 반지름이 정수 픽셀로 갇혀 0.3 m 요청이 0.4 m 로 반올림된다.

    ppm=5 에서 차선 0.4 m 의 1/2(0.2) 와 3/4(0.3) 를 구분하려면 짝수 커널이 필요하다.
    """
    from mapbake.raster import shrink_outer
    R = 200
    disc = np.zeros((2 * R + 40, 2 * R + 40), dtype=np.uint8)
    import cv2
    cv2.circle(disc, (R + 20, R + 20), R, 255, -1)

    def eroded_radius(trim):
        out = shrink_outer(disc, 5.0, trim)
        return float(((out > 0).sum() / np.pi) ** 0.5)

    r2, r3, r4 = (eroded_radius(t) for t in (0.2, 0.3, 0.4))
    # 세 값이 서로 구분되어야 한다 (예전에는 0.3 과 0.4 가 같았다)
    assert r2 > r3 > r4
    assert (R - r3) / 5.0 == pytest.approx(0.3, abs=0.08)


@needs_map
def test_outer_trim_cuts_road_and_lane_by_same_amount(data, grid):
    """차선 바깥 절반을 자르면 도로도 같은 양만큼 줄어야 바깥 끝이 맞는다."""
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)

    plain_road = bake_drivable(data, grid, centerlines=fixed)
    plain_lane = bake_lane(data, grid, centerlines=fixed)
    assert int(((plain_lane.image > 0) & (plain_road.image == 0)).sum()) > 0, \
        '깎기 전에는 차선이 도로 밖으로 나가 있어야 이 테스트가 의미가 있다'

    trim_road = bake_drivable(data, grid, outer_trim_m=0.3, centerlines=fixed)
    trim_lane = bake_lane(data, grid, clip_to=trim_road.image, centerlines=fixed)

    # 도로는 줄어들되 연결성은 유지된다 (차로 사이는 침식 영향을 안 받는다)
    assert trim_road.stats['filled_px'] < plain_road.stats['filled_px']
    import cv2
    n_before = cv2.connectedComponentsWithStats(
        (plain_road.image > 0).astype(np.uint8), 8)[0]
    n_after = cv2.connectedComponentsWithStats(
        (trim_road.image > 0).astype(np.uint8), 8)[0]
    assert n_after == n_before

    # 차선은 전부 도로 안에 있어야 한다
    outside = int(((trim_lane.image > 0) & (trim_road.image == 0)).sum())
    assert outside == 0, f'차선 {outside} px 가 아직 도로 밖이다'
    assert trim_lane.stats['clipped_to_road_m2'] > 0


@needs_map
def test_lane_clip_is_optional(data, grid):
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    res = bake_lane(data, grid, clip_to=None, centerlines=fixed)
    assert res.stats['clipped_to_road_m2'] == 0.0


# ------------------------------------------------ 최외곽 차선 안쪽 이동

@needs_map
def test_outer_side_flags_detect_road_edges(data):
    """차로 옆에 이웃이 있는지로 최외곽을 판정한다."""
    from mapbake.topology import outer_side_flags, lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    flags = outer_side_flags(data.valid_links, fixed)
    assert len(flags) == len(data.valid_links)
    both = sum(1 for l, r in flags.values() if l and r)
    one = sum(1 for l, r in flags.values() if l != r)
    none = sum(1 for l, r in flags.values() if not l and not r)
    # 단일 차로 도로(양쪽 최외곽), 끝 차로(한쪽), 가운데 차로(없음) 가 다 있어야 한다
    assert both > 0 and one > 0 and none > 0
    # 가운데 차로가 최외곽으로 잡히면 안 된다: 이웃이 양쪽에 있는 링크를 하나 확인
    import collections
    by_road = collections.defaultdict(list)
    for l in data.valid_links:
        by_road[l.road_id].append(l)
    multi = [ls for ls in by_road.values() if len(ls) >= 3]
    assert multi, '3차로 이상 도로가 있어야 이 검사가 의미 있다'
    # 가운데 차로가 양쪽 다 최외곽으로 잡히면 안 된다. 다만 차로가 한 점으로
    # 모이는 합류형 도로에서는 옆을 짚어도 이웃이 안 잡히므로, 나란한 도로가
    # 하나라도 제대로 판정되는지로 확인한다.
    mids = [sorted(ls, key=lambda l: l.ego_lane or 0)[1] for ls in multi]
    assert any(flags[m.idx] != (True, True) for m in mids), \
        '나란한 다차로 도로의 가운데 차로는 최외곽이 아니어야 한다'


@needs_map
def test_outer_shift_preserves_lane_thickness(data, grid):
    """도로를 깎으면 최외곽 차선도 옮겨야 두께가 남는다.

    옮기지 않고 잘라내기만 하면 최외곽 차선이 반쪽(1 px)으로 줄어든다.
    """
    from mapbake.topology import lane_pinch_corrections
    fixed = lane_pinch_corrections(data.valid_links)
    road = bake_drivable(data, grid, outer_trim_m=0.3, centerlines=fixed)

    clipped = bake_lane(data, grid, clip_to=road.image, centerlines=fixed)
    shifted = bake_lane(data, grid, clip_to=road.image, outer_shift_m=0.5,
                        centerlines=fixed)

    assert clipped.stats['outer_shifted_sides'] == 0
    assert shifted.stats['outer_shifted_sides'] > 100
    # 옮기면 차선이 더 많이 남는다 (두께 보존)
    assert shifted.stats['filled_px'] > clipped.stats['filled_px'] * 1.3
    # 그래도 도로 밖으로 나가지는 않는다
    assert int(((shifted.image > 0) & (road.image == 0)).sum()) == 0


# ------------------------------------------------ 외부 파트용 조회 모듈

@needs_map
def test_drivable_query_matches_raster():
    """다른 파트에 넘길 조회 모듈이 래스터와 같은 답을 내야 한다."""
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from drivable_query import DrivableMask
    h5 = PKG_ROOT / 'out' / 'link_node_map.h5'
    if not h5.is_file():
        pytest.skip('구운 H5 가 없다')

    dm = DrivableMask.load(h5)
    from live_check import BakedMap
    ref = BakedMap.load(h5)
    assert dm.map_id == ref.map_id
    assert dm.pixels_per_meter == ref.grid.pixels_per_meter

    rng = np.random.default_rng(3)
    pts = rng.uniform([-400, -770], [110, 560], size=(5000, 2))
    got = dm.contains(pts)
    px = ref.grid.to_pixel(pts)
    h, w = ref.road.shape
    ok = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
    want = np.zeros(len(pts), dtype=bool)
    want[ok] = ref.road[px[ok, 1], px[ok, 0]] > 0
    assert np.array_equal(got, want)

    # 맵 범위 밖은 False
    assert not dm.contains(np.array([[1e6, 1e6]]))[0]


@needs_map
def test_drivable_query_margin_and_map_guard():
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from drivable_query import DrivableMask
    h5 = PKG_ROOT / 'out' / 'link_node_map.h5'
    if not h5.is_file():
        pytest.skip('구운 H5 가 없다')

    tight = DrivableMask.load(h5)
    loose = DrivableMask.load(h5, margin_m=1.0)
    assert int((loose.mask > 0).sum()) > int((tight.mask > 0).sum())

    # 맵 판본이 다르면 막아야 한다 (좌표가 통째로 어긋난다)
    tight.check_map(tight.map_id)
    with pytest.raises(ValueError):
        tight.check_map('other_map')


@needs_map
def test_drivable_query_obb_uses_corners():
    """중심점만 보면 경계에서 들락거린다. 귀퉁이까지 봐야 한다."""
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from drivable_query import DrivableMask
    h5 = PKG_ROOT / 'out' / 'link_node_map.h5'
    if not h5.is_file():
        pytest.skip('구운 H5 가 없다')
    dm = DrivableMask.load(h5)

    # 도로 위 한 점을 찾아 중심으로 쓴다
    ys, xs = np.nonzero(dm.mask[::37, ::37])
    k = len(ys) // 2
    cx, cy = dm.world_offset + np.array([xs[k] * 37, ys[k] * 37]) / dm.pixels_per_meter
    assert dm.obb_overlaps(float(cx), float(cy), 4.6, 1.9, 0.0)
    assert dm.obb_inside_ratio(float(cx), float(cy), 4.6, 1.9, 0.0) > 0.5
    # 맵 밖은 걸치지 않는다
    assert not dm.obb_overlaps(1e6, 1e6, 4.6, 1.9, 0.0)


@needs_map
def test_drivable_query_handles_other_enu_origins():
    """라이다 쪽 ENU 원점이 다르거나 UTM 절대좌표로 들어와도 맞춰줘야 한다."""
    import sys as _sys
    _sys.path.insert(0, str(PKG_ROOT))
    from drivable_query import DrivableMask
    h5 = PKG_ROOT / 'out' / 'link_node_map.h5'
    if not h5.is_file():
        pytest.skip('구운 H5 가 없다')

    ref = DrivableMask.load(h5)
    rng = np.random.default_rng(7)
    pts = rng.uniform([-400, -770], [110, 560], size=(2000, 2))
    base = ref.contains(pts)
    assert base.any() and not base.all()
    assert ref.looks_like_same_frame(pts)

    utm_origin = ref.local_origin_in_global[:2]

    # 1) UTM 절대좌표로 들어오는 경우
    utm_pts = pts + utm_origin
    d = DrivableMask.load(h5)
    assert not d.looks_like_same_frame(utm_pts)      # 보정 전에는 맵 밖
    assert not d.contains(utm_pts).any()
    d.set_utm_input()
    assert d.looks_like_same_frame(utm_pts)
    assert np.array_equal(d.contains(utm_pts), base)

    # 2) 원점이 다른 ENU
    other = np.array([302700.0, 4124000.0])
    d2 = DrivableMask.load(h5)
    d2.set_enu_origin_utm(float(other[0]), float(other[1]))
    assert np.array_equal(d2.contains(pts + utm_origin - other), base)

    # 3) 원점을 위경도로 주는 경우
    import pyproj
    tf = pyproj.Transformer.from_crs('EPSG:32652', 'EPSG:4326', always_xy=True)
    lon, lat = tf.transform(other[0], other[1])
    d3 = DrivableMask.load(h5)
    d3.set_enu_origin_wgs84(float(lat), float(lon))
    got = d3.contains(pts + utm_origin - other)
    assert float((got == base).mean()) > 0.999
