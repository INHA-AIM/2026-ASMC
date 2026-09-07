#!/usr/bin/env python3
"""구운 주행가능영역을 다른 파트(라이다 등)에서 조회하는 최소 모듈.

의존성은 numpy / h5py / opencv 뿐이고, 이 저장소의 다른 코드를 import 하지 않는다.
파일 하나와 H5 하나만 넘기면 된다.

    from drivable_query import DrivableMask

    dm = DrivableMask.load('link_node_map.h5', margin_m=1.0)
    dm.check_map('waize_katri_2025-10-23')          # 맵 판본 확인 (권장)

    keep = dm.contains(points_xy)                    # (N,2) 맵 로컬 좌표 -> (N,) bool
    on   = dm.obb_overlaps(cx, cy, length, width, yaw_deg)

좌표계: 구운 맵은 **원점이 UTM52N (302595.0, 4124145.0) 인 ENU** 다 (E=+x, N=+y).
라이다 점이 이미 같은 원점의 ENU 라면 변환이 필요 없다 — 그대로 넣으면 된다.
원점이 다르면 한 번만 알려주면 이후 그 좌표계로 넣을 수 있다::

    dm.set_enu_origin_utm(302595.0, 4124145.0)   # 원점을 UTM 으로
    dm.set_enu_origin_wgs84(37.242949, 126.774419)
    dm.set_utm_input()                            # UTM 절대좌표를 그대로 넣을 때

원점이 맞는지 헷갈리면 `looks_like_same_frame(points)` 로 확인한다.
조회가 전부 False 로 나오면 십중팔구 원점 문제다.

주의 (실측 기준):
  - 2D 마스크다. 높이는 보지 않고 x, y 만으로 판정한다.
  - 판정은 **측위 정확도에 종속**된다. 도로 가장자리 객체는 위치가 0.2 m 만 틀려도
    밖으로 나간다. margin_m 으로 여유를 줄 것.
  - 실측 노면이 아니라 링크 중심선 x 차로폭에서 유도한 근사치다. 갓길이 없고,
    도로 폭을 실제에 맞추려 좌우 0.3 m 를 깎아둔 상태라 경계가 빡빡하다.
  - **객체를 버리는 하드 게이트로 쓰지 말 것.** 계산 범위를 좁히는 용도로 쓰고,
    최종 판단은 뒷단에 남겨두는 것이 안전하다.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import h5py
import numpy as np


@dataclass
class DrivableMask:
    mask: np.ndarray              # (H, W) uint8, 0/255
    pixels_per_meter: float
    world_offset: np.ndarray      # (2,) 맵 로컬 좌표 원점 (m)
    map_id: str
    local_origin_in_global: np.ndarray   # (3,) UTM 원점
    margin_m: float = 0.0
    # 들어오는 점 좌표에 더해 맵 로컬로 맞추는 보정값. ENU 원점이 같으면 0 이다.
    frame_offset: np.ndarray = None

    @classmethod
    def load(cls, h5_path, margin_m: float = 0.0, layer: str = 'road') -> 'DrivableMask':
        """H5 를 읽어 조회기를 만든다. margin_m > 0 이면 그만큼 팽창해 둔다."""
        with h5py.File(str(h5_path), 'r') as hf:
            mask = np.asarray(hf[layer], dtype=np.uint8)
            ppm = float(hf.attrs['pixels_per_meter'])
            off = np.asarray(hf.attrs['world_offset_in_meters'], dtype=np.float64)
            map_id = str(hf.attrs.get('map_id', ''))
            origin = np.asarray(hf.attrs.get('local_origin_in_global', [0, 0, 0]),
                                dtype=np.float64)
        if margin_m > 0:
            # 커널 크기를 반 픽셀 단위로 잡는다 (홀수만 쓰면 정수 픽셀에 갇힌다)
            size = max(3, int(round(2.0 * margin_m * ppm)) + 1)
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                              (size, size)))
        return cls(mask=mask, pixels_per_meter=ppm, world_offset=off,
                   map_id=map_id, local_origin_in_global=origin, margin_m=margin_m,
                   frame_offset=np.zeros(2))

    # ------------------------------------------------------------------ 좌표계
    def set_enu_origin_utm(self, easting: float, northing: float) -> None:
        """상대 좌표계의 원점을 UTM52N 으로 알려준다.

        구운 맵은 원점이 UTM52N (302595.0, 4124145.0) 인 ENU 다. 라이다 쪽 ENU 가
        같은 원점이면 이 호출이 필요 없다. 원점이 다르면 여기서 한 번 맞춰두면
        이후 contains() 에 그 좌표계 값을 그대로 넣을 수 있다.
        """
        self.frame_offset = (np.array([easting, northing], dtype=np.float64)
                             - self.local_origin_in_global[:2])

    def set_enu_origin_wgs84(self, lat: float, lon: float,
                             utm_epsg: int = 32652) -> None:
        """상대 좌표계의 원점을 위경도로 알려준다."""
        import pyproj
        tf = pyproj.Transformer.from_crs('EPSG:4326', f'EPSG:{utm_epsg}',
                                         always_xy=True)
        e, n = tf.transform(lon, lat)
        self.set_enu_origin_utm(float(e), float(n))

    def set_utm_input(self) -> None:
        """UTM 절대좌표(easting, northing)를 그대로 넣을 때."""
        self.frame_offset = -self.local_origin_in_global[:2].copy()

    def looks_like_same_frame(self, points_xy: np.ndarray) -> bool:
        """넣은 좌표가 맵 범위 안인지로 원점이 맞는지 대략 확인한다.

        원점이 다르면 값이 맵 밖으로 몰린다. 조회 결과가 전부 False 로 나올 때
        먼저 이걸 확인할 것.
        """
        h, w = self.mask.shape
        lo = self.world_offset
        hi = lo + np.array([w, h]) / self.pixels_per_meter
        pts = self._to_local(points_xy)
        inside = ((pts >= lo) & (pts <= hi)).all(axis=1)
        return bool(inside.mean() > 0.5)

    def _to_local(self, points_xy: np.ndarray) -> np.ndarray:
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if self.frame_offset is None:
            return pts
        return pts + self.frame_offset

    def check_map(self, expected_map_id: str) -> None:
        """맵 판본 확인. 다른 판본이면 좌표가 통째로 어긋나므로 반드시 확인할 것."""
        if self.map_id != expected_map_id:
            raise ValueError(
                f'맵 판본 불일치: H5 는 {self.map_id!r}, 기대값은 {expected_map_id!r}. '
                f'좌표계가 다르면 조회 결과 전체가 틀린다.')

    def to_pixel(self, points_xy: np.ndarray) -> np.ndarray:
        """맵 로컬 (N,2) m -> 픽셀 (N,2) int32 (col, row)."""
        pts = self._to_local(points_xy)
        return np.rint(self.pixels_per_meter * (pts - self.world_offset)).astype(np.int32)

    def contains(self, points_xy: np.ndarray) -> np.ndarray:
        """맵 로컬 (N,2) m -> (N,) bool. 맵 범위를 벗어난 점은 False."""
        px = self.to_pixel(points_xy)
        h, w = self.mask.shape
        ok = ((px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h))
        out = np.zeros(px.shape[0], dtype=bool)
        if ok.any():
            out[ok] = self.mask[px[ok, 1], px[ok, 0]] > 0
        return out

    def obb_corners(self, cx: float, cy: float, length_m: float, width_m: float,
                    yaw_deg: float) -> np.ndarray:
        """OBB 중심 + 네 귀퉁이 (5, 2)."""
        yaw = math.radians(yaw_deg)
        f = np.array([math.cos(yaw), math.sin(yaw)])
        r = np.array([f[1], -f[0]])
        hl, hw = 0.5 * max(length_m, 1e-3), 0.5 * max(width_m, 1e-3)
        c = np.array([cx, cy], dtype=np.float64)
        return np.stack([c,
                         c + hl * f + hw * r, c + hl * f - hw * r,
                         c - hl * f + hw * r, c - hl * f - hw * r])

    def obb_overlaps(self, cx: float, cy: float, length_m: float, width_m: float,
                     yaw_deg: float) -> bool:
        """OBB 가 주행가능영역에 걸치는가.

        중심점 하나만 보면 경계에서 들락거리므로 중심 + 네 귀퉁이를 함께 본다.
        하나라도 안이면 True (놓치는 쪽보다 남기는 쪽으로 기울인다).
        """
        return bool(self.contains(self.obb_corners(cx, cy, length_m, width_m,
                                                   yaw_deg)).any())

    def obb_inside_ratio(self, cx: float, cy: float, length_m: float, width_m: float,
                         yaw_deg: float, samples: int = 5) -> float:
        """OBB 를 격자로 훑어 주행가능영역에 든 비율. 이진 판정 대신 점수로 쓸 때."""
        yaw = math.radians(yaw_deg)
        f = np.array([math.cos(yaw), math.sin(yaw)])
        r = np.array([f[1], -f[0]])
        u = np.linspace(-0.5, 0.5, samples) * max(length_m, 1e-3)
        v = np.linspace(-0.5, 0.5, samples) * max(width_m, 1e-3)
        uu, vv = np.meshgrid(u, v)
        pts = (np.array([cx, cy]) + uu.ravel()[:, None] * f + vv.ravel()[:, None] * r)
        return float(self.contains(pts).mean())

    def local_from_wgs84(self, lat, lon, utm_epsg: int = 32652) -> np.ndarray:
        """위경도 -> 맵 로컬 미터. GPS 로 들어올 때만 필요하다."""
        import pyproj
        tf = pyproj.Transformer.from_crs('EPSG:4326', f'EPSG:{utm_epsg}',
                                         always_xy=True)
        e, n = tf.transform(np.asarray(lon), np.asarray(lat))
        return np.stack([np.asarray(e) - self.local_origin_in_global[0],
                         np.asarray(n) - self.local_origin_in_global[1]], axis=-1)


def _demo() -> None:
    import time
    here = Path(__file__).resolve().parent
    dm = DrivableMask.load(here / 'out' / 'link_node_map.h5', margin_m=1.0)
    print(f'map_id={dm.map_id}  ppm={dm.pixels_per_meter}  '
          f'margin={dm.margin_m} m  shape={dm.mask.shape}')
    rng = np.random.default_rng(0)
    pts = rng.uniform([-400, -770], [110, 560], size=(200_000, 2))
    dm.contains(pts)
    t = time.perf_counter()
    for _ in range(10):
        keep = dm.contains(pts)
    print(f'20만 점 조회 {((time.perf_counter() - t) / 10) * 1000:.2f} ms, '
          f'주행가능영역 안 {100 * keep.mean():.1f}%')
    print('OBB 예:', dm.obb_overlaps(-31.99, -249.29, 4.6, 1.9, 64.2),
          f"(내부 비율 {dm.obb_inside_ratio(-31.99, -249.29, 4.6, 1.9, 64.2):.2f})")


if __name__ == '__main__':
    _demo()
