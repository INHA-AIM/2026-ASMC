"""월드 좌표 <-> 픽셀 좌표 규약과 래스터 캔버스.

좌표 매핑은 여기 한 곳에만 정의한다. CARLA/carla-roach 의 ``_world_to_pixel`` 과
동일하게 OpenCV (col, row) = (x, y) 순서를 쓴다::

    col = ppm * (wx - offset_x)
    row = ppm * (wy - offset_y)

레이어가 여러 개일 때 이 규약이 파일마다 흩어지면 축이 뒤바뀌는 버그가 난다.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class RasterGrid:
    """월드 AABB 를 픽셀 격자에 대응시키는 규약."""

    world_offset: np.ndarray  # (2,) float32 = (min_x, min_y)
    pixels_per_meter: float
    width_px: int
    height_px: int

    @property
    def width_m(self) -> float:
        return self.width_px / self.pixels_per_meter

    @property
    def height_m(self) -> float:
        return self.height_px / self.pixels_per_meter

    @classmethod
    def from_bounds(
        cls,
        min_x: float,
        min_y: float,
        max_x: float,
        max_y: float,
        pixels_per_meter: float,
        margin_m: float,
    ) -> 'RasterGrid':
        ox = float(min_x) - margin_m
        oy = float(min_y) - margin_m
        w_m = (float(max_x) + margin_m) - ox
        h_m = (float(max_y) + margin_m) - oy
        w_px = max(16, int(np.ceil(pixels_per_meter * w_m)))
        h_px = max(16, int(np.ceil(pixels_per_meter * h_m)))
        return cls(
            world_offset=np.array([ox, oy], dtype=np.float32),
            pixels_per_meter=float(pixels_per_meter),
            width_px=w_px,
            height_px=h_px,
        )

    def to_pixel(self, points_xy: np.ndarray) -> np.ndarray:
        """월드 (N, 2) -> 픽셀 (N, 2) int32, OpenCV (col, row)."""
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return np.zeros((0, 2), dtype=np.int32)
        out = self.pixels_per_meter * (pts - self.world_offset.astype(np.float64))
        return np.round(out).astype(np.int32)

    def to_pixel_f(self, points_xy: np.ndarray) -> np.ndarray:
        """to_pixel 의 반올림 없는 버전. 워핑 행렬처럼 서브픽셀이 필요한 곳에 쓴다.

        정수로 반올림하면 ppm=5 에서 0.2 m 단위로 튀어 저속 주행이 뚝뚝 끊긴다.
        """
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return np.zeros((0, 2), dtype=np.float64)
        return self.pixels_per_meter * (pts - self.world_offset.astype(np.float64))

    def to_world(self, points_cr: np.ndarray) -> np.ndarray:
        """픽셀 (N, 2) -> 월드 (N, 2) float32. 왕복 검증용."""
        pts = np.asarray(points_cr, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return np.zeros((0, 2), dtype=np.float32)
        return (pts / self.pixels_per_meter + self.world_offset).astype(np.float32)

    def meters_to_px(self, meters: float, minimum: int = 1) -> int:
        return max(minimum, int(round(meters * self.pixels_per_meter)))

    def blank(self) -> np.ndarray:
        return np.zeros((self.height_px, self.width_px), dtype=np.uint8)


def compute_bounds(point_arrays: Iterable[np.ndarray]) -> Tuple[float, float, float, float]:
    """여러 (N, 2) 배열을 감싸는 AABB."""
    mins: List[np.ndarray] = []
    maxs: List[np.ndarray] = []
    for arr in point_arrays:
        a = np.asarray(arr, dtype=np.float64).reshape(-1, 2)
        if a.size == 0:
            continue
        mins.append(a.min(axis=0))
        maxs.append(a.max(axis=0))
    if not mins:
        raise ValueError('좌표가 하나도 없습니다.')
    lo = np.min(np.stack(mins, axis=0), axis=0)
    hi = np.max(np.stack(maxs, axis=0), axis=0)
    return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])


def draw_polyline(
    canvas: np.ndarray,
    grid: RasterGrid,
    points_xy: np.ndarray,
    thickness_px: int,
    value: int = 255,
) -> None:
    px = grid.to_pixel(points_xy)
    if px.shape[0] < 2:
        return
    cv2.polylines(
        canvas,
        [px.reshape(-1, 1, 2)],
        isClosed=False,
        color=int(value),
        thickness=int(thickness_px),
        lineType=cv2.LINE_8,
    )


def draw_dashed_polyline(
    canvas: np.ndarray,
    grid: RasterGrid,
    points_xy: np.ndarray,
    thickness_px: int,
    dash_m: float,
    gap_m: float,
    value: int = 255,
) -> None:
    """등간격 리샘플된 폴리라인을 파선으로 그린다."""
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 2:
        return
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    period = dash_m + gap_m
    if period <= 0:
        draw_polyline(canvas, grid, pts, thickness_px, value)
        return
    on = (s % period) < dash_m
    start = None
    for i in range(len(on)):
        if on[i] and start is None:
            start = i
        elif not on[i] and start is not None:
            if i - start >= 2:
                draw_polyline(canvas, grid, pts[start:i], thickness_px, value)
            start = None
    if start is not None and len(on) - start >= 2:
        draw_polyline(canvas, grid, pts[start:], thickness_px, value)


def fill_thin_gaps(
    canvas: np.ndarray,
    pixels_per_meter: float,
    max_width_m: float,
    max_area_m2: float = 20.0,
) -> np.ndarray:
    """래스터화로 생긴 내부 틈만 메우고, 인도 섬 같은 실제 빈 공간은 남긴다.

    두 조건 중 하나라도 맞으면 메운다:

    - **가늘다** — 최대 내접원 반경(거리변환의 최대값)이 `max_width_m/2` 미만.
      인접 차로 통로 사이의 길고 좁은 틈이 여기 걸린다.
    - **아주 작다** — 면적이 `max_area_m2` 미만. 링크가 갈라지는 지점의 작은
      삼각형 틈은 반경이 1 m 를 살짝 넘기도 해서(실측 1.16 m) 폭 조건만으로는
      안 걸린다.

    면적 하나로만 거르면 안 된다. 처음에 50 m^2 임계를 썼더니 교차로 코너의 인도
    섬(48 / 50.4 / 56.8 / 57.6 m^2)이 임계값 양옆에 몰려, 같은 교차로의 네 모서리
    중 하나만 메워졌다. 실측상 20 m^2 미만 구멍 38개는 전부 내접반경 1.88 m 이하로
    인도 섬이 아니고, 남는 것 중 가장 작은 게 22.9 m^2 다.
    """
    if max_width_m <= 0 and max_area_m2 <= 0:
        return canvas
    bg = (canvas == 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(bg, connectivity=8)
    if count <= 1:
        return canvas
    border = np.unique(
        np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]])
    )
    on_border = np.zeros(count, dtype=bool)
    on_border[border] = True

    # 거리변환 최대값 = 그 구멍에 들어가는 가장 큰 원의 반경
    dist = cv2.distanceTransform(bg, cv2.DIST_L2, 5) / float(pixels_per_meter)
    max_radius = np.zeros(count, dtype=np.float64)
    np.maximum.at(max_radius, labels.ravel(), dist.ravel())

    area_m2 = stats[:, cv2.CC_STAT_AREA] / float(pixels_per_meter ** 2)
    fill = np.zeros(count, dtype=bool)
    fill[1:] = (~on_border[1:]) & (
        (max_radius[1:] < 0.5 * max_width_m) | (area_m2[1:] < max_area_m2)
    )
    if not fill.any():
        return canvas
    out = canvas.copy()
    out[fill[labels]] = 255
    return out


def shrink_outer(canvas: np.ndarray, pixels_per_meter: float, trim_m: float) -> np.ndarray:
    """도로 바깥 경계를 안쪽으로 trim_m 만큼 깎는다 (침식).

    차선은 차로 경계선 위에 두께를 갖고 그려지므로, 도로 최외곽에서는 선의 바깥
    절반이 도로 밖으로 나간다. 그 절반을 잘라내려면 도로도 같은 양만큼 줄여야
    둘의 바깥 끝이 맞는다.

    차로와 차로 사이는 도로가 이어져 있어 침식의 영향을 받지 않는다.
    """
    if trim_m <= 0:
        return canvas
    # 커널 반지름 = trim_m * ppm. 홀수 크기만 쓰면 반지름이 정수 픽셀로 갇혀
    # ppm=5 에서 0.2 m 단위밖에 못 깎는다 (0.3 m 요청이 0.4 m 로 반올림된다).
    # 짝수 크기를 허용하면 반 픽셀(=0.1 m) 단위가 된다.
    size = max(3, int(round(2.0 * trim_m * pixels_per_meter)) + 1)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.erode(canvas, k)


def close_gaps(canvas: np.ndarray, kernel_px: int) -> np.ndarray:
    """모폴로지 닫힘. 인접 차로 폴리라인 사이의 틈/계단 아티팩트를 메운다."""
    if kernel_px <= 1:
        return canvas
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    return cv2.morphologyEx(canvas, cv2.MORPH_CLOSE, k)
