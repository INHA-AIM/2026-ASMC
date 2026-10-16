"""정적 장애물 구간 주변 맵: 전역경로 좌표계, 차선, 주행가능영역, 거리 지도."""
from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
from matplotlib.path import Path as MplPath
from scipy import ndimage
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree

from scen_runner.config import load_yaml, repo_path

RESAMPLE_STEP_M = 0.05


class RouteFrame:
    """전역경로를 따라가는 (s, d) 좌표계. d 는 진행방향 왼쪽이 +."""

    def __init__(self, path_xyz: np.ndarray, s_range: tuple[float, float],
                 margin_m: float = 12.0):
        seg = np.hypot(np.diff(path_xyz[:, 0]), np.diff(path_xyz[:, 1]))
        station = np.r_[0.0, np.cumsum(seg)]
        keep = np.r_[True, seg > 1e-6]
        path_xyz, station = path_xyz[keep], station[keep]

        s0 = max(station[0], s_range[0] - margin_m)
        s1 = min(station[-1], s_range[1] + margin_m)
        self.s = np.arange(s0, s1, RESAMPLE_STEP_M)
        x = np.interp(self.s, station, path_xyz[:, 0])
        y = np.interp(self.s, station, path_xyz[:, 1])
        self.z = np.interp(self.s, station, path_xyz[:, 2])
        # 0.5 m 간격 원본의 꺾임이 곡률로 튀지 않도록 약 2 m 창으로 다듬는다.
        win = 41
        self.x = savgol_filter(x, win, 3)
        self.y = savgol_filter(y, win, 3)
        heading = np.unwrap(np.arctan2(np.gradient(self.y), np.gradient(self.x)))
        self.heading = savgol_filter(heading, win, 2)
        self.kappa = savgol_filter(np.gradient(self.heading, self.s), win, 2)
        self._tree = cKDTree(np.c_[self.x, self.y])

    @property
    def s_min(self) -> float:
        return float(self.s[0])

    @property
    def s_max(self) -> float:
        return float(self.s[-1])

    def sample(self, s):
        """경로 위 점의 x, y, heading, 곡률."""
        s = np.asarray(s, dtype=float)
        return (np.interp(s, self.s, self.x), np.interp(s, self.s, self.y),
                np.interp(s, self.s, self.heading), np.interp(s, self.s, self.kappa))

    def z_at(self, s):
        return np.interp(np.asarray(s, dtype=float), self.s, self.z)

    def to_xy(self, s, d):
        x, y, th, _ = self.sample(s)
        d = np.asarray(d, dtype=float)
        return x - d * np.sin(th), y + d * np.cos(th)

    def project(self, x, y):
        """점의 (s, d). 경로 창 밖이면 가장 가까운 끝점 기준."""
        pts = np.c_[np.ravel(x), np.ravel(y)]
        _, idx = self._tree.query(pts)
        th = self.heading[idx]
        dx, dy = pts[:, 0] - self.x[idx], pts[:, 1] - self.y[idx]
        along = dx * np.cos(th) + dy * np.sin(th)
        s = self.s[idx] + along
        d = -dx * np.sin(th) + dy * np.cos(th)
        shape = np.shape(x)
        return s.reshape(shape), d.reshape(shape)


@dataclass
class Marking:
    marking_id: str
    feature_type: str
    color: str
    pattern: str
    points: np.ndarray
    forbidden: bool


@dataclass
class DistanceGrid:
    """경로 주변 격자. 칸마다 실선까지, 주행가능영역 밖까지의 거리(m)."""

    x0: float
    y0: float
    res: float
    drivable: np.ndarray
    edge_dist: np.ndarray
    line_dist: np.ndarray

    def _index(self, x, y):
        ix = np.floor((np.asarray(x) - self.x0) / self.res).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.y0) / self.res).astype(np.int64)
        h, w = self.drivable.shape
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        return np.clip(ix, 0, w - 1), np.clip(iy, 0, h - 1), inside

    def edge_distance(self, x, y):
        ix, iy, inside = self._index(x, y)
        return np.where(inside, self.edge_dist[iy, ix], 0.0)

    def line_distance(self, x, y):
        ix, iy, inside = self._index(x, y)
        return np.where(inside, self.line_dist[iy, ix], np.inf)

    def distances(self, x, y):
        """Same edge/line distances with one coordinate-to-grid conversion."""
        ix, iy, inside = self._index(x, y)
        return (np.where(inside, self.edge_dist[iy, ix], 0.0),
                np.where(inside, self.line_dist[iy, ix], np.inf))

    def is_drivable(self, x, y):
        ix, iy, inside = self._index(x, y)
        return inside & self.drivable[iy, ix]

    @property
    def extent(self):
        h, w = self.drivable.shape
        return (self.x0, self.x0 + w * self.res, self.y0, self.y0 + h * self.res)


@dataclass
class StaticZoneScene:
    route: RouteFrame
    zone_s: tuple[float, float]
    stopline_s: float
    checkpoint_xy: tuple[float, float]
    checkpoint_s: float
    checkpoint_radius: float
    markings: list[Marking]
    drivable_polygons: list[np.ndarray]
    grid: DistanceGrid
    view_s: tuple[float, float]
    sources: dict = field(default_factory=dict)
    reference_label: str = "정지선"


def _line_rule_match(rule: dict, feature_type: str, attrs: dict) -> bool:
    for key, value in rule.items():
        actual = feature_type if key == "feature_type" else attrs.get(key)
        if actual != value:
            return False
    return True


def _polyline_xy(geometry) -> np.ndarray:
    coords = geometry.get("coordinates") if isinstance(geometry, dict) else geometry
    if coords and isinstance(coords[0][0], (list, tuple)):
        coords = coords[0]
    return np.asarray([[c[0], c[1]] for c in coords], dtype=float)


def _densify(points: np.ndarray, step: float) -> np.ndarray:
    out = [points[:1]]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)) / step)))
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        out.append(a + (b - a) * t)
    return np.vstack(out)


def _bbox_overlap(points: np.ndarray, bbox) -> bool:
    xmin, xmax, ymin, ymax = bbox
    return bool(np.any((points[:, 0] >= xmin) & (points[:, 0] <= xmax)
                       & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)))


def load_scene(cfg: dict) -> StaticZoneScene:
    mcfg, zcfg = cfg["map"], cfg["zone"]
    meta = load_yaml(repo_path(mcfg["competition_metadata"]))
    zones = {z["id"]: z for z in meta["mission_zones"]}
    zone = zones[zcfg["mission_zone_id"]]
    stopline_s = float(zones[zcfg["stopline_zone_id"]]["route_station_start_m"])
    cp = next(c for c in meta["checkpoints"] if str(c["id"]) == str(zcfg["checkpoint_id"]))

    view_s = tuple(float(v) for v in zcfg["view_s_range"])
    path_xyz = np.loadtxt(repo_path(mcfg["global_path"]))[:, :3]
    route = RouteFrame(path_xyz, view_s)
    link_range = zcfg.get("evaluation_s_range")
    if link_range:
        stopline_s = float(link_range[0])
        end_s = float(link_range[1])
        end_x, end_y = route.to_xy(end_s, 0.0)
        cp = {"pose": {"position": {"x": float(end_x), "y": float(end_y)}},
              "route_station_m": end_s, "pass_radius_m": 0.0}

    # 계산 격자: 경로 창 + 좌우 여유
    pad = float(zcfg["grid_padding_m"])
    m = (route.s >= view_s[0]) & (route.s <= view_s[1])
    bbox = (route.x[m].min() - pad, route.x[m].max() + pad,
            route.y[m].min() - pad, route.y[m].max() + pad)
    res = float(zcfg["grid_resolution_m"])
    w = int(np.ceil((bbox[1] - bbox[0]) / res))
    h = int(np.ceil((bbox[3] - bbox[2]) / res))
    gx = bbox[0] + (np.arange(w) + 0.5) * res
    gy = bbox[2] + (np.arange(h) + 0.5) * res

    # 주행가능영역
    drivable_map = json.loads(repo_path(mcfg["drivable_map"]).read_text(encoding="utf-8"))
    polygons = []
    drivable = np.zeros((h, w), dtype=bool)
    for area in drivable_map["drivable_areas"]:
        poly = np.asarray(area["polygon"], dtype=float)[:, :2]
        if not _bbox_overlap(poly, bbox):
            continue
        polygons.append(poly)
        x_lo, x_hi = np.searchsorted(gx, [poly[:, 0].min(), poly[:, 0].max()])
        y_lo, y_hi = np.searchsorted(gy, [poly[:, 1].min(), poly[:, 1].max()])
        if x_hi <= x_lo or y_hi <= y_lo:
            continue
        xx, yy = np.meshgrid(gx[x_lo:x_hi], gy[y_lo:y_hi])
        inside = MplPath(poly).contains_points(np.c_[xx.ravel(), yy.ravel()])
        drivable[y_lo:y_hi, x_lo:x_hi] |= inside.reshape(xx.shape)
    # 인접 polygon 사이 0.1~0.2 m 이음매가 연석처럼 보이지 않게 메운다.
    drivable = ndimage.binary_closing(drivable, iterations=3) | drivable

    # 차선 도색
    marking_doc = json.loads(repo_path(mcfg["lane_markings"]).read_text(encoding="utf-8"))
    rules = cfg.get("forbidden_lines", [])
    markings = []
    line_center = np.zeros((h, w), dtype=bool)
    for feat in marking_doc["features"]:
        if feat.get("feature_type") not in ("lane_marking", "stopline"):
            continue
        pts = _polyline_xy(feat["geometry"])
        if len(pts) < 2 or not _bbox_overlap(pts, bbox):
            continue
        attrs = feat.get("attributes", {})
        forbidden = any(_line_rule_match(r, feat["feature_type"], attrs) for r in rules)
        markings.append(Marking(
            marking_id=feat["id"].split(":")[-2] if ":" in feat["id"] else feat["id"],
            feature_type=feat["feature_type"], color=attrs.get("color", ""),
            pattern=attrs.get("pattern", ""), points=pts, forbidden=forbidden))
        if forbidden:
            dense = _densify(pts, res * 0.4)
            ix = np.floor((dense[:, 0] - bbox[0]) / res).astype(int)
            iy = np.floor((dense[:, 1] - bbox[2]) / res).astype(int)
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            line_center[iy[ok], ix[ok]] = True

    edge_dist = ndimage.distance_transform_edt(drivable) * res
    if line_center.any():
        # 도색 폭 0.15 m 의 절반을 빼서 "도색 가장자리까지 거리"로 만든다.
        line_dist = ndimage.distance_transform_edt(~line_center) * res - 0.075
    else:
        line_dist = np.full((h, w), np.inf)

    grid = DistanceGrid(x0=bbox[0], y0=bbox[2], res=res, drivable=drivable,
                        edge_dist=edge_dist, line_dist=line_dist)
    return StaticZoneScene(
        route=route,
        zone_s=tuple(map(float, link_range)) if link_range else
               (float(zone["route_station_start_m"]), float(zone["route_station_end_m"])),
        stopline_s=stopline_s,
        checkpoint_xy=(float(cp["pose"]["position"]["x"]), float(cp["pose"]["position"]["y"])),
        checkpoint_s=float(cp["route_station_m"]),
        checkpoint_radius=float(cp["pass_radius_m"]),
        markings=markings,
        drivable_polygons=polygons,
        grid=grid,
        view_s=view_s,
        sources={k: str(repo_path(v)) for k, v in mcfg.items()},
        reference_label="구간 시작" if link_range else "정지선",
    )


@lru_cache(maxsize=3)
def _scene_template(settings, signature):
    # signature participates in the cache key; load_scene reads those files.
    scene = load_scene(json.loads(settings))
    for array in (scene.route.s, scene.route.x, scene.route.y, scene.route.z,
                  scene.route.heading, scene.route.kappa, scene.route._tree.data,
                  scene.grid.drivable, scene.grid.edge_dist, scene.grid.line_dist,
                  *(m.points for m in scene.markings), *scene.drivable_polygons):
        array.setflags(write=False)
    return scene


def reusable_scene(cfg):
    """Reuse map geometry only; actor positions and feasibility results stay fresh."""
    geometry = dict(map=cfg["map"], zone=cfg["zone"],
                    forbidden_lines=cfg.get("forbidden_lines", []))
    settings = json.dumps(geometry, sort_keys=True)
    signature = []
    for name in ("competition_metadata", "global_path", "drivable_map", "lane_markings"):
        path = repo_path(cfg["map"][name]).resolve()
        stat = path.stat()
        signature.append((str(path), stat.st_dev, stat.st_ino, stat.st_size,
                          stat.st_mtime_ns, stat.st_ctime_ns))
    template = _scene_template(settings, tuple(signature))
    # Writable wrappers are private to each consumer. The expensive arrays
    # and nearest-route index are shared read-only and bounded to three maps.
    scene = copy.copy(template)
    scene.route = copy.copy(template.route)
    scene.grid = copy.copy(template.grid)
    scene.markings = [copy.copy(m) for m in template.markings]
    scene.drivable_polygons = list(template.drivable_polygons)
    scene.sources = dict(template.sources)
    return scene
