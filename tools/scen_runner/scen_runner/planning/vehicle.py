"""차량 외형과 장애물 상자."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class VehicleSpec:
    length: float
    width: float
    wheelbase: float
    front_overhang: float
    rear_overhang: float
    max_wheel_angle_deg: float
    circle_count: int = 8

    @classmethod
    def from_config(cls, cfg: dict) -> "VehicleSpec":
        v = cfg["vehicle"]
        return cls(length=float(v["length"]), width=float(v["width"]),
                   wheelbase=float(v["wheelbase"]),
                   front_overhang=float(v["front_overhang"]),
                   rear_overhang=float(v["rear_overhang"]),
                   max_wheel_angle_deg=float(v["max_wheel_angle_deg"]))

    @property
    def front_extent(self) -> float:
        """뒷바퀴 축에서 앞 범퍼까지."""
        return self.length - self.rear_overhang

    @property
    def circle_offsets(self) -> np.ndarray:
        """차체를 덮는 원들의 중심(뒷바퀴 축 기준 앞쪽 거리)."""
        n = self.circle_count
        return -self.rear_overhang + (np.arange(n) + 0.5) * self.length / n

    @property
    def circle_radius(self) -> float:
        return math.hypot(0.5 * self.width, 0.5 * self.length / self.circle_count)

    @property
    def kinematic_max_curvature(self) -> float:
        return math.tan(math.radians(self.max_wheel_angle_deg)) / self.wheelbase

    def circle_centers(self, x, y, heading):
        """포즈 배열 (N,) -> 원 중심 (N, C) x, y."""
        off = self.circle_offsets[None, :]
        x = np.asarray(x)[:, None]
        y = np.asarray(y)[:, None]
        th = np.asarray(heading)[:, None]
        return x + off * np.cos(th), y + off * np.sin(th)

    def footprint(self, x: float, y: float, heading: float) -> np.ndarray:
        """뒷바퀴 축 포즈의 차체 사각형 꼭짓점 (4, 2)."""
        local = np.array([[-self.rear_overhang, -0.5 * self.width],
                          [self.front_extent, -0.5 * self.width],
                          [self.front_extent, 0.5 * self.width],
                          [-self.rear_overhang, 0.5 * self.width]])
        c, s = math.cos(heading), math.sin(heading)
        return local @ np.array([[c, s], [-s, c]]) + np.array([x, y])


@dataclass
class Obstacle:
    """지면 위 3D 상자. (x, y) 는 상자 중심, yaw 는 진행방향(rad)."""

    x: float
    y: float
    yaw: float
    length: float
    width: float
    height: float
    model: str = "custom_box"
    morai_model: str | None = None
    z: float = 0.0
    source: str = "gui"
    actor_id: str | None = None
    tag: str = ""
    extra: dict = field(default_factory=dict)

    def corners(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hl, hw = 0.5 * self.length, 0.5 * self.width
        local = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        return local @ np.array([[c, s], [-s, c]]) + np.array([self.x, self.y])

    def signed_distance(self, px, py):
        """점에서 상자까지 거리. 안쪽이면 음수."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(px) - self.x, np.asarray(py) - self.y
        u = np.abs(dx * c + dy * s) - 0.5 * self.length
        v = np.abs(-dx * s + dy * c) - 0.5 * self.width
        outside = np.hypot(np.maximum(u, 0.0), np.maximum(v, 0.0))
        return outside + np.minimum(np.maximum(u, v), 0.0)

    def to_dict(self) -> dict:
        return {"model": self.model, "morai_model": self.morai_model,
                "x": self.x, "y": self.y, "z": self.z,
                "yaw_deg": math.degrees(self.yaw), "length": self.length,
                "width": self.width, "height": self.height, "source": self.source,
                "actor_id": self.actor_id, "tag": self.tag}

    @classmethod
    def from_dict(cls, d: dict) -> "Obstacle":
        return cls(x=float(d["x"]), y=float(d["y"]), yaw=math.radians(float(d["yaw_deg"])),
                   length=float(d["length"]), width=float(d["width"]),
                   height=float(d.get("height", 1.0)), model=d.get("model", "custom_box"),
                   morai_model=d.get("morai_model"), z=float(d.get("z", 0.0)),
                   source=d.get("source", "file"), actor_id=d.get("actor_id"),
                   tag=d.get("tag", ""))
