"""
MORAI ego vehicle: Ioniq 5 (rear-axle origin).

/Ego_topic position = 후륜 차축 중심 (rear axle center).
LBC warp anchor pixel (160, 260) maps to:
  world point = rear_axle + (pixels_ahead / PIXELS_PER_METER) * forward
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .spec import PIXELS_PER_METER, PIXELS_AHEAD_VEHICLE


@dataclass(frozen=True)
class MoraiIoniq5Spec:
    """Exterior dimensions from MORAI Ioniq 5 model."""

    length_m: float = 4.635
    width_m: float = 1.892
    height_m: float = 2.434
    wheelbase_m: float = 3.000
    front_overhang_m: float = 0.845
    rear_overhang_m: float = 0.700
    min_turning_radius_m: float = 5.87
    max_wheel_angle_deg: float = 40.0

    @property
    def front_axle_from_rear_axle_m(self) -> float:
        return self.wheelbase_m

    @property
    def front_bumper_from_rear_axle_m(self) -> float:
        return self.wheelbase_m + self.front_overhang_m

    @property
    def geometric_center_from_rear_axle_m(self) -> float:
        front_extent = self.wheelbase_m + self.front_overhang_m
        rear_extent = self.rear_overhang_m
        return 0.5 * (front_extent - rear_extent)

    @property
    def rear_axle_to_bbox_center_m(self) -> float:
        return self.length_m * 0.5 - self.rear_overhang_m

    def footprint_polygon_world(
        self,
        rear_axle_x: float,
        rear_axle_y: float,
        yaw_deg: float,
    ) -> np.ndarray:
        """
        Vehicle rectangle in world ENU (m), rear-axle origin.
        Order: front-left, front-right, rear-right, rear-left (CCW, forward = +f).
        """
        yaw = math.radians(yaw_deg)
        f = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
        r = np.array([math.cos(yaw - 0.5 * math.pi), math.sin(yaw - 0.5 * math.pi)], dtype=np.float32)
        c = np.array([rear_axle_x, rear_axle_y], dtype=np.float32)
        back = self.rear_overhang_m
        front = self.front_bumper_from_rear_axle_m
        hw = 0.5 * self.width_m
        fl = c + front * f + hw * r
        fr = c + front * f - hw * r
        rr = c - back * f - hw * r
        rl = c - back * f + hw * r
        return np.stack([fl, fr, rr, rl], axis=0).astype(np.float32)

    def pixels_ahead(
        self,
        target: str = "rear_axle",
        ppm: int = PIXELS_PER_METER,
    ) -> int:
        if target == "rear_axle":
            return 0
        if target == "geometric_center":
            return int(round(self.geometric_center_from_rear_axle_m * ppm))
        if target == "front_axle":
            return int(round(self.front_axle_from_rear_axle_m * ppm))
        if target == "lbc_official":
            return PIXELS_AHEAD_VEHICLE
        raise ValueError(f"unknown pixels_ahead target: {target}")


IONIQ5 = MoraiIoniq5Spec()

MORAI_EGO_PIXELS_AHEAD = IONIQ5.pixels_ahead("rear_axle")
LBC_TRAINING_PIXELS_AHEAD = IONIQ5.pixels_ahead("lbc_official")

# BGR for OpenCV overlays on RGB visualization (converted in visualizer)
EGO_FOOTPRINT_FILL_BGR = (220, 180, 80)
EGO_FOOTPRINT_OUTLINE_BGR = (255, 255, 255)
EGO_REAR_AXLE_BGR = (255, 255, 255)
