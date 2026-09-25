"""High-level LBC BEV renderer for MORAI KATRI maps."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .dynamic_layers import (
    PedestrianState,
    TrafficLightState,
    TrafficLightStoplineMapper,
    VehicleState,
    render_dynamic_layers,
)
from .map_baker import bake_maps, compute_world_bounds
from .spec import (
    PIXELS_AHEAD_VEHICLE,
    crop_birdview,
    crop_visualization_from_full,
    stack_birdview,
    visualize_birdview,
)
from .static_layers import extract_static_layers
from .ws_root import asmc_ws_root


@dataclass
class EgoState:
    x: float
    y: float
    yaw_deg: float


class LBCRenderer:
    def __init__(
        self,
        aim_ws_root: str | Path | None = None,
        *,
        road_json: Optional[Path] = None,
        lane_json: Optional[Path] = None,
        tl_json: Optional[Path] = None,
        link_json: Optional[Path] = None,
        pixels_ahead: int = PIXELS_AHEAD_VEHICLE,
    ):
        root = Path(aim_ws_root) if aim_ws_root is not None else asmc_ws_root()
        self.road_json = road_json or (
            root / 'mgeo_toolkit/data/KATRI/road_mesh_out_line.json'
        )
        self.lane_json = lane_json or (root / 'R_KR_PG_KATRI/lane_boundary_set.json')
        self.tl_json = tl_json or (root / 'R_KR_PG_KATRI/traffic_light_set.json')
        self.link_json = link_json or (root / 'R_KR_PG_KATRI/link_set.json')
        self.pixels_ahead = int(pixels_ahead)
        self._baked = bake_maps(self.road_json, self.lane_json)
        self._bounds = compute_world_bounds(self.road_json, self.lane_json)
        self._tl_mapper: Optional[TrafficLightStoplineMapper] = None

    def _ensure_tl_mapper(self) -> Optional[TrafficLightStoplineMapper]:
        if self._tl_mapper is None and self.tl_json.is_file():
            self._tl_mapper = TrafficLightStoplineMapper(
                self.tl_json, self.lane_json, self.link_json
            )
        return self._tl_mapper

    @property
    def map_bounds(self):
        return self._bounds

    @property
    def baked_maps(self):
        return self._baked

    def default_ego(self) -> EgoState:
        min_x, min_y, max_x, max_y = self._bounds
        return EgoState(x=0.5 * (min_x + max_x), y=0.5 * (min_y + max_y), yaw_deg=0.0)

    def render(
        self,
        ego_x: float,
        ego_y: float,
        yaw_deg: float,
        vehicles: Optional[Sequence[VehicleState]] = None,
        pedestrians: Optional[Sequence[PedestrianState]] = None,
        traffic_lights: Optional[Sequence[TrafficLightState]] = None,
        crop_dx: int = 0,
        crop_dy: int = 0,
        pixels_ahead: Optional[int] = None,
    ) -> Dict:
        vehicles = list(vehicles or [])
        pedestrians = list(pedestrians or [])
        traffic_lights = list(traffic_lights or [])
        ahead = self.pixels_ahead if pixels_ahead is None else int(pixels_ahead)
        road, lane, bev_affine = extract_static_layers(
            self._baked, ego_x, ego_y, yaw_deg, pixels_ahead=ahead
        )
        veh, ped, tl_r, tl_y, tl_g = render_dynamic_layers(
            bev_affine,
            self._baked,
            vehicles,
            pedestrians,
            traffic_lights,
        )
        birdview = stack_birdview(road, lane, tl_r, tl_y, tl_g, veh, ped)
        cropped = crop_birdview(birdview, dx=crop_dx, dy=crop_dy)
        vis_full = visualize_birdview(birdview)
        channels = {
            'road': road,
            'lane': lane,
            'traffic_red': tl_r,
            'traffic_yellow': tl_y,
            'traffic_green': tl_g,
            'vehicle': veh,
            'pedestrian': ped,
        }
        return {
            'birdview': birdview,
            'cropped': cropped,
            'channels': channels,
            'bev_affine': bev_affine,
            'pixels_ahead': ahead,
            'visualization': vis_full,
            'visualization_cropped': crop_visualization_from_full(
                vis_full, dx=crop_dx, dy=crop_dy
            ),
        }
