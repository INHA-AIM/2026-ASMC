"""One frame's geometry shared by RGB and semantic comparison panels.

Planes arrive from the family worker's existing IMU decision. This module
performs raster projection only: no IMU lookup, fitting, or second correction
of an already projected BEV. Original camera images are warped once.
"""
from dataclasses import replace

import cv2
import numpy as np

from .calibration import GroundPlane, calibration_id
from .fusion import CameraBevFusion, build_quality_maps
from .projector import CameraBevProjector
from .visualization import fuse_projected_rgb


class ComparisonProjection:
    def __init__(self, cameras, grid, projector_options=None, quality_options=None):
        self.cameras, self.grid = cameras, grid
        self.projector_options = dict(projector_options or {})
        self.quality_options = dict(quality_options or {})
        self.calibration_ids = {v: calibration_id(c) for v, c in cameras.items()}
        self.nominal = self._geometry(cameras)

    def _geometry(self, cameras):
        projectors = {v: CameraBevProjector(c, self.grid, **self.projector_options)
                      for v, c in cameras.items()}
        quality = build_quality_maps(projectors, self.grid, **self.quality_options)
        return projectors, quality

    def render(self, planes, rgb, semantics):
        """Return RGB and fused semantic layers using the SAME per-view map."""
        if set(planes) != set(self.cameras) or set(rgb) != set(planes) or set(semantics) != set(planes):
            raise ValueError('comparison requires all configured views')
        nominal = all(tuple(planes[v]) == c.ground_plane.coefficients
                      for v, c in self.cameras.items())
        if nominal:
            projectors, quality = self.nominal
        else:
            cameras = {v: replace(c, ground_plane=GroundPlane(
                frame_id='base_link', dz_dx=planes[v][0], dz_dy=planes[v][1],
                z_at_origin_m=planes[v][2], source='family_projection_context'))
                for v, c in self.cameras.items()}
            projectors, quality = self._geometry(cameras)
        size = (self.grid.width_px, self.grid.height_px)
        rgb_projected, semantic_projected = {}, {}
        for view, projector in projectors.items():
            image = rgb[view]
            camera = self.cameras[view]
            if image.dtype != np.uint8 or image.shape != (camera.height, camera.width, 3):
                raise ValueError('invalid original RGB shape/dtype for ' + view)
            rgb_projected[view] = cv2.warpPerspective(
                image, projector.homography.bev_from_image, size,
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            semantic_projected[view] = projector.project(*semantics[view])
        fused = CameraBevFusion(quality).fuse(semantic_projected)
        return fuse_projected_rgb(rgb_projected, quality), dict(
            drivable=fused.drivable_probability, road_marking=fused.road_marking_class_id,
            confidence=fused.road_marking_confidence, coverage=fused.coverage,
            source_count=fused.source_count)
