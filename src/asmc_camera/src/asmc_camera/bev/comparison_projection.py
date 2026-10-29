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

    def geometry(self, planes):
        if set(planes) != set(self.cameras):
            raise ValueError("projection requires every configured camera plane")
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
        return projectors, quality

    def render(self, planes, rgb, semantics):
        """Return RGB and fused semantic layers using the SAME per-view map."""
        if set(planes) != set(self.cameras) or set(rgb) != set(planes) or set(semantics) != set(planes):
            raise ValueError('comparison requires all configured views')
        projectors, quality = self.geometry(planes)
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

    def drivable(self, planes, probabilities, masks, visibility_margin_px=1, include_front=False):
        """Semantic-only experiment using the exact family planes.

        No RGB, IMU lookup, JPEG, or repeated fitting. Ego pixels remove camera
        support, never fill it with free space. A strong overlapping view's
        non-free observation cannot be averaged away by another camera.
        Low probability behind an obstacle remains unknown/occupied; this
        raster does not claim to recover the obstacle's depth or length.
        """
        if not isinstance(visibility_margin_px,int) or not 1<=visibility_margin_px<=16:
            raise ValueError('invalid source visibility margin')
        if set(probabilities) != set(self.cameras) or set(masks) != set(self.cameras):
            raise ValueError('drivable requires all views and visibility masks')
        projectors, quality = self.geometry(planes)
        size = (self.grid.width_px, self.grid.height_px)
        values, weights, original_support = [], [], []
        for view, projector in projectors.items():
            camera = self.cameras[view]
            p = projector._validate_source(probabilities[view], 'drivable')
            mask = masks[view]
            if mask.dtype != np.bool_ or mask.shape != p.shape:
                raise ValueError('invalid ego visibility mask')
            # Bilinear projection must not borrow an excluded source neighbour.
            # Original-space resampling halo is excluded as UNKNOWN, never free.
            kernel=2*visibility_margin_px+1
            visible = cv2.erode((~mask).astype(np.uint8), np.ones((kernel,kernel), np.uint8))
            support = cv2.warpPerspective(visible, projector.homography.bev_from_image,
                size, flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            weight = quality[view] * (support > 0)
            values.append(cv2.warpPerspective(p, projector.homography.bev_from_image,
                size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT))
            weights.append(weight)
            original_support.append(quality[view] > 1e-6)
        weights = np.stack(weights)
        strongest = np.max(weights, axis=0)
        usable = (weights > 1e-6) & (weights >= .25*strongest)
        observed = np.any(usable, axis=0)
        fused = np.min(np.where(usable, np.stack(values), 255), axis=0).astype(np.uint8)
        fused[~observed] = 0
        #128 is specifically ego-occluded, not observed free.0 is outside
        # usable camera support. A semantic obstacle/shadow remains coverage255
        # with low probability and cannot use an ego-occlusion exception.
        coverage=np.where(observed,255,np.where(np.any(original_support,axis=0),128,0)).astype(np.uint8)
        result=dict(drivable=fused, coverage=coverage,
                    source_count=np.sum(usable, axis=0).astype(np.uint8))
        if include_front:
            # Reuse the SAME warp/plane/mask; no extra inference/projection.
            i=list(projectors).index('front')
            seen=weights[i]>1e-6
            result['front_drivable']=np.where(seen,values[i],0).astype(np.uint8)
            result['front_coverage']=np.where(seen,255,
                np.where(original_support[i],128,0)).astype(np.uint8)
        return result
