"""Sparse attitude-only ground correction; network and static pixel gates stay fixed."""
import numpy as np
from .model_grid_projection import MetricEvidence
from asmc_camera.runtime.ground_projection import attitude_camera  # compatibility import


def correct_evidence(projector, evidence, corrected_camera):
    """Move selected nominal rays to the attitude plane; preserve confidences/quality.

    Sparse remap avoids rebuilding full-resolution lookup tables each frame.
    Nominal pixel selection is deliberately frozen; newly exposed pixels outside
    that gate are not recovered by this candidate. No past-frame accumulation.
    """
    if not len(evidence.xy) or np.array_equal(
            projector.camera.ground_to_image_homography, corrected_camera.ground_to_image_homography):
        return evidence
    transform = np.linalg.solve(corrected_camera.ground_to_image_homography,
                                projector.camera.ground_to_image_homography)
    points = np.column_stack((evidence.xy, np.ones(len(evidence.xy)))) @ transform.T
    denominator = points[:, 2]
    xy = np.full((len(points), 2), np.nan)
    safe = np.abs(denominator) > 1e-9
    xy[safe] = points[safe, :2] / denominator[safe, None]
    depth = corrected_camera.ground_to_optical(corrected_camera.ground_plane.xyz(xy))[:, 2]
    valid = (np.isfinite(xy).all(axis=1) & (depth > projector.min_camera_depth_m) &
             (np.linalg.norm(xy, axis=1) <= projector.max_ground_range_m) &
             (xy[:, 0] >= projector.x_bounds_m[0]) & (xy[:, 0] <= projector.x_bounds_m[1]) &
             (xy[:, 1] >= projector.y_bounds_m[0]) & (xy[:, 1] <= projector.y_bounds_m[1]))
    return MetricEvidence(np.ascontiguousarray(xy[valid], dtype=np.float32),
                          evidence.class_id[valid], evidence.confidence[valid],
                          evidence.geometric_quality[valid], evidence.source_mask[valid])
