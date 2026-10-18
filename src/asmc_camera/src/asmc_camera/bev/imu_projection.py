"""Sparse attitude-only ground correction; network and static pixel gates stay fixed."""
from dataclasses import replace
import numpy as np
from .calibration import GroundPlane, _rotation_x, _rotation_y, _rotation_z
from .model_grid_projection import MetricEvidence
from asmc_camera.runtime.sensor_timing import quaternion_rotation


def attitude_camera(camera, quaternion, imu_mount_rotation_deg=(0., 0., 0.), max_tilt_deg=8.):
    """Flat world normal in body, with fixed normal distance and no heave estimate.

    R_WB = R_WI R_BI.T. Camera extrinsics stay fixed: mount pitch is applied once.
    Yaw alone does not change a flat-world ground normal. No Euler subtraction.
    """
    roll, pitch, yaw = np.radians(np.asarray(imu_mount_rotation_deg, dtype=float))
    body_from_imu = _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
    world_from_body = quaternion_rotation(quaternion) @ body_from_imu.T
    normal = world_from_body.T @ np.array([0., 0., 1.])
    if normal[2] < np.cos(np.deg2rad(max_tilt_deg)):
        raise ValueError('attitude outside validated tilt range')
    height = -float(camera.ground_plane.z_at_origin_m)
    if height <= 0 or abs(camera.ground_plane.dz_dx) + abs(camera.ground_plane.dz_dy) > 1e-12:
        raise ValueError('attitude candidate requires the nominal flat-plane profile')
    plane = GroundPlane('base_link', -height/normal[2], -normal[0]/normal[2],
                        -normal[1]/normal[2], 'imu_source_time_flat_world')
    return replace(camera, ground_plane=plane)


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
