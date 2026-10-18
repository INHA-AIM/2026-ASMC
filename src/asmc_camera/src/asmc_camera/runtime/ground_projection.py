"""Frame-time camera geometry selection, independent of inference and lane fitting.

Callers supply original image headers and bridge timing records. Network pixels
stay unchanged. The selected plane is used once when projecting pixels/metric
geometry; consumers of a published selection must not query IMU a second time.
"""
from dataclasses import dataclass, replace
from types import MappingProxyType
from typing import Mapping, Optional
import time

import numpy as np
from asmc_camera.bev.calibration import CameraCalibration, GroundPlane, _rotation_x, _rotation_y, _rotation_z
from asmc_camera.runtime.sensor_timing import quaternion_rotation


def attitude_camera(camera, quaternion, imu_mount_rotation_deg=(0., 0., 0.), max_tilt_deg=8.):
    """Flat world normal in body, with fixed normal distance and no heave estimate.

    R_WB = R_WI R_BI.T. Camera extrinsics stay fixed: mount pitch is applied once.
    Yaw alone does not change a flat-world ground normal. No Euler subtraction.
    """
    if not 0 < max_tilt_deg <= 8.:
        raise ValueError('validated tilt limit must be in (0,8] degrees')
    mount = np.asarray(imu_mount_rotation_deg, dtype=float)
    if mount.shape != (3,) or not np.isfinite(mount).all():
        raise ValueError('invalid IMU mount rotation')
    roll, pitch, yaw = np.radians(mount)
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


@dataclass(frozen=True)
class ProjectionDecision:
    cameras: Mapping[str, CameraCalibration]
    details: Mapping[str, Mapping[str, object]]
    reason: str
    source_skew_ms: Optional[float]


def select_ground_projection(cameras, image_headers, sensor_timings, timeline, config):
    """All supplied views use their image-time attitude, or all remain nominal.

    No ROS subscriptions, model-grid classes, family parameters or mutable node
    state are owned here. YOLO ground contacts, semantic projection and fitting
    can share this selection policy without depending on one another's solver.
    An empty cameras mapping signals that callers must use all nominal cameras.
    """
    corrected, details = {}, {}
    reason = 'corrected'
    deadline = time.monotonic() + float(config.get('max_wait_ms', 0.)) / 1000.
    for view, camera in cameras.items():
        header, raw = image_headers.get(view), sensor_timings.get(view)
        if header is None or raw is None:
            reason = 'missing_source_timing'; break
        expected = (camera.image_topic, header.frame_id, header.stamp.to_nsec(), header.seq)
        actual = (raw.sensor_topic, raw.sensor_frame_id, raw.published_stamp.to_nsec(),
                  raw.sensor_wire_sequence)
        if not raw.sensor_wire_sequence_valid or expected != actual:
            reason = 'source_identity_mismatch'; break
        pose, why = timeline.wait_attitude(raw, config.get('imu_frame_id', 'imu'),
            min(10., max(0., 1000 * (deadline - time.monotonic()))))
        if pose is None:
            reason = why; break
        try:
            corrected[view] = attitude_camera(camera, pose.quaternion,
                config.get('imu_mount_rotation_deg', [0., 0., 0.]),
                float(config.get('max_tilt_deg', 8.)))
        except ValueError:
            reason = 'attitude_out_of_range'; break
        details[view] = dict(source_ns=pose.source_ns, bracket_ms=pose.bracket_ms,
            quaternion=list(pose.quaternion), plane=corrected[view].ground_plane.coefficients,
            bridge_session_id=pose.bridge_session_id)
    skew = ((max(d['source_ns'] for d in details.values()) -
             min(d['source_ns'] for d in details.values())) / 1e6 if details else None)
    if reason == 'corrected' and (not details or skew > float(config.get('max_source_skew_ms', 35.))):
        reason = 'camera_source_skew'
    if reason != 'corrected':
        corrected = {}
    return ProjectionDecision(MappingProxyType(corrected), MappingProxyType(details), reason, skew)
