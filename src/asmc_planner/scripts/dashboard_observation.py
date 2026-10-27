"""Bounded, observer-only local scene. No pose extrapolation or control output.

ROS source stamps and monotonic reception ages are kept separate. Coordinates
remain in the source frame; a local drawing is never placed at the last GPS pose.
"""
import math
import threading
import time


def stamp_ns(header):
    return int(header.stamp.to_nsec())


def xy(point):
    values = [float(point.x), float(point.y)]
    return values if all(math.isfinite(v) for v in values) else None


class LocalScene:
    KEYS = {"family", "path", "objects", "controller", "bbox", "selector", "mode"}

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.items = {}
        self.accepted = {}
        self.epoch = None
        self.retired = set()

    def put(self, key, payload, stamp=0, frame="", valid=True, reason="",
            epoch=None, sequence=None):
        if key not in self.KEYS:
            raise ValueError("unknown scene key")
        received = self.clock()
        with self.lock:
            previous = self.accepted.get(key)
            if key in {"family", "path", "objects"} and frame != "base_link":
                valid, reason = False, "base_link_required"
            if key in {"family", "path", "objects", "controller", "bbox"} and stamp <= 0:
                valid, reason = False, "source_stamp_missing"
            if epoch is not None:
                if not epoch[0]:
                    valid, reason = False, "producer_identity_missing"
                elif self.epoch is not None and epoch != self.epoch:
                    if epoch in self.retired or (epoch[0] == self.epoch[0] and epoch[1] < self.epoch[1]):
                        valid, reason = False, "retired_epoch"
                    elif len(self.retired) >= 128:
                        valid, reason = False, "epoch_limit_restart_dashboard"
                    else:
                        self.retired.add(self.epoch)
                        self.epoch = epoch
                        previous = None
                elif self.epoch is None:
                    self.epoch = epoch
                if previous and previous.get("epoch") == epoch and sequence is not None:
                    if sequence <= previous.get("sequence", -1):
                        valid, reason = False, "duplicate_or_regressing_scan"
            if previous and stamp and stamp < previous["stamp_ns"]:
                valid, reason = False, "source_stamp_regression"
            # Invalid input replaces visible geometry immediately. Never reuse a
            # previous good scene as a newly received free-space observation.
            self.items[key] = dict(payload=payload if valid else {}, stamp_ns=stamp,
                                   frame_id=frame, valid=valid, reason=reason,
                                   received=received, epoch=epoch, sequence=sequence)
            if valid:
                self.accepted[key] = self.items[key]

    def snapshot(self, ros_now_ns):
        now = self.clock()
        with self.lock:
            items = dict(self.items)
        result = {}
        for key, entry in items.items():
            source_age = ((int(ros_now_ns) - entry["stamp_ns"]) / 1e9
                          if entry["stamp_ns"] else None)
            receipt_age = now - entry["received"]
            # Selector diagnostics are 1 Hz; this is display freshness only,
            # never the selector's 250 ms command watchdog.
            limit = (0.35 if key in {"family", "path", "objects"} else
                     2.5 if key in {"selector", "mode"} else 0.5)
            fresh = 0 <= receipt_age <= limit and (source_age is None or -0.05 <= source_age <= limit)
            result[key] = {k: v for k, v in entry.items() if k not in {"received", "epoch", "sequence"}}
            result[key].update(source_age_s=source_age, receive_age_s=receipt_age,
                               freshness_limit_s=limit,
                               available=bool(entry["valid"] and fresh))
        return result


def family_payload(message):
    lines, omitted = [], 0
    # Split on INVALID support and on observed/inferred transitions. Never draw
    # a line through an unobserved gap just to make the display smooth.
    for member in message.boundaries[:3]:
        if len(member.polyline) != len(member.support_state):
            return {}, False, "support_length_mismatch"
        current, state = [], None
        for point, support in zip(member.polyline[:256], member.support_state[:256]):
            p = xy(point)
            if support not in (1, 2) or p is None or support != state:
                if len(current) >= 2:
                    lines.append(dict(role=int(member.role), support=state, points=current))
                current = []
            state = support
            if support in (1, 2) and p is not None:
                current.append(p)
        if len(current) >= 2:
            lines.append(dict(role=int(member.role), support=state, points=current))
        omitted += max(0, len(member.polyline) - 256)
    return dict(lines=lines, config_id=message.config_sha256,
                deployment_allowed=bool(message.deployment_allowed),
                omitted_points=omitted, x_min=message.valid_x_min_m,
                x_max=message.valid_x_max_m), bool(message.valid), message.status_reason


def markers_payload(message):
    lines, stamps, omitted = [], set(), 0
    allowed = {"own_lane_centerline", "adjacent_lane_centerline", "unassigned_lane_centerline", "selected_centerline"}
    for marker in message.markers:
        if marker.action != 0 or marker.type != 4 or marker.ns not in allowed:
            continue
        if marker.header.frame_id != "base_link":
            return {}, 0, False, "base_link_required"
        stamps.add(stamp_ns(marker.header))
        if len(lines) >= 32:
            omitted += len(marker.points)
            continue
        points = [xy(p) for p in marker.points[:256]]
        if any(p is None for p in points):
            return {}, 0, False, "nonfinite_marker"
        lines.append(dict(kind=marker.ns, points=points))
        omitted += max(0, len(marker.points) - 256)
    if len(stamps) != 1:
        return {}, 0, False, "no_path_or_mixed_source_stamps"
    return dict(lines=lines, omitted_points=omitted), stamps.pop(), True, ""


def objects_payload(message):
    observation = message.observation
    array = observation.objects
    output = []
    if len(array.objects) != len(message.tracking):
        return {}, False, "tracking_alignment_mismatch"
    if array.status not in (array.OBSERVED, array.OBSERVED_EMPTY):
        return {}, False, "local_observation_unavailable"
    if (array.status == array.OBSERVED_EMPTY) != (len(array.objects) == 0):
        return {}, False, "empty_status_mismatch"
    for obj, track in zip(array.objects[:256], message.tracking[:256]):
        if obj.track_id != track.track_id or obj.observation_state != obj.OBSERVED:
            return {}, False, "unobserved_or_mismatched_track"
        corners = [xy(p) for p in obj.obb_corners]
        if len(corners) != 8 or any(p is None for p in corners):
            return {}, False, "invalid_obb"
        output.append(dict(id=obj.track_id, classification=obj.classification,
                           camera_class=obj.camera_class, corners=corners,
                           motion_state=int(track.motion_state),
                           absolute_velocity_valid=bool(obj.absolute_velocity_valid)))
    return dict(objects=output, count=len(array.objects),
                omitted_objects=max(0, len(array.objects)-256),
                producer_session_id=observation.producer_session_id,
                generation=observation.generation, scan_sequence=observation.scan_sequence,
                config_id=observation.config_id, calibration_id=observation.calibration_id), True, ""
