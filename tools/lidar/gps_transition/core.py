"""L1-B2 isolated GPS-health/output selector. No ROS topics or tracker mutation.

All times are integers in one explicitly named clock. GPS health is deliberately
independent of the observation-time pose used by the L1-B1 geometry core.
"""

from dataclasses import dataclass
import copy
import math


@dataclass(frozen=True)
class Policy:
    max_age_ns: int = 500_000_000
    recovery_fixes: int = 3
    max_speed_mps: float = 40.0  # experimental gate, not a competition speed rule
    jump_slack_m: float = 1.5
    reanchor_tolerance_m: float = 2.0


class GpsGate:
    def __init__(self, clock="ros_receive", policy=Policy()):
        self.clock = clock
        self.policy = policy
        self.state = "startup"
        self.reason = "no_gps"
        self.last_arrival_ns = None
        self.last_event_arrival_ns = None
        self.last_stamp_ns = None
        self.last_good = None  # (x, y, stamp_ns); do not replace on invalid/jump
        self.jump_limit_m = None  # freeze the first rejection threshold
        self.fixes = 0
        self.events = 0

    def snapshot(self):
        return {"state": self.state, "reason": self.reason,
                "latest_gps_stamp_ns": self.last_stamp_ns,
                "last_good_stamp_ns": self.last_good[2] if self.last_good else None,
                "consecutive_fixes": self.fixes}

    def tick(self, now_ns):
        if not isinstance(now_ns, int) or now_ns <= 0:
            raise ValueError("positive integer now_ns required")
        if self.last_event_arrival_ns is not None and now_ns < self.last_event_arrival_ns:
            raise ValueError("replay arrival order regressed")
        self.last_event_arrival_ns = now_ns
        if (self.last_arrival_ns is not None and
                now_ns - self.last_arrival_ns > self.policy.max_age_ns and
                self.state != "stale"):
            self.state, self.reason, self.fixes = "stale", "gps_timeout", 0
        return self.snapshot()

    def feed(self, *, arrival_ns, stamp_ns, clock, status, latitude, longitude,
             altitude, xy_m=None, trusted_reanchor=None):
        self.tick(arrival_ns)
        self.events += 1
        if clock != self.clock:
            self.state, self.reason, self.fixes = "invalid", "clock_mismatch", 0
            return self.snapshot()
        if (not isinstance(stamp_ns, int) or stamp_ns <= 0 or stamp_ns > arrival_ns or
                (self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns)):
            self.state, self.reason, self.fixes = "invalid", "stamp_invalid", 0
            return self.snapshot()
        self.last_stamp_ns, self.last_arrival_ns = stamp_ns, arrival_ns
        if arrival_ns - stamp_ns > self.policy.max_age_ns:
            self.state, self.reason, self.fixes = "stale", "old_gps_stamp", 0
            return self.snapshot()
        values = (latitude, longitude, altitude)
        if (not isinstance(status, int) or status <= 0 or
                any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values) or
                not -90 <= latitude <= 90 or not -180 <= longitude <= 180 or
                (latitude == 0 and longitude == 0)):
            self.state, self.reason, self.fixes = "invalid", "invalid_fix", 0
            return self.snapshot()
        if (xy_m is None or len(xy_m) != 2 or
                any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in xy_m)):
            self.state, self.reason, self.fixes = "invalid", "projection_unavailable", 0
            return self.snapshot()
        x, y = xy_m
        if self.last_good:
            dx = math.hypot(x - self.last_good[0], y - self.last_good[1])
            elapsed = max(0, stamp_ns - self.last_good[2]) / 1e9
            limit = self.policy.max_speed_mps * elapsed + self.policy.jump_slack_m
            if self.jump_limit_m is not None:
                limit = min(limit, self.jump_limit_m)
            plausible = dx <= limit
            if not plausible:
                # A different, explicitly named pose source may authorize reanchoring.
                anchor_ok = (isinstance(trusted_reanchor, dict) and
                             trusted_reanchor.get("source") not in (None, "", "gps") and
                             trusted_reanchor.get("valid") is True and
                             trusted_reanchor.get("stamp_ns") == stamp_ns and
                             isinstance(trusted_reanchor.get("xy_m"), (list, tuple)) and
                             len(trusted_reanchor["xy_m"]) == 2 and
                             all(isinstance(v, (int, float)) and math.isfinite(v)
                                 for v in trusted_reanchor["xy_m"]) and
                             math.hypot(x - trusted_reanchor["xy_m"][0],
                                        y - trusted_reanchor["xy_m"][1]) <=
                             self.policy.reanchor_tolerance_m)
                if not anchor_ok:
                    if self.jump_limit_m is None:
                        self.jump_limit_m = limit
                    self.state, self.reason, self.fixes = "jump_rejected", "implausible_jump", 0
                    return self.snapshot()
                self.last_good = None
                self.fixes = 0
        self.last_good = (x, y, stamp_ns)
        self.jump_limit_m = None
        self.fixes += 1
        if self.fixes >= self.policy.recovery_fixes:
            self.state, self.reason = "ready", "consecutive_valid_fixes"
        else:
            self.state, self.reason = "recovering", "waiting_for_fixes"
        return self.snapshot()


def select_output(core_result, gate, *, processing_ns, detector_ran,
                  pose_trusted, vehicle_state_ready, trajectory_ready):
    """Select fixed-frame candidates; never fabricate fresh empty or map geometry.

    `pose_trusted` must be observation-time pose evidence independent of raw GPS
    health. This selector cannot establish it from a GPS coordinate alone.
    """
    health = gate.tick(processing_ns)
    stamp = core_result.get("stamp_ns") if core_result else None
    result = {"input_id": core_result.get("input_id") if core_result else None,
              "stamp_ns": stamp, "gps": health,
              "local": {"frame": "base_link", "status": "sensor_missing", "objects": []},
              "map": {"frame": "map", "status": "inactive", "objects": []},
              "planner_ready": False, "planner_reason": "unavailable"}
    if core_result is None:
        result["map"]["status"] = "sensor_missing"
        result["planner_reason"] = "sensor_missing"
        return result
    if not core_result.get("valid"):
        reason = core_result.get("reason", "invalid_observation")
        result["local"]["status"] = "invalid_observation:" + reason
        result["map"]["status"] = "invalid_observation:" + reason
        result["planner_reason"] = "invalid_observation"
        return result
    if not isinstance(stamp, int) or stamp <= 0 or stamp > processing_ns:
        result["local"]["status"] = "invalid_observation:stamp_invalid"
        result["map"]["status"] = "invalid_observation:stamp_invalid"
        result["planner_reason"] = "invalid_observation"
        return result
    if not detector_ran:
        result["local"]["status"] = "unassessed_detector_not_run"
        result["map"]["status"] = "unassessed_detector_not_run"
        result["planner_reason"] = "detector_not_run"
        return result
    objects = core_result.get("objects", [])
    if any(not o.get("valid") or o.get("base") is None for o in objects):
        result["local"]["status"] = "invalid_object_geometry"
        result["map"]["status"] = "invalid_object_geometry"
        result["planner_reason"] = "invalid_object_geometry"
        return result
    result["local"]["objects"] = [
        {"id": o.get("id"), "classification": o.get("classification"),
         "geometry": copy.deepcopy(o["base"]),
         "absolute_velocity": copy.deepcopy(o.get("absolute_velocity_base"))}
        for o in objects]
    result["local"]["status"] = "objects" if objects else "observed_empty"
    if health["state"] != "ready":
        result["map"]["status"] = "gps_" + health["state"]
        result["planner_reason"] = "gps_unavailable"
        return result
    if health["latest_gps_stamp_ns"] is not None and stamp < health["latest_gps_stamp_ns"]:
        result["map"]["status"] = "gps_evidence_after_observation"
        result["planner_reason"] = "observation_precedes_gps_health"
        return result
    if not pose_trusted or core_result.get("pose_reason") != "valid":
        result["map"]["status"] = "pose_unavailable"
        result["planner_reason"] = "pose_unavailable"
        return result
    if any(o.get("map_reason") != "valid" or o.get("map") is None for o in objects):
        result["map"]["status"] = "map_geometry_unavailable"
        result["planner_reason"] = "map_geometry_unavailable"
        return result
    result["map"]["objects"] = [
        {"id": o.get("id"), "classification": o.get("classification"),
         "geometry": copy.deepcopy(o["map"]),
         "absolute_velocity": copy.deepcopy(o.get("absolute_velocity_map"))}
        for o in objects]
    result["map"]["status"] = "objects" if objects else "observed_empty"
    if not vehicle_state_ready:
        result["planner_reason"] = "vehicle_state_unavailable"
    elif not trajectory_ready:
        result["planner_reason"] = "trajectory_unavailable"
    else:
        result["planner_ready"], result["planner_reason"] = True, "ready"
    return result
