"""Read-only LOCAL comparison: executed stops, failed plans and input evidence.

Uses the native driving interval, not warmup. A valid-path gap is never called
an actuator gap, and semantic groups never imply physical motion or safe stops.
"""
from collections import Counter
import json
from pathlib import Path

from .analyze_local import quantiles


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()] if Path(path).exists() else []


def summarize(root):
    root = Path(root)
    events = rows(root / "events.jsonl")
    release = next((r["ros_time"] for r in events if r.get("event") == "ego_released"), float("inf"))
    native = [r for r in events if r.get("event") == "native_sample" and r.get("ros_time", 0) >= release]
    if not native:
        return dict(run=root.name, driving=False, result=json.loads((root/"trial-result.json").read_text()) if (root/"trial-result.json").exists() else {})
    end = native[-1]["ros_time"]
    plans = [r for r in rows(root / "decisions.jsonl") if release <= r.get("decision_stamp", 0) <= end]
    control = [r for r in rows(root / "control.jsonl") if release <= r.get("decision_stamp", 0) <= end]
    intervals, first, last = [], None, None
    for r in native:
        t = r["ros_time"]
        if abs(r["ego"]["speed_kmh"]) < .5 and t > release+2:
            if first is None:
                first = t
            last = t
        elif first is not None:
            if last-first >= .5:
                intervals.append((first, last))
            first = last = None
    if first is not None and last-first >= .5:
        intervals.append((first, last))
    episodes = []
    for start, stop in intervals:
        preceding = [r for r in plans if start-.5 <= r["decision_stamp"] <= stop]
        episodes.append(dict(start_s=start-release, end_s=stop-release, duration_s=stop-start,
            plan_reasons=dict(Counter(r["reason"] for r in preceding)),
            input_valid_plans=sum(bool(r.get("scene_valid")) for r in preceding),
            failed_plans=sum(not r.get("valid") for r in preceding)))
    valid = [r for r in plans if r.get("valid")]
    stop_commands = [r for r in control if r.get("command_published") and not r.get("valid")]
    def gaps(name, valid_only=False):
        rr = [r for r in rows(root/name) if release <= r.get("received", 0) <= end and (not valid_only or r.get("valid"))]
        # Receipt monotonic time is authoritative; ROS time selects the interval.
        ts = [r["received_steady"] for r in rr if "received_steady" in r]
        return quantiles([1000*(b-a) for a,b in zip(ts,ts[1:])])
    ref = Counter(r.get("input_join",{}).get("reference_source", "unavailable") for r in plans)
    stationary_assets = Counter()
    for r in plans:
        for o in r.get("scene",{}).get("obstacles",[]):
            if o.get("local_motion_prediction"):
                stationary_assets[(o.get("camera_class","unknown"),o.get("classification","unknown"))] += 1
    return dict(run=root.name, driving=True, begin=release, seconds=end-release,
        result=json.loads((root/"trial-result.json").read_text()),
        plan_reasons=dict(Counter(r["reason"] for r in plans)),
        control_reasons=dict(Counter(r["reason"] for r in control)),
        stop_command_reasons=dict(Counter(r["reason"] for r in stop_commands)),
        actual_stationary_episodes=episodes, stationary_seconds=sum(e["duration_s"] for e in episodes),
        input_invalid_plans=sum(not r.get("scene_valid") for r in plans), plans=len(plans),
        path_sources=dict(Counter(r.get("path_source","unavailable") for r in valid)),
        reference_sources=dict(ref),
        camera_age_ms=quantiles([1000*r["inputs"]["camera_age_s"] for r in plans if "camera_age_s" in r.get("inputs",{})]),
        lidar_age_ms=quantiles([1000*r["inputs"]["lidar_age_s"] for r in plans if "lidar_age_s" in r.get("inputs",{})]),
        processing_ms=quantiles([r["processing_ms"] for r in plans]),
        control_processing_ms=quantiles([r["processing_ms"] for r in control if r.get("valid")]),
        valid_path_gap_ms=gaps("deliveries.jsonl",True),
        actual_speed_kph=quantiles([r["ego"]["speed_kmh"] for r in native]),
        forecast_snapshot_classes=[dict(camera_class=k[0],group=k[1],snapshots=v) for k,v in stationary_assets.items()],
        interpretation="Native driving interval; source/execution gaps remain distinct. Stops and semantic labels are evidence, not a safety verdict. Contact qualification remains in trial-result/native evidence.",
        speed_samples=[[r["ros_time"]-release,r["ego"]["speed_kmh"]] for r in native[::2]])
