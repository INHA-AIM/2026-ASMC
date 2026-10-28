"""Audit source/arrival timing and attitude around local invalid plans in a bag."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from scen_runner.records.analyze_local import quantiles


def audit(root):
    import rosbag

    root = Path(root)
    frames = defaultdict(dict)
    events = []
    attitudes = []
    family = Counter()
    context = Counter()
    latest = {}
    gaps = defaultdict(list)
    previous = {}
    prefix = "/experimental/shadow/semantic_space_trial/"
    names = {
        prefix + "context": "context",
        prefix + "coverage": "coverage",
        prefix + "drivable_probability": "probability",
        "/perception/lane/boundary_family": "family",
        "/perception/lidar/tracks_local": "lidar",
        "/imu": "imu",
        "/planning/local/trace": "trace",
    }
    for bagfile in sorted(root.glob("*.bag")):
        with rosbag.Bag(str(bagfile)) as bag:
            for topic, m, received in bag.read_messages(topics=list(names)):
                kind = names[topic]
                arrival = received.to_sec()
                if kind == "trace":
                    row = json.loads(m.data)
                    if not row["valid"]:
                        events.append(
                            dict(
                                reason=row["reason"],
                                time=row["decision_stamp"],
                                latest=dict(latest),
                                attitude=attitudes[-1] if attitudes else None,
                            )
                        )
                    continue
                header = m.observation.objects.header if kind == "lidar" else m.header
                stamp = header.stamp.to_nsec()
                t = header.stamp.to_sec()
                if kind in previous and 0 < t - previous[kind] < 3:
                    gaps[kind].append((t - previous[kind]) * 1000)
                previous[kind] = t
                latest[kind] = dict(
                    source=t, arrival=arrival, age_at_arrival=arrival - t
                )
                if kind == "family":
                    family[str(m.valid) + ":" + m.status_reason] += 1
                if kind == "context":
                    context[str(m.imu_corrected)] += 1
                if kind in ("family", "context", "probability", "coverage"):
                    frames[stamp][kind] = arrival
                if kind == "imu":
                    q = m.orientation
                    roll = math.atan2(
                        2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)
                    )
                    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
                    attitudes.append(
                        dict(
                            time=t,
                            roll_deg=math.degrees(roll),
                            pitch_deg=math.degrees(pitch),
                        )
                    )
    complete = [
        max(v.values()) - k / 1e9
        for k, v in frames.items()
        if all(x in v for x in ("family", "context", "coverage", "probability"))
    ]
    result = dict(
        family=dict(family),
        imu_corrected_context=dict(context),
        bundle_arrival_age_s=quantiles(complete),
        source_gap_ms={k: quantiles(v) for k, v in gaps.items()},
        roll_abs_deg=quantiles([abs(x["roll_deg"]) for x in attitudes]),
        pitch_abs_deg=quantiles([abs(x["pitch_deg"]) for x in attitudes]),
        invalid_events=events,
        interpretation="arrival ages include processing and transport; correlation is not a posture-causality claim",
    )
    with (root / "input-timing-audit.json").open("x") as f:
        json.dump(result, f, indent=2)
    return {k: v for k, v in result.items() if k != "invalid_events"}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    print(json.dumps(audit(p.parse_args().root), indent=2))
