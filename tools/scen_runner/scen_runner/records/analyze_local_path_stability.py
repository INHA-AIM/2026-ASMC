"""Compare successive geometric paths at the same current local positions.

Recorded trajectory ego_at_plan supplies exact relative-frame pose, never GT.
Only same-session plans and the native drive interval are compared.
"""

import argparse
import bisect
import json
import math
from pathlib import Path
from .analyze_local import quantiles


def lateral(path, x):
    if (len(path) < 2 or any(b[0] <= a[0] for a, b in zip(path, path[1:]))
            or x < path[0][0] or x > path[-1][0]):
        return None
    i = max(1, bisect.bisect_left([p[0] for p in path], x))
    a, b = path[i - 1], path[i]
    if b[0] - a[0] <= 1e-8:
        return None
    return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])


def align(path, old, current):
    ox, oy, oa = old
    x, y, a = current
    c, s = math.cos(a), math.sin(a)
    ca, sa = math.cos(oa), math.sin(oa)
    out = []
    for p in path:
        dx, dy = ox + ca * p[0] - sa * p[1] - x, oy + sa * p[0] + ca * p[1] - y
        out.append([c * dx + s * dy, -s * dx + c * dy])
    return out


def analyze(root, output):
    import rosbag

    root = Path(root)
    rows = [json.loads(x) for x in (root / "decisions.jsonl").read_text().splitlines()]
    events = [json.loads(x) for x in (root / "events.jsonl").read_text().splitlines()]
    native = [e for e in events if e.get("event") == "native_sample"]
    begin, end = native[0]["ros_time"], native[-1]["ros_time"]
    poses = {}
    for p in sorted(root.glob("sensors*.bag")):
        with rosbag.Bag(str(p)) as bag:
            for _, m, _ in bag.read_messages(topics=["/planning/local/trajectory"]):
                q = m.ego_at_plan.pose.pose.orientation
                point = m.ego_at_plan.pose.pose.position
                angle = math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                )
                poses[(m.header.frame_id, m.plan_sequence)] = [point.x, point.y, angle]
    previous = None
    changes = {5: [], 10: []}
    samples = []
    skipped = 0
    for r in rows:
        if not r["valid"] or not begin <= r["decision_stamp"] <= end:
            previous = None
            continue
        pose = poses.get((r["frame"], r["plan_sequence"]))
        if pose is None:
            previous = None
            skipped += 1
            continue
        if (
            previous
            and r["frame"] == previous[0]["frame"]
            and 0 < r["decision_stamp"] - previous[0]["decision_stamp"] <= 0.3
        ):
            old = align(previous[0]["path"], previous[1], pose)
            for distance in changes:
                a, b = lateral(r["path"], distance), lateral(old, distance)
                if a is not None and b is not None:
                    changes[distance].append(abs(a - b))
                    samples.append(
                        dict(
                            plan_sequence=r["plan_sequence"],
                            x_m=distance,
                            change_m=a - b,
                        )
                    )
        previous = (r, pose)
    result = dict(
        jump_abs_m={str(x): quantiles(v) for x, v in changes.items()},
        count={str(x): len(v) for x, v in changes.items()},
        missing_pose=skipped,
        interpretation="same-position replanning consistency; not independent tracking accuracy",
        samples=samples,
    )
    with Path(output or root / "path-stability.json").open("x") as f:
        json.dump(result, f, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    analyze(a.root, a.output)
