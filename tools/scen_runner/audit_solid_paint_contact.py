#!/usr/bin/env python3
"""Audit recorded MORAI body/solid-paint contact; not the referee wheel signal.

The competition rule counts a wheel touching solid paint for three seconds.
Oracle bounding boxes cover the wheels, so this is a conservative proxy, not
an official penalty verdict. Missing oracle samples make the result inconclusive.
"""

import argparse
import json
from pathlib import Path

from shapely.affinity import rotate, translate
import math

from shapely.geometry import LineString, Point, box
from shapely.strtree import STRtree


def _episodes(rows, contacts):
    episodes = []
    beginning = None
    for i, contact in enumerate(contacts):
        if contact and beginning is None:
            beginning = i
        if beginning is not None and (not contact or i == len(contacts) - 1):
            last = i if contact else i - 1
            episodes.append({"start_station": rows[beginning]["station"],
                             "end_station": rows[last]["station"],
                             "duration_s": rows[last]["monotonic"] -
                                           rows[beginning]["monotonic"]})
            beginning = None
    return episodes


def audit(trial, lines_path, start, end, half_track_m=0.814,
          wheelbase_m=3.0, wheel_patch_radius_m=0.15):
    rows = [json.loads(line) for line in (trial / "oracle.jsonl").open() if line.strip()]
    rows = [row for row in rows if start <= row["station"] <= end]
    if not rows:
        raise ValueError(f"no oracle samples in station window: {trial}")
    centers = [row["bounding_box"]["center"] for row in rows]
    nearby = box(min(p[0] for p in centers) - 8, min(p[1] for p in centers) - 8,
                 max(p[0] for p in centers) + 8, max(p[1] for p in centers) + 8)
    paints = []
    for feature in json.loads(lines_path.read_text())["features"]:
        attrs = feature["attributes"]
        if (feature["feature_type"] != "lane_marking" or
                attrs.get("pattern") != "solid" or
                attrs.get("color") not in ("yellow", "white")):
            continue
        line = LineString([(p[0], p[1]) for p in feature["geometry"]["coordinates"]])
        if line.intersects(nearby):
            paints.append(line.buffer(attrs.get("line_width_m", 0.0) / 2,
                                      cap_style=2))
    tree = STRtree(paints)
    contacts = []
    estimated_wheel_contacts = []
    for row in rows:
        if not row["observation_complete"]:
            contacts.append(None)
            estimated_wheel_contacts.append(None)
            continue
        bbox = row["bounding_box"]
        body = box(-bbox["half_length"], -bbox["half_width"],
                   bbox["half_length"], bbox["half_width"])
        body = translate(rotate(body, bbox["center"][2], origin=(0, 0),
                                use_radians=True), *bbox["center"][:2])
        contacts.append(any(body.intersects(paints[int(i)]) for i in tree.query(body)))
        heading = row["yaw"]
        wheel_contact = False
        for forward in (0.0, wheelbase_m):
            for lateral in (-half_track_m, half_track_m):
                wheel = Point(row["x"] + forward * math.cos(heading) -
                              lateral * math.sin(heading),
                              row["y"] + forward * math.sin(heading) +
                              lateral * math.cos(heading)).buffer(wheel_patch_radius_m)
                if any(wheel.intersects(paints[int(i)]) for i in tree.query(wheel)):
                    wheel_contact = True
        estimated_wheel_contacts.append(wheel_contact)
    gaps = [i for i in range(1, len(rows))
            if rows[i]["monotonic"] - rows[i - 1]["monotonic"] > .2]
    episodes = _episodes(rows, contacts)
    wheel_episodes = _episodes(rows, estimated_wheel_contacts)
    longest = max((e["duration_s"] for e in episodes), default=0.0)
    wheel_longest = max((e["duration_s"] for e in wheel_episodes), default=0.0)
    # Route stations are sampled at vehicle-status cadence (up to about 1 m
    # between samples at the tested speed), not at exact window boundaries.
    covered_window = (min(row["station"] for row in rows) <= start + 1.0 and
                      max(row["station"] for row in rows) >= end - 1.0)
    complete = covered_window and not gaps and all(c is not None for c in contacts)
    return {"trial": str(trial), "scope": "conservative_body_proxy_not_wheels",
            "samples": len(rows), "observation_complete": complete,
            "station_window_covered": covered_window,
            "maximum_sample_gap_s": max((rows[i]["monotonic"] -
                                          rows[i - 1]["monotonic"]
                                          for i in range(1, len(rows))), default=0.0),
            "episodes": episodes, "longest_contact_s": longest,
            "total_contact_s": sum(e["duration_s"] for e in episodes),
            "below_three_second_referee_window_proxy": complete and longest < 3.0,
            "below_two_point_five_second_planning_target": complete and longest < 2.5,
            "estimated_wheel_proxy": {
                "scope": "geometric_estimate_not_EgoDetailStatus",
                "half_track_m": half_track_m, "wheelbase_m": wheelbase_m,
                "wheel_patch_radius_m": wheel_patch_radius_m,
                "episodes": wheel_episodes,
                "longest_contact_s": wheel_longest,
                "below_three_seconds": complete and wheel_longest < 3.0}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, nargs="+")
    parser.add_argument("--lines", type=Path,
                        default=Path(__file__).resolve().parents[2] /
                        "src/asmc_planner/config/hd_map/lines.json")
    parser.add_argument("--start", type=float, default=280.0)
    parser.add_argument("--end", type=float, default=350.0)
    parser.add_argument("--half-track-m", type=float, default=0.814)
    parser.add_argument("--wheelbase-m", type=float, default=3.0)
    parser.add_argument("--wheel-patch-radius-m", type=float, default=0.15)
    args = parser.parse_args()
    if min(args.half_track_m, args.wheelbase_m, args.wheel_patch_radius_m) < 0:
        parser.error("wheel geometry must be nonnegative")
    for trial in args.trial:
        print(json.dumps(audit(trial, args.lines, args.start, args.end,
                               args.half_track_m, args.wheelbase_m,
                               args.wheel_patch_radius_m),
                         ensure_ascii=False))


if __name__ == "__main__":
    main()
