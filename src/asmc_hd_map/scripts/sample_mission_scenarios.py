#!/usr/bin/env python3
"""Print sampled actor spawn poses for one mission zone as JSON lines.

Example:
  python3 src/asmc_hd_map/scripts/sample_mission_scenarios.py \
    --zone roundabout_candidate --role npc_vehicle --n 10 --seed 0
Without --role every actor role of the zone is sampled. Poses use the map
frame and MORAI yaw degrees (atan2(dy, dx)), so they can seed a gRPC spawn or
an offline planner check.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    root = Path(__file__).resolve().parents[3]
    package_src = root / "src/asmc_hd_map/src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))
    from asmc_hd_map.mission_scenario import MissionScenarioSpec

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--zone", help="mission zone id; omit to list zones and roles")
    parser.add_argument("--role")
    parser.add_argument("--n", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--margin", type=float, default=2.0, help="longitudinal margin from link ends [m]")
    parser.add_argument("--jitter", type=float, default=0.0, help="uniform lateral jitter [m]")
    args = parser.parse_args(argv)

    spec = MissionScenarioSpec.load(args.repo_root, args.source)
    for warning in spec.warnings:
        print("WARNING " + warning, file=sys.stderr)
    if not args.zone:
        for zone in spec.zones():
            roles = spec.actor_roles(zone["id"])
            print("%-40s %-20s %8.1f-%8.1f m  roles=%s" % (
                zone["id"], zone["mission_type"], zone["route_station_start_m"],
                zone["route_station_end_m"], ",".join(roles) or "-",
            ))
        return 0
    roles = [args.role] if args.role else spec.actor_roles(args.zone)
    if not roles:
        print("zone %s has no actor sources" % args.zone, file=sys.stderr)
        return 1
    rng = random.Random(args.seed)
    for role in roles:
        for _ in range(args.n):
            pose = spec.sample_actor_pose(args.zone, role, rng, args.margin, args.jitter)
            print(json.dumps(pose, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
