#!/usr/bin/env python3
"""Generate an explicit refinement profile from the selected main YAML.

Never imports an old full snapshot or changes camera model, fusion views,
rear/front clustering tolerances, voxel size or semantic-motion policy.
"""
import argparse
import copy
from pathlib import Path
import yaml


def configured(base, profile="integrated", tuning="keep"):
    if profile not in ("disabled", "steel", "wall", "integrated"):
        raise ValueError("unknown refinement profile")
    if tuning not in ("keep", "low-obstacle"):
        raise ValueError("unknown tuning policy")
    result = copy.deepcopy(base)
    wall = profile in ("wall", "integrated")
    steel = profile in ("steel", "integrated")
    result.setdefault("camera_point_refinement", {}).update(
        enabled=wall, steel_fragment_merge=steel,
        vehicle_scanline_grouping=wall, independent_wall_cleanup=wall,
        detached_wall_cleanup=wall, steel_minimum_confidence=0.5,
        steel_maximum_gap_m=2.8, steel_minimum_ownership=0.8)
    tunnel = result["experimental"]["tunnel_filter"]
    tunnel["wall_geometry_precedence"] = wall
    tunnel["low_obstacle_evidence"] = profile != "disabled"
    tunnel.setdefault("protection", {})["motion_compensated_fallback"] = wall
    if tuning == "low-obstacle":
        result["lidar"]["Clustering"]["MinPoints"] = 3
        result["ground"].update(ground_distance_threshold_near_m=0.23,
                               ground_distance_threshold_far_m=0.40,
                               ground_threshold_far_range_m=32.0)
        tunnel.setdefault("original_parameters", {}).update(
            cluster_min_points=3, ground_remove_distance_m=0.20)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", choices=("disabled", "steel", "wall", "integrated"), default="integrated")
    parser.add_argument("--tuning", choices=("keep", "low-obstacle"), default="keep")
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error("input must remain unchanged; select a new output file")
    data = configured(yaml.safe_load(args.input.read_text()), args.profile, args.tuning)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with args.output.open("x") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
    except FileExistsError:
        parser.error("output already exists; select a new output file")


if __name__ == "__main__":
    main()
