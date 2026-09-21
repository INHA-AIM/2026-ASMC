#!/usr/bin/env python3
"""Dev-only MORAI teleport regression for HD-map current-lane association.

The tool never starts/restarts the world.  It attaches to the existing Ego,
tests only the competition traversable high-speed lanes, and restores the
original Ego transform in ``finally``.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
GRPC_ROOT = REPO_ROOT / "tools" / "grpc_inha_univ" / "src"
for path in (GRPC_ROOT, GRPC_ROOT / "api", GRPC_ROOT / "proto"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import rospy  # noqa: E402
from asmc_msgs.msg import LaneArray, LaneAssociation  # noqa: E402
from api.morai_sim_client import MoraiSimClient  # noqa: E402
from api.vehicle import Vehicle  # noqa: E402
from proto.morai.common.type_pb2 import Transform  # noqa: E402


USABLE_LINKS = {
    "A2256W000411": 5,  # entry merge lane
    "A2256W000409": 4,
    "A2256W000410": 3,
    "A2256W000418": 2,
}
EXCLUDED_BUS_LINKS = {
    "A2256W000429",
    "A2256W000433",
    "A2256W000421",
    "A2256W000425",
    "A2256W000424",
}


def _sample_polyline(points, fraction):
    xy = [(float(point.x), float(point.y)) for point in points]
    lengths = [math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(xy, xy[1:])]
    target = sum(lengths) * float(fraction)
    accumulated = 0.0
    for index, length in enumerate(lengths):
        if accumulated + length >= target and length > 1e-9:
            ratio = (target - accumulated) / length
            x0, y0 = xy[index]
            x1, y1 = xy[index + 1]
            return (
                x0 + ratio * (x1 - x0),
                y0 + ratio * (y1 - y0),
                math.atan2(y1 - y0, x1 - x0),
            )
        accumulated += length
    x0, y0 = xy[-2]
    x1, y1 = xy[-1]
    return x1, y1, math.atan2(y1 - y0, x1 - x0)


def _transform(x, y, z, yaw_rad):
    result = Transform()
    result.location.x = float(x)
    result.location.y = float(y)
    result.location.z = float(z)
    result.rotation.z = math.degrees(float(yaw_rad))
    return result


def _wait_at(x, y, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    last = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        last = rospy.wait_for_message(
            "/hd_map/local/current_lane", LaneAssociation, timeout=timeout_sec
        )
        distance = math.hypot(
            float(last.reference_point.x) - x, float(last.reference_point.y) - y
        )
        if distance <= 1.0 and last.state != LaneAssociation.STATE_STALE:
            return last
    raise RuntimeError(
        "current-lane topic did not reach teleport target; last=%s"
        % (last.link_id if last is not None else "none")
    )


def _result_dict(label, expected_link, message):
    passed = bool(message.valid and message.link_id == expected_link)
    return {
        "label": label,
        "expected_link": expected_link,
        "actual_link": message.link_id,
        "lane_number": int(message.lane_number),
        "state": int(message.state),
        "clearance_m": float(message.polygon_clearance_m),
        "lateral_offset_m": float(message.signed_lateral_offset_m),
        "transition_pending": bool(message.transition_pending),
        "transition_candidate": message.transition_candidate_lane_id,
        "passed": passed,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="host.docker.internal")
    parser.add_argument("--port", type=int, default=7789)
    parser.add_argument("--client-key", default="hd_map_lane_association_audit")
    parser.add_argument("--settle-sec", type=float, default=0.55)
    parser.add_argument("--timeout-sec", type=float, default=4.0)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    rospy.init_node("hd_map_lane_association_live_audit", anonymous=True)
    lane_array = rospy.wait_for_message("/hd_map/global/lanes", LaneArray, timeout=5.0)
    by_link = {lane.link_id: lane for lane in lane_array.lanes}

    missing = sorted(set(USABLE_LINKS) - set(by_link))
    leaked_bus = sorted(EXCLUDED_BUS_LINKS & set(by_link))
    if missing or leaked_bus:
        raise RuntimeError(
            f"competition lane inventory mismatch missing={missing} leaked_bus={leaked_bus}"
        )

    client = MoraiSimClient(args.client_key)
    client.connect(args.host, args.port)
    if not client.is_connected():
        raise RuntimeError(f"cannot connect MORAI gRPC at {args.host}:{args.port}")
    ego = Vehicle(client._sim_adapter, args.client_key, "Ego")
    original_state = ego.get_actor_state()
    if original_state is None:
        raise RuntimeError("cannot read existing Ego state")
    original_transform = Transform()
    original_transform.CopyFrom(original_state.transform)
    original_speed = math.sqrt(
        float(original_state.velocity.x) ** 2
        + float(original_state.velocity.y) ** 2
        + float(original_state.velocity.z) ** 2
    )
    z_m = float(original_transform.location.z)
    results = []

    def place(label, expected_link, x, y, yaw):
        if not ego.set_transform(_transform(x, y, z_m, yaw)):
            raise RuntimeError(f"teleport failed: {label}")
        ego.set_velocity(0.0)
        time.sleep(max(0.0, args.settle_sec))
        message = _wait_at(x, y, args.timeout_sec)
        result = _result_dict(label, expected_link, message)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return message

    try:
        samples = {}
        for link_id in USABLE_LINKS:
            samples[link_id] = _sample_polyline(by_link[link_id].centerline, 0.50)
            x, y, yaw = samples[link_id]
            message = place(f"center:{link_id}", link_id, x, y, yaw)
            if int(message.lane_number) != USABLE_LINKS[link_id]:
                results[-1]["passed"] = False
                results[-1]["lane_number_error"] = (
                    f"expected {USABLE_LINKS[link_id]}"
                )

        # Explicitly cross the A409(4) / A410(3) shared boundary.  The exact
        # midpoint is treated as belonging to the previous lane; the switch
        # happens only after two distinct poses at least 0.30 m inside A410.
        x4, y4, yaw4 = samples["A2256W000409"]
        x3, y3, yaw3 = samples["A2256W000410"]
        place("transition:start_lane4", "A2256W000409", x4, y4, yaw4)
        boundary_x, boundary_y = (x4 + x3) / 2.0, (y4 + y3) / 2.0
        place("transition:shared_boundary", "A2256W000409", boundary_x, boundary_y, yaw4)
        dx, dy = x3 - x4, y3 - y4
        norm = math.hypot(dx, dy)
        target_x = boundary_x + 0.38 * dx / norm
        target_y = boundary_y + 0.38 * dy / norm
        # MORAI keeps publishing GPS while stationary, so the 0.55 s settle
        # interval intentionally supplies the two consecutive 5 Hz samples.
        # The pure unit test separately asserts the first-sample pending state.
        place("transition:confirmed_lane3", "A2256W000410", target_x, target_y, yaw3)
        tangent_x, tangent_y = math.cos(yaw3), math.sin(yaw3)
        place(
            "transition:lane3_continuity",
            "A2256W000410",
            target_x + 0.10 * tangent_x,
            target_y + 0.10 * tangent_y,
            yaw3,
        )
    finally:
        ego.set_transform(original_transform)
        ego.set_velocity(original_speed)
        client.disconnect()

    summary = {
        "build_id": lane_array.build_id,
        "geometry_sha256": lane_array.geometry_sha256,
        "reference": "map-frame base_link rear-axle center",
        "usable_lane_numbers": [2, 3, 4],
        "merge_lane_number": 5,
        "excluded_bus_links": sorted(EXCLUDED_BUS_LINKS),
        "passed": all(result["passed"] for result in results),
        "results": results,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
