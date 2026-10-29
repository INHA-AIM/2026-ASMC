#!/usr/bin/env python3
"""Read-only short GPS-outage pilot: native C++ motion and pinned AIMDrive core.

Native simulator poses are evaluation ONLY. Each relative comparison uses one
initial position translation, no fitted yaw/scale and no subsequent GT updates.
A GPS-anchor variant masks every subsequent GPS fix. This tool publishes nothing.
"""
import argparse
import bisect
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def stats(values):
    a = sorted(values)
    return {
        k: a[min(len(a) - 1, round(q * (len(a) - 1)))] if a else None
        for k, q in [("p50", 0.5), ("p95", 0.95), ("max", 1)]
    }


def interpolate(rows, t):
    if not rows:
        return None
    times = [r[0] for r in rows]
    k = bisect.bisect_left(times, t)
    if k == 0:
        return rows[0][1:] if abs(t - times[0]) < 0.05 else None
    if k == len(rows):
        return rows[-1][1:] if abs(t - times[-1]) < 0.05 else None
    a, b = rows[k - 1], rows[k]
    if b[0] - a[0] > 0.15:
        return None
    q = (t - a[0]) / (b[0] - a[0])
    return tuple(x + q * (y - x) for x, y in zip(a[1:], b[1:]))


def causal_point(rows, arrivals, target, available):
    """Anchor using only already received states; short forward extrapolation.

    This is an offline adapter, not a replacement production position estimator.
    Future states may interpolate evaluation truth, never the GPS anchor.
    """
    past = [
        r
        for r in rows
        if r[0] <= target and arrivals.get(r[0], float("inf")) <= available
    ]
    if not past or target - past[-1][0] > 0.05:
        return None
    last = past[-1]
    if target == last[0]:
        return last[1:]
    if len(past) < 2:
        return None
    previous = past[-2]
    dt = last[0] - previous[0]
    if not 0 < dt <= 0.15:
        return None
    return tuple(
        x + (x - y) * (target - last[0]) / dt for x, y in zip(last[1:], previous[1:])
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path)
    p.add_argument("native", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--aimdrive", type=Path, required=True)
    p.add_argument("--probe", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(exist_ok=False)
    sys.path.insert(0, str(a.aimdrive))
    from AIM2026.Localization.gps_aligned_hold import MotionTrack
    import rosbag
    from pyproj import Proj, Transformer

    qrows = []
    arrivals = {}
    speeds = deque(maxlen=64)
    gps = []
    motion_inputs = []
    reject = Counter()
    aim_speed = []
    aim_imu = []
    with rosbag.Bag(str(a.bag)) as bag:
        for topic, m, receipt in bag.read_messages(
            topics=["/Ego_topic", "/imu", "/gps"]
        ):
            t = m.header.stamp.to_sec()
            recv = receipt.to_nsec()
            if topic == "/gps":
                if m.latitude and m.longitude and m.status > 0:
                    gps.append(
                        dict(
                            t=t,
                            arrival=receipt.to_sec(),
                            lat=m.latitude,
                            lon=m.longitude,
                            east=m.eastOffset,
                            north=m.northOffset,
                        )
                    )
                continue
            if topic == "/Ego_topic":
                speeds.append((t, m.velocity.x, recv))
                continue
            eligible = [s for s in speeds if s[0] <= t]
            if not eligible or t - eligible[-1][0] > 0.1:
                reject["speed_alignment"] += 1
                continue
            v = eligible[-1][1]
            q = m.orientation
            g = m.angular_velocity
            n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
            if not math.isfinite(n) or abs(n - 1) > 0.05:
                reject["quaternion"] += 1
                continue
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            qrows.append((t, yaw))
            arrivals[t] = receipt.to_sec()
            motion_inputs.append((t, v, q.x, q.y, q.z, q.w, g.x, g.y, g.z))
            identity = dict(received_ns=recv, source_stamp_ns=m.header.stamp.to_nsec())
            # Existing local boundary uses the latest already received speed at
            # each IMU time. Apply that SAME resampling to both numeric cores.
            aim_speed.append((dict(signed_speed=v * 3.6), identity))
            aim_imu.append(
                (
                    dict(
                        yaw=yaw,
                        quaternion=(q.x, q.y, q.z, q.w),
                        angular_velocity=(g.x, g.y, g.z),
                    ),
                    identity,
                )
            )
    if not motion_inputs:
        raise ValueError("no usable motion inputs")
    projection = Proj(proj="utm", zone=52, datum="WGS84")
    convert = Transformer.from_crs(4326, 32652, always_xy=True)
    if gps:
        lat, lon = gps[0]["lat"], gps[0]["lon"]
        basis = "first recorded valid GPS, fixed convergence for this short pilot"
    else:
        lon, lat = Transformer.from_crs(32652, 4326, always_xy=True).transform(
            302595.0, 4124145.0
        )
        basis = "AIMDrive map origin; no recorded valid GPS anchor available"
    convergence = math.radians(projection.get_factors(lon, lat).meridian_convergence)
    text = "".join(
        " ".join(format(x, ".17g") for x in row) + "\n" for row in motion_inputs
    )
    native_result = subprocess.run(
        [str(a.probe)], input=text, text=True, capture_output=True, check=True
    )
    (a.output / "probe-input.txt").write_text(text)
    (a.output / "probe-output.txt").write_text(native_result.stdout)
    cpp = []
    epochs = Counter()
    initial = qrows[0][1] + convergence
    for l in native_result.stdout.splitlines():
        t, x, y, yaw, epoch = map(float, l.split())
        epochs[int(epoch)] += 1
        if epoch == 0:
            cpp.append(
                (
                    t,
                    math.cos(initial) * x - math.sin(initial) * y,
                    math.sin(initial) * x + math.cos(initial) * y,
                )
            )
    start = time.perf_counter()
    track = MotionTrack(lambda: convergence)
    track.update(aim_speed, aim_imu, aim_speed[-1][1]["received_ns"])
    duration = (time.perf_counter() - start) * 1000
    aim = [(s["source_stamp_ns"] / 1e9, *s["xy"]) for s in track.states]
    truth = []
    for l in a.native.read_text().splitlines():
        r = json.loads(l)
        if r.get("event") == "native_sample":
            truth.append(
                (
                    r["ros_time"],
                    r["ego"]["pose"]["x"],
                    r["ego"]["pose"]["y"],
                    r["ego"]["speed_kmh"],
                    math.radians(r["ego"]["pose"]["yaw"]),
                )
            )
        elif "pose" in r and "ros_ns" in r:
            t = r["pose"]["location"]
            truth.append(
                (
                    r["ros_ns"] / 1e9,
                    t["x"],
                    t["y"],
                    r["speed_mps"] * 3.6,
                    math.radians(r["pose"]["rotation"]["z"]),
                )
            )
    if not truth:
        raise ValueError("no native evaluation poses")
    results = {}
    traces = {}
    for name, rows in [("local_gyro_cpp", cpp), ("aimdrive_absolute_imu", aim)]:
        anchor = None
        errors = []
        lateral = []
        longitudinal = []
        distance = 0.0
        last = None
        record = []
        for t, x, y, v, yaw in truth:
            est = interpolate(rows, t)
            if est is None:
                continue
            if anchor is None:
                anchor = (x - est[0], y - est[1])
            dx, dy = est[0] + anchor[0] - x, est[1] + anchor[1] - y
            e = math.hypot(dx, dy)
            errors.append(e)
            lateral.append(abs(-math.sin(yaw) * dx + math.cos(yaw) * dy))
            longitudinal.append(abs(math.cos(yaw) * dx + math.sin(yaw) * dy))
            if last:
                distance += math.hypot(x - last[0], y - last[1])
            last = (x, y)
            record.append(
                dict(
                    stamp=t,
                    x=est[0] + anchor[0],
                    y=est[1] + anchor[1],
                    truth_x=x,
                    truth_y=y,
                    truth_yaw=yaw,
                    error_m=e,
                )
            )
        if not record:
            raise ValueError("no comparable timestamps for " + name)
        results[name] = dict(
            samples=len(errors),
            relative_error_m=stats(errors),
            lateral_abs_m=stats(lateral),
            longitudinal_abs_m=stats(longitudinal),
            final_error_m=errors[-1],
            seconds=record[-1]["stamp"] - record[0]["stamp"],
            native_travel_m=distance,
            anchor="one initial translation from evaluation truth; no GT yaw/scale or later correction",
        )
        traces[name] = record
        # One GPS fix before native evaluation begins, then a synthetic blackout.
        # The antenna lever is removed once. No later fix influences this variant.
        candidates = [
            g
            for g in gps
            if g["arrival"] <= record[0]["stamp"]
            and causal_point(rows, arrivals, g["t"], g["arrival"]) is not None
        ]
        if not candidates:
            candidates = [
                g
                for g in gps
                if g["arrival"] <= record[-1]["stamp"]
                and causal_point(rows, arrivals, g["t"], g["arrival"]) is not None
            ][:1]
        if candidates:
            g = candidates[-1]
            local = causal_point(rows, arrivals, g["t"], g["arrival"])
            heading_rows = [
                r for r in qrows if r[0] <= g["t"] and arrivals[r[0]] <= g["arrival"]
            ]
            heading = (
                heading_rows[-1][1:]
                if heading_rows and g["t"] - heading_rows[-1][0] <= 0.05
                else None
            )
            if heading is not None:
                yaw = heading[0] + convergence
                east, north = convert.transform(g["lon"], g["lat"])
                offset = (
                    east - g["east"] - 0.35 * math.cos(yaw) - local[0],
                    north - g["north"] - 0.35 * math.sin(yaw) - local[1],
                )
                absolute = []
                absolute_lateral = []
                absolute_longitudinal = []
                for r in record:
                    if r["stamp"] < g["arrival"]:
                        continue
                    state = interpolate(rows, r["stamp"])
                    dx = state[0] + offset[0] - r["truth_x"]
                    dy = state[1] + offset[1] - r["truth_y"]
                    yaw = r["truth_yaw"]
                    absolute.append(math.hypot(dx, dy))
                    absolute_lateral.append(
                        abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)
                    )
                    absolute_longitudinal.append(
                        abs(math.cos(yaw) * dx + math.sin(yaw) * dy)
                    )
                results[name]["single_gps_anchor_error_m"] = stats(absolute)
                results[name]["single_gps_anchor_lateral_abs_m"] = stats(
                    absolute_lateral
                )
                results[name]["single_gps_anchor_longitudinal_abs_m"] = stats(
                    absolute_longitudinal
                )
                results[name]["gps_anchor_stamp"] = g["t"]
                results[name][
                    "gps_anchor_note"
                ] = "causal past-state anchor with <=50ms bounded forward extrapolation; all later GPS withheld; source header is receive policy, acquisition latency not independently calibrated; SDK reference-point equivalence is not certified"
        else:
            results[name][
                "gps_anchor_note"
            ] = "unavailable: no causal valid GPS before evaluation"
    result = dict(
        tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        probe_binary_sha256=hashlib.sha256(a.probe.read_bytes()).hexdigest(),
        aimdrive_revision=(
            subprocess.check_output(
                ["git", "-C", str(a.aimdrive), "rev-parse", "HEAD"], text=True
            ).strip()
            if (a.aimdrive / ".git").exists()
            else json.loads((a.aimdrive / "baseline.json").read_text())["revision"]
        ),
        bag=str(a.bag),
        basis=basis,
        convergence_deg=math.degrees(convergence),
        motion_inputs=len(motion_inputs),
        rejected=dict(reject),
        cpp_epochs=dict(epochs),
        aim_failure=track.failure,
        aim_core_batch_ms=duration,
        max_native_speed_kph=max(r[3] for r in truth),
        results=results,
        limitations=[
            "offline motion feasibility only; no planner/control command published",
            "not a closed-loop map/frenet avoidance acceptance",
            "GT used only for scoring and one explicitly labelled relative translation",
            "constant convergence approximation for this short trajectory; no tunnel wall/map snapping",
        ],
    )
    (a.output / "summary.json").write_text(json.dumps(result, indent=2))
    (a.output / "paths.json").write_text(json.dumps(traces))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
