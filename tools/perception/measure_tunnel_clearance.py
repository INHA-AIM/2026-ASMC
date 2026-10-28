#!/usr/bin/env python3
"""Offline observed-paint to inner vertical-surface clearance, not a road ROI.

Uses scan-exact runtime transforms and the recorded LOCAL odometry. No model,
planner input or map is changed. Inferred/invalid family support is excluded.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import html
import json
from pathlib import Path

import numpy as np


def rotation(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


def quantiles(v):
    return (
        dict(
            zip(
                ("p05", "p50", "p95", "min", "max"),
                map(float, np.quantile(v, [0.05, 0.5, 0.95, 0, 1])),
            )
        )
        if len(v)
        else {}
    )


def interpolate_pose(left, right, fraction):
    """Interpolate a bracketed SE(2) sample without crossing the long yaw arc."""
    left, right = np.asarray(left), np.asarray(right)
    delta = right - left
    delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
    return left + fraction * delta


def wall_band(points, line, centre, sign):
    """Mode of offset from observed paint with vertical/longitudinal support."""
    x, y, z = points.T
    good = (abs(x - centre) <= 1) & (x >= line[0, 0]) & (x <= line[-1, 0])
    q = points[good]
    if len(q) < 30:
        return None
    gap = sign * (q[:, 1] - np.interp(q[:, 0], line[:, 0], line[:, 1]))
    valid = (gap > 0) & (gap < 3)
    q, gap = q[valid], gap[valid]
    if len(gap) < 30:
        return None
    hist, edges = np.histogram(gap, bins=np.arange(0, 3.051, 0.05))
    index = int(np.argmax(np.convolve(hist, [1, 1, 1], mode="same")))
    anchor = (edges[index] + edges[index + 1]) * 0.5
    inlier = abs(gap - anchor) <= 0.10
    surface = q[inlier]
    if len(surface) < 30 or np.ptp(surface[:, 0]) < 1.5 or np.ptp(surface[:, 2]) < 0.5:
        return None
    if len(set(np.floor(surface[:, 2] / 0.25).astype(int))) < 3:
        return None
    # Report range and residual; a thin pole/isolated OBB is not a wall model.
    estimate = float(np.median(gap[inlier]))
    return dict(
        gap_m=estimate,
        residual_p95_m=float(np.quantile(abs(gap[inlier] - estimate), 0.95)),
        points=int(inlier.sum()),
        x_span_m=float(np.ptp(surface[:, 0])),
        z_span_m=float(np.ptp(surface[:, 2])),
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--camera-config", type=Path, required=True)
    a = p.parse_args()
    if not any((parent / "registration.json").is_file() for parent in a.output.parents):
        raise ValueError("output must be within a new registered run")
    a.output.mkdir(exist_ok=False)
    import rosbag
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import cv2
    import yaml
    from asmc_camera.bev.calibration import (
        calibrations_from_dict,
        calibration_id,
        GroundPlane,
    )
    from dataclasses import replace

    cameras = calibrations_from_dict(yaml.safe_load(a.camera_config.read_text()))
    traces, families, contexts, odometry = {}, {}, {}, []
    required = {
        "/velodyne_points",
        "/perception/lidar/trace",
        "/perception/lane/boundary_family",
        "/perception/lane/ground_projection",
        "/planning/local/odometry",
    }
    with rosbag.Bag(str(a.bag)) as bag:
        missing_topics = sorted(required - set(bag.get_type_and_topic_info().topics))
    if missing_topics:
        report = dict(
            status="unmeasurable", missing_topics=missing_topics, bag=str(a.bag)
        )
        (a.output / "summary.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
        return
    with rosbag.Bag(str(a.bag)) as bag:
        for topic, m, _ in bag.read_messages(
            topics=[
                "/perception/lidar/trace",
                "/perception/lane/boundary_family",
                "/perception/lane/ground_projection",
                "/planning/local/odometry",
            ]
        ):
            if topic.endswith("/trace"):
                row = json.loads(m.data)
                traces[row["stamp_ns"]] = row
            elif topic.endswith("/boundary_family"):
                families[m.header.stamp.to_nsec()] = m
            elif topic.endswith("/ground_projection"):
                contexts[m.header.stamp.to_nsec()] = m
            else:
                q = m.pose.pose.orientation
                odometry.append(
                    (
                        m.header.stamp.to_sec(),
                        m.header.frame_id,
                        m.pose.pose.position.x,
                        m.pose.pose.position.y,
                        np.arctan2(
                            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                        ),
                    )
                )
    odometry.sort()
    times = [r[0] for r in odometry]

    def pose(t):
        i = bisect_left(times, t)
        if i == 0 or i == len(times):
            return None
        l, r = odometry[i - 1], odometry[i]
        if l[1] != r[1] or r[0] - l[0] > 0.1:
            return None
        f = (t - l[0]) / (r[0] - l[0])
        return l[1], interpolate_pose(l[2:], r[2:], f)

    stamps = sorted(set(families) & set(contexts))
    rows = []
    missing = Counter()
    cards = []
    overlays = []
    with rosbag.Bag(str(a.bag)) as bag:
        for _, cloud, _ in bag.read_messages(topics=["/velodyne_points"]):
            ns = cloud.header.stamp.to_nsec()
            trace = traces.get(ns)
            if not trace or not trace.get("processed"):
                missing["no_scan_exact_transform"] += 1
                continue
            if trace.get("map", {}).get("valid", True):
                missing["outside_gps_denied_section"] += 1
                continue
            i = bisect_left(stamps, ns)
            choices = stamps[max(0, i - 1) : i + 1]
            if not choices:
                missing["no_camera_context"] += 1
                continue
            fs = min(choices, key=lambda v: abs(v - ns))
            if abs(fs - ns) > 35000000:
                missing["camera_scan_skew"] += 1
                continue
            f, context = families[fs], contexts[fs]
            if not f.valid or not context.imu_corrected:
                missing["family_or_plane_invalid"] += 1
                continue
            sp, fp = pose(ns * 1e-9), pose(fs * 1e-9)
            if sp is None or fp is None or sp[0] != fp[0]:
                missing["motion_unavailable"] += 1
                continue
            if cloud.is_bigendian or cloud.row_step != cloud.width * cloud.point_step:
                missing["unsupported_cloud_layout"] += 1
                continue
            offsets = {field.name: field.offset for field in cloud.fields}
            dtype = np.dtype(
                dict(
                    names=["x", "y", "z"],
                    formats=["<f4"] * 3,
                    offsets=[offsets[k] for k in ("x", "y", "z")],
                    itemsize=cloud.point_step,
                )
            )
            raw = np.frombuffer(cloud.data, dtype=dtype)
            xyz = np.column_stack([raw[k] for k in ("x", "y", "z")])
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            # Runtime contract: base_from_input maps LEVELED sensor points.
            # Compose the recorded leveling first, so raw points get mount once.
            base = np.asarray(trace["base_from_input"]).reshape(4, 4)
            level = np.asarray(trace["leveling"]).reshape(3, 3)
            points = (xyz @ level.T) @ base[:3, :3].T + base[:3, 3]
            local = trace["local"]
            points = points @ rotation(local["roll_rad"], local["pitch_rad"]).T
            origin, target = sp[1], fp[1]
            angle = origin[2] - target[2]
            rot = np.array(
                [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
            )
            delta = origin[:2] - target[:2]
            c, s = np.cos(target[2]), np.sin(target[2])
            shift = np.array(
                [c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]]
            )
            points[:, :2] = points[:, :2] @ rot.T + shift
            front = list(context.views).index("front")
            aa, bb, cc = context.plane_coefficients[front * 3 : front * 3 + 3]
            R = rotation(np.arctan2(-bb, 1), np.arctan2(aa, np.sqrt(1 + bb * bb)))
            ground = cc / np.sqrt(1 + aa * aa + bb * bb)
            points = points[
                (points[:, 2] > ground + 0.25) & (points[:, 2] < ground + 1.8)
            ]
            lines = {}
            for member in f.boundaries:
                if member.role not in (1, 3):
                    continue
                # Separate runs: never interpolate across invalid/inferred paint.
                chunks = []
                current = []
                for q, state in zip(member.polyline, member.support_state):
                    if state == 1:
                        current.append([q.x, q.y, q.z])
                    else:
                        if len(current) >= 2:
                            chunks.append(np.asarray(current) @ R.T)
                        current = []
                if len(current) >= 2:
                    chunks.append(np.asarray(current) @ R.T)
                lines[member.role] = chunks
            sample = []
            for role, sign in ((1, 1), (3, -1)):
                for centre in (3.0, 5.0, 7.0, 9.0, 11.0, 13.0):
                    line = next(
                        (
                            q
                            for q in lines.get(role, [])
                            if q[0, 0] <= centre - 1 and q[-1, 0] >= centre + 1
                        ),
                        None,
                    )
                    if line is None:
                        missing["paint_not_observed_continuously"] += 1
                        continue
                    band = wall_band(points, line, centre, sign)
                    if band is None:
                        missing["vertical_surface_unresolved"] += 1
                        continue
                    row = dict(
                        stamp_ns=ns,
                        family_stamp_ns=fs,
                        source_skew_ms=(ns - fs) * 1e-6,
                        side="left" if role == 1 else "right",
                        x_m=centre,
                        **band,
                    )
                    rows.append(row)
                    sample.append(row)
            if (
                sample
                and len(cards) < 12
                and (not cards or ns - cards[-1]["stamp_ns"] > 2000000000)
            ):
                fig, ax = plt.subplots(figsize=(6, 8))
                ax.scatter(
                    points[:, 1], points[:, 0], s=0.5, c=points[:, 2], cmap="viridis"
                )
                for chunks in lines.values():
                    for line in chunks:
                        ax.plot(line[:, 1], line[:, 0], color="red", linewidth=2)
                for r in sample:
                    sign = 1 if r["side"] == "left" else -1
                    role = 1 if sign == 1 else 3
                    line = next(
                        q for q in lines[role] if q[0, 0] <= r["x_m"] <= q[-1, 0]
                    )
                    y = np.interp(r["x_m"], line[:, 0], line[:, 1])
                    ax.plot([y, y + sign * r["gap_m"]], [r["x_m"]] * 2, "k-")
                    ax.text(y, r["x_m"], f" {r['gap_m']:.2f}m", fontsize=8)
                ax.set(
                    xlim=(9, -6),
                    ylim=(0, 16),
                    xlabel="vehicle left y (m)",
                    ylabel="forward x (m)",
                    title="Observed family paint / LiDAR inner surface",
                )
                fig.tight_layout()
                name = "case-%02d.png" % len(cards)
                fig.savefig(a.output / name, dpi=120)
                plt.close(fig)
                cards.append(dict(stamp_ns=ns, family_stamp_ns=fs, image=name))
                overlays.append((len(cards) - 1, context, R, ground, lines, sample))
    # Ground-foot re-projection is for visual verification, not an independent
    # wall detector or an assumption that raised wall returns lie on the road.
    needed = {
        h.stamp.to_nsec() for _, ctx, _, _, _, _ in overlays for h in ctx.image_headers
    }
    rgb = {}
    topics = [
        "/image_jpeg/compressed",
        "/cam_left/image_jpeg/compressed",
        "/cam_right/image_jpeg/compressed",
    ]
    with rosbag.Bag(str(a.bag)) as bag:
        for topic, m, _ in bag.read_messages(topics=topics):
            if m.header.stamp.to_nsec() in needed:
                rgb[(topics.index(topic), m.header.stamp.to_nsec())] = m.data
    for index, ctx, R, ground, lines, sample in overlays:
        images = []
        for vi, view in enumerate(("front", "left", "right")):
            if view not in ctx.views:
                continue
            ci = list(ctx.views).index(view)
            data = rgb.get((vi, ctx.image_headers[ci].stamp.to_nsec()))
            if data is None:
                missing["overlay_exact_rgb_missing"] += 1
                continue
            if ctx.calibration_ids[ci] != calibration_id(cameras[view]):
                missing["overlay_calibration_mismatch"] += 1
                continue
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            coefficients = ctx.plane_coefficients[3 * ci : 3 * ci + 3]
            cam = replace(
                cameras[view],
                ground_plane=GroundPlane(
                    frame_id="base_link",
                    dz_dx=coefficients[0],
                    dz_dy=coefficients[1],
                    z_at_origin_m=coefficients[2],
                    source="recorded",
                ),
            )

            def pixels(q):
                base = np.column_stack([q[:, 0], q[:, 1], np.full(len(q), ground)]) @ R
                uv, depth = cam.project_ground(base[:, :2])
                valid = depth > 0.1
                valid &= (
                    (uv[:, 0] >= 0)
                    & (uv[:, 0] < image.shape[1])
                    & (uv[:, 1] >= 0)
                    & (uv[:, 1] < image.shape[0])
                )
                return uv, valid

            for chunks in lines.values():
                for line in chunks:
                    uv, visible = pixels(line[:, :2])
                    for k in range(1, len(uv)):
                        if visible[k - 1] and visible[k]:
                            cv2.line(
                                image,
                                tuple(uv[k - 1].astype(int)),
                                tuple(uv[k].astype(int)),
                                (0, 0, 255),
                                2,
                            )
            for r in sample:
                role = 1 if r["side"] == "left" else 3
                sign = 1 if role == 1 else -1
                line = next(q for q in lines[role] if q[0, 0] <= r["x_m"] <= q[-1, 0])
                x = r["x_m"]
                y = np.interp(x, line[:, 0], line[:, 1]) + sign * r["gap_m"]
                uv, visible = pixels(np.array([[x, y]]))
                if visible[0]:
                    cv2.circle(image, tuple(uv[0].astype(int)), 5, (255, 255, 0), -1)
            cv2.putText(
                image,
                view + " red=observed paint cyan=wall ground-foot",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
            filename = "rgb-%02d-%s.jpg" % (index, view)
            cv2.imwrite(str(a.output / filename), image)
            images.append(filename)
        cards[index]["rgb"] = images
    summary = dict(
        bag=str(a.bag),
        samples=len(rows),
        scans=len(set(r["stamp_ns"] for r in rows)),
        by_side={
            side: quantiles([r["gap_m"] for r in rows if r["side"] == side])
            for side in ("left", "right")
        },
        by_distance={
            f"{side}:{x}": quantiles(
                [r["gap_m"] for r in rows if r["side"] == side and r["x_m"] == x]
            )
            for side in ("left", "right")
            for x in (3.0, 5.0, 7.0, 9.0, 11.0, 13.0)
        },
        missing=dict(missing),
        cards=cards,
        limitations=[
            "Sensor-derived inner vertical surface, not independently validated collision mesh.",
            "Only continuous OBSERVED paint, GPS-denied frames and recorded odometry; no ROI/default adoption.",
            "Offline interpolation of recorded pose; no new planner localization or wall detector.",
            "Repeated scans share surfaces; counts are not independent ground-truth observations.",
        ],
    )
    (a.output / "measurements.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows)
    )
    (a.output / "summary.json").write_text(json.dumps(summary, indent=2))
    (a.output / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><h1>Tunnel paint / wall clearance</h1><p>Red: observed family paint. Points: 0.25–1.8m above nominal ground. Black: measured offset. Not a deployable free-space width.</p><pre>'
        + html.escape(json.dumps(summary, indent=2))
        + "</pre>"
        + "".join(
            '<article><img width="480" src="'
            + c["image"]
            + '">'
            + "".join('<img width="640" src="' + v + '">' for v in c.get("rgb", []))
            + "</article>"
            for c in cards
        )
    )
    print(
        json.dumps(
            {k: v for k, v in summary.items() if k not in ("cards", "by_distance")},
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
