#!/usr/bin/env python3
"""Compare map-frame LiDAR velocity vectors with MORAI /Object_topic GT.

The Main node publishes /lidar/tracked_objects_map in SI units.  The repository's
MORAI UDP bridge publishes ObjectStatus.velocity as km/h, despite the generic ROS
message carrying no unit annotation.  This tool makes both assumptions explicit
and configurable from the command line.
"""

import argparse
import bisect
import csv
import math
import os
import sys
from collections import Counter

import numpy as np

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag을 찾을 수 없습니다. ROS Noetic 환경을 source하세요.") from exc

try:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/asmc_matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit("matplotlib이 필요합니다: sudo apt install python3-matplotlib") from exc


GT_TOPIC = "/Object_topic"
LIDAR_TOPIC = "/lidar/tracked_objects_map"


def message_time(message, bag_time):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return stamp.to_sec()
    return bag_time.to_sec()


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def gt_velocity_mps(obj, unit, velocity_frame):
    scale = 1.0 / 3.6 if unit == "kmh" else 1.0
    raw_x = float(obj.velocity.x) * scale
    raw_y = float(obj.velocity.y) * scale
    raw_z = float(obj.velocity.z) * scale
    if velocity_frame == "map":
        return raw_x, raw_y, raw_z
    heading = math.radians(float(obj.heading))
    return raw_x * math.cos(heading), raw_x * math.sin(heading), raw_z


def gt_vehicles(message, args):
    objects = list(message.npc_list)
    if args.include_pedestrians:
        objects += list(message.pedestrian_list)
    result = []
    for obj in objects:
        if args.gt_id is not None and int(obj.unique_id) != args.gt_id:
            continue
        vx, vy, vz = gt_velocity_mps(
            obj, args.gt_velocity_unit, args.gt_velocity_frame)
        speed = math.hypot(vx, vy)
        if speed * 3.6 < args.min_gt_speed_kph:
            continue
        result.append({
            "id": int(obj.unique_id),
            "name": str(obj.name),
            "x": float(obj.position.x),
            "y": float(obj.position.y),
            "z": float(obj.position.z),
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "speed": speed,
        })
    return result


def lidar_objects(message, args):
    result = []
    for obj in message.objects:
        if args.track_id is not None and int(obj.id) != args.track_id:
            continue
        if int(obj.age) < args.min_track_age or int(obj.miss_count) != 0:
            continue
        result.append({
            "id": int(obj.id),
            "classification": str(obj.classification),
            "x": float(obj.obb_center.x),
            "y": float(obj.obb_center.y),
            "z": float(obj.obb_center.z),
            "vx": float(obj.velocity_mps.x),
            "vy": float(obj.velocity_mps.y),
            "vz": float(obj.velocity_mps.z),
            "speed": math.hypot(
                float(obj.velocity_mps.x), float(obj.velocity_mps.y)),
            "age": int(obj.age),
        })
    return result


def associate(gt_objects, tracked_objects, gate_m):
    """Deterministic one-to-one nearest-position association inside a gate."""
    candidates = []
    for gi, gt in enumerate(gt_objects):
        for li, lidar in enumerate(tracked_objects):
            distance = math.hypot(gt["x"] - lidar["x"], gt["y"] - lidar["y"])
            if distance <= gate_m:
                candidates.append((distance, gi, li))
    candidates.sort()
    used_gt = set()
    used_lidar = set()
    matches = []
    for distance, gi, li in candidates:
        if gi in used_gt or li in used_lidar:
            continue
        used_gt.add(gi)
        used_lidar.add(li)
        matches.append((gt_objects[gi], tracked_objects[li], distance))
    return matches


def nearest_index(times, target):
    index = bisect.bisect_left(times, target)
    choices = []
    if index < len(times):
        choices.append(index)
    if index > 0:
        choices.append(index - 1)
    if not choices:
        return None
    return min(choices, key=lambda i: abs(times[i] - target))


def load_messages(bag_path, time_source):
    gt_raw = []
    lidar_raw = []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, bag_time in bag.read_messages(
                topics=[GT_TOPIC, LIDAR_TOPIC]):
            header_time = message_time(message, bag_time)
            item = (header_time, bag_time.to_sec(), message)
            if topic == GT_TOPIC:
                gt_raw.append(item)
            else:
                lidar_raw.append(item)

    resolved_source = time_source
    if time_source == "auto":
        # A live Main node can stamp its output with a simulator/GPS clock while
        # the UDP GT bridge stamps with ROS wall time.  rosbag receipt time is
        # the only common clock in that case.
        if gt_raw and lidar_raw:
            gt_offset = float(np.median([
                header - bag for header, bag, _ in gt_raw]))
            lidar_offset = float(np.median([
                header - bag for header, bag, _ in lidar_raw]))
            resolved_source = (
                "header" if abs(gt_offset - lidar_offset) <= 1.0 else "bag")
        else:
            resolved_source = "bag"

    index = 0 if resolved_source == "header" else 1
    gt = [(item[index], item[2]) for item in gt_raw]
    lidar = [(item[index], item[2]) for item in lidar_raw]
    gt.sort(key=lambda item: item[0])
    lidar.sort(key=lambda item: item[0])
    return gt, lidar, resolved_source


def make_rows(gt_messages, lidar_messages, args):
    gt_times = [item[0] for item in gt_messages]
    rows = []
    for lidar_time, lidar_message in lidar_messages:
        target_gt_time = lidar_time + args.time_offset_sec
        gt_index = nearest_index(gt_times, target_gt_time)
        if gt_index is None:
            continue
        gt_time, gt_message = gt_messages[gt_index]
        sync_dt = gt_time - lidar_time
        if abs(sync_dt - args.time_offset_sec) > args.max_sync_gap:
            continue
        ground_truth = gt_vehicles(gt_message, args)
        tracked = lidar_objects(lidar_message, args)
        for gt, lidar, distance in associate(
                ground_truth, tracked, args.association_gate):
            error_e = lidar["vx"] - gt["vx"]
            error_n = lidar["vy"] - gt["vy"]
            vector_error = math.hypot(error_e, error_n)
            speed_error = lidar["speed"] - gt["speed"]
            direction_error_deg = float("nan")
            if gt["speed"] >= args.direction_min_speed:
                direction_error_deg = math.degrees(wrap_pi(
                    math.atan2(lidar["vy"], lidar["vx"]) -
                    math.atan2(gt["vy"], gt["vx"])))
            rows.append({
                "timestamp": lidar_time,
                "sync_dt_sec": sync_dt,
                "gt_id": gt["id"],
                "gt_name": gt["name"],
                "track_id": lidar["id"],
                "track_class": lidar["classification"],
                "track_age": lidar["age"],
                "association_distance_m": distance,
                "gt_x": gt["x"],
                "gt_y": gt["y"],
                "lidar_x": lidar["x"],
                "lidar_y": lidar["y"],
                "gt_v_e_mps": gt["vx"],
                "gt_v_n_mps": gt["vy"],
                "lidar_v_e_mps": lidar["vx"],
                "lidar_v_n_mps": lidar["vy"],
                "error_e_mps": error_e,
                "error_n_mps": error_n,
                "vector_error_mps": vector_error,
                "gt_speed_kph": gt["speed"] * 3.6,
                "lidar_speed_kph": lidar["speed"] * 3.6,
                "speed_error_kph": speed_error * 3.6,
                "direction_error_deg": direction_error_deg,
            })
    return rows


def metric_summary(rows):
    e_e = np.asarray([row["error_e_mps"] for row in rows])
    e_n = np.asarray([row["error_n_mps"] for row in rows])
    vector = np.asarray([row["vector_error_mps"] for row in rows])
    speed_kph = np.asarray([row["speed_error_kph"] for row in rows])
    direction = np.asarray([
        row["direction_error_deg"] for row in rows
        if math.isfinite(row["direction_error_deg"])
    ])
    summary = {
        "samples": len(rows),
        "e_mae_mps": float(np.mean(np.abs(e_e))),
        "e_rmse_mps": float(np.sqrt(np.mean(e_e ** 2))),
        "e_bias_mps": float(np.mean(e_e)),
        "n_mae_mps": float(np.mean(np.abs(e_n))),
        "n_rmse_mps": float(np.sqrt(np.mean(e_n ** 2))),
        "n_bias_mps": float(np.mean(e_n)),
        "vector_mae_mps": float(np.mean(vector)),
        "vector_p95_mps": float(np.percentile(vector, 95)),
        "speed_mae_kph": float(np.mean(np.abs(speed_kph))),
        "speed_rmse_kph": float(np.sqrt(np.mean(speed_kph ** 2))),
        "speed_bias_kph": float(np.mean(speed_kph)),
        "direction_mae_deg": (
            float(np.mean(np.abs(direction))) if direction.size else float("nan")),
    }
    return summary


def write_summary(path, summary, args, selected_pair):
    with open(path, "w", encoding="utf-8") as output:
        output.write("LiDAR velocity vector vs MORAI GT\n")
        output.write(f"GT velocity unit: {args.gt_velocity_unit}\n")
        output.write(f"GT velocity frame: {args.gt_velocity_frame}\n")
        output.write(f"Time source: {args.resolved_time_source}\n")
        output.write(f"Selected plot pair (gt, track): {selected_pair}\n\n")
        for key, value in summary.items():
            if isinstance(value, float):
                output.write(f"{key}: {value:.6f}\n")
            else:
                output.write(f"{key}: {value}\n")


def make_plots(rows, output_dir):
    pair_counts = Counter((row["gt_id"], row["track_id"]) for row in rows)
    selected_pair, _ = pair_counts.most_common(1)[0]
    selected = [
        row for row in rows
        if (row["gt_id"], row["track_id"]) == selected_pair
    ]
    selected.sort(key=lambda row: row["timestamp"])
    t0 = selected[0]["timestamp"]
    time_sec = np.asarray([row["timestamp"] - t0 for row in selected])

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    axes[0].plot(time_sec, [row["gt_speed_kph"] for row in selected], label="GT")
    axes[0].plot(time_sec, [row["lidar_speed_kph"] for row in selected], label="LiDAR")
    axes[0].set_ylabel("Speed [km/h]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time_sec, [row["gt_v_e_mps"] for row in selected], label="GT vE")
    axes[1].plot(time_sec, [row["lidar_v_e_mps"] for row in selected], label="LiDAR vE")
    axes[1].set_ylabel("East [m/s]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(time_sec, [row["gt_v_n_mps"] for row in selected], label="GT vN")
    axes[2].plot(time_sec, [row["lidar_v_n_mps"] for row in selected], label="LiDAR vN")
    axes[2].set_ylabel("North [m/s]")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(time_sec, [row["vector_error_mps"] for row in selected])
    axes[3].set_ylabel("Vector error [m/s]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True)
    fig.suptitle(f"GT {selected_pair[0]} vs LiDAR track {selected_pair[1]}")
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "velocity_timeseries.png"), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].scatter(
        [row["gt_v_e_mps"] for row in rows],
        [row["lidar_v_e_mps"] for row in rows], s=8, alpha=0.5, label="vE")
    axes[0].scatter(
        [row["gt_v_n_mps"] for row in rows],
        [row["lidar_v_n_mps"] for row in rows], s=8, alpha=0.5, label="vN")
    limits = axes[0].get_xlim()
    low = min(limits[0], axes[0].get_ylim()[0])
    high = max(limits[1], axes[0].get_ylim()[1])
    axes[0].plot([low, high], [low, high], "k--", linewidth=1)
    axes[0].set_xlabel("GT component [m/s]")
    axes[0].set_ylabel("LiDAR component [m/s]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].hist([row["vector_error_mps"] for row in rows], bins=40)
    axes[1].set_xlabel("Velocity vector error [m/s]")
    axes[1].set_ylabel("Samples")
    axes[1].grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "velocity_error_summary.png"), dpi=160)
    plt.close(fig)
    return selected_pair


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag file")
    parser.add_argument("--output-dir")
    parser.add_argument("--gt-velocity-unit", choices=("kmh", "mps"), default="kmh")
    parser.add_argument(
        "--gt-velocity-frame", choices=("map", "heading"), default="heading",
        help="map: x/y are map components; heading: x is forward speed")
    parser.add_argument(
        "--time-source", choices=("auto", "header", "bag"), default="auto",
        help="auto: use header stamps only when GT/LiDAR share one clock")
    parser.add_argument("--association-gate", type=float, default=5.0)
    parser.add_argument("--max-sync-gap", type=float, default=0.10)
    parser.add_argument("--time-offset-sec", type=float, default=0.0)
    parser.add_argument("--min-track-age", type=int, default=3)
    parser.add_argument("--min-gt-speed-kph", type=float, default=1.0)
    parser.add_argument("--direction-min-speed", type=float, default=1.0)
    parser.add_argument("--gt-id", type=int, help="compare only this MORAI unique_id")
    parser.add_argument("--track-id", type=int, help="compare only this LiDAR track ID")
    parser.add_argument("--include-pedestrians", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if not os.path.isfile(args.bag):
        raise SystemExit(f"bag 파일이 없습니다: {args.bag}")
    output_dir = args.output_dir or os.path.splitext(args.bag)[0] + "_velocity_analysis"
    os.makedirs(output_dir, exist_ok=True)

    gt_messages, lidar_messages, resolved_time_source = load_messages(
        args.bag, args.time_source)
    args.resolved_time_source = resolved_time_source
    if not gt_messages:
        raise SystemExit(f"{GT_TOPIC} 메시지가 bag에 없습니다.")
    if not lidar_messages:
        raise SystemExit(f"{LIDAR_TOPIC} 메시지가 bag에 없습니다.")

    rows = make_rows(gt_messages, lidar_messages, args)
    if not rows:
        raise SystemExit(
            "매칭된 샘플이 없습니다. topic, 시간 동기, map 좌표 및 association gate를 확인하세요.")

    csv_path = os.path.join(output_dir, "velocity_samples.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    selected_pair = make_plots(rows, output_dir)
    summary = metric_summary(rows)
    summary_path = os.path.join(output_dir, "summary.txt")
    write_summary(summary_path, summary, args, selected_pair)

    print(f"시간 기준: {resolved_time_source}")
    print(f"분석 완료: {len(rows)} samples")
    print(f"CSV: {csv_path}")
    print(f"요약: {summary_path}")
    print(f"그래프: {os.path.join(output_dir, 'velocity_timeseries.png')}")
    print(f"벡터 오차 MAE: {summary['vector_mae_mps']:.3f} m/s")
    print(f"속력 오차 MAE: {summary['speed_mae_kph']:.3f} km/h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
