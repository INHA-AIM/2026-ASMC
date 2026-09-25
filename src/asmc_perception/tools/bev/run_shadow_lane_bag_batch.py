#!/usr/bin/env python3
"""Replay a curated shadow-run matrix through the headless lane pipeline.

The raw bags remain immutable. Each run receives an independent result bag and
the batch audit records selection lineage plus final structured diagnostics.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from asmc_perception.offline.bev.bag_batch import (
    diagnostic_values,
    load_bag_selection,
    result_quality_gates,
    sha256_file,
)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument(
        "--model-id", default="twinlite_v6_shadow_boundary_typed_v1"
    )
    parser.add_argument("--rosbag-root", default="/data/rosbags")
    parser.add_argument(
        "--scenarios",
        default="shadow_clear,shadow_static_avoid_v1",
        help="comma-separated scenarios from the selection manifest",
    )
    parser.add_argument("--only-run-id", action="append", default=[])
    parser.add_argument("--max-runs", type=int)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--topic-timeout-sec", type=float, default=30.0)
    parser.add_argument("--min-free-gb", type=float, default=5.0)
    parser.add_argument(
        "--result-profile",
        default="shadow_boundary_result",
        help=(
            "result bag allowlist; use shadow_runtime_profile_result for a "
            "headless timing run without Marker subscribers"
        ),
    )
    parser.add_argument(
        "--external-pipeline",
        action="store_true",
        help=(
            "reuse an already running inference/BEV/lane pipeline; useful when "
            "the GPU container has read-only /data and the ROS container records"
        ),
    )
    parser.add_argument(
        "--enable-shared-family-debug",
        action="store_true",
        help="record the evaluated shared-family debug sidecar as a runtime baseline",
    )
    parser.add_argument(
        "--enable-boundary-family-candidate",
        action="store_true",
        help=(
            "publish and record the typed /perception/lane/boundary_family "
            "candidate without requiring Marker visualization"
        ),
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--summary-out")
    return parser.parse_args()


def write_yaml_atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=False)
    temporary.replace(path)


def terminate(process, timeout=20.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def start_process(command, log_path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=stream,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        start_new_session=True,
    )
    process._asmc_log_stream = stream
    return process


def close_process_log(process):
    if process is not None and hasattr(process, "_asmc_log_stream"):
        process._asmc_log_stream.close()


def wait_for(predicate, timeout, description, processes=()):
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        for process in processes:
            if process is not None and process.poll() is not None:
                raise RuntimeError(
                    "{} exited with rc={} while waiting for {}".format(
                        process.args[1] if len(process.args) > 1 else process.args[0],
                        process.returncode,
                        description,
                    )
                )
        if predicate():
            return
        time.sleep(0.2)
    raise RuntimeError("timed out waiting for {}".format(description))


def metadata_status(path):
    if not path.is_file():
        return None
    try:
        with path.open("r", encoding="utf-8") as stream:
            return (yaml.safe_load(stream) or {}).get("status")
    except (OSError, yaml.YAMLError):
        return None


def result_summary(item, result_root, elapsed_s, require_boundary_family=False):
    metadata_path = result_root / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    camera = diagnostic_values(metadata, "camera_semantic_perception/inference")
    bev = diagnostic_values(metadata, "drivable_bev/fusion")
    lane = diagnostic_values(metadata, "drivable_bev/lane_vectorizer")
    shared = diagnostic_values(
        metadata, "drivable_bev/shared_lane_family_debug"
    )
    gates = result_quality_gates(
        metadata, require_boundary_family=require_boundary_family
    )
    return {
        "run_id": item.run_id,
        "scenario": item.scenario,
        "weather": item.weather,
        "sim_hour": item.sim_hour,
        "recorded_controller": item.recorded_controller,
        "effective_controller": item.effective_controller,
        "status": metadata.get("status"),
        "elapsed_wall_s": round(float(elapsed_s), 3),
        "result_metadata": str(metadata_path),
        "result_bags": metadata.get("bags") or [],
        "camera_diagnostics": camera,
        "bev_diagnostics": bev,
        "lane_diagnostics": lane,
        "shared_family_diagnostics": shared,
        "topic_statistics": metadata.get("topic_statistics") or {},
        "quality_gates": gates,
        "quality_gate_passed": all(gates.values()),
    }


def commands_for(item, args):
    replay = [
        "roslaunch", "data_collection", "bag_replay.launch",
        "bag_path:={}".format(item.raw_path),
        "required_profile:=camera_raw",
        "rate:={}".format(args.rate),
        "wait_for_start:=true",
        "post_roll_sec:=2.0",
    ]
    pipeline = [
        "roslaunch", "asmc_perception", "perception.launch",
        "start_camera_lane:=true",
        "start_traffic_light:=false",
        "start_visualization:=false",
        "camera_checkpoint:={}".format(Path(args.checkpoint).expanduser().resolve()),
        "camera_device:={}".format(args.device),
        "lane_enable_shared_family_debug:={}".format(
            "true" if args.enable_shared_family_debug else "false"
        ),
        "lane_enable_boundary_family_candidate:={}".format(
            "true" if args.enable_boundary_family_candidate else "false"
        ),
    ]
    recorder = [
        "roslaunch", "data_collection", "bag_recorder.launch",
        "profile:={}".format(args.result_profile),
        "parent_raw_run_id:={}".format(item.run_id),
        "model_id:={}".format(args.model_id),
        "checkpoint:={}".format(Path(args.checkpoint).expanduser().resolve()),
        "topic_timeout_sec:={}".format(args.topic_timeout_sec),
        "min_free_gb:={}".format(args.min_free_gb),
    ]
    return replay, pipeline, recorder


def main():
    args = parse_args()
    # The original debug switch remains a compatibility alias for the typed
    # candidate output. The new switch permits headless typed output without
    # Marker rendering.
    args.enable_boundary_family_candidate = bool(
        args.enable_boundary_family_candidate or args.enable_shared_family_debug
    )
    if args.rate <= 0.0 or args.topic_timeout_sec <= 0.0:
        raise SystemExit("rate and topic timeout must be positive")
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    if not checkpoint.is_file():
        raise SystemExit("checkpoint not found: {}".format(checkpoint))
    scenarios = [value.strip() for value in args.scenarios.split(",") if value.strip()]
    _, selected = load_bag_selection(args.selection, args.rosbag_root, scenarios)
    requested_ids = set(args.only_run_id)
    if requested_ids:
        selected = tuple(item for item in selected if item.run_id in requested_ids)
        missing = requested_ids.difference(item.run_id for item in selected)
        if missing:
            raise SystemExit("requested run IDs are not selected: {}".format(sorted(missing)))
    if args.max_runs is not None:
        if args.max_runs <= 0:
            raise SystemExit("max-runs must be positive")
        selected = selected[: args.max_runs]
    if not selected:
        raise SystemExit("no runs selected")

    selection_path = Path(args.selection).expanduser().resolve()
    default_summary = (
        selection_path.parent
        / "{}_batch_summary.yaml".format(args.model_id)
    )
    summary_path = Path(args.summary_out).expanduser().resolve() if args.summary_out else default_summary
    rosbag_root = Path(args.rosbag_root).expanduser().resolve()
    document = {
        "schema_version": "asmc-shadow-lane-batch-1.0.0",
        "status": "planned" if args.dry_run else "running",
        "started_at_utc": utc_now(),
        "selection": {
            "path": str(selection_path),
            "sha256": sha256_file(selection_path),
        },
        "checkpoint": {"path": str(checkpoint), "sha256": sha256_file(checkpoint)},
        "model_id": args.model_id,
        "rate": args.rate,
        "external_pipeline": args.external_pipeline,
        "shared_family_debug_enabled": args.enable_shared_family_debug,
        "boundary_family_candidate_enabled": args.enable_boundary_family_candidate,
        "result_profile": args.result_profile,
        "runs": [],
    }
    plan = []
    for item in selected:
        replay, pipeline, recorder = commands_for(item, args)
        plan.append(
            {
                "run_id": item.run_id,
                "condition": [item.scenario, item.weather, item.sim_hour],
                "commands": [replay, pipeline, recorder],
            }
        )
    if args.dry_run:
        document["plan"] = plan
        document["finished_at_utc"] = utc_now()
        write_yaml_atomic(summary_path, document)
        print(yaml.safe_dump(document, allow_unicode=True, sort_keys=False))
        return 0

    import rosgraph
    import rospy
    from std_srvs.srv import Trigger

    try:
        rosgraph.Master("/shadow_lane_batch").getPid()
    except Exception as exc:
        raise SystemExit("ROS master is unavailable: {}".format(exc))
    rospy.init_node("shadow_lane_batch", anonymous=True, disable_signals=True)
    write_yaml_atomic(summary_path, document)
    logs = summary_path.parent / (summary_path.stem + "_logs")

    try:
        for index, item in enumerate(selected, 1):
            result_root = rosbag_root / "results" / item.run_id / args.model_id
            metadata_path = result_root / "metadata.yaml"
            if result_root.exists():
                if args.resume and metadata_status(metadata_path) == "complete":
                    value = result_summary(
                        item,
                        result_root,
                        0.0,
                        require_boundary_family=args.enable_boundary_family_candidate,
                    )
                    value["batch_action"] = "skipped_complete"
                    document["runs"].append(value)
                    write_yaml_atomic(summary_path, document)
                    continue
                raise RuntimeError(
                    "result already exists; use a new model-id or --resume: {}".format(result_root)
                )

            replay_command, pipeline_command, recorder_command = commands_for(item, args)
            print("[{}/{}] {}".format(index, len(selected), item.run_id), flush=True)
            replay = pipeline = recorder = None
            started = time.monotonic()
            try:
                replay = start_process(replay_command, logs / (item.run_id + "_replay.log"))
                rospy.wait_for_service("/asmc_bag_replay/start", timeout=args.topic_timeout_sec)
                if not args.external_pipeline:
                    pipeline = start_process(
                        pipeline_command, logs / (item.run_id + "_pipeline.log")
                    )

                def pipeline_ready():
                    published = dict(rospy.get_published_topics())
                    required = {
                        "/perception/camera/diagnostics",
                        "/perception/bev/diagnostics",
                        "/perception/lane/boundary_observations",
                        "/perception/lane/diagnostics",
                    }
                    if args.enable_shared_family_debug:
                        required.add(
                            "/perception/lane/debug/shared_family/diagnostics"
                        )
                    if args.enable_boundary_family_candidate:
                        required.add("/perception/lane/boundary_family")
                    return required.issubset(published)

                wait_for(
                    pipeline_ready,
                    args.topic_timeout_sec,
                    "headless lane pipeline topics",
                    (replay, pipeline),
                )
                recorder = start_process(recorder_command, logs / (item.run_id + "_recorder.log"))
                wait_for(
                    lambda: metadata_status(metadata_path) == "recording",
                    args.topic_timeout_sec,
                    "result recorder metadata",
                    (replay, pipeline, recorder),
                )
                start = rospy.ServiceProxy("/asmc_bag_replay/start", Trigger)
                response = start()
                if not response.success:
                    raise RuntimeError("replay start rejected: {}".format(response.message))
                timeout = item.duration_s / args.rate + 90.0
                try:
                    replay.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    raise RuntimeError("replay exceeded {:.1f}s timeout".format(timeout))
                if replay.returncode != 0:
                    raise RuntimeError("replay exited with rc={}".format(replay.returncode))
                terminate(recorder)
                if not args.external_pipeline:
                    terminate(pipeline)
                if metadata_status(metadata_path) != "complete":
                    raise RuntimeError("result recorder did not finalize metadata")
                value = result_summary(
                    item,
                    result_root,
                    time.monotonic() - started,
                    require_boundary_family=args.enable_boundary_family_candidate,
                )
                value["batch_action"] = "recorded"
                document["runs"].append(value)
                write_yaml_atomic(summary_path, document)
            except Exception as exc:
                document["runs"].append(
                    {
                        "run_id": item.run_id,
                        "status": "failed",
                        "error": "{}: {}".format(type(exc).__name__, exc),
                    }
                )
                document["status"] = "failed"
                document["finished_at_utc"] = utc_now()
                write_yaml_atomic(summary_path, document)
                raise
            finally:
                terminate(recorder)
                if not args.external_pipeline:
                    terminate(pipeline)
                terminate(replay)
                close_process_log(recorder)
                close_process_log(pipeline)
                close_process_log(replay)
                time.sleep(0.5)
    except KeyboardInterrupt:
        document["status"] = "interrupted"
        document["finished_at_utc"] = utc_now()
        write_yaml_atomic(summary_path, document)
        return 130
    except Exception as exc:
        print("batch failed: {}".format(exc), file=sys.stderr)
        return 2

    document["status"] = (
        "complete"
        if all(row.get("quality_gate_passed", False) for row in document["runs"])
        else "complete_with_gate_failures"
    )
    document["finished_at_utc"] = utc_now()
    write_yaml_atomic(summary_path, document)
    print("batch {}: {}".format(document["status"], summary_path))
    return 0 if document["status"] == "complete" else 3


if __name__ == "__main__":
    raise SystemExit(main())
