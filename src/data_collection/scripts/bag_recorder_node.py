#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Profile-based rosbag recorder with immutable raw/result lineage."""

from __future__ import print_function

import glob
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone

import rosgraph
import rospy
import rospkg
import yaml
from diagnostic_msgs.msg import DiagnosticArray

from data_collection.bag_profile import BagProfileError, load_bag_profile
from data_collection.bag_run import (
    build_rosbag_command,
    default_rosbag_root,
    make_bag_run_id,
    missing_recorded_topics,
    recorded_topic_counts,
    sha256_file,
    slug,
)
from data_collection.capture_grpc import resolve_proto_root
from data_collection.environment_grpc import MoraiEnvironmentClient


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_yaml_atomic(path, value):
    temporary = path + ".tmp"
    with open(temporary, "w") as stream:
        yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=True)
    os.replace(temporary, path)


def git_metadata(repo_root):
    def run(*args):
        git_dir = os.path.join(repo_root, ".git")
        return subprocess.check_output(
            ["git", "--git-dir={}".format(git_dir),
             "--work-tree={}".format(repo_root)] + list(args),
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()

    result = {"root": repo_root, "commit": "unknown", "branch": "unknown"}
    try:
        result["commit"] = run("rev-parse", "HEAD")
        result["branch"] = run("rev-parse", "--abbrev-ref", "HEAD")
        result["dirty"] = bool(run("status", "--porcelain"))
    except (OSError, subprocess.CalledProcessError) as exc:
        result["error"] = "{}: {}".format(type(exc).__name__, exc)
        result["dirty"] = None
    return result


class TopicCounter(object):
    def __init__(self, topics):
        self._lock = threading.Lock()
        self._values = {
            topic: {
                "messages": 0,
                "first_wall": None,
                "last_wall": None,
                "start_window": [],
                "end_window": deque(maxlen=512),
            }
            for topic in topics
        }
        self._latest_diagnostics = {}
        self._subscribers = [
            rospy.Subscriber(
                topic,
                DiagnosticArray if topic.endswith("/diagnostics") else rospy.AnyMsg,
                self._callback,
                callback_args=topic, queue_size=1, buff_size=1 << 24
            )
            for topic in topics
        ]

    def _callback(self, _message, topic):
        now = time.monotonic()
        with self._lock:
            value = self._values[topic]
            value["messages"] += 1
            if value["first_wall"] is None:
                value["first_wall"] = now
            value["last_wall"] = now
            if now - value["first_wall"] <= 5.0:
                value["start_window"].append(now)
            value["end_window"].append(now)
            while value["end_window"] and now - value["end_window"][0] > 5.0:
                value["end_window"].popleft()
            if topic.endswith("/diagnostics") and hasattr(_message, "status"):
                self._latest_diagnostics[topic] = {
                    status.name: {
                        "level": int(status.level),
                        "message": status.message,
                        "values": dict((item.key, item.value) for item in status.values),
                    }
                    for status in _message.status
                }

    def summary(self):
        def window_rate(values):
            if len(values) < 2 or values[-1] <= values[0]:
                return 0.0
            return (len(values) - 1) / (values[-1] - values[0])

        result = {}
        with self._lock:
            values = {}
            for key, value in self._values.items():
                copied = dict(value)
                copied["start_window"] = list(value["start_window"])
                copied["end_window"] = list(value["end_window"])
                values[key] = copied
        for topic, value in values.items():
            duration = None
            overall_rate = 0.0
            if value["first_wall"] is not None and value["last_wall"] is not None:
                duration = max(0.0, value["last_wall"] - value["first_wall"])
                if value["messages"] > 1 and duration > 0:
                    overall_rate = (value["messages"] - 1) / duration
            result[topic] = {
                "messages": value["messages"],
                "observed_seconds": None if duration is None else round(duration, 3),
                "average_hz": round(overall_rate, 3),
                "start_window_hz": round(
                    window_rate(value["start_window"]), 3
                ),
                "end_window_hz": round(window_rate(value["end_window"]), 3),
            }
        return result

    def diagnostics(self):
        with self._lock:
            return dict(self._latest_diagnostics)

    def close(self):
        for subscriber in self._subscribers:
            subscriber.unregister()


class BagRecorderNode(object):
    def __init__(self):
        self.package_path = rospkg.RosPack().get_path("data_collection")
        self.repo_root = os.path.abspath(os.path.join(self.package_path, "..", ".."))
        profile_dir = os.path.join(self.package_path, "config", "bag_profiles")
        self.profile = load_bag_profile(
            profile_dir, str(rospy.get_param("~profile", "full_raw"))
        )
        self.profile_name = self.profile["name"]
        self.process = None
        self.counter = None
        self.environment_client = None
        self.environment_thread = None
        self.environment_stop = threading.Event()
        self._stop_lock = threading.Lock()
        self._finalized = False
        self._requested_stop = False

        scenario_value = str(rospy.get_param("~scenario", "")).strip()
        controller_value = str(rospy.get_param("~controller", "")).strip()
        self.scenario = slug(scenario_value, fallback="scenario")
        self.controller = slug(controller_value, fallback="controller")
        self.map_slug = slug(
            rospy.get_param("~map_slug", self.profile.get("map", "kcity2025")),
            fallback="kcity2025",
        )
        self.root = os.path.abspath(
            os.path.expanduser(rospy.get_param("~rosbag_root", "") or default_rosbag_root())
        )
        self.topic_timeout = float(rospy.get_param("~topic_timeout_sec", 10.0))
        self.min_free_gb = float(rospy.get_param("~min_free_gb", 20.0))
        self.split_gb = float(rospy.get_param("~split_gb", 4.0))
        self.buffer_mb = int(rospy.get_param("~buffer_mb", 1024))
        self.environment_poll_sec = float(
            rospy.get_param("~environment_poll_sec", 2.0)
        )

        self.parent_raw_run_id = str(
            rospy.get_param("~parent_raw_run_id", "")
        ).strip()
        self.parent_metadata = None
        model_value = str(rospy.get_param("~model_id", "")).strip()
        self.model_id = slug(model_value, fallback="model", max_length=80)
        self.checkpoint = str(rospy.get_param("~checkpoint", "")).strip()
        if self.profile["kind"] == "raw" and (
            not scenario_value or not controller_value
        ):
            raise BagProfileError(
                "raw recording requires explicit scenario and controller"
            )
        if self.profile["kind"] == "result" and not model_value:
            raise BagProfileError("result recording requires explicit model_id")
        if self.profile["kind"] == "result" and not self.checkpoint:
            raise BagProfileError("result recording requires explicit checkpoint")
        if self.profile["kind"] == "result" and not os.path.isfile(self.checkpoint):
            raise BagProfileError(
                "result checkpoint not found: {}".format(self.checkpoint)
            )

        # Do not keep a gRPC channel alive across the rosbag subprocess fork.
        # A one-shot client obtains run naming metadata and is closed immediately;
        # the timeline client is opened only after rosbag has started.
        initial_environment = (
            self._environment_snapshot_once()
            if self.profile["kind"] == "raw"
            else {"weather": "from_parent", "sim_hour": None,
                  "simulator_version": "from_parent"}
        )
        weather_override = str(rospy.get_param("~weather", "")).strip()
        sim_hour_override = str(rospy.get_param("~sim_hour", "")).strip()
        weather = weather_override or initial_environment.get("weather", "unknown")
        sim_hour = (
            int(sim_hour_override) if sim_hour_override
            else initial_environment.get("sim_hour")
        )
        self.naming_environment = {
            "weather": weather,
            "weather_source": "launch_override" if weather_override else "grpc",
            "sim_hour": sim_hour,
            "sim_hour_source": "launch_override" if sim_hour_override else "grpc",
        }

        requested_run_id = str(rospy.get_param("~run_id", "")).strip()
        auto_run_id = make_bag_run_id(
            self.map_slug, self.scenario, weather, sim_hour, self.controller
        )
        self.run_id = slug(requested_run_id, fallback=auto_run_id, max_length=120) \
            if requested_run_id else auto_run_id
        if self.profile["kind"] == "result":
            parent_metadata = self._parent_metadata()
            self.parent_metadata = parent_metadata
            self.run_id = self.model_id
            self.scenario = slug(parent_metadata.get("scenario"), fallback="unknown")
            self.controller = slug(parent_metadata.get("controller"), fallback="replay")
            self.map_slug = slug(parent_metadata.get("map"), fallback=self.map_slug)
            initial_environment = parent_metadata.get(
                "environment_initial", initial_environment
            )
            self.naming_environment = parent_metadata.get(
                "naming_environment", self.naming_environment
            )

        self.run_root = self._resolve_run_root()
        self.bags_dir = os.path.join(self.run_root, "bags")
        self.metadata_path = os.path.join(self.run_root, "metadata.yaml")
        self.timeline_path = os.path.join(
            self.run_root, "environment_timeline.jsonl"
        )

        published = self._wait_for_topics()
        topics = list(self.profile["required_topics"])
        topics.extend(
            topic for topic in self.profile["optional_topics"] if topic in published
        )
        self._reject_duplicate_publishers(topics)
        self._validate_timestamp_contract()
        self._prepare_storage()
        self.recorded_topics = topics
        self.topic_types = dict((topic, published[topic]) for topic in topics)
        self.counter = TopicCounter(topics)

        output_name = self.model_id if self.profile["kind"] == "result" else self.run_id
        output_prefix = os.path.join(self.bags_dir, output_name)
        self.command = build_rosbag_command(
            self.profile, output_prefix, topics,
            split_gb=self.split_gb, buffer_mb=self.buffer_mb
        )
        self.metadata = self._initial_metadata(initial_environment)
        write_yaml_atomic(self.metadata_path, self.metadata)
        self.initial_environment = initial_environment
        rospy.on_shutdown(self.request_stop)

    def _new_environment_client(self):
        proto_root = resolve_proto_root(
            self.package_path,
            str(rospy.get_param("~grpc_proto_root", "")),
        )
        client = MoraiEnvironmentClient(
            str(rospy.get_param("~grpc_address", "127.0.0.1")),
            int(rospy.get_param("~grpc_port", 7789)),
            proto_root,
            rpc_timeout_sec=float(rospy.get_param("~grpc_timeout_sec", 1.0)),
        )
        client.wait_ready(
            float(rospy.get_param("~grpc_connect_timeout_sec", 2.0))
        )
        return client

    def _environment_snapshot_once(self):
        fallback = {
            "weather": "unknown",
            "sim_hour": None,
            "simulator_version": "unknown",
        }
        client = None
        try:
            client = self._new_environment_client()
            return client.snapshot()
        except Exception as exc:
            fallback["errors"] = {
                "connection": "{}: {}".format(type(exc).__name__, exc)
            }
            rospy.logwarn("MORAI environment metadata unavailable: %s", exc)
            return fallback
        finally:
            if client is not None:
                client.close()

    def _connect_environment_timeline(self):
        try:
            self.environment_client = self._new_environment_client()
        except Exception as exc:
            self.environment_client = None
            rospy.logwarn("MORAI environment timeline unavailable: %s", exc)

    def _resolve_run_root(self):
        if self.profile["kind"] == "raw":
            return os.path.join(self.root, "raw", self.run_id)
        if not self.parent_raw_run_id:
            raise BagProfileError("result profile requires parent_raw_run_id")
        parent = os.path.join(self.root, "raw", self.parent_raw_run_id)
        if not os.path.isdir(parent):
            raise BagProfileError("parent raw run not found: {}".format(parent))
        return os.path.join(
            self.root, "results", self.parent_raw_run_id, self.model_id
        )

    def _parent_metadata(self):
        if not self.parent_raw_run_id:
            raise BagProfileError("result profile requires parent_raw_run_id")
        path = os.path.join(
            self.root, "raw", self.parent_raw_run_id, "metadata.yaml"
        )
        if not os.path.isfile(path):
            raise BagProfileError("parent raw metadata not found: {}".format(path))
        with open(path, "r") as stream:
            value = yaml.safe_load(stream) or {}
        if value.get("status") != "complete":
            raise BagProfileError("parent raw run must have status=complete")
        return value

    def _prepare_storage(self):
        os.makedirs(self.root, exist_ok=True)
        free_gb = shutil.disk_usage(self.root).free / float(1024 ** 3)
        if free_gb < self.min_free_gb:
            raise OSError(
                "insufficient free space: {:.2f} GiB < {:.2f} GiB".format(
                    free_gb, self.min_free_gb
                )
            )
        if os.path.exists(self.run_root):
            raise OSError("run already exists; refusing overwrite: {}".format(self.run_root))
        os.makedirs(self.bags_dir, exist_ok=False)

    def _wait_for_topics(self):
        deadline = time.monotonic() + self.topic_timeout
        published = {}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            published = dict(rospy.get_published_topics())
            missing = set(self.profile["required_topics"]).difference(published)
            if not missing:
                break
            time.sleep(0.2)
        missing = set(self.profile["required_topics"]).difference(published)
        if missing:
            raise BagProfileError(
                "required topics are not published: {}".format(sorted(missing))
            )
        for topic, expected in (self.profile.get("expected_types") or {}).items():
            if topic in published and published[topic] != expected:
                raise BagProfileError(
                    "{} type mismatch: expected {}, got {}".format(
                        topic, expected, published[topic]
                    )
                )
        return published

    @staticmethod
    def _publishers_by_topic():
        state = rosgraph.Master(rospy.get_name()).getSystemState()
        return dict(state[0])

    def _reject_duplicate_publishers(self, topics):
        publishers = self._publishers_by_topic()
        duplicates = dict(
            (topic, publishers[topic])
            for topic in topics
            if len(publishers.get(topic, [])) > 1
        )
        if duplicates:
            raise BagProfileError(
                "multiple publishers on canonical topics: {}".format(duplicates)
            )

    def _validate_timestamp_contract(self):
        expected = self.profile["timestamp_mode"]
        if expected == "replay":
            if not bool(rospy.get_param("/use_sim_time", False)):
                raise BagProfileError(
                    "result recording requires /use_sim_time=true; start the paused "
                    "bag replay controller before inference and result recording"
                )
            return
        master = rosgraph.Master(rospy.get_name())
        for namespace in ("/morai_sensor_bridge", "/morai_udp_bridge"):
            try:
                master.lookupNode(namespace)
            except rosgraph.MasterError:
                # ROS parameters survive a node restart and can otherwise make
                # an inactive bridge look like a live source of this run.
                continue
            key = namespace + "/timestamp_mode"
            if rospy.has_param(key):
                actual = str(rospy.get_param(key))
                if actual != expected:
                    raise BagProfileError(
                        "{} requires timestamp_mode={}, but {}={}".format(
                            self.profile_name, expected, key, actual
                        )
                    )

    def _initial_metadata(self, environment):
        checkpoint = None
        if self.checkpoint:
            checkpoint = {"path": self.checkpoint, "exists": os.path.isfile(self.checkpoint)}
            if checkpoint["exists"]:
                checkpoint["sha256"] = sha256_file(self.checkpoint)
        if self.profile["kind"] == "result":
            bridge_configuration = (
                self.parent_metadata.get("bridge_configuration", {})
                if self.parent_metadata else {}
            )
            bridge_configuration_source = "parent_raw"
        else:
            bridge_configuration = {
                "morai_sensor_bridge": rospy.get_param(
                    "/morai_sensor_bridge", {}
                ),
                "morai_udp_bridge": rospy.get_param("/morai_udp_bridge", {}),
            }
            bridge_configuration_source = "live_ros_parameters"
        return {
            "schema_version": "asmc-rosbag-1.0.0",
            "status": "recording",
            "run_id": self.run_id,
            "profile": self.profile_name,
            "profile_source": self.profile.get("resolved_from"),
            "kind": self.profile["kind"],
            "transport": self.profile["transport"],
            "timestamp_mode": self.profile["timestamp_mode"],
            "timestamp_provenance": self.profile.get("timestamp_provenance", {}),
            "privileged_gt": self.profile["privileged_gt"],
            "deployment_allowed": self.profile["deployment_allowed"],
            "sensor_set": self.profile.get("sensor_set", "unknown"),
            "map": self.map_slug,
            "scenario": self.scenario,
            "controller": self.controller,
            "started_at_utc": utc_now_iso(),
            "environment_initial": environment,
            "naming_environment": self.naming_environment,
            "git": git_metadata(self.repo_root),
            "ros": {
                "use_sim_time": bool(rospy.get_param("/use_sim_time", False)),
                "topics": self.topic_types,
            },
            "bridge_configuration": bridge_configuration,
            "bridge_configuration_source": bridge_configuration_source,
            "replay_configuration": (
                rospy.get_param("/asmc_bag_replay", {})
                if self.profile["kind"] == "result" else None
            ),
            "recording": {
                "compression": self.profile["compression"],
                "split_gb": self.split_gb,
                "buffer_mb": self.buffer_mb,
                "command": self.command,
            },
            "parent_raw_run_id": self.parent_raw_run_id or None,
            "model_id": self.model_id if self.profile["kind"] == "result" else None,
            "checkpoint": checkpoint,
        }

    def _start_environment_timeline(self, initial):
        with open(self.timeline_path, "w") as stream:
            stream.write(json.dumps(
                {"recorded_at_utc": utc_now_iso(), "environment": initial},
                ensure_ascii=False, sort_keys=True
            ) + "\n")
        if self.environment_client is None:
            return

        def worker():
            while not self.environment_stop.wait(self.environment_poll_sec):
                value = self.environment_client.snapshot()
                record = {"recorded_at_utc": utc_now_iso(), "environment": value}
                try:
                    with open(self.timeline_path, "a") as stream:
                        stream.write(json.dumps(
                            record, ensure_ascii=False, sort_keys=True
                        ) + "\n")
                except OSError as exc:
                    rospy.logwarn_throttle(5.0, "environment timeline write failed: %s", exc)

        self.environment_thread = threading.Thread(
            target=worker, name="bag-environment-timeline"
        )
        self.environment_thread.daemon = True
        self.environment_thread.start()

    def run(self):
        rospy.loginfo(
            "rosbag recorder ready: profile=%s run=%s transport=%s root=%s",
            self.profile_name, self.run_id, self.profile["transport"], self.run_root
        )
        try:
            self.process = subprocess.Popen(self.command, start_new_session=True)
            if self.profile["kind"] == "raw":
                self._connect_environment_timeline()
                self._start_environment_timeline(self.initial_environment)
            while not rospy.is_shutdown():
                return_code = self.process.poll()
                if return_code is not None:
                    if not self._requested_stop:
                        rospy.logerr(
                            "rosbag record exited unexpectedly: rc=%d", return_code
                        )
                    break
                time.sleep(0.2)
        finally:
            self._finalize()

    def request_stop(self):
        with self._stop_lock:
            if self._requested_stop:
                return
            self._requested_stop = True
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def _finalize(self):
        with self._stop_lock:
            if self._finalized:
                return
            self._finalized = True
        return_code = None
        if self.process is not None:
            if self.process.poll() is None:
                try:
                    if not self._requested_stop:
                        self._requested_stop = True
                        os.killpg(self.process.pid, signal.SIGINT)
                    return_code = self.process.wait(timeout=30.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        return_code = self.process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        return_code = self.process.wait()
            else:
                return_code = self.process.returncode
        self.environment_stop.set()
        if self.environment_thread is not None:
            self.environment_thread.join(timeout=3.0)
        if self.environment_client is not None:
            self.environment_client.close()
        topic_stats = self.counter.summary() if self.counter is not None else {}
        bridge_diagnostics = self.counter.diagnostics() if self.counter is not None else {}
        if self.counter is not None:
            self.counter.close()

        bag_files = sorted(glob.glob(os.path.join(self.bags_dir, "*.bag")))
        active_files = sorted(glob.glob(os.path.join(self.bags_dir, "*.active")))
        bag_info = [
            {
                "file": os.path.basename(path),
                "bytes": os.path.getsize(path),
                "sha256": sha256_file(path),
            }
            for path in bag_files
        ]
        status = (
            "complete"
            if self._requested_stop
            and return_code in (0, None)
            and bag_files
            and not active_files
            else "interrupted"
        )
        recorded_counts = None
        missing_recorded = []
        recorded_counts_error = None
        if self.profile.get("require_nonempty_topics") and bag_files and not active_files:
            try:
                recorded_counts = recorded_topic_counts(bag_files)
                missing_recorded = missing_recorded_topics(
                    self.profile["required_topics"], recorded_counts
                )
            except Exception as exc:
                recorded_counts_error = "{}: {}".format(type(exc).__name__, exc)
            if missing_recorded or recorded_counts_error:
                status = "incomplete" if status == "complete" else status
                rospy.logerr(
                    "GT bag missing recorded topics=%s; bag inspection error=%s",
                    missing_recorded, recorded_counts_error
                )
        self.metadata.update(
            {
                "status": status,
                "closed_at_utc": utc_now_iso(),
                "rosbag_return_code": return_code,
                "topic_statistics": topic_stats,
                "recorded_topic_counts": recorded_counts,
                "missing_required_message_topics": missing_recorded,
                "recorded_topic_counts_error": recorded_counts_error,
                "diagnostics_final": bridge_diagnostics,
                "bags": bag_info,
                "active_files": [os.path.basename(path) for path in active_files],
            }
        )
        write_yaml_atomic(self.metadata_path, self.metadata)
        rospy.loginfo("rosbag run closed: status=%s root=%s", status, self.run_root)


def main():
    rospy.init_node("asmc_bag_recorder")
    try:
        BagRecorderNode().run()
    except (BagProfileError, OSError, ValueError) as exc:
        rospy.logfatal("bag recorder could not start: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
