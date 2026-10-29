#!/usr/bin/env python3
"""Serve dashboard assets and proxy custom behavior messages over HTTP."""

import argparse
import sys
import functools
import http.server
import json
import os
import threading
import time
from urllib.parse import parse_qs, urlsplit
from sensor_msgs.msg import Image
from asmc_msgs.msg import CameraObjectObservation
from local_debug_cache import LocalDebugCache

import rospy
from asmc_msgs.msg import (BehaviorCommand, BehaviorContext,
                           FrenetFeedback, Trajectory,
                           VehicleState)
from planning_inputs.msg import DetectedObjectArray
from asmc_msgs.msg import UrbanFeatureDebug
from morai_msgs.msg import ObjectStatusList
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import UInt8
from asmc_msgs.msg import LaneBoundaryFamily, LidarTrackingObservation
from lane_boundary_adapter.msg import CameraBBoxLaneResult
from dashboard_startup import run_on_port
from dashboard_observation import LocalScene, family_payload, markers_payload, objects_payload, stamp_ns

local_scene = LocalScene()
local_debug = LocalDebugCache()


TRAJECTORY_FAILURE_NAMES = {
    0: "NONE",
    1: "COLLISION_BLOCKED",
    2: "GENERATION",
    3: "STATE_STALE",
    4: "BEHAVIOR_STALE",
    5: "OBJECTS_STALE",
    6: "INVALID_INPUT",
    7: "UNSUPPORTED_BEHAVIOR",
    8: "NOT_INITIALIZED",
}
FRENET_RESULT_NAMES = {
    0: "NONE",
    1: "NOT_INITIALIZED",
    2: "INVALID_COMMAND",
    3: "STALE_COMMAND",
    4: "TARGET_PROJECTION_FAILED",
    5: "TARGET_OBJECT_LOST",
    6: "TRANSIENT_GENERATION_FAILURE",
    7: "CURVATURE_BLOCKED",
    8: "COLLISION_BLOCKED",
    9: "TOO_LATE_TO_CANCEL",
}


def ros_to_dict(value):
    if hasattr(value, "to_sec"):
        return value.to_sec()
    if hasattr(value, "__slots__"):
        return {name: ros_to_dict(getattr(value, name)) for name in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_to_dict(item) for item in value]
    return value


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    snapshot = {"interface_version": "v1", "context": None, "command": None,
                "feedback": None, "feature": None, "trajectory": None,
                "ego": None, "objects": None, "lidar_objects": None,
                "lidar_motion_debug": None, "debug_config": None,
                "planning_markers": None, "recent_planning_issue": None,
                "received_at": {}}
    snapshot_lock = threading.Lock()
    trial_identity = os.environ.get("ASMC_TRIAL_MONITOR_IDENTITY", "")

    def guess_type(self, path):
        # Explicit UTF-8 for Korean labels in HTML/CSS/JS; keep binary MIME types.
        kind = super().guess_type(path)
        if kind.startswith("text/") or kind in ("application/javascript", "application/json"):
            return kind + "; charset=utf-8"
        return kind

    def end_headers(self):
        if self.trial_identity:
            self.send_header("X-ASMC-Trial-Identity", self.trial_identity)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        if urlsplit(self.path).path == "/api/local-scene":
            args = parse_qs(urlsplit(self.path).query)
            with self.snapshot_lock:
                plan = self.snapshot.get("local_experiment") or {}
            wanted = args.get("sequence", [""])[0]
            requested_frame=args.get('frame',[None])[0]
            requested_run=args.get('run_id',[None])[0]
            try:
                selected_stamp=local_debug.plan_stamp(int(wanted),requested_run,requested_frame)
            except ValueError:selected_stamp=None
            # Older clients without an epoch may request the current plan only.
            if requested_frame is None and wanted==str(plan.get('plan_sequence')):
                selected_stamp=plan.get('scene_stamp_ns',0)
            if selected_stamp is None:
                self.send_error(409, "plan epoch changed or no longer cached")
                return
            body = json.dumps(local_debug.snapshot(selected_stamp), separators=(",", ":")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.split("?", 1)[0] == "/api/snapshot":
            with self.snapshot_lock:
                # Encoding candidate paths must not block high-rate ROS callbacks.
                snapshot = dict(self.snapshot)
                snapshot["received_at"] = dict(self.snapshot["received_at"])
            # Receive age uses server monotonic time, source age uses ROS time.
            # Browser/Windows clocks never decide ROS observation freshness.
            received = snapshot.get("received_at", {}).get("local_experiment")
            snapshot["local_receive_age_s"] = time.time()-received if received else None
            snapshot["shadow"] = local_scene.snapshot(rospy.Time.now().to_nsec())
            body = json.dumps(snapshot, separators=(",", ":")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def log_message(self, fmt, *args):
        # Avoid printing one line for every 10 Hz snapshot request.
        if not self.path.startswith("/api/"):
            super().log_message(fmt, *args)


def set_local_experiment(key, message):
    """Optional LOCAL-1 observer; no control or map pose fallback."""
    if len(message.data) > 256 * 1024:
        return
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError):
        return
    if not isinstance(value, dict):
        return
    if key=='local_experiment':local_debug.plan(value)
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot[key] = value
        DashboardHandler.snapshot["received_at"][key] = time.time()


def set_local_camera(message):
    d = message.detections
    local_debug.camera(dict(view=d.header.frame_id, stamp_ns=str(d.header.stamp.to_nsec()),
        valid=d.valid, width=d.image_width, height=d.image_height,
        session=message.producer_session_id, generation=message.generation,
        boxes=[dict(name=b.source_class_name, group=b.class_name, confidence=b.confidence,
                    bbox=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in d.boxes[:64]],
        omitted=max(0,len(d.boxes)-64)))


def set_snapshot(key, message):
    converted = ros_to_dict(message)
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot[key] = converted
        DashboardHandler.snapshot["received_at"][key] = time.time()


def _set_recent_planning_issue_locked(source, reason, detail="",
                                      command_id=None):
    DashboardHandler.snapshot["recent_planning_issue"] = {
        "source": source,
        "reason": reason,
        "detail": detail,
        "command_id": command_id,
        "observed_at": time.time(),
    }


def set_frenet_feedback(message):
    """Retain brief failures that the browser's 10 Hz polling could miss."""
    converted = ros_to_dict(message)
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot["feedback"] = converted
        DashboardHandler.snapshot["received_at"]["feedback"] = time.time()
        abnormal = (not message.command_accepted or
                    message.trajectory_source ==
                    FrenetFeedback.TRAJECTORY_SAFE_STOP or
                    message.result_code != FrenetFeedback.RESULT_NONE or
                    message.execution_state == FrenetFeedback.FAILED)
        if abnormal:
            reason = (message.result_detail or
                      FRENET_RESULT_NAMES.get(int(message.result_code),
                                              "FRENET_PLAN_INVALID"))
            _set_recent_planning_issue_locked(
                "FRENET", reason, message.result_detail, message.command_id)


def set_trajectory(message):
    """Retain invalid/one-point STOP trajectory reasons between snapshots."""
    converted = ros_to_dict(message)
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot["trajectory"] = converted
        DashboardHandler.snapshot["received_at"]["trajectory"] = time.time()
        one_point_stop = (
            message.valid and len(message.points) == 1 and
            abs(message.points[0].velocity) <= 1e-6)
        if not message.valid or one_point_stop:
            failure_name = TRAJECTORY_FAILURE_NAMES.get(
                int(message.failure_reason),
                "FAILURE_{}".format(int(message.failure_reason)))
            if message.failure_reason == Trajectory.FAILURE_NONE:
                command = DashboardHandler.snapshot.get("command") or {}
                maneuver = command.get("maneuver_type")
                if maneuver == BehaviorCommand.STOP:
                    failure_name = "NORMAL_STOP_OR_HOLD"
                elif maneuver == BehaviorCommand.EMERGENCY_STOP:
                    failure_name = "EMERGENCY_STOP"
                else:
                    failure_name = "ONE_POINT_STOP"
            _set_recent_planning_issue_locked(
                "TRAJECTORY", failure_name,
                "valid={} points={} failure_reason={}".format(
                    bool(message.valid), len(message.points),
                    int(message.failure_reason)),
                (DashboardHandler.snapshot.get("command") or {}).get(
                    "command_id"))


def set_ros_log(message):
    """Reuse rosout diagnostics instead of introducing a debug topic."""
    marker = "MPC fail-safe STOP:"
    if marker not in message.msg:
        return
    reason = message.msg.split(marker, 1)[1].strip() or "UNKNOWN"
    with DashboardHandler.snapshot_lock:
        _set_recent_planning_issue_locked("MPC", reason, message.msg)


def set_shadow_family(message):
    payload, valid, reason = family_payload(message)
    local_scene.put("family", payload, stamp_ns(message.header), message.header.frame_id, valid, reason)


def set_shadow_markers(message):
    payload, stamp, valid, reason = markers_payload(message)
    local_scene.put("path", payload, stamp, "base_link", valid, reason)


def set_shadow_objects(message):
    payload, valid, reason = objects_payload(message)
    o = message.observation
    local_scene.put("objects", payload, stamp_ns(o.objects.header), o.objects.header.frame_id,
                    valid, reason, (o.producer_session_id, int(o.generation)), int(o.scan_sequence))


def set_shadow_diagnostics(message):
    for status in message.status:
        if status.name == "camera_bbox_lane_change":
            payload = {entry.key: entry.value for entry in status.values[:80]}
            payload.update(reason=status.message, level=int(status.level))
            local_scene.put("controller", payload, stamp_ns(message.header))
            break


def set_shadow_bbox(message):
    boxes = []
    for i, box in enumerate(message.boxes[:100]):
        boxes.append(dict(label=box.label, bbox=[box.xmin, box.ymin, box.xmax, box.ymax],
                          assignment=int(message.assignments[i]) if i < len(message.assignments) else None,
                          reason=message.box_reasons[i] if i < len(message.box_reasons) else "missing_assignment"))
    # Keep invalid-result diagnostics visible; never use them as free-space.
    local_scene.put("bbox", dict(valid=message.valid, reason=message.reason,
                                 own_lane=message.own_lane, boxes=boxes,
                                 omitted_boxes=max(0, len(message.boxes)-100)), stamp_ns(message.header))


def set_control_selector(message):
    fields = dict(item.split("=", 1) for item in message.data.split() if "=" in item)
    local_scene.put("selector", fields)


def set_lidar_motion_debug(message):
    """Decode the debug-only JSON stream without changing planner messages."""
    try:
        payload = json.loads(message.data)
    except (TypeError, ValueError) as error:
        rospy.logwarn_throttle(2.0, "Invalid LiDAR motion debug JSON: %s", error)
        return
    if (not isinstance(payload, dict) or
            payload.get("schema") not in {
                "lidar_motion_debug_v1", "lidar_motion_debug_v2"} or
            payload.get("frame_id") != "map" or
            not isinstance(payload.get("objects"), list)):
        rospy.logwarn_throttle(
            2.0, "Rejected LiDAR motion debug payload: schema/frame/objects")
        return
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot["lidar_motion_debug"] = payload
        DashboardHandler.snapshot["received_at"]["lidar_motion_debug"] = time.time()


def load_debug_config():
    ego_width = float(rospy.get_param("/behavior_planner/vehicle/width", 1.85))
    path_margin = float(rospy.get_param(
        "/behavior_planner/avoid/path_margin_m", 0.4))
    lateral_ceiling = float(rospy.get_param(
        "/behavior_planner/avoid/max_lateral_offset", 1.75))
    return {
        "avoid_max_distance_m": float(rospy.get_param(
            "/behavior_planner/avoid/max_target_distance", 120.0)),
        "ego_width_m": ego_width,
        "path_margin_m": path_margin,
        "lateral_ceiling_m": lateral_ceiling,
        "path_corridor_half_width_m": min(
            lateral_ceiling, 0.5 * ego_width + path_margin),
    }


def set_planning_markers(message):
    """Store only browser-relevant paths, not the full MarkerArray payload."""
    paths = {"valid": [], "rejected": [], "selected": []}
    for marker in message.markers:
        if marker.action != marker.ADD or marker.type != marker.LINE_STRIP:
            continue
        points = [[point.x, point.y] for point in marker.points]
        if marker.ns == "frenet_candidates_valid":
            paths["valid"].append(points)
        elif marker.ns == "frenet_candidates_rejected":
            paths["rejected"].append(points)
        elif marker.ns == "frenet_selected_trajectory":
            paths["selected"] = points
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot["planning_markers"] = paths
        DashboardHandler.snapshot["received_at"]["planning_markers"] = time.time()


def run_dashboard(server=None, reserved_port=None):
    rospy.init_node("behavior_dashboard_server")
    port = int(rospy.get_param("~port", 8088))
    if reserved_port is not None and port != reserved_port:
        raise ValueError("dashboard --port and ROS ~port disagree")
    package_path = rospy.get_param("~package_path")
    interface_version = rospy.get_param("~interface_version", "v1")
    DashboardHandler.snapshot["interface_version"] = interface_version
    if interface_version == "v2":
        # Diagnostics only: no control publishers, inference or duplicate fits.
        rospy.Subscriber("/planning/mode", UInt8,
                         lambda m: local_scene.put("mode", dict(mode=int(m.data))), queue_size=1)
        rospy.Subscriber("/planning/control_selector/diagnostics", String, set_control_selector, queue_size=1)
        rospy.Subscriber("/planning/camera_bbox_lane_change/diagnostics", DiagnosticArray, set_shadow_diagnostics, queue_size=1)
        rospy.Subscriber("/planning/camera_bbox_lane_change/markers", MarkerArray, set_shadow_markers, queue_size=1)
        rospy.Subscriber("/planning/camera_bbox_lane/result", CameraBBoxLaneResult, set_shadow_bbox, queue_size=1)
        rospy.Subscriber("/perception/lane/boundary_family", LaneBoundaryFamily, set_shadow_family, queue_size=1)
        rospy.Subscriber("/perception/lidar/tracks_local", LidarTrackingObservation, set_shadow_objects, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~command_topic", "/behavior/command_v2"),
            BehaviorCommand,
            lambda message: set_snapshot("command", message), queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~feedback_topic", "/planning/frenet_feedback_v2"),
            FrenetFeedback,
            set_frenet_feedback, queue_size=10)
        lidar_objects_topic = rospy.get_param(
            "~lidar_objects_topic", "/planning/objects")
        rospy.Subscriber(
            lidar_objects_topic, DetectedObjectArray,
            lambda message: set_snapshot("lidar_objects", message),
            queue_size=1)
        lidar_motion_debug_topic = rospy.get_param(
            "~lidar_motion_debug_topic", "")
        if lidar_motion_debug_topic:
            rospy.Subscriber(
                lidar_motion_debug_topic, String,
                set_lidar_motion_debug, queue_size=1)
        rospy.loginfo("Dashboard LiDAR objects: %s", lidar_objects_topic)
        rospy.loginfo(
            "Dashboard LiDAR motion diagnostics: %s",
            lidar_motion_debug_topic or "disabled (typed objects)")
    else:
        rospy.Subscriber(
            rospy.get_param("~context_topic", "/behavior/context"),
            BehaviorContext,
            lambda message: set_snapshot("context", message), queue_size=1)
        rospy.Subscriber(
            "/Object_topic", ObjectStatusList,
            lambda message: set_snapshot("objects", message), queue_size=1)
    rospy.Subscriber(
        "/behavior/feature_debug", UrbanFeatureDebug,
        lambda message: set_snapshot("feature", message), queue_size=1)
    rospy.Subscriber(
        "/planning/trajectory", Trajectory,
        set_trajectory, queue_size=10)
    rospy.Subscriber(
        rospy.get_param("~rosout_topic", "/rosout_agg"), Log,
        set_ros_log, queue_size=20)
    rospy.Subscriber(
        "/vehicle/state", VehicleState,
        lambda message: set_snapshot("ego", message), queue_size=1)
    rospy.Subscriber(
        "/planning/markers", MarkerArray,
        set_planning_markers, queue_size=1)
    rospy.Subscriber("/planning/local/trace", String,
                     lambda m: set_local_experiment("local_experiment", m), queue_size=1)
    rospy.Subscriber("/planning/local/control_trace", String,
                     lambda m: set_local_experiment("local_control", m), queue_size=1)
    prefix = "/experimental/shadow/semantic_space_trial/"
    for key, topic in (("probability", "drivable_probability"), ("coverage", "coverage")):
        rospy.Subscriber(prefix + topic, Image,
            lambda m, key=key: local_debug.grid(key, m.header.stamp.to_nsec(), m.width, m.height, m.step, m.encoding, m.data),
            queue_size=1)
    rospy.Subscriber("/perception/camera/objects/observations", CameraObjectObservation,
                     set_local_camera, queue_size=3)
    DashboardHandler.snapshot["debug_config"] = load_debug_config()
    handler = functools.partial(DashboardHandler, directory=package_path)
    if server is None:  # Legacy direct rosrun invocation without --port.
        server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    else:
        server.RequestHandlerClass = handler
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    page = "index_v2.html" if interface_version == "v2" else "index.html"
    rospy.loginfo("Behavior dashboard (%s): http://localhost:%d/web/%s",
                  interface_version, port, page)
    rospy.on_shutdown(server.shutdown)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.port is None:
        return run_dashboard()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    return run_on_port(args.port, DashboardHandler,
                       lambda server: run_dashboard(server, args.port))


if __name__ == "__main__":
    sys.exit(main())
