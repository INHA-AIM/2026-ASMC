#!/usr/bin/env python3
"""Serve dashboard assets and proxy custom behavior messages over HTTP."""

import functools
import http.server
import json
import threading
import time

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

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/api/snapshot":
            with self.snapshot_lock:
                # Encoding candidate paths must not block high-rate ROS callbacks.
                snapshot = dict(self.snapshot)
                snapshot["received_at"] = dict(self.snapshot["received_at"])
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
        if not self.path.startswith("/api/snapshot"):
            super().log_message(fmt, *args)


def set_snapshot(key, message):
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot[key] = ros_to_dict(message)
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


def main():
    rospy.init_node("behavior_dashboard_server")
    port = int(rospy.get_param("~port", 8088))
    package_path = rospy.get_param("~package_path")
    interface_version = rospy.get_param("~interface_version", "v1")
    DashboardHandler.snapshot["interface_version"] = interface_version
    if interface_version == "v2":
        rospy.Subscriber(
            rospy.get_param("~command_topic", "/behavior/command_v2"),
            BehaviorCommand,
            lambda message: set_snapshot("command", message), queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~feedback_topic", "/planning/frenet_feedback_v2"),
            FrenetFeedback,
            set_frenet_feedback, queue_size=10)
        lidar_objects_topic = rospy.get_param(
            "~lidar_objects_topic", "/perception/objects_lidar_test")
        rospy.Subscriber(
            lidar_objects_topic, DetectedObjectArray,
            lambda message: set_snapshot("lidar_objects", message),
            queue_size=1)
        lidar_motion_debug_topic = rospy.get_param(
            "~lidar_motion_debug_topic", "/lidar/motion_debug")
        rospy.Subscriber(
            lidar_motion_debug_topic, String,
            set_lidar_motion_debug, queue_size=1)
        rospy.loginfo("Dashboard LiDAR objects: %s", lidar_objects_topic)
        rospy.loginfo(
            "Dashboard LiDAR motion diagnostics: %s",
            lidar_motion_debug_topic)
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
    DashboardHandler.snapshot["debug_config"] = load_debug_config()
    handler = functools.partial(DashboardHandler, directory=package_path)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    page = "index_v2.html" if interface_version == "v2" else "index.html"
    rospy.loginfo("Behavior dashboard (%s): http://localhost:%d/web/%s",
                  interface_version, port, page)
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
