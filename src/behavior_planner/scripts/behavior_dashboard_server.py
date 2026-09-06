#!/usr/bin/env python3
"""Serve dashboard assets and proxy custom behavior messages over HTTP."""

import functools
import http.server
import json
import threading

import rospy
from asmc_msgs.msg import BehaviorContext, Trajectory, VehicleState
from behavior_planner.msg import UrbanFeatureDebug
from morai_msgs.msg import ObjectStatusList
from visualization_msgs.msg import MarkerArray


def ros_to_dict(value):
    if hasattr(value, "to_sec"):
        return value.to_sec()
    if hasattr(value, "__slots__"):
        return {name: ros_to_dict(getattr(value, name)) for name in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_to_dict(item) for item in value]
    return value


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    snapshot = {"context": None, "feature": None, "trajectory": None,
                "ego": None, "objects": None, "planning_markers": None}
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


def main():
    rospy.init_node("behavior_dashboard_server")
    port = int(rospy.get_param("~port", 8088))
    package_path = rospy.get_param("~package_path")
    rospy.Subscriber(
        "/behavior/context", BehaviorContext,
        lambda message: set_snapshot("context", message), queue_size=1)
    rospy.Subscriber(
        "/behavior/feature_debug", UrbanFeatureDebug,
        lambda message: set_snapshot("feature", message), queue_size=1)
    rospy.Subscriber(
        "/planning/trajectory", Trajectory,
        lambda message: set_snapshot("trajectory", message), queue_size=1)
    rospy.Subscriber(
        "/vehicle/state", VehicleState,
        lambda message: set_snapshot("ego", message), queue_size=1)
    rospy.Subscriber(
        "/Object_topic", ObjectStatusList,
        lambda message: set_snapshot("objects", message), queue_size=1)
    rospy.Subscriber(
        "/planning/markers", MarkerArray,
        set_planning_markers, queue_size=1)
    handler = functools.partial(DashboardHandler, directory=package_path)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo("Behavior dashboard: http://localhost:%d/web/index.html", port)
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
