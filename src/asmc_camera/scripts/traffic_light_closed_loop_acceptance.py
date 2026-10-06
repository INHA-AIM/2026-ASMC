#!/usr/bin/env python3
"""ROS acceptance test for the typed traffic-light planning loop.

The test deliberately uses isolated topics and starts no trajectory planner or
controller.  It exercises the production behavior_node, route/MGeo data and
ROS messages from a pose on A2256W000728 immediately before the straight
signal at A2256W000599.
"""

import json
import os
import sys
import threading
import time

import rospy

from asmc_msgs.msg import BehaviorCommand, TrafficLightDetection, VehicleState
from behavior_planner.msg import UrbanFeatureDebug


class AcceptanceFailure(RuntimeError):
    pass


class TrafficLightClosedLoopAcceptance:
    def __init__(self):
        self.vehicle_topic = rospy.get_param(
            "~vehicle_topic", "/acceptance/vehicle_state"
        )
        self.detection_topic = rospy.get_param(
            "~detection_topic", "/acceptance/traffic_light/detection"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/acceptance/behavior/command_v2"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/acceptance/behavior/feature_debug"
        )
        self.report_path = rospy.get_param(
            "~report_path", "/tmp/asmc_traffic_light_closed_loop_acceptance.json"
        )
        self.timeout_s = float(rospy.get_param("~timeout_s", 12.0))
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))

        # First centerline sample on A2256W000728, 8.95 m before its stop-line
        # node.  Values are the repository MGeo map frame, not base_link.
        self.x = float(rospy.get_param("~x", -60.10581400350202))
        self.y = float(rospy.get_param("~y", -43.4557633921504))
        self.yaw = float(rospy.get_param("~yaw", 1.5764114027309468))
        self.speed_mps = float(rospy.get_param("~speed_mps", 4.0))
        self.expected_link = rospy.get_param("~expected_link", "A2256W000728")
        self.expected_signal_link = rospy.get_param(
            "~expected_signal_link", "A2256W000599"
        )
        self.expected_traffic_light = rospy.get_param(
            "~expected_traffic_light", "C1256W000016"
        )

        self._lock = threading.Lock()
        self._command = None
        self._debug = None

        self.vehicle_pub = rospy.Publisher(
            self.vehicle_topic, VehicleState, queue_size=2
        )
        self.detection_pub = rospy.Publisher(
            self.detection_topic, TrafficLightDetection, queue_size=2
        )
        self.command_sub = rospy.Subscriber(
            self.command_topic, BehaviorCommand, self._command_callback, queue_size=10
        )
        self.debug_sub = rospy.Subscriber(
            self.debug_topic, UrbanFeatureDebug, self._debug_callback, queue_size=10
        )

    def _command_callback(self, message):
        with self._lock:
            self._command = message

    def _debug_callback(self, message):
        with self._lock:
            self._debug = message

    def _publish_inputs(self, label, state):
        stamp = rospy.Time.now()

        vehicle = VehicleState()
        vehicle.header.stamp = stamp
        vehicle.header.frame_id = "map"
        vehicle.source = VehicleState.SOURCE_EGO_STATUS
        vehicle.valid = True
        vehicle.x = self.x
        vehicle.y = self.y
        vehicle.yaw = self.yaw
        vehicle.speed = self.speed_mps
        self.vehicle_pub.publish(vehicle)

        detection = TrafficLightDetection()
        detection.header.stamp = stamp
        detection.header.frame_id = "camera_extra"
        detection.state = state
        detection.label = label
        detection.confidence = 0.99
        detection.valid = True
        detection.stale = False
        detection.inference_latency_ms = 1.0
        detection.model_id = "p4c4_acceptance_fixture"
        detection.candidate_labels = [label]
        detection.candidate_confidences = [0.99]
        self.detection_pub.publish(detection)

    @staticmethod
    def _command_record(message):
        return {
            "command_id": int(message.command_id),
            "maneuver_type": int(message.maneuver_type),
            "execution_request": int(message.execution_request),
            "source_link_id": message.source_link_id,
            "has_target_pose": bool(message.has_target_pose),
            "target_x": float(message.target_pose_map.position.x),
            "target_y": float(message.target_pose_map.position.y),
            "reason": message.reason,
        }

    @staticmethod
    def _debug_record(message):
        return {
            "current_link_id": message.current_link_id,
            "has_stop_line": bool(message.has_stop_line),
            "distance_to_stop_line_m": float(message.distance_to_stop_line),
            "traffic_light_id": message.traffic_light_id,
            "traffic_light_state": int(message.traffic_light_state),
            "traffic_light_raw_label": message.traffic_light_raw_label,
            "related_signal": message.related_signal,
            "signal_controlled_link_id": message.signal_controlled_link_id,
            "selected_behavior": int(message.selected_behavior),
        }

    def _wait_for(self, label, state, predicate, description):
        deadline = time.monotonic() + self.timeout_s
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_inputs(label, state)
            with self._lock:
                command = self._command
                debug = self._debug
            if command is not None and debug is not None and predicate(command, debug):
                return command, debug
            rate.sleep()
        raise AcceptanceFailure("timeout waiting for " + description)

    def _assert_route_context(self, debug):
        failures = []
        if debug.current_link_id != self.expected_link:
            failures.append(
                "current_link_id={} expected={}".format(
                    debug.current_link_id, self.expected_link
                )
            )
        if not debug.has_stop_line:
            failures.append("has_stop_line=false")
        if debug.traffic_light_id != self.expected_traffic_light:
            failures.append(
                "traffic_light_id={} expected={}".format(
                    debug.traffic_light_id, self.expected_traffic_light
                )
            )
        if debug.signal_controlled_link_id != self.expected_signal_link:
            failures.append(
                "signal_controlled_link_id={} expected={}".format(
                    debug.signal_controlled_link_id, self.expected_signal_link
                )
            )
        if debug.related_signal != "straight":
            failures.append("related_signal={} expected=straight".format(debug.related_signal))
        if failures:
            raise AcceptanceFailure("; ".join(failures))

    def run(self):
        connection_deadline = time.monotonic() + self.timeout_s
        while not rospy.is_shutdown() and time.monotonic() < connection_deadline:
            if (
                self.vehicle_pub.get_num_connections() > 0
                and self.detection_pub.get_num_connections() > 0
            ):
                break
            rospy.sleep(0.05)
        else:
            raise AcceptanceFailure("behavior subscribers did not connect")

        # Establish a non-stopping baseline before changing the same typed
        # stream to red.  Internal debug enum GREEN is 3.
        initial_green, green_debug = self._wait_for(
            "green",
            TrafficLightDetection.STATE_GREEN,
            lambda command, debug: command.maneuver_type == BehaviorCommand.KEEP
            and debug.traffic_light_raw_label == "green"
            and debug.traffic_light_state == 3,
            "initial typed GREEN -> KEEP",
        )
        self._assert_route_context(green_debug)

        # Internal debug enum RED is 1.  STOP must include the map-frame target
        # generated from the physical stop line and configured rear-axle offset.
        red_command, red_debug = self._wait_for(
            "red",
            TrafficLightDetection.STATE_RED,
            lambda command, debug: command.maneuver_type == BehaviorCommand.STOP
            and command.has_target_pose
            and debug.traffic_light_raw_label == "red"
            and debug.traffic_light_state == 1,
            "typed RED -> planned STOP",
        )
        self._assert_route_context(red_debug)

        restarted_command, restarted_debug = self._wait_for(
            "green",
            TrafficLightDetection.STATE_GREEN,
            lambda command, debug: command.maneuver_type == BehaviorCommand.KEEP
            and debug.traffic_light_raw_label == "green"
            and debug.traffic_light_state == 3
            and command.command_id != red_command.command_id,
            "typed GREEN -> new KEEP command",
        )
        self._assert_route_context(restarted_debug)

        return {
            "schema_version": 1,
            "acceptance_id": "p4c4_traffic_light_behavior_closed_loop_v1",
            "passed": True,
            "scope": "isolated ROS decision loop; no trajectory/controller/UDP output",
            "fixture": {
                "map_frame": "map",
                "link_id": self.expected_link,
                "signal_controlled_link_id": self.expected_signal_link,
                "traffic_light_id": self.expected_traffic_light,
                "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
            },
            "initial_green": {
                "command": self._command_record(initial_green),
                "debug": self._debug_record(green_debug),
            },
            "red_stop": {
                "command": self._command_record(red_command),
                "debug": self._debug_record(red_debug),
            },
            "green_restart": {
                "command": self._command_record(restarted_command),
                "debug": self._debug_record(restarted_debug),
            },
        }


def _write_report(path, report):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")


def main():
    rospy.init_node("traffic_light_closed_loop_acceptance", anonymous=False)
    acceptance = TrafficLightClosedLoopAcceptance()
    try:
        report = acceptance.run()
        _write_report(acceptance.report_path, report)
        rospy.loginfo(
            "P4-C4 PASS: typed RED produced STOP and typed GREEN produced a new KEEP; report=%s",
            acceptance.report_path,
        )
        return 0
    except Exception as error:  # Report evidence even when roslaunch fails.
        report = {
            "schema_version": 1,
            "acceptance_id": "p4c4_traffic_light_behavior_closed_loop_v1",
            "passed": False,
            "error": str(error),
        }
        _write_report(acceptance.report_path, report)
        rospy.logerr("P4-C4 FAIL: %s; report=%s", error, acceptance.report_path)
        return 1


if __name__ == "__main__":
    sys.exit(main())
