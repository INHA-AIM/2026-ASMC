#!/usr/bin/env python3
"""Guarded MORAI red-stop / green-restart vehicle acceptance.

This is deliberately separate from the deterministic decision-loop test.  It
attaches to an already running local UDP bridge and MORAI gRPC server, then
drives the production behavior -> Frenet -> MPC -> /ctrl_cmd -> UDP chain.
The existing Ego pose, speed, gear and control mode are restored in ``finally``.

The test uses a synthetic *typed* traffic-light result.  It therefore accepts
the planning/control response to perception, not camera classifier accuracy.
"""

import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

import rospy

from asmc_msgs.msg import BehaviorCommand, TrafficLightDetection, VehicleState
from asmc_msgs.msg import UrbanFeatureDebug
from morai_msgs.msg import CtrlCmd


class AcceptanceFailure(RuntimeError):
    pass


def _copy_transform(source):
    from proto.morai.common.type_pb2 import Transform

    result = Transform()
    result.CopyFrom(source)
    return result


def _make_transform(x, y, z, yaw_deg):
    from proto.morai.common.type_pb2 import Transform

    result = Transform()
    result.location.x = float(x)
    result.location.y = float(y)
    result.location.z = float(z)
    result.rotation.z = float(yaw_deg)
    return result


class TrafficLightVehicleAcceptance:
    def __init__(self):
        if not bool(rospy.get_param("~allow_vehicle_control", False)):
            raise AcceptanceFailure(
                "vehicle control is locked; pass allow_vehicle_control:=true "
                "only in a controlled simulator"
            )

        self.repo_root = Path(rospy.get_param("~repo_root", "/root/ws"))
        self.grpc_host = rospy.get_param("~grpc_host", "host.docker.internal")
        self.grpc_port = int(rospy.get_param("~grpc_port", 7789))
        self.report_path = rospy.get_param(
            "~report_path", "/tmp/asmc_traffic_light_vehicle_acceptance.json"
        )

        # A2256W000202 centerline, about 32 m before the physical stop line at
        # the end of A2256W000728.  MORAI's actor Z is about 0.36 m above MGeo.
        self.start_x = float(rospy.get_param("~start_x", -59.98509757156367))
        self.start_y = float(rospy.get_param("~start_y", -66.53153144890034))
        self.start_z = float(rospy.get_param("~start_z", 28.7487))
        self.start_yaw_deg = float(rospy.get_param("~start_yaw_deg", 90.3183423))
        self.stopline_x = float(rospy.get_param("~stopline_x", -60.15604465093929))
        self.stopline_y = float(rospy.get_param("~stopline_y", -34.51018160348758))

        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.ready_timeout_s = float(rospy.get_param("~ready_timeout_s", 12.0))
        self.move_timeout_s = float(rospy.get_param("~move_timeout_s", 15.0))
        self.stop_timeout_s = float(rospy.get_param("~stop_timeout_s", 18.0))
        self.restart_timeout_s = float(rospy.get_param("~restart_timeout_s", 12.0))
        self.stop_speed_mps = float(rospy.get_param("~stop_speed_mps", 0.25))
        self.restart_speed_mps = float(rospy.get_param("~restart_speed_mps", 0.75))
        self.minimum_stopline_clearance_m = float(
            rospy.get_param("~minimum_stopline_clearance_m", 2.5)
        )

        self._lock = threading.Lock()
        self._vehicle = None
        self._command = None
        self._debug = None
        self._control = None
        self._stage = "preflight"
        self._samples = []
        self._last_sample_time = 0.0
        self._signal_label = "green"
        self._signal_state = TrafficLightDetection.STATE_GREEN
        self._forward_control = False
        self._forward_control_ever_enabled = False

        self.signal_pub = rospy.Publisher(
            "/perception/traffic_light/detection",
            TrafficLightDetection,
            queue_size=2,
        )
        # MPC is remapped to a private acceptance topic.  Nothing reaches the
        # real bridge until the original Ego state has been captured and this
        # node explicitly opens the gate.
        self.bridge_control_pub = rospy.Publisher(
            "/ctrl_cmd", CtrlCmd, queue_size=2
        )
        rospy.Subscriber("/vehicle/state", VehicleState, self._vehicle_callback, queue_size=10)
        rospy.Subscriber(
            "/behavior/command_v2", BehaviorCommand, self._command_callback, queue_size=10
        )
        rospy.Subscriber(
            "/behavior/feature_debug", UrbanFeatureDebug, self._debug_callback, queue_size=10
        )
        rospy.Subscriber(
            "/acceptance/ctrl_cmd", CtrlCmd, self._control_callback, queue_size=10
        )

        self._client = None
        self._ego = None
        self._original = None

    def _vehicle_callback(self, message):
        with self._lock:
            self._vehicle = message

    def _command_callback(self, message):
        with self._lock:
            self._command = message

    def _debug_callback(self, message):
        with self._lock:
            self._debug = message

    def _control_callback(self, message):
        with self._lock:
            self._control = message
            should_forward = self._forward_control
        if should_forward:
            self.bridge_control_pub.publish(message)

    def _snapshot(self):
        with self._lock:
            return (
                copy.deepcopy(self._vehicle),
                copy.deepcopy(self._command),
                copy.deepcopy(self._debug),
                copy.deepcopy(self._control),
            )

    def _publish_signal(self):
        message = TrafficLightDetection()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera_extra"
        message.state = self._signal_state
        message.label = self._signal_label
        message.confidence = 0.99
        message.valid = True
        message.stale = False
        message.inference_latency_ms = 1.0
        message.model_id = "p4c4_vehicle_acceptance_fixture"
        message.candidate_labels = [self._signal_label]
        message.candidate_confidences = [0.99]
        self.signal_pub.publish(message)

    def _record(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_sample_time < 0.2:
            return
        self._last_sample_time = now
        vehicle, command, debug, control = self._snapshot()
        if vehicle is None:
            return
        self._samples.append(
            {
                "elapsed_s": round(now - self._samples_start_time, 3),
                "stage": self._stage,
                "x": float(vehicle.x),
                "y": float(vehicle.y),
                "speed_mps": float(vehicle.speed),
                "link_id": debug.current_link_id if debug is not None else "",
                "distance_to_stopline_m": (
                    float(debug.distance_to_stop_line) if debug is not None else None
                ),
                "maneuver_type": (
                    int(command.maneuver_type) if command is not None else None
                ),
                "command_id": int(command.command_id) if command is not None else None,
                "ctrl_velocity": float(control.velocity) if control is not None else None,
                "ctrl_brake": float(control.brake) if control is not None else None,
            }
        )

    def _set_signal(self, label, state, stage):
        self._signal_label = label
        self._signal_state = state
        self._stage = stage
        rospy.loginfo("P4-C4 vehicle acceptance stage=%s signal=%s", stage, label)

    def _wait(self, timeout_s, predicate, description):
        deadline = time.monotonic() + timeout_s
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_signal()
            self._record()
            snapshot = self._snapshot()
            if predicate(*snapshot):
                self._record(force=True)
                return snapshot
            rate.sleep()
        raise AcceptanceFailure("timeout waiting for " + description)

    def _connect_grpc(self):
        grpc_root = self.repo_root / "tools" / "grpc_inha_univ" / "src"
        for path in (grpc_root, grpc_root / "api", grpc_root / "proto"):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))

        from api.morai_sim_client import MoraiSimClient
        from api.vehicle import Vehicle

        self._client = MoraiSimClient("p4c4_vehicle_acceptance")
        self._client.connect(self.grpc_host, self.grpc_port)
        if not self._client.is_connected():
            raise AcceptanceFailure(
                "cannot connect MORAI gRPC at {}:{}".format(
                    self.grpc_host, self.grpc_port
                )
            )
        self._ego = Vehicle(
            self._client._sim_adapter, "p4c4_vehicle_acceptance", "Ego"
        )
        state = self._ego.get_actor_state()
        if state is None:
            raise AcceptanceFailure("cannot read existing Ego state")
        self._original = {
            "transform": _copy_transform(state.transform),
            "speed_mps": math.sqrt(
                float(state.velocity.x) ** 2
                + float(state.velocity.y) ** 2
                + float(state.velocity.z) ** 2
            ),
            "control_mode": int(self._ego.get_control_mode()),
            "gear": int(state.vehicle_state.gear_mode),
        }

    def _place_and_enable_control(self):
        from proto.morai.actor.actor_enum_pb2 import (
            GEAR_MODE_D,
            VEHICLE_CONTROL_AUTO_MODE,
            VEHICLE_CONTROL_KEYBOARD,
        )

        # Ignore any live /ctrl_cmd while moving the actor into the fixture.
        if not self._ego.set_control_mode(VEHICLE_CONTROL_KEYBOARD):
            raise AcceptanceFailure("failed to enter temporary keyboard mode")
        if not self._ego.set_transform(
            _make_transform(
                self.start_x, self.start_y, self.start_z, self.start_yaw_deg
            )
        ):
            raise AcceptanceFailure("failed to teleport Ego to acceptance fixture")
        self._ego.set_velocity(0.0)
        rospy.sleep(0.7)

        # Let behavior/Frenet form a valid green trajectory before UDP control
        # is accepted by MORAI.
        self._set_signal("green", TrafficLightDetection.STATE_GREEN, "green_ready")
        self._wait(
            self.ready_timeout_s,
            lambda vehicle, command, debug, control: vehicle is not None
            and vehicle.valid
            and math.hypot(vehicle.x - self.start_x, vehicle.y - self.start_y) < 2.0
            and command is not None
            and command.maneuver_type == BehaviorCommand.KEEP
            and debug is not None
            and debug.traffic_light_raw_label == "green"
            and control is not None,
            "green KEEP trajectory/control readiness at teleported pose",
        )
        if not self._ego.set_control_mode(VEHICLE_CONTROL_AUTO_MODE):
            raise AcceptanceFailure("failed to enter MORAI AUTO control mode")
        if not self._ego.set_vehicle_gear(GEAR_MODE_D):
            raise AcceptanceFailure("failed to select drive gear")
        with self._lock:
            self._forward_control = True
            self._forward_control_ever_enabled = True

    def _stop_before_restore(self):
        """Close the MPC gate and bring the physical actor to rest via UDP."""
        with self._lock:
            self._forward_control = False
            was_enabled = self._forward_control_ever_enabled
        if not was_enabled:
            return

        stop = CtrlCmd()
        stop.longlCmdType = 1
        stop.accel = 0.0
        stop.brake = 1.0
        stop.steering = 0.0
        stop.velocity = 0.0
        stop.acceleration = 0.0

        deadline = time.monotonic() + 5.0
        stopped_since = None
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.bridge_control_pub.publish(stop)
            vehicle, _, _, _ = self._snapshot()
            if vehicle is not None and vehicle.speed <= 0.10:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                if time.monotonic() - stopped_since >= 0.5:
                    break
            else:
                stopped_since = None
            rate.sleep()
        # Ensure the final datagram is consumed before gRPC restores control
        # mode.  No producer publishes /ctrl_cmd after this point.
        rospy.sleep(0.2)

    def _restore(self):
        if self._ego is None or self._original is None:
            return
        errors = []
        try:
            # The UDP gate is already closed and Ego is stationary.  Apply
            # the original mode last because MORAI transform/gear setters may
            # temporarily select AUTO internally.
            if not self._ego.set_transform(self._original["transform"]):
                errors.append("transform")
            if not self._ego.set_velocity(self._original["speed_mps"]):
                errors.append("speed")
            if not self._ego.set_vehicle_gear(self._original["gear"]):
                errors.append("gear")
            if not self._ego.set_control_mode(self._original["control_mode"]):
                errors.append("control mode")
            rospy.sleep(0.5)
            restored = self._ego.get_actor_state()
            restored_mode = int(self._ego.get_control_mode())
            if restored is None:
                errors.append("post-restore state unavailable")
            else:
                position_error = math.hypot(
                    float(restored.transform.location.x)
                    - float(self._original["transform"].location.x),
                    float(restored.transform.location.y)
                    - float(self._original["transform"].location.y),
                )
                restored_speed = math.sqrt(
                    float(restored.velocity.x) ** 2
                    + float(restored.velocity.y) ** 2
                    + float(restored.velocity.z) ** 2
                )
                if position_error > 0.5:
                    errors.append("position error {:.3f}m".format(position_error))
                if abs(restored_speed - self._original["speed_mps"]) > 0.2:
                    errors.append("speed error {:.3f}m/s".format(restored_speed))
                if int(restored.vehicle_state.gear_mode) != self._original["gear"]:
                    errors.append("gear verification")
            if restored_mode != self._original["control_mode"]:
                errors.append("control mode verification")
        except Exception as error:
            errors.append(str(error))
        if errors:
            rospy.logerr("Ego restore incomplete: %s", ", ".join(errors))
            raise AcceptanceFailure(
                "Ego restore incomplete: " + ", ".join(errors)
            )
        else:
            rospy.loginfo("Ego pose, speed, gear and control mode restored")

    def run(self):
        self._samples_start_time = time.monotonic()
        self._connect_grpc()
        try:
            self._place_and_enable_control()

            moving_vehicle, initial_command, initial_debug, initial_ctrl = self._wait(
                self.move_timeout_s,
                lambda vehicle, command, debug, control: vehicle is not None
                and vehicle.speed >= 1.0
                and math.hypot(vehicle.x - self.start_x, vehicle.y - self.start_y) >= 1.0,
                "physical movement under initial GREEN",
            )
            initial_command_id = int(initial_command.command_id)
            peak_green_speed = max(
                sample["speed_mps"]
                for sample in self._samples
                if sample["stage"] in ("green_ready",)
            )

            self._set_signal("red", TrafficLightDetection.STATE_RED, "red_braking")
            red_snapshot = self._wait(
                self.stop_timeout_s,
                lambda vehicle, command, debug, control: command is not None
                and command.maneuver_type == BehaviorCommand.STOP
                and command.has_target_pose
                and debug is not None
                and debug.traffic_light_raw_label == "red"
                and control is not None
                and control.brake > 0.05,
                "typed RED -> STOP command and physical brake output",
            )
            red_command_id = int(red_snapshot[1].command_id)

            stopped_since = [None]

            def stopped_before_line(vehicle, command, debug, control):
                if vehicle is None:
                    stopped_since[0] = None
                    return False
                clearance = math.hypot(
                    float(vehicle.x) - self.stopline_x,
                    float(vehicle.y) - self.stopline_y,
                )
                if (
                    vehicle.speed <= self.stop_speed_mps
                    and clearance >= self.minimum_stopline_clearance_m
                ):
                    if stopped_since[0] is None:
                        stopped_since[0] = time.monotonic()
                    return time.monotonic() - stopped_since[0] >= 1.0
                stopped_since[0] = None
                return False

            stopped_vehicle, stop_command, stop_debug, stop_ctrl = self._wait(
                self.stop_timeout_s,
                stopped_before_line,
                "Ego held stopped before the physical stop line",
            )
            stop_x = float(stopped_vehicle.x)
            stop_y = float(stopped_vehicle.y)
            stop_clearance = math.hypot(
                stop_x - self.stopline_x, stop_y - self.stopline_y
            )
            stop_speed = float(stopped_vehicle.speed)

            self._set_signal(
                "green", TrafficLightDetection.STATE_GREEN, "green_restart"
            )
            restart_origin = (stop_x, stop_y)
            restarted_vehicle, restarted_command, restarted_debug, restarted_ctrl = self._wait(
                self.restart_timeout_s,
                lambda vehicle, command, debug, control: vehicle is not None
                and vehicle.speed >= self.restart_speed_mps
                and math.hypot(
                    vehicle.x - restart_origin[0], vehicle.y - restart_origin[1]
                )
                >= 1.0
                and command is not None
                and command.maneuver_type == BehaviorCommand.KEEP
                and command.command_id != red_command_id
                and debug is not None
                and debug.traffic_light_raw_label == "green",
                "typed GREEN -> new KEEP and physical restart",
            )

            return {
                "schema_version": 1,
                "acceptance_id": "p4c4_traffic_light_vehicle_closed_loop_v1",
                "passed": True,
                "scope": (
                    "typed perception result -> behavior -> Frenet -> MPC -> "
                    "/ctrl_cmd -> MORAI UDP vehicle motion"
                ),
                "excluded_scope": ["camera_traffic_light_classification_accuracy"],
                "fixture": {
                    "start": {
                        "x": self.start_x,
                        "y": self.start_y,
                        "z": self.start_z,
                        "yaw_deg": self.start_yaw_deg,
                    },
                    "start_link_id": "A2256W000202",
                    "stopline_link_id": "A2256W000728",
                    "signal_controlled_link_id": "A2256W000599",
                    "traffic_light_id": "C1256W000016",
                    "stopline": {"x": self.stopline_x, "y": self.stopline_y},
                },
                "observed": {
                    "initial_keep_command_id": initial_command_id,
                    "red_stop_command_id": red_command_id,
                    "restart_keep_command_id": int(restarted_command.command_id),
                    "peak_initial_green_speed_mps": peak_green_speed,
                    "stop": {
                        "x": stop_x,
                        "y": stop_y,
                        "speed_mps": stop_speed,
                        "physical_stopline_clearance_m": stop_clearance,
                        "route_stopline_distance_m": float(
                            stop_debug.distance_to_stop_line
                        ),
                    },
                    "restart": {
                        "x": float(restarted_vehicle.x),
                        "y": float(restarted_vehicle.y),
                        "speed_mps": float(restarted_vehicle.speed),
                        "progress_m": math.hypot(
                            restarted_vehicle.x - stop_x,
                            restarted_vehicle.y - stop_y,
                        ),
                    },
                },
                "samples": self._samples,
            }
        finally:
            try:
                self._stop_before_restore()
            finally:
                try:
                    self._restore()
                finally:
                    if self._client is not None:
                        self._client.disconnect()


def _write_report(path, report):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main():
    rospy.init_node("traffic_light_vehicle_acceptance", anonymous=False)
    acceptance = None
    try:
        acceptance = TrafficLightVehicleAcceptance()
        report = acceptance.run()
        _write_report(acceptance.report_path, report)
        rospy.loginfo(
            "P4-C4 VEHICLE PASS: physical red stop and green restart; report=%s",
            acceptance.report_path,
        )
        return 0
    except Exception as error:
        path = (
            acceptance.report_path
            if acceptance is not None
            else "/tmp/asmc_traffic_light_vehicle_acceptance.json"
        )
        samples = acceptance._samples if acceptance is not None else []
        report = {
            "schema_version": 1,
            "acceptance_id": "p4c4_traffic_light_vehicle_closed_loop_v1",
            "passed": False,
            "error": str(error),
            "samples": samples,
        }
        _write_report(path, report)
        rospy.logerr("P4-C4 VEHICLE FAIL: %s; report=%s", error, path)
        return 1


if __name__ == "__main__":
    sys.exit(main())
