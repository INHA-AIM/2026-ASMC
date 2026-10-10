#!/usr/bin/env python3
"""Run one MORAI pedal calibration trial and record it to a rosbag.

This node owns /ctrl_cmd during a trial. Do not run the autonomy launch at the
same time. A trial must be explicitly armed with ``_armed:=true``.
"""

import json
import os
import threading
import time
from datetime import datetime

import rosbag
import rosgraph
import rospkg
import rospy
import yaml
from asmc_msgs.msg import VehicleState
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class PedalCalibrationNode:
    def __init__(self):
        self.mode = str(rospy.get_param("~mode", "brake")).lower()
        self.pedal = float(rospy.get_param("~pedal", 0.2))
        self.target_speed = float(rospy.get_param("~target_speed_mps", 10.0))
        self.approach_accel = float(rospy.get_param("~approach_accel", 0.25))
        self.max_speed = float(rospy.get_param("~max_speed_mps", 12.0))
        self.max_trial_time = float(rospy.get_param("~max_trial_time_s", 45.0))
        self.stop_speed = float(rospy.get_param("~stop_speed_mps", 0.08))
        self.stop_hold = float(rospy.get_param("~stop_hold_s", 1.0))
        self.pre_roll = float(rospy.get_param("~pre_roll_s", 2.0))
        self.coast_before_brake = float(
            rospy.get_param("~coast_before_brake_s", 0.5))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.armed = bool(rospy.get_param("~armed", False))
        self.state_topic = str(rospy.get_param("~state_topic", "/vehicle/state"))
        self.ctrl_topic = str(rospy.get_param("~ctrl_topic", "/ctrl_cmd"))

        if self.mode not in ("accel", "brake"):
            raise ValueError("~mode must be 'accel' or 'brake'")
        if not 0.0 < self.pedal <= 1.0:
            raise ValueError("~pedal must be in (0, 1]")
        if not 0.0 < self.approach_accel <= 1.0:
            raise ValueError("~approach_accel must be in (0, 1]")
        if self.target_speed <= 0.0 or self.max_speed < self.target_speed:
            raise ValueError("require 0 < target_speed_mps <= max_speed_mps")
        if self.pre_roll < 0.0 or self.coast_before_brake < 0.0:
            raise ValueError("pre-roll and coast durations must be non-negative")
        if not self.armed:
            raise RuntimeError("trial is disarmed; pass _armed:=true after clearing the test area")

        package_path = rospkg.RosPack().get_path("asmc_planner")
        log_dir = os.path.join(package_path, "logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = "pedal_{}_p{:03d}_v{:03d}_{}.bag".format(
            self.mode, int(round(self.pedal * 100)),
            int(round(self.target_speed * 10)), timestamp)
        requested_path = str(rospy.get_param("~bag_path", ""))
        self.bag_path = requested_path or os.path.join(log_dir, default_name)
        if not os.path.isabs(self.bag_path):
            self.bag_path = os.path.join(log_dir, self.bag_path)
        self.params_path = os.path.splitext(self.bag_path)[0] + "_params.yaml"
        with open(self.params_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(self._trial_parameters(), stream, sort_keys=False)

        self.lock = threading.Lock()
        self.bag = rosbag.Bag(self.bag_path, "w")
        self.speed = None
        self.state_valid = False
        self.last_state_wall = None
        self.phase = "WAIT_STATE"
        self.phase_started = rospy.Time.now()
        self.trial_started = None
        self.stopped_since = None
        self.finished = False
        self.closed = False

        self.ctrl_pub = rospy.Publisher(self.ctrl_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/pedal_calibration/status", String, queue_size=10, latch=True)
        self.metadata_pub = rospy.Publisher(
            "/pedal_calibration/metadata", String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.state_topic, VehicleState, self._state_cb,
                             queue_size=100),
            rospy.Subscriber("/Ego_topic", EgoVehicleStatus,
                             self._record_cb, callback_args="/Ego_topic",
                             queue_size=100),
            rospy.Subscriber("/gps", GPSMessage, self._record_cb,
                             callback_args="/gps", queue_size=100),
            rospy.Subscriber("/imu", Imu, self._record_cb,
                             callback_args="/imu", queue_size=200),
        ]
        rospy.on_shutdown(self._shutdown)
        rospy.sleep(0.8)
        self._assert_exclusive_control()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.control_rate)), self._tick)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self._status_tick)
        self._publish_metadata()
        self._publish_status("READY")
        rospy.logwarn("PEDAL CALIBRATION ARMED: mode=%s pedal=%.3f target=%.2f m/s",
                      self.mode, self.pedal, self.target_speed)
        rospy.loginfo("Recording calibration bag: %s", self.bag_path)
        rospy.loginfo("Recording calibration parameters: %s", self.params_path)

    def _trial_parameters(self):
        return {
            "format_version": 1,
            "vehicle": "MORAI IONIQ 5",
            "longitudinal_command_type": 1,
            "mode": self.mode,
            "pedal": self.pedal,
            "target_speed_mps": self.target_speed,
            "approach_accel": self.approach_accel,
            "max_speed_mps": self.max_speed,
            "max_trial_time_s": self.max_trial_time,
            "stop_speed_mps": self.stop_speed,
            "stop_hold_s": self.stop_hold,
            "pre_roll_s": self.pre_roll,
            "coast_before_brake_s": self.coast_before_brake,
            "control_rate_hz": self.control_rate,
            "state_topic": self.state_topic,
            "ctrl_topic": self.ctrl_topic,
            "recorded_topics": [
                self.ctrl_topic, self.state_topic, "/Ego_topic", "/gps",
                "/imu", "/pedal_calibration/status",
                "/pedal_calibration/metadata",
            ],
        }

    def _write(self, topic, message):
        with self.lock:
            if not self.closed:
                self.bag.write(topic, message, rospy.Time.now())

    def _record_cb(self, message, topic):
        self._write(topic, message)

    def _state_cb(self, message):
        self.speed = float(message.speed)
        self.state_valid = bool(message.valid)
        self.last_state_wall = rospy.get_time()
        self._write(self.state_topic, message)

    def _assert_exclusive_control(self):
        state = rosgraph.Master(rospy.get_name()).getSystemState()
        publishers = dict(state[0]).get(self.ctrl_topic, [])
        others = [name for name in publishers if name != rospy.get_name()]
        if others:
            raise RuntimeError("other {} publishers are active: {}".format(
                self.ctrl_topic, ", ".join(others)))

    @staticmethod
    def _command(accel=0.0, brake=0.0):
        command = CtrlCmd()
        command.longlCmdType = 1
        command.accel = accel
        command.brake = brake
        command.steering = 0.0
        return command

    def _publish_command(self, accel=0.0, brake=0.0):
        command = self._command(accel, brake)
        self.ctrl_pub.publish(command)
        self._write(self.ctrl_topic, command)

    def _publish_status(self, event=""):
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "event": event,
            "mode": self.mode,
            "phase": self.phase,
            "pedal": self.pedal,
            "target_speed_mps": self.target_speed,
            "speed_mps": self.speed,
            "phase_elapsed_s": (rospy.Time.now() - self.phase_started).to_sec(),
            "bag_path": self.bag_path,
        }
        message = String(data=json.dumps(payload, sort_keys=True))
        self.status_pub.publish(message)
        self._write("/pedal_calibration/status", message)

    def _publish_metadata(self):
        message = String(data=json.dumps(self._trial_parameters(), sort_keys=True))
        self.metadata_pub.publish(message)
        self._write("/pedal_calibration/metadata", message)

    def _status_tick(self, _event):
        if not self.finished:
            self._publish_status("SAMPLE")

    def _set_phase(self, phase):
        if phase != self.phase:
            self.phase = phase
            self.phase_started = rospy.Time.now()
            self.stopped_since = None
            self._publish_status("PHASE_CHANGE")
            rospy.loginfo("Pedal calibration phase: %s", phase)

    def _state_is_fresh(self):
        return (self.state_valid and self.last_state_wall is not None and
                rospy.get_time() - self.last_state_wall < 0.25)

    def _tick(self, _event):
        if self.finished:
            return
        now = rospy.Time.now()
        if not self._state_is_fresh():
            self._publish_command(brake=1.0)
            self._set_phase("WAIT_STATE")
            return

        if self.trial_started is None:
            self.trial_started = now
        if (now - self.trial_started).to_sec() > self.max_trial_time:
            rospy.logerr("Calibration trial timeout; applying emergency brake")
            self._set_phase("ABORT_TIMEOUT")
            self._publish_command(brake=1.0)
            self._finish(False)
            return
        if self.speed > self.max_speed:
            rospy.logerr("Calibration exceeded max speed; applying emergency brake")
            self._set_phase("ABORT_OVERSPEED")
            self._publish_command(brake=1.0)
            self._finish(False)
            return

        if self.phase == "WAIT_STATE":
            self._set_phase("PRE_ROLL")

        if self.phase == "PRE_ROLL":
            self._publish_command(brake=0.3)
            if (now - self.phase_started).to_sec() >= self.pre_roll:
                self._set_phase("ACCELERATE")
            else:
                return

        if self.phase == "ACCELERATE":
            accel = self.pedal if self.mode == "accel" else self.approach_accel
            if self.speed >= self.target_speed:
                self._set_phase("COAST" if self.mode == "brake"
                                else "SAFETY_STOP")
            else:
                self._publish_command(accel=accel)
                return

        if self.phase == "COAST":
            self._publish_command()
            if ((now - self.phase_started).to_sec() >=
                    self.coast_before_brake):
                self._set_phase("MEASURE_BRAKE")
            else:
                return

        if self.phase == "MEASURE_BRAKE":
            self._publish_command(brake=self.pedal)
        elif self.phase == "SAFETY_STOP":
            self._publish_command(brake=1.0)

        if self.phase in ("MEASURE_BRAKE", "SAFETY_STOP"):
            if self.speed <= self.stop_speed:
                if self.stopped_since is None:
                    self.stopped_since = now
                if (now - self.stopped_since).to_sec() >= self.stop_hold:
                    self._set_phase("COMPLETE")
                    self._publish_command(brake=1.0)
                    self._finish(True)
            else:
                self.stopped_since = None

    def _finish(self, success):
        self._publish_status("COMPLETE" if success else "ABORT")
        self.finished = True
        rospy.loginfo("Calibration %s; bag saved: %s",
                      "complete" if success else "aborted", self.bag_path)
        rospy.signal_shutdown("calibration trial finished")

    def _shutdown(self):
        if self.closed:
            return
        # Publish repeatedly because UDP control can lose a single packet.
        for _ in range(5):
            self.ctrl_pub.publish(self._command(brake=1.0))
            time.sleep(0.03)
        with self.lock:
            if not self.closed:
                self.bag.close()
                self.closed = True


def main():
    rospy.init_node("pedal_calibration")
    try:
        PedalCalibrationNode()
        rospy.spin()
    except Exception as error:  # Ensure roslaunch reports configuration errors.
        rospy.logfatal("Pedal calibration refused to start: %s", error)
        raise


if __name__ == "__main__":
    main()
