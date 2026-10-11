#!/usr/bin/env python3
"""Measure MORAI steady-state steering curvature and save a rosbag.

The node exclusively owns /ctrl_cmd. Run it in a clear, flat open area with
the autonomy/MPC launch stopped. A trial must be explicitly armed.
"""

import json
import math
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


class SteeringCalibrationNode:
    def __init__(self):
        self.steering_deg = float(rospy.get_param("~steering_deg", 20.0))
        self.steering_rad = math.radians(self.steering_deg)
        self.target_speed = float(rospy.get_param("~target_speed_mps", 2.5))
        self.speed_deadband = float(rospy.get_param("~speed_deadband_mps", 0.2))
        self.approach_accel = float(rospy.get_param("~approach_accel", 0.18))
        self.hold_accel = float(rospy.get_param("~hold_accel", 0.08))
        self.hold_brake = float(rospy.get_param("~hold_brake", 0.05))
        self.straight_roll = float(rospy.get_param("~straight_roll_s", 1.0))
        self.settle_time = float(rospy.get_param("~settle_time_s", 4.0))
        self.sample_time = float(rospy.get_param("~sample_time_s", 8.0))
        self.stop_speed = float(rospy.get_param("~stop_speed_mps", 0.08))
        self.stop_hold = float(rospy.get_param("~stop_hold_s", 1.0))
        self.max_speed = float(rospy.get_param("~max_speed_mps", 4.0))
        self.max_trial_time = float(rospy.get_param("~max_trial_time_s", 45.0))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.armed = bool(rospy.get_param("~armed", False))
        self.state_topic = str(rospy.get_param("~state_topic", "/vehicle/state"))
        self.ctrl_topic = str(rospy.get_param("~ctrl_topic", "/ctrl_cmd"))

        if not self.armed:
            raise RuntimeError("trial is disarmed; clear the area and pass _armed:=true")
        if not 0.0 < abs(self.steering_deg) <= 40.0:
            raise ValueError("~steering_deg must be in [-40, 40] excluding zero")
        if not 0.5 <= self.target_speed <= self.max_speed:
            raise ValueError("require 0.5 <= target_speed_mps <= max_speed_mps")
        if self.settle_time < 1.0 or self.sample_time < 3.0:
            raise ValueError("settle_time_s >= 1 and sample_time_s >= 3 are required")
        for name, value in (("approach_accel", self.approach_accel),
                            ("hold_accel", self.hold_accel),
                            ("hold_brake", self.hold_brake)):
            if not 0.0 <= value <= 1.0:
                raise ValueError("~{} must be in [0, 1]".format(name))

        package_path = rospkg.RosPack().get_path("asmc_planner")
        log_dir = os.path.join(package_path, "logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        side = "left" if self.steering_deg > 0.0 else "right"
        default_name = "steering_{}_{:03d}deg_v{:03d}_{}.bag".format(
            side, int(round(abs(self.steering_deg))),
            int(round(self.target_speed * 10.0)), timestamp)
        requested_path = str(rospy.get_param("~bag_path", ""))
        self.bag_path = requested_path or os.path.join(log_dir, default_name)
        if not os.path.isabs(self.bag_path):
            self.bag_path = os.path.join(log_dir, self.bag_path)
        self.params_path = os.path.splitext(self.bag_path)[0] + "_params.yaml"

        self.lock = threading.Lock()
        self.bag = rosbag.Bag(self.bag_path, "w")
        self.speed = None
        self.yaw_rate = None
        self.state_valid = False
        self.last_state_wall = None
        self.phase = "WAIT_STATE"
        self.phase_started = rospy.Time.now()
        self.trial_started = None
        self.stopped_since = None
        self.finished = False
        self.closed = False

        self.ctrl_pub = rospy.Publisher(self.ctrl_topic, CtrlCmd, queue_size=1)
        self.status_topic = "/steering_calibration/status"
        self.metadata_topic = "/steering_calibration/metadata"
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10,
                                          latch=True)
        self.metadata_pub = rospy.Publisher(self.metadata_topic, String,
                                            queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.state_topic, VehicleState, self._state_cb,
                             queue_size=100),
            rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._record_cb,
                             callback_args="/Ego_topic", queue_size=100),
            rospy.Subscriber("/gps", GPSMessage, self._record_cb,
                             callback_args="/gps", queue_size=100),
            rospy.Subscriber("/imu", Imu, self._record_cb,
                             callback_args="/imu", queue_size=200),
        ]
        with open(self.params_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(self._parameters(), stream, sort_keys=False)

        rospy.on_shutdown(self._shutdown)
        rospy.sleep(0.8)
        self._assert_exclusive_control()
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.control_rate)),
                                 self._tick)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self._status_tick)
        self._publish_metadata()
        self._publish_status("READY")
        rospy.logwarn("STEERING CALIBRATION ARMED: steer=%+.1f deg target=%.2f m/s",
                      self.steering_deg, self.target_speed)
        rospy.loginfo("Recording calibration bag: %s", self.bag_path)

    def _parameters(self):
        return {
            "format_version": 1,
            "vehicle": "MORAI IONIQ 5",
            "longitudinal_command_type": 1,
            "steering_deg": self.steering_deg,
            "steering_rad": self.steering_rad,
            "target_speed_mps": self.target_speed,
            "speed_deadband_mps": self.speed_deadband,
            "approach_accel": self.approach_accel,
            "hold_accel": self.hold_accel,
            "hold_brake": self.hold_brake,
            "straight_roll_s": self.straight_roll,
            "settle_time_s": self.settle_time,
            "sample_time_s": self.sample_time,
            "max_speed_mps": self.max_speed,
            "control_rate_hz": self.control_rate,
            "state_topic": self.state_topic,
            "ctrl_topic": self.ctrl_topic,
            "recorded_topics": [self.ctrl_topic, self.state_topic, "/Ego_topic",
                                "/gps", "/imu", self.status_topic,
                                self.metadata_topic],
        }

    def _write(self, topic, message):
        with self.lock:
            if not self.closed:
                self.bag.write(topic, message, rospy.Time.now())

    def _record_cb(self, message, topic):
        self._write(topic, message)

    def _state_cb(self, message):
        self.speed = float(message.speed)
        self.yaw_rate = float(message.yaw_rate)
        self.state_valid = bool(message.valid)
        self.last_state_wall = rospy.get_time()
        self._write(self.state_topic, message)

    def _assert_exclusive_control(self):
        publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0]).get(
            self.ctrl_topic, [])
        others = [name for name in publishers if name != rospy.get_name()]
        if others:
            raise RuntimeError("other {} publishers are active: {}".format(
                self.ctrl_topic, ", ".join(others)))

    @staticmethod
    def _command(accel=0.0, brake=0.0, steering=0.0):
        command = CtrlCmd()
        command.longlCmdType = 1
        command.accel = accel
        command.brake = brake
        command.steering = steering
        return command

    def _publish_command(self, accel=0.0, brake=0.0, steering=0.0):
        command = self._command(accel, brake, steering)
        self.ctrl_pub.publish(command)
        self._write(self.ctrl_topic, command)

    def _speed_hold_command(self, steering):
        if self.speed < self.target_speed - self.speed_deadband:
            self._publish_command(accel=self.hold_accel, steering=steering)
        elif self.speed > self.target_speed + self.speed_deadband:
            self._publish_command(brake=self.hold_brake, steering=steering)
        else:
            self._publish_command(steering=steering)

    def _set_phase(self, phase):
        if phase != self.phase:
            self.phase = phase
            self.phase_started = rospy.Time.now()
            self.stopped_since = None
            self._publish_status("PHASE_CHANGE")
            rospy.loginfo("Steering calibration phase: %s", phase)

    def _publish_status(self, event=""):
        curvature = None
        if self.speed is not None and self.yaw_rate is not None and self.speed > 0.3:
            curvature = self.yaw_rate / self.speed
        payload = {
            "stamp": rospy.Time.now().to_sec(), "event": event,
            "phase": self.phase, "steering_deg": self.steering_deg,
            "steering_rad": self.steering_rad, "target_speed_mps": self.target_speed,
            "speed_mps": self.speed, "yaw_rate_radps": self.yaw_rate,
            "instant_curvature_1pm": curvature,
            "phase_elapsed_s": (rospy.Time.now() - self.phase_started).to_sec(),
            "bag_path": self.bag_path,
        }
        message = String(data=json.dumps(payload, sort_keys=True))
        self.status_pub.publish(message)
        self._write(self.status_topic, message)

    def _publish_metadata(self):
        message = String(data=json.dumps(self._parameters(), sort_keys=True))
        self.metadata_pub.publish(message)
        self._write(self.metadata_topic, message)

    def _status_tick(self, _event):
        if not self.finished:
            self._publish_status("SAMPLE")

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
            return self._abort("ABORT_TIMEOUT", "Calibration trial timed out")
        if self.speed > self.max_speed:
            return self._abort("ABORT_OVERSPEED", "Calibration exceeded max speed")

        if self.phase == "WAIT_STATE":
            self._set_phase("STRAIGHT_ROLL")
        if self.phase == "STRAIGHT_ROLL":
            if self.speed < self.target_speed:
                self._publish_command(accel=self.approach_accel)
                return
            self._publish_command()
            if (now - self.phase_started).to_sec() >= self.straight_roll:
                self._set_phase("SETTLE_TURN")
            return
        if self.phase == "SETTLE_TURN":
            self._speed_hold_command(self.steering_rad)
            if (now - self.phase_started).to_sec() >= self.settle_time:
                self._set_phase("MEASURE_TURN")
            return
        if self.phase == "MEASURE_TURN":
            self._speed_hold_command(self.steering_rad)
            if (now - self.phase_started).to_sec() >= self.sample_time:
                self._set_phase("SAFETY_STOP")
            return
        if self.phase == "SAFETY_STOP":
            self._publish_command(brake=1.0, steering=0.0)
            if self.speed <= self.stop_speed:
                if self.stopped_since is None:
                    self.stopped_since = now
                if (now - self.stopped_since).to_sec() >= self.stop_hold:
                    self._set_phase("COMPLETE")
                    self._finish(True)
            else:
                self.stopped_since = None

    def _abort(self, phase, message):
        rospy.logerr(message)
        self._set_phase(phase)
        self._publish_command(brake=1.0)
        self._finish(False)

    def _finish(self, success):
        self._publish_status("COMPLETE" if success else "ABORT")
        self.finished = True
        rospy.loginfo("Steering calibration %s; bag saved: %s",
                      "complete" if success else "aborted", self.bag_path)
        rospy.signal_shutdown("steering calibration trial finished")

    def _shutdown(self):
        if self.closed:
            return
        for _ in range(5):
            self.ctrl_pub.publish(self._command(brake=1.0))
            time.sleep(0.03)
        with self.lock:
            if not self.closed:
                self.bag.close()
                self.closed = True


def main():
    rospy.init_node("steering_calibration")
    try:
        SteeringCalibrationNode()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Steering calibration refused to start: %s", error)
        raise


if __name__ == "__main__":
    main()
