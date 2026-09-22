#!/usr/bin/env python3
"""
MORAI UDP <-> ROS1 bridge

rosbridge(websocket/JSON) 없이 UDP 바이너리를 직접 파싱/생성한다.

  UDP -> ROS   /Ego_topic      morai_msgs/EgoVehicleStatus
               /Object_topic   morai_msgs/ObjectStatusList
               /CollisionData  morai_msgs/CollisionData
  ROS -> UDP   /ctrl_cmd       morai_msgs/CtrlCmd

단위 / ASMC beta_drive 필드:
  EgoVehicleStatus.velocity     m/s   <- UDP km/h 변환
  EgoVehicleStatus.wheel_angle  deg   <- UDP front_steer
  ObjectStatus.velocity         km/h
  CtrlCmd.steering              rad   -> UDP -1~1 정규화
"""

import math
import socket
import struct
import threading
import time
from collections import deque

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from morai_msgs.msg import (CollisionData, CtrlCmd, EgoVehicleStatus,
                            ObjectStatus, ObjectStatusList)

KMH2MS = 1.0 / 3.6
AUX = 4 + 12
TAIL = 2

# --------------------------------------------------------------------------
# Ego Vehicle Status
#   body 공통부 (offset)
#     0  uint32 sec / 4 uint32 nsec / 8 int8 ctrl_mode / 9 int8 gear
#    10  f signed_velocity(km/h)   14 i map_data_id
#    18  f accel   22 f brake
#    26  3f size            38 3f overhang/wheelbase/rear_overhang
#    50  3f position        62 3f roll/pitch/heading(deg)
#    74  3f velocity(km/h)  86 3f angular_velocity(deg/s)
#    98  3f acceleration    110 f front_steer(deg)
#   이후 버전별 분기 (패킷 전체 길이로 판별)
# --------------------------------------------------------------------------
EGO_HDR = b"#MoraiInfo$"
EGO_PRE = len(EGO_HDR) + AUX          # 27
EGO_HEAD_FMT = "<IIbb"
EGO_COMMON_FMT = "<25f"               # offset 10~109
LINK_LEN = 38

# total_len -> (rear_steer_off, link_off, ext_off, ext_count)
EGO_VARIANTS = {
    181: (None, 114, 152, 0),         # beta: 공통부 + link_id, 확장 필드 없음
    229: (None, 114, 152, 12),        # rear_steer 없음, 확장 12개
    245: (114,  118, 156, 15),        # 공식 예제 기준
}
EXT_NAMES = [
    "tire_lateral_force_fl", "tire_lateral_force_fr",
    "tire_lateral_force_rl", "tire_lateral_force_rr",
    "side_slip_angle_fl", "side_slip_angle_fr",
    "side_slip_angle_rl", "side_slip_angle_rr",
    "tire_cornering_stiffness_fl", "tire_cornering_stiffness_fr",
    "tire_cornering_stiffness_rl", "tire_cornering_stiffness_rr",
    "distance_left_lane_boundary", "distance_right_lane_boundary",
    "cross_track_error",
]

# --------------------------------------------------------------------------
OBJ_HDR = b"#MoraiObjInfo$"
OBJ_PRE = len(OBJ_HDR) + AUX          # 30
OBJ_ONE_FMT = "<hh" + "fff" + "f" + "fff" + "fff" + "fff" + "fff" + "38s"
OBJ_ONE = 106                          # 신버전(link_id 포함). 구버전은 68
OBJ_N = 20

COL_HDR = b"#CollisionData$"
COL_PRE = len(COL_HDR) + AUX          # 31
COL_ONE_FMT = "<hh" + "fff" + "fff"
COL_ONE = 28
COL_N = 5

CMD_HDR = b"#MoraiCtrlCmd$"
CMD_FMT_27 = "<BBB" + "ffffff"        # 신버전(rear_steer 포함), 전체 59B
CMD_FMT_23 = "<BBB" + "fffff"         # 구버전, 전체 55B


def latency_summary(samples):
    """Return a compact rolling latency summary without external dependencies."""
    values = sorted(float(value) for value in samples)
    if not values:
        return {"samples": 0, "mean_ms": 0.0, "p50_ms": 0.0,
                "p95_ms": 0.0, "max_ms": 0.0}

    def percentile(percent):
        position = (len(values) - 1) * float(percent) / 100.0
        low = int(math.floor(position))
        high = int(math.ceil(position))
        if low == high:
            return values[low]
        return (values[low] * (high - position)
                + values[high] * (position - low))

    return {
        "samples": len(values),
        "mean_ms": sum(values) / len(values),
        "p50_ms": percentile(50),
        "p95_ms": percentile(95),
        "max_ms": values[-1],
    }


def vec3(x, y, z):
    v = Vector3()
    v.x, v.y, v.z = float(x), float(y), float(z)
    return v


class MoraiUdpBridge(object):

    def __init__(self):
        p = rospy.get_param
        self.ego_port = p("~ego_port", 9099)
        self.obj_port = p("~object_port", 9021)
        self.col_port = p("~collision_port", 9031)
        self.cmd_ip = p("~cmd_ip", "127.0.0.1")
        self.cmd_port = p("~cmd_port", 9093)          # MORAI 의 Host PORT
        self.max_steer_deg = float(p("~max_steer_deg", 40.0))
        self.cmd_bytes = int(p("~cmd_payload_bytes", 27))
        self.invert_steer = bool(p("~invert_steer", False))
        self.frame_id = p("~frame_id", "base_link")
        self.ego_vel_ms = bool(p("~ego_velocity_in_ms", True))
        self.runtime_profile = str(p("~runtime_profile", "morai_udp"))
        self.transport_profile = str(p("~transport_profile", "local"))
        self.timestamp_mode = str(p("~timestamp_mode", "receive")).strip().lower()
        if self.timestamp_mode not in ("receive", "source"):
            raise ValueError("timestamp_mode must be receive or source")

        self.pub_ego = rospy.Publisher("/Ego_topic", EgoVehicleStatus,
                                       queue_size=1)
        self.pub_obj = rospy.Publisher("/Object_topic", ObjectStatusList,
                                       queue_size=1)
        self.pub_col = rospy.Publisher("/CollisionData", CollisionData,
                                       queue_size=1)
        self.pub_diagnostics = rospy.Publisher(
            "/morai_udp_bridge/diagnostics", DiagnosticArray, queue_size=1
        )

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.on_ctrl_cmd, queue_size=1)

        self.last_link_id = ""
        self.stats = {"ego": 0, "obj": 0, "col": 0, "cmd": 0}
        # Rolling callback wall time. UDP inputs cover validated datagram
        # parse->ROS publish enqueue; cmd covers ROS callback->UDP sendto.
        self.processing_ms = {
            name: deque(maxlen=4096) for name in self.stats
        }
        self.last_activity_monotonic = dict((name, None) for name in self.stats)
        self.timestamp_fallbacks = {"ego": 0, "obj": 0, "col": 0}
        self.timestamp_regressions = {"ego": 0, "obj": 0, "col": 0}
        self._last_source_ns = {}
        self._started_monotonic = time.monotonic()
        self._last_report_monotonic = self._started_monotonic
        self._last_report_stats = dict(self.stats)
        self.warned = set()

        for port, kind, hdr, cb in (
                (self.ego_port, "ego", EGO_HDR, self.handle_ego),
                (self.obj_port, "obj", OBJ_HDR, self.handle_obj),
                (self.col_port, "col", COL_HDR, self.handle_col)):
            t = threading.Thread(target=self.rx_loop,
                                 args=(port, kind, hdr, cb))
            t.daemon = True
            t.start()

        rospy.Timer(rospy.Duration(5.0), self.report)

    # ---------------- 수신 루프 ----------------
    def rx_loop(self, port, kind, hdr, cb):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            sock.bind(("0.0.0.0", port))
        except Exception as e:
            rospy.logerr("bind %d (%s) failed: %s", port, kind, e)
            return
        sock.settimeout(1.0)
        rospy.loginfo("listening %-3s on udp/%d", kind, port)

        while not rospy.is_shutdown():
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                continue
            if not data.startswith(hdr):
                if port not in self.warned:
                    self.warned.add(port)
                    rospy.logwarn("udp/%d: expected %s, got %s",
                                  port, hdr, data[:16])
                continue
            try:
                processing_started = time.perf_counter_ns()
                cb(data)
                self.processing_ms[kind].append(
                    (time.perf_counter_ns() - processing_started) / 1000000.0
                )
                self.stats[kind] += 1
                self.last_activity_monotonic[kind] = time.monotonic()
            except struct.error as e:
                rospy.logwarn_throttle(5.0, "parse error udp/%d (%dB): %s",
                                       port, len(data), e)
            except Exception as e:
                rospy.logwarn_throttle(5.0, "handle %s udp/%d (%dB): %s",
                                       kind, port, len(data), e)
        sock.close()

    # ---------------- Ego ----------------
    def handle_ego(self, data):
        var = EGO_VARIANTS.get(len(data))
        if var is None:
            if "egolen" not in self.warned:
                self.warned.add("egolen")
                rospy.logerr("Ego %dB 는 미지의 버전입니다 (181/229/245 만 지원). "
                             "morai_ego_probe.py 로 구조를 확인하세요.",
                             len(data))
            return
        rear_off, link_off, ext_off, ext_n = var

        body = data[EGO_PRE:-TAIL]
        sec, nsec, ctrl_mode, gear = struct.unpack(EGO_HEAD_FMT, body[0:10])
        f = struct.unpack(EGO_COMMON_FMT, body[10:110])
        front_steer = struct.unpack("<f", body[110:114])[0]
        ext = struct.unpack("<%df" % ext_n,
                            body[ext_off:ext_off + ext_n * 4])

        m = EgoVehicleStatus()
        m.header.stamp = self.packet_stamp(sec, nsec, "ego")
        m.header.frame_id = self.frame_id
        m.unique_id = -1

        m.accel = f[2]
        m.brake = f[3]
        m.position = vec3(f[10], f[11], f[12])
        m.heading = f[15]

        k = KMH2MS if self.ego_vel_ms else 1.0
        m.velocity = vec3(f[16] * k, f[17] * k, f[18] * k)
        # beta_drive 에는 angular_velocity 없음 (va msg 전용)
        if hasattr(m, "angular_velocity"):
            m.angular_velocity = vec3(f[19], f[20], f[21])
        m.acceleration = vec3(f[22], f[23], f[24])

        # beta_drive: wheel_angle / va: front_steer_angle
        if hasattr(m, "wheel_angle"):
            m.wheel_angle = front_steer
        if hasattr(m, "front_steer_angle"):
            m.front_steer_angle = front_steer
        if hasattr(m, "rear_steer_angle") and rear_off is not None:
            m.rear_steer_angle = struct.unpack(
                "<f", body[rear_off:rear_off + 4])[0]

        for name, val in zip(EXT_NAMES, ext):
            if hasattr(m, name):
                setattr(m, name, val)

        self.last_link_id = body[link_off:link_off + LINK_LEN] \
            .decode("utf-8", "ignore").strip("\x00 ")

        self.pub_ego.publish(m)

    # ---------------- Object ----------------
    def handle_obj(self, data):
        body = data[OBJ_PRE:-TAIL]
        sec, nsec = struct.unpack("<II", body[0:8])
        off = 8

        msg = ObjectStatusList()
        msg.header.stamp = self.packet_stamp(sec, nsec, "obj")
        msg.header.frame_id = self.frame_id

        for _ in range(OBJ_N):
            if off + OBJ_ONE > len(body):
                break
            v = struct.unpack(OBJ_ONE_FMT, body[off:off + OBJ_ONE])
            off += OBJ_ONE
            obj_id, obj_type = v[0], v[1]
            if obj_id == 0 and abs(v[2]) < 1e-6 and abs(v[3]) < 1e-6:
                continue

            o = ObjectStatus()
            o.unique_id = obj_id
            o.type = obj_type
            o.name = v[18].decode("utf-8", "ignore").strip("\x00 ")
            o.position = vec3(v[2], v[3], v[4])
            o.heading = v[5]
            o.size = vec3(v[6], v[7], v[8])
            o.velocity = vec3(v[12], v[13], v[14])     # km/h
            o.acceleration = vec3(v[15], v[16], v[17])

            if obj_type == 0:
                msg.pedestrian_list.append(o)
            elif obj_type == 1:
                msg.npc_list.append(o)
            else:
                msg.obstacle_list.append(o)

        msg.num_of_npcs = len(msg.npc_list)
        msg.num_of_pedestrian = len(msg.pedestrian_list)
        msg.num_of_obstacle = len(msg.obstacle_list)
        self.pub_obj.publish(msg)

    # ---------------- Collision ----------------
    def handle_col(self, data):
        body = data[COL_PRE:-TAIL]
        sec, nsec = struct.unpack("<II", body[0:8])
        off = 8
        msg = CollisionData()
        msg.header.stamp = self.packet_stamp(sec, nsec, "col")
        msg.header.frame_id = self.frame_id
        hit = False
        for _ in range(COL_N):
            if off + COL_ONE > len(body):
                break
            v = struct.unpack(COL_ONE_FMT, body[off:off + COL_ONE])
            off += COL_ONE
            if v[1] == 0 and abs(v[2]) < 1e-6 and abs(v[3]) < 1e-6:
                continue
            o = ObjectStatus()
            o.type, o.unique_id = v[0], v[1]
            o.position = vec3(v[2], v[3], v[4])
            msg.collision_object.append(o)
            msg.global_offset_x, msg.global_offset_y, msg.global_offset_z = \
                v[5], v[6], v[7]
            hit = True
        if hit:
            self.pub_col.publish(msg)

    # ---------------- Ctrl Cmd 송신 ----------------
    def on_ctrl_cmd(self, msg):
        processing_started = time.perf_counter_ns()
        # beta_drive: steering / va: front_steer, rear_steer  [rad]
        if hasattr(msg, "steering"):
            steer_rad = float(msg.steering)
            rear_rad = 0.0
        else:
            steer_rad = float(getattr(msg, "front_steer", 0.0))
            rear_rad = float(getattr(msg, "rear_steer", 0.0))
        steer = math.degrees(steer_rad) / self.max_steer_deg
        rear = math.degrees(rear_rad) / self.max_steer_deg
        if self.invert_steer:
            steer, rear = -steer, -rear
        steer = max(-1.0, min(1.0, steer))
        rear = max(-1.0, min(1.0, rear))

        base = [2,                       # ctrl_mode : 2 = AutoMode
                4,                       # gear      : 4 = D
                msg.longlCmdType,
                msg.velocity,            # longlCmdType==2 (km/h)
                msg.acceleration,        # longlCmdType==3 (m/s^2)
                msg.accel,
                msg.brake,
                steer]
        if self.cmd_bytes == 27:
            payload = struct.pack(CMD_FMT_27, *(base + [rear]))
        else:
            payload = struct.pack(CMD_FMT_23, *base)

        pkt = (CMD_HDR + struct.pack("<i", len(payload))
               + b"\x00" * 12 + payload + b"\r\n")
        try:
            self.cmd_sock.sendto(pkt, (self.cmd_ip, self.cmd_port))
            self.processing_ms["cmd"].append(
                (time.perf_counter_ns() - processing_started) / 1000000.0
            )
            self.stats["cmd"] += 1
            self.last_activity_monotonic["cmd"] = time.monotonic()
        except Exception as e:
            rospy.logwarn_throttle(5.0, "ctrl_cmd send failed: %s", e)

    # ---------------- 상태 ----------------
    def packet_stamp(self, sec, nsec, kind):
        if self.timestamp_mode == "receive":
            return rospy.Time.now()
        if int(sec) <= 0 or not 0 <= int(nsec) < 1000000000:
            self.timestamp_fallbacks[kind] += 1
            return self.fallback_stamp(kind)
        stamp = rospy.Time(int(sec), int(nsec))
        value_ns = int(stamp.to_nsec())
        previous = self._last_source_ns.get(kind)
        if previous is not None and value_ns < previous:
            self.timestamp_fallbacks[kind] += 1
            self.timestamp_regressions[kind] += 1
            return self.fallback_stamp(kind)
        self._last_source_ns[kind] = value_ns
        return stamp

    def fallback_stamp(self, kind):
        stamp = rospy.Time.now()
        value_ns = int(stamp.to_nsec())
        previous = self._last_source_ns.get(kind)
        if previous is not None and value_ns <= previous:
            value_ns = previous + 1
            stamp = rospy.Time(value_ns // 1000000000, value_ns % 1000000000)
        self._last_source_ns[kind] = value_ns
        return stamp

    def report(self, _):
        now = time.monotonic()
        report_elapsed = max(1e-6, now - self._last_report_monotonic)
        interval_rates = dict(
            (
                name,
                max(0, count - self._last_report_stats.get(name, 0))
                / report_elapsed,
            )
            for name, count in self.stats.items()
        )
        rospy.loginfo(
            "rx ego=%d(%.1fHz) obj=%d(%.1fHz) col=%d(%.1fHz) | "
            "tx cmd=%d(%.1fHz) | link=%s",
            self.stats["ego"], interval_rates["ego"],
            self.stats["obj"], interval_rates["obj"],
            self.stats["col"], interval_rates["col"],
            self.stats["cmd"], interval_rates["cmd"],
            self.last_link_id or "-",
        )
        self._last_report_stats = dict(self.stats)
        self._last_report_monotonic = now
        if self.stats["ego"] == 0:
            rospy.logwarn("Ego 수신 0건. MORAI Destination PORT 와 "
                          "~ego_port 가 같은지 확인하세요.")
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        elapsed = max(1e-6, now - self._started_monotonic)
        for kind in ("ego", "obj", "col", "cmd"):
            metrics = latency_summary(self.processing_ms[kind])
            last_activity = self.last_activity_monotonic[kind]
            last_age_sec = (
                -1.0 if last_activity is None else max(0.0, now - last_activity)
            )
            status = DiagnosticStatus()
            status.name = "morai_udp_bridge/{}".format(kind)
            status.hardware_id = "morai_udp"
            status.level = (
                DiagnosticStatus.OK if self.stats[kind] > 0 or kind == "cmd"
                else DiagnosticStatus.WARN
            )
            if self.stats[kind] > 0:
                status.message = "active"
            elif kind == "cmd":
                status.message = "no control command"
            else:
                status.message = "no data"
            status.values = [
                KeyValue("runtime_profile", self.runtime_profile),
                KeyValue("transport_profile", self.transport_profile),
                KeyValue("timestamp_mode", self.timestamp_mode),
                KeyValue(
                    "direction",
                    "ros_to_udp" if kind == "cmd" else "udp_to_ros",
                ),
                KeyValue("messages", str(self.stats[kind])),
                KeyValue("average_hz", "{:.3f}".format(self.stats[kind] / elapsed)),
                KeyValue("interval_hz", "{:.3f}".format(interval_rates[kind])),
                KeyValue("last_message_age_sec", "{:.6f}".format(last_age_sec)),
                KeyValue("processing_samples", str(metrics["samples"])),
                KeyValue("processing_mean_ms", "{:.6f}".format(metrics["mean_ms"])),
                KeyValue("processing_p50_ms", "{:.6f}".format(metrics["p50_ms"])),
                KeyValue("processing_p95_ms", "{:.6f}".format(metrics["p95_ms"])),
                KeyValue("processing_max_ms", "{:.6f}".format(metrics["max_ms"])),
                KeyValue(
                    "timestamp_fallbacks",
                    str(self.timestamp_fallbacks.get(kind, 0)),
                ),
                KeyValue(
                    "timestamp_regressions",
                    str(self.timestamp_regressions.get(kind, 0)),
                ),
            ]
            message.status.append(status)
        self.pub_diagnostics.publish(message)


def main():
    rospy.init_node("morai_udp_bridge")
    MoraiUdpBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
