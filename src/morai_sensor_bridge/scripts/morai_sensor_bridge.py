#!/usr/bin/env python3
"""
MORAI Sensor UDP -> ROS1 bridge

Sensor Setting(UDP) 으로 나오는 센서를 ROS 토픽으로 변환한다.
Network Settings(Ego/Simulator) 와는 프로토콜이 완전히 다르므로
morai_udp_bridge 와 별도 노드로 돌린다.

  Camera×N JPEG 프래그먼트 -> sensor_msgs/CompressedImage
  Semantic Camera JPEG      -> sensor_msgs/CompressedImage (동일 UDP 프래그먼트)
  3D Lidar Velodyne raw     -> sensor_msgs/PointCloud2
  GPS      NMEA ascii       -> morai_msgs/GPSMessage + sensor_msgs/NavSatFix
  IMU      #IMUData$        -> sensor_msgs/Imu

포맷 근거: MORAI-Autonomous/MORAI-SensorExample
"""

import socket
import struct
import threading
import time
from collections import deque

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import (CompressedImage, Image, Imu, NavSatFix,
                             NavSatStatus, PointCloud2, PointField)
from morai_msgs.msg import GPSMessage

VERTICAL_ANGLES = {
    "HDL-32E": [-30.67, -9.33, -29.33, -8.0, -28.0, -6.67, -26.67, -5.33,
                -25.33, -4.0, -24.0, -2.67, -22.67, -1.33, -21.33, 0.0,
                -20.0, 1.33, -18.67, 2.67, -17.33, 4.0, -16.0, 5.33,
                -14.67, 6.67, -13.33, 8.0, -12.0, 9.33, -10.67, 10.67],
    "VLP-16": [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
               -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0],
    "VLP-32C": [-25.0, -1.0, -1.667, -15.639, -11.31, 0.0, -0.667, -8.843,
                -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4.0,
                -4.667, 1.667, 1.0, -3.667, -3.333, 3.333, 2.333, -2.667,
                -3.0, 7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333],
}

# AIM06 Sensor Dest 기본 (Host/Dest 표의 Destination)
DEFAULT_CAMERAS = [
    {"name": "cam1", "port": 9291,
     "topic": "/image_jpeg/compressed", "frame": "camera_front"},
    {"name": "cam2", "port": 9293,
     "topic": "/cam_left/image_jpeg/compressed", "frame": "camera_left"},
    {"name": "cam3", "port": 9295,
     "topic": "/cam_right/image_jpeg/compressed", "frame": "camera_right"},
    {"name": "cam4", "port": 9297,
     "topic": "/cam_extra/image_jpeg/compressed", "frame": "camera_extra"},
]

# ASMC_sensor_set_GT.json Semantic (cameraType=1) — front/left/right 정렬
DEFAULT_SEMANTIC_CAMERAS = [
    {"name": "sem_front", "port": 9305,
     "topic": "/sem_front/image_jpeg/compressed", "frame": "camera_front_semantic"},
    {"name": "sem_left", "port": 9307,
     "topic": "/sem_left/image_jpeg/compressed", "frame": "camera_left_semantic"},
    {"name": "sem_right", "port": 9309,
     "topic": "/sem_right/image_jpeg/compressed", "frame": "camera_right_semantic"},
]


def latency_summary(samples):
    values = sorted(float(value) for value in samples)
    if not values:
        return {"samples": 0, "mean_ms": 0.0, "p50_ms": 0.0,
                "p95_ms": 0.0, "max_ms": 0.0}

    def percentile(percent):
        position = (len(values) - 1) * float(percent) / 100.0
        low = int(np.floor(position))
        high = int(np.ceil(position))
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


def parse_camera_packet(block):
    """Return ``(sec, nsec, index, payload, is_final)`` for one MORAI JPEG block."""
    if len(block) < 21 or block[:3] != b"MOR":
        raise ValueError("invalid camera packet header or length")
    sec, nsec, index, size = struct.unpack("<IIii", block[3:19])
    tail = block[-2:]
    available = len(block) - 21
    if index < 0 or size < 0 or size > available or tail not in (b"AI", b"EI"):
        raise ValueError("invalid camera packet index, size, or tail")
    return sec, nsec, index, block[19:19 + size], tail == b"EI"


class SensorBridge(object):

    def __init__(self):
        p = rospy.get_param
        self.frame_lidar = p("~lidar_frame", "velodyne")
        self.frame_imu = p("~imu_frame", "imu")
        self.frame_gps = p("~gps_frame", "gps")
        self.simulator_profile = str(
            p("~simulator_profile", "dev_26r1_h3")
        )
        self.timestamp_mode = str(p("~timestamp_mode", "receive")).strip().lower()
        if self.timestamp_mode not in ("receive", "source"):
            raise ValueError("timestamp_mode must be receive or source")
        # NMEA GGA에는 MORAI map 원점 offset이 포함되지 않는다. GPSMessage를
        # 사용하는 인지/플래닝 모듈이 같은 local-map 좌표를 만들 수 있도록
        # launch에서 주입받아 메시지에 명시한다.
        self.gps_east_offset = float(p("~gps_east_offset", 0.0))
        self.gps_north_offset = float(p("~gps_north_offset", 0.0))
        self.publish_raw = bool(p("~publish_raw_image", False))

        # MORAI의 단일 GPS/IMU UDP 스트림을 ROS 변환과 standalone planner가
        # 동시에 사용할 수 있도록 원본 datagram을 planner 전용 포트로 복제한다.
        self.planner_forward_ip = str(p("~planner_forward_ip", "127.0.0.1"))
        self.planner_forward_ports = {
            "gps": int(p("~planner_gps_port", 0)),
            "imu": int(p("~planner_imu_port", 0)),
        }
        self.planner_forward_sockets = {}
        for name, port in self.planner_forward_ports.items():
            if port > 0:
                self.planner_forward_sockets[name] = socket.socket(
                    socket.AF_INET, socket.SOCK_DGRAM)
                rospy.loginfo("forward %-5s raw UDP -> %s:%d",
                              name, self.planner_forward_ip, port)

        self.stats = {}
        self.errors = {}
        self.timestamp_fallbacks = {}
        self.timestamp_regressions = {}
        self.processing_ms = {}
        self.assembly_ms = {}
        self.last_received_monotonic = {}
        # v2 does not transmit these GT-only streams. Their idle sockets are
        # expected in the union-port bridge; verify-gt promotes them to
        # required inputs when a GT run is requested.
        self.optional_streams = set()
        self._last_source_ns = {}
        self._started_monotonic = time.monotonic()
        self._last_report_monotonic = self._started_monotonic
        self._last_report_stats = {}
        self.pub_diagnostics = rospy.Publisher(
            "/morai_sensor_bridge/diagnostics", DiagnosticArray, queue_size=1
        )
        self._start_all(p)
        rospy.Timer(rospy.Duration(5.0), self._report)

    def _start_all(self, p):
        if p("~enable_camera", True):
            cams = list(p("~cameras", DEFAULT_CAMERAS) or [])
            if not cams:
                # 단일 포트 호환
                cams = [{"name": "cam1",
                         "port": int(p("~camera_port", 9291)),
                         "topic": p("~camera_topic", "/image_jpeg/compressed"),
                         "frame": p("~camera_frame", "camera_front")}]
            if p("~enable_semantic_camera", False):
                sem = p("~semantic_cameras", DEFAULT_SEMANTIC_CAMERAS)
                if sem:
                    self.optional_streams.update(
                        str(camera.get("name", "semantic")) for camera in sem
                    )
                    cams.extend(sem)
            for cam in cams:
                self._start_camera(cam)

        if p("~enable_lidar", True):
            model = self._norm_model(p("~lidar_model", "VLP-16"))
            if model is None:
                rospy.logerr("알 수 없는 lidar_model. 가능: %s",
                             list(VERTICAL_ANGLES))
            else:
                n = int(p("~lidar_packets_per_frame", 0))
                if n <= 0:
                    rate = float(p("~lidar_rotation_rate", 10.0))
                    channel = len(VERTICAL_ANGLES[model])
                    per_pkt = 24 if channel == 16 else 12
                    n = max(1, int(round((36000.0 / (rate * 2.0)) / per_pkt)))
                self._start_lidar({
                    "name": "lidar",
                    "port": int(p("~lidar_port", 9299)),
                    "topic": str(p("~lidar_topic", "/velodyne_points")),
                    "model": model,
                    "packets_per_frame": n,
                    "value_field": "intensity",
                })

                # Legacy host-port mirror support. It remains an independent
                # stream so it cannot corrupt the primary frame buffer.
                host_p = int(p("~lidar_host_port", 0))
                if host_p > 0 and host_p != int(p("~lidar_port", 9299)):
                    self._start_lidar({
                        "name": "lidar_h",
                        "port": host_p,
                        "topic": str(p("~lidar_host_topic", "/velodyne_points_host")),
                        "model": model,
                        "packets_per_frame": n,
                        "value_field": "intensity",
                    })

                if p("~enable_instance_lidar", False):
                    self.optional_streams.add("lidar_instance")
                    self._start_lidar({
                        "name": "lidar_instance",
                        "port": int(p("~instance_lidar_port", 9311)),
                        "topic": str(
                            p(
                                "~instance_lidar_topic",
                                "/velodyne_points_instance",
                            )
                        ),
                        "model": model,
                        "packets_per_frame": n,
                        "value_field": "instance_id",
                    })


        if p("~enable_gps", True):
            self.pub_gps = rospy.Publisher("/gps", GPSMessage, queue_size=1)
            self.pub_fix = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
            self._spawn("gps", int(p("~gps_port", 9301)), self._gps)

        if p("~enable_imu", True):
            self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=1)
            self._spawn("imu", int(p("~imu_port", 9303)), self._imu)

    def _start_camera(self, cam):
        name = str(cam.get("name", "cam"))
        port = int(cam["port"])
        topic = str(cam.get("topic", "/image_jpeg/compressed"))
        frame = str(cam.get("frame", "camera"))
        pub_c = rospy.Publisher(topic, CompressedImage, queue_size=1)
        pub_r = None
        if self.publish_raw:
            raw_topic = topic.replace("/compressed", "") \
                if topic.endswith("/compressed") else topic + "_raw"
            if raw_topic == topic:
                raw_topic = topic + "/raw"
            pub_r = rospy.Publisher(raw_topic, Image, queue_size=1)
        rospy.loginfo("camera %s udp/%d -> %s", name, port, topic)

        def loop(sock, stat_name):
            self._camera_loop(sock, stat_name, pub_c, pub_r, frame)

        self._spawn(name, port, loop)

    def _start_lidar(self, config):
        """Start one fully independent Velodyne stream."""
        model = config["model"]
        vertical = np.array([VERTICAL_ANGLES[model]], dtype=np.float32)
        channel = vertical.shape[1]
        publisher = rospy.Publisher(config["topic"], PointCloud2, queue_size=1)
        rospy.loginfo(
            "%s model=%s ch=%d packets_per_frame=%d udp/%d -> %s field=%s",
            config["name"],
            model,
            channel,
            config["packets_per_frame"],
            config["port"],
            config["topic"],
            config["value_field"],
        )

        def loop(sock, stat_name):
            self._lidar_loop(
                sock,
                stat_name,
                publisher,
                vertical,
                int(config["packets_per_frame"]),
                config["value_field"],
            )

        self._spawn(config["name"], config["port"], loop)

    @staticmethod
    def _norm_model(name):
        key = str(name).upper().replace("_", "").replace("-", "").replace(" ", "")
        table = {"VLP16": "VLP-16", "VLP32C": "VLP-32C",
                 "HDL32E": "HDL-32E", "HDL32": "HDL-32E"}
        return table.get(key)

    def _spawn(self, name, port, fn):
        self.stats[name] = 0
        self.errors[name] = 0
        self.timestamp_fallbacks[name] = 0
        self.timestamp_regressions[name] = 0
        self.processing_ms[name] = deque(maxlen=4096)
        self.assembly_ms[name] = deque(maxlen=4096)
        self.last_received_monotonic[name] = None
        self._last_report_stats[name] = 0
        t = threading.Thread(target=self._guard, args=(name, port, fn))
        t.daemon = True
        t.start()

    def _guard(self, name, port, fn):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        try:
            sock.bind(("0.0.0.0", port))
        except Exception as e:
            rospy.logerr("bind %d (%s) failed: %s", port, name, e)
            return
        sock.settimeout(1.0)
        rospy.loginfo("listening %-8s on udp/%d", name, port)
        try:
            fn(sock, name)
        except Exception as e:
            rospy.logerr("%s thread died: %s", name, e)
        finally:
            sock.close()

    def _camera_loop(self, sock, name, pub_c, pub_r, frame_id):
        buf = bytearray()
        active_stamp = None
        expected_index = 0
        assembly_started_ns = None
        while not rospy.is_shutdown():
            try:
                blk, _ = sock.recvfrom(65000)
            except socket.timeout:
                continue
            try:
                sec, nsec, index, payload, is_final = parse_camera_packet(blk)
            except (struct.error, ValueError):
                self.errors[name] += 1
                continue

            stamp_key = (sec, nsec)
            if index == 0:
                buf = bytearray()
                active_stamp = stamp_key
                expected_index = 0
                assembly_started_ns = time.perf_counter_ns()
            if active_stamp != stamp_key or index != expected_index:
                self.errors[name] += 1
                buf = bytearray()
                active_stamp = None
                expected_index = 0
                assembly_started_ns = None
                continue
            buf.extend(payload)
            expected_index += 1
            if not is_final:
                continue

            processing_started_ns = time.perf_counter_ns()
            stamp = self._stamp_from_sec_nsec(sec, nsec, name)
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.format = "jpeg"
            m.data = bytes(buf)
            pub_c.publish(m)

            if pub_r is not None:
                try:
                    import cv2
                    img = cv2.imdecode(np.frombuffer(bytes(buf), np.uint8),
                                       cv2.IMREAD_COLOR)
                    if img is not None:
                        r = Image()
                        r.header.stamp = stamp
                        r.header.frame_id = frame_id
                        r.height, r.width = img.shape[:2]
                        r.encoding = "bgr8"
                        r.is_bigendian = 0
                        r.step = r.width * 3
                        r.data = img.tobytes()
                        pub_r.publish(r)
                except ImportError:
                    rospy.logwarn_once("cv2 없음 -> publish_raw_image 무시")

            buf = bytearray()
            active_stamp = None
            expected_index = 0
            completed_ns = time.perf_counter_ns()
            self._record_processing(name, processing_started_ns, completed_ns)
            if assembly_started_ns is not None:
                self.assembly_ms[name].append(
                    (completed_ns - assembly_started_ns) / 1000000.0
                )
            assembly_started_ns = None
            self._mark_message(name)

    def _lidar_loop(self, sock, name, publisher, vertical,
                    packets_per_frame, value_field):
        """Velodyne UDP. 패킷을 모으되 타임아웃 시에도 부분 프레임을 살린다."""
        ch = vertical.shape[1]
        cos_v = np.cos(np.deg2rad(vertical))
        sin_v = np.sin(np.deg2rad(vertical))
        need = packets_per_frame
        min_pkts = max(12, need // 4)  # 최소 이만큼이면 한 프레임으로 발행
        chunks = []
        device_times = []
        assembly_started_ns = None
        raw_seen = 0

        while not rospy.is_shutdown():
            try:
                pkt, _ = sock.recvfrom(2048)
            except socket.timeout:
                if len(chunks) >= min_pkts:
                    self._lidar_publish(
                        chunks, device_times, name, publisher,
                        ch, cos_v, sin_v, value_field, assembly_started_ns
                    )
                    chunks = []
                    device_times = []
                    assembly_started_ns = None
                continue

            raw_seen += 1
            if raw_seen == 1:
                preview = "".join(chr(b) if 32 <= b < 127 else "."
                                  for b in pkt[:min(40, len(pkt))])
                rospy.loginfo("%s first UDP %dB (expect ~1206 Velodyne) preview=%r",
                              name, len(pkt), preview)
                if pkt.startswith(b"$GP") or pkt.startswith(b"$GN"):
                    rospy.logerr(
                        "%s port에 GPS NMEA가 들어옴 — LiDAR Dest가 이 포트가 아님. "
                        "SIM에서 LiDAR Dest=%d, GPS는 다음 포트로 옮기세요.",
                        name, sock.getsockname()[1])
                elif len(pkt) < 100:
                    rospy.logwarn(
                        "%s 패킷이 너무 짧음 (%dB). Velodyne raw(~1206B)가 아님.",
                        name, len(pkt))
            if len(pkt) < 100:
                self.errors[name] += 1
                continue
            # 1200B 데이터부 (표준 1206 = 1200 + 6 timestamp)
            if not chunks:
                assembly_started_ns = time.perf_counter_ns()
            chunks.append(pkt[:1200] if len(pkt) >= 1200 else pkt.ljust(1200, b"\x00")[:1200])
            if len(pkt) >= 1204:
                device_times.append(struct.unpack("<I", pkt[1200:1204])[0])
            else:
                device_times.append(None)

            if len(chunks) >= need:
                self._lidar_publish(
                    chunks, device_times, name, publisher,
                    ch, cos_v, sin_v, value_field, assembly_started_ns
                )
                chunks = []
                device_times = []
                assembly_started_ns = None

    def _lidar_publish(self, chunks, device_times, name, publisher,
                       ch, cos_v, sin_v, value_field,
                       assembly_started_ns=None):
        processing_started_ns = time.perf_counter_ns()
        try:
            b = np.frombuffer(b"".join(chunks), dtype=np.uint8).reshape([-1, 100])
        except ValueError as e:
            rospy.logwarn_throttle(5.0, "%s reshape fail (%d pkts): %s",
                                   name, len(chunks), e)
            return False

        if ch == 16:
            az = np.zeros((b.shape[0] * 2,), dtype=np.float32)
            base = b[:, 2].astype(np.float32) + 256.0 * b[:, 3]
            az[0::2] = base
            az[1::2] = base + 20.0
        else:
            az = (b[:, 2].astype(np.float32) + 256.0 * b[:, 3])

        dist = (b[:, 4::3].astype(np.float32)
                + 256.0 * b[:, 5::3].astype(np.float32)) * 2.0
        raw_value = b[:, 6::3].reshape(-1)

        az = az.reshape([-1, 1]) / 100.0
        dist = dist.reshape([-1, ch]) / 1000.0

        sin_a = np.sin(np.deg2rad(az))
        cos_a = np.cos(np.deg2rad(az))
        # Velodyne azimuth is clockwise from the sensor's forward direction.
        # Publish in the ROS vehicle convention: +X forward and +Y left.
        x = (dist * cos_v * cos_a).reshape(-1)
        y = -(dist * cos_v * sin_a).reshape(-1)
        z = (dist * sin_v).reshape(-1)

        valid = dist.reshape(-1) > 0.01
        device_us = next((value for value in reversed(device_times)
                          if value is not None), None)
        stamp = self._stamp_from_velodyne(device_us, name)
        publisher.publish(
            self._make_cloud(
                x[valid], y[valid], z[valid], raw_value[valid],
                stamp, value_field
            )
        )
        completed_ns = time.perf_counter_ns()
        self._record_processing(name, processing_started_ns, completed_ns)
        if assembly_started_ns is not None:
            self.assembly_ms[name].append(
                (completed_ns - assembly_started_ns) / 1000000.0
            )
        self._mark_message(name)
        return True

    def _make_cloud(self, x, y, z, value, stamp, value_field):
        n = int(x.shape[0])
        value_dtype = np.uint32 if value_field == "instance_id" else np.float32
        arr = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32),
                                 ("z", np.float32),
                                 (value_field, value_dtype)])
        arr["x"], arr["y"], arr["z"] = x, y, z
        arr[value_field] = value.astype(value_dtype, copy=False)

        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_lidar
        m.height = 1
        m.width = n
        m.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField(
                value_field,
                12,
                PointField.UINT32 if value_field == "instance_id"
                else PointField.FLOAT32,
                1,
            ),
        ]
        m.is_bigendian = False
        m.point_step = 16
        m.row_step = 16 * n
        m.is_dense = True
        m.data = arr.tobytes()
        return m

    def _fallback_stamp(self, name, regression=False):
        self.timestamp_fallbacks[name] = self.timestamp_fallbacks.get(name, 0) + 1
        if regression:
            self.timestamp_regressions[name] = \
                self.timestamp_regressions.get(name, 0) + 1
        stamp = rospy.Time.now()
        value_ns = int(stamp.to_nsec())
        previous = self._last_source_ns.get(name)
        if previous is not None and value_ns <= previous:
            value_ns = previous + 1
            stamp = rospy.Time(value_ns // 1000000000, value_ns % 1000000000)
        self._last_source_ns[name] = value_ns
        return stamp

    def _accept_source_stamp(self, stamp, name):
        value_ns = int(stamp.to_nsec())
        previous = self._last_source_ns.get(name)
        if previous is not None and value_ns < previous:
            return self._fallback_stamp(name, regression=True)
        self._last_source_ns[name] = value_ns
        return stamp

    def _stamp_from_sec_nsec(self, sec, nsec, name):
        if self.timestamp_mode == "receive":
            return rospy.Time.now()
        if int(sec) <= 0 or not 0 <= int(nsec) < 1000000000:
            return self._fallback_stamp(name)
        try:
            return self._accept_source_stamp(
                rospy.Time(int(sec), int(nsec)), name
            )
        except (TypeError, ValueError):
            return self._fallback_stamp(name)

    def _stamp_from_velodyne(self, device_us, name):
        """Resolve Velodyne microseconds-since-hour to the nearest ROS hour."""
        if self.timestamp_mode == "receive":
            return rospy.Time.now()
        if device_us is None or not 0 <= int(device_us) < 3600 * 1000000:
            return self._fallback_stamp(name)
        now = rospy.Time.now().to_sec()
        hour = int(now // 3600) * 3600
        offset = float(device_us) / 1000000.0
        candidates = (hour - 3600 + offset, hour + offset, hour + 3600 + offset)
        resolved = min(candidates, key=lambda value: abs(value - now))
        try:
            return self._accept_source_stamp(rospy.Time.from_sec(resolved), name)
        except (TypeError, ValueError):
            return self._fallback_stamp(name)

    def _gps(self, sock, name):
        while not rospy.is_shutdown():
            try:
                raw, _ = sock.recvfrom(8196)
            except socket.timeout:
                continue
            processing_started_ns = time.perf_counter_ns()
            self._forward_to_planner(name, raw)
            try:
                line = raw.decode("ascii", "ignore").split("\r\n")[0]
            except Exception:
                continue
            f = line.split(",")
            if len(f) < 10 or "GGA" not in f[0]:
                continue
            try:
                lat = self._nmea_deg(f[2], f[3])
                lon = self._nmea_deg(f[4], f[5])
                status = int(f[6]) if f[6] else 0
                alt = float(f[9]) if f[9] else 0.0
            except (ValueError, IndexError):
                continue

            stamp = rospy.Time.now()
            g = GPSMessage()
            g.header.stamp = stamp
            g.header.frame_id = self.frame_gps
            g.latitude, g.longitude, g.altitude = lat, lon, alt
            g.status = status
            g.eastOffset = self.gps_east_offset
            g.northOffset = self.gps_north_offset
            self.pub_gps.publish(g)

            fx = NavSatFix()
            fx.header = g.header
            fx.status.status = (NavSatStatus.STATUS_FIX if status > 0
                                else NavSatStatus.STATUS_NO_FIX)
            fx.status.service = NavSatStatus.SERVICE_GPS
            fx.latitude, fx.longitude, fx.altitude = lat, lon, alt
            self.pub_fix.publish(fx)
            self._record_processing(name, processing_started_ns)
            self._mark_message(name)

    @staticmethod
    def _nmea_deg(val, hemi):
        if not val:
            return 0.0
        v = float(val)
        deg = int(v / 100)
        dec = deg + (v - deg * 100) / 60.0
        return -dec if hemi in ("S", "W") else dec

    def _imu(self, sock, name):
        while not rospy.is_shutdown():
            try:
                raw, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            processing_started_ns = time.perf_counter_ns()
            self._forward_to_planner(name, raw)
            if raw[0:9] != b"#IMUData$" or len(raw) < 113:
                continue
            sec, nsec = struct.unpack("<II", raw[25:33])
            d = struct.unpack("<10d", raw[33:113])

            m = Imu()
            m.header.stamp = self._stamp_from_sec_nsec(sec, nsec, name)
            m.header.frame_id = self.frame_imu
            m.orientation.w, m.orientation.x = d[0], d[1]
            m.orientation.y, m.orientation.z = d[2], d[3]
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z \
                = d[4], d[5], d[6]
            m.linear_acceleration.x, m.linear_acceleration.y, \
                m.linear_acceleration.z = d[7], d[8], d[9]
            self.pub_imu.publish(m)
            self._record_processing(name, processing_started_ns)
            self._mark_message(name)

    def _record_processing(self, name, started_ns, completed_ns=None):
        if completed_ns is None:
            completed_ns = time.perf_counter_ns()
        self.processing_ms[name].append(
            (completed_ns - started_ns) / 1000000.0
        )

    def _mark_message(self, name):
        self.stats[name] += 1
        self.last_received_monotonic[name] = time.monotonic()

    def _forward_to_planner(self, name, raw):
        forward_sock = self.planner_forward_sockets.get(name)
        port = self.planner_forward_ports.get(name, 0)
        if forward_sock is None or port <= 0:
            return
        try:
            forward_sock.sendto(raw, (self.planner_forward_ip, port))
        except OSError as e:
            rospy.logwarn_throttle(
                5.0, "%s planner UDP forward failed: %s", name, e)

    def _report(self, _):
        now = time.monotonic()
        report_elapsed = max(1e-6, now - self._last_report_monotonic)
        interval_rates = {}
        for name, count in self.stats.items():
            previous = self._last_report_stats.get(name, 0)
            interval_rates[name] = max(0, count - previous) / report_elapsed
        rospy.loginfo(
            "rx " + "  ".join(
                "%s=%d(%.1fHz)" % (name, count, interval_rates[name])
                for name, count in sorted(self.stats.items())
            )
        )
        self._last_report_stats = dict(self.stats)
        self._last_report_monotonic = now
        for k, v in self.stats.items():
            if v == 0 and k not in self.optional_streams:
                rospy.logwarn_throttle(
                    30.0, "%s 수신 0건 - MORAI Sensor Destination Port 확인.", k)
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        elapsed = max(1e-6, time.monotonic() - self._started_monotonic)
        for name in sorted(self.stats):
            processing = latency_summary(self.processing_ms[name])
            assembly = latency_summary(self.assembly_ms[name])
            last_received = self.last_received_monotonic[name]
            last_age_sec = (
                -1.0 if last_received is None else max(0.0, now - last_received)
            )
            status = DiagnosticStatus()
            status.name = "morai_sensor_bridge/{}".format(name)
            status.hardware_id = "morai_udp"
            optional_idle = (
                self.stats[name] == 0 and name in self.optional_streams
            )
            status.level = (
                DiagnosticStatus.OK
                if self.stats[name] > 0 or optional_idle
                else DiagnosticStatus.WARN
            )
            if self.stats[name] > 0:
                status.message = "receiving"
            elif optional_idle:
                status.message = "optional GT stream idle"
            else:
                status.message = "no data"
            status.values = [
                KeyValue("simulator_profile", self.simulator_profile),
                KeyValue("timestamp_mode", self.timestamp_mode),
                KeyValue("optional_stream", str(name in self.optional_streams).lower()),
                KeyValue("messages", str(self.stats[name])),
                KeyValue("average_hz", "{:.3f}".format(self.stats[name] / elapsed)),
                KeyValue("interval_hz", "{:.3f}".format(interval_rates[name])),
                KeyValue("last_message_age_sec", "{:.6f}".format(last_age_sec)),
                KeyValue("processing_samples", str(processing["samples"])),
                KeyValue("processing_mean_ms", "{:.6f}".format(processing["mean_ms"])),
                KeyValue("processing_p50_ms", "{:.6f}".format(processing["p50_ms"])),
                KeyValue("processing_p95_ms", "{:.6f}".format(processing["p95_ms"])),
                KeyValue("processing_max_ms", "{:.6f}".format(processing["max_ms"])),
                KeyValue("assembly_samples", str(assembly["samples"])),
                KeyValue("assembly_p50_ms", "{:.6f}".format(assembly["p50_ms"])),
                KeyValue("assembly_p95_ms", "{:.6f}".format(assembly["p95_ms"])),
                KeyValue("parse_errors", str(self.errors.get(name, 0))),
                KeyValue(
                    "timestamp_fallbacks",
                    str(self.timestamp_fallbacks.get(name, 0)),
                ),
                KeyValue(
                    "timestamp_regressions",
                    str(self.timestamp_regressions.get(name, 0)),
                ),
            ]
            message.status.append(status)
        self.pub_diagnostics.publish(message)


def main():
    rospy.init_node("morai_sensor_bridge")
    SensorBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
