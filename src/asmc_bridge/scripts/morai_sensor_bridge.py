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
  GPS + IMU                 -> TF map -> base_link (~publish_map_tf)

포맷 근거: MORAI-Autonomous/MORAI-SensorExample
"""

import socket
import struct
import threading
import time
import uuid

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import (CompressedImage, Image, Imu, NavSatFix,
                             NavSatStatus, PointCloud2, PointField)
from morai_msgs.msg import GPSMessage
from asmc_msgs.msg import SensorFrameTiming
from asmc_bridge.frame_timing import SourceClockTracker, publish_sensor, receipt_fields
from asmc_bridge.telemetry import BridgeTelemetry, publish_report, start_reporting

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


def velodyne_packet_azimuth(packet):
    """Return the first firing-block azimuth in hundredths of a degree."""
    if len(packet) < 4:
        raise ValueError("Velodyne packet is too short for an azimuth")
    return struct.unpack_from("<H", packet, 2)[0]


class VelodyneFrameAssembler(object):
    """Group raw packets by physical revolution or a legacy fixed count."""

    def __init__(self, mode, packets_per_frame):
        if mode not in ("azimuth", "packet_count"):
            raise ValueError("lidar_frame_mode must be azimuth or packet_count")
        self.mode = mode
        self.packets_per_frame = max(1, int(packets_per_frame))
        self.minimum_packets = max(12, self.packets_per_frame // 4)
        self.chunks = []
        self.device_times = []
        self.receipts = []
        self.completed_receipts = ()
        self.started_ns = None
        self.previous_azimuth = None
        self.synchronized = mode == "packet_count"
        self.boundaries = 0
        self.discarded_revolutions = 0

    def _take(self):
        completed = (self.chunks, self.device_times, self.started_ns)
        self.completed_receipts = tuple(self.receipts)
        self.chunks = []
        self.device_times = []
        self.receipts = []
        self.started_ns = None
        return completed

    def push(self, packet, device_time, arrival_ns, receipt=None):
        azimuth = velodyne_packet_azimuth(packet)
        completed = None
        wrapped = (
            self.previous_azimuth is not None
            and self.previous_azimuth - azimuth > 18000
        )
        if self.mode == "azimuth" and wrapped:
            self.boundaries += 1
            if self.synchronized and len(self.chunks) >= self.minimum_packets:
                completed = self._take()
            else:
                self.discarded_revolutions += 1
                self.chunks = []
                self.device_times = []
                self.receipts = []
                self.started_ns = None
            # The packet after this boundary starts a complete revolution.
            self.synchronized = True

        if not self.chunks:
            self.started_ns = arrival_ns
        self.chunks.append(packet)
        self.device_times.append(device_time)
        self.receipts.append(receipt)
        self.previous_azimuth = azimuth
        if len(self.chunks) > max(4096, self.packets_per_frame * 4):
            # No-wrap/malformed streams must not grow payload or timing memory.
            self._take()
            self.completed_receipts = ()
            self.synchronized = self.mode == 'packet_count'
            self.discarded_revolutions += 1
            return None

        if (self.mode == "packet_count"
                and len(self.chunks) >= self.packets_per_frame):
            completed = self._take()
        return completed

    def flush(self):
        completed = None
        if self.synchronized and len(self.chunks) >= self.minimum_packets:
            completed = self._take()
        else:
            if self.chunks:
                self.discarded_revolutions += 1
            self.chunks = []
            self.device_times = []
            self.receipts = []
            self.started_ns = None
        if self.mode == "azimuth":
            # A timeout loses the physical revolution boundary. Resynchronize
            # at the next wrap instead of joining two partial revolutions.
            self.synchronized = False
        self.previous_azimuth = None
        return completed


class SensorBridge(object):

    def __init__(self):
        p = rospy.get_param
        self.frame_lidar = p("~lidar_frame", "velodyne")
        self.frame_imu = p("~imu_frame", "imu")
        self.frame_gps = p("~gps_frame", "gps")
        self.runtime_profile = str(p("~runtime_profile", "morai_udp"))
        self.transport_profile = str(p("~transport_profile", "local"))
        self.stream_profile = str(p("~stream_profile", "competition"))
        if self.stream_profile not in ("competition", "development"):
            raise ValueError("stream_profile must be competition or development")
        if self.transport_profile == "lan2lan" and self.stream_profile != "competition":
            raise ValueError("lan2lan transport only permits the competition stream profile")
        if self.stream_profile == "competition" and (
                bool(p("~enable_semantic_camera", False))
                or bool(p("~enable_instance_lidar", False))):
            raise ValueError(
                "competition stream profile forbids Semantic and Instance sensors"
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

        # map -> base_link 동적 TF. 이걸 발행하지 않으면 map 프레임이 TF 트리에
        # 없어서, 메시지에 frame_id="map" 이 붙어 있어도 RViz 에서 velodyne
        # 프레임 데이터와 겹쳐 볼 수 없다 (Fixed Frame=map 이 실패한다).
        # 위치는 GPS, 방위는 IMU orientation 을 쓴다 — 둘 다 허용 센서다.
        # 기본은 꺼 둔다. 측위 노드가 발행하는 authoritative map -> base_link와
        # 동시에 켜면 TF authority가 충돌하므로 디버그/융합 실행에서만 opt-in한다.
        self.publish_map_tf = bool(p("~publish_map_tf", False))
        self.frame_map = str(p("~map_frame", "map"))
        self.frame_base = str(p("~base_frame", "base_link"))
        self.utm_epsg = int(p("~utm_epsg", 32652))
        self._tf_bcast = None
        self._utm = None
        self._last_imu_q = None

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

        self.telemetry = BridgeTelemetry()
        self.stale_after = dict(p("~stale_after", {}) or {})
        # Only the explicit local development profile starts GT-only streams;
        # verify-gt then promotes them to required inputs for that run.
        self.optional_streams = set()
        self._last_source_ns = {}
        self._header_fallback_reason = {}
        self._timing_trackers = {}
        self._timing_sequences = {}
        self.bridge_session_id = str(uuid.uuid4())
        self.use_sim_time = bool(p('/use_sim_time', False))
        self.pub_timing = rospy.Publisher(
            '/morai_sensor_bridge/frame_timing', SensorFrameTiming, queue_size=128)
        self.pub_diagnostics = rospy.Publisher(
            "/morai_sensor_bridge/diagnostics", DiagnosticArray, queue_size=1
        )
        self._start_all(p)
        start_reporting(self, 'morai_sensor_bridge')

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
                    rate = float(p("~lidar_rotation_rate", 15.0))
                    channel = len(VERTICAL_ANGLES[model])
                    per_pkt = 24 if channel == 16 else 12
                    n = max(1, int(round((36000.0 / (rate * 2.0)) / per_pkt)))
                frame_mode = str(
                    p("~lidar_frame_mode", "azimuth")
                ).strip().lower()
                if frame_mode not in ("azimuth", "packet_count"):
                    raise ValueError(
                        "lidar_frame_mode must be azimuth or packet_count"
                    )
                self._start_lidar({
                    "name": "lidar",
                    "port": int(p("~lidar_port", 9299)),
                    "topic": str(p("~lidar_topic", "/velodyne_points")),
                    "model": model,
                    "packets_per_frame": n,
                    "frame_mode": frame_mode,
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
                        "frame_mode": frame_mode,
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
                        "frame_mode": frame_mode,
                        "value_field": "instance_id",
                    })


        if p("~enable_gps", True):
            self.pub_gps = rospy.Publisher("/gps", GPSMessage, queue_size=1)
            self.pub_fix = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
            self._spawn("gps", int(p("~gps_port", 9301)), self._gps)

        if p("~enable_imu", True):
            # Pose samples arrive in short bursts, including repeated raw stamps.
            # A video-style queue of one dropped distinct source poses in live
            # timing joins. Keep a small bounded burst buffer; consumers still
            # choose their own latest/bracketed policy. No header-clock change.
            self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=16)
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
            "%s model=%s ch=%d frame_mode=%s fallback_packets=%d udp/%d -> %s field=%s",
            config["name"],
            model,
            channel,
            config["frame_mode"],
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
                config["frame_mode"],
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
        self.telemetry.register(name, optional=name in self.optional_streams,
            stale_after=float(self.stale_after.get(name, self.stale_after.get('default', .5))))
        t = threading.Thread(target=self._guard, args=(name, port, fn))
        t.daemon = True
        t.start()

    def _guard(self, name, port, fn):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.telemetry.configure_socket(name, sock)
        try:
            sock.bind(("0.0.0.0", port))
        except Exception as e:
            self.telemetry.receiver(name, 'failed', str(e))
            rospy.logerr("bind %d (%s) failed: %s", port, name, e)
            sock.close()
            return
        sock.settimeout(1.0)
        rospy.loginfo("listening %-8s on udp/%d", name, port)
        self.telemetry.receiver(name, 'running')
        try:
            fn(sock, name)
        except Exception as e:
            self.telemetry.receiver(name, 'failed', str(e))
            rospy.logerr("%s thread died: %s", name, e)
        finally:
            self.telemetry.receiver_exited(name, rospy.is_shutdown())
            sock.close()

    def _camera_loop(self, sock, name, pub_c, pub_r, frame_id):
        buf = bytearray()
        active_stamp = None
        expected_index = 0
        assembly_started_ns = None
        receipts = []
        while not rospy.is_shutdown():
            try:
                blk, _, receipt = self.telemetry.receive_timed(name, sock, 65000)
            except socket.timeout:
                if active_stamp is not None:
                    self.telemetry.increment(name, 'assembly_timeouts')
                    self.telemetry.increment(name, 'incomplete_frames')
                    buf, receipts, active_stamp = bytearray(), [], None
                    expected_index, assembly_started_ns = 0, None
                continue
            if blk is None:
                continue
            try:
                sec, nsec, index, payload, is_final = parse_camera_packet(blk)
            except (struct.error, ValueError):
                self.telemetry.increment(name, 'parse_errors')
                continue

            stamp_key = (sec, nsec)
            if index == 0:
                if active_stamp is not None:
                    self.telemetry.increment(name, 'incomplete_frames')
                buf = bytearray()
                active_stamp = stamp_key
                expected_index = 0
                assembly_started_ns = time.perf_counter_ns()
                receipts = []
            if active_stamp != stamp_key or index != expected_index:
                self.telemetry.increment(name, 'parse_errors')
                if active_stamp is not None:
                    self.telemetry.increment(name, 'incomplete_frames')
                buf = bytearray()
                active_stamp = None
                expected_index = 0
                assembly_started_ns = None
                receipts = []
                continue
            buf.extend(payload)
            receipts.append(receipt)
            expected_index += 1
            if len(buf) > 16 * 1024 * 1024 or expected_index > 4096:
                self.telemetry.increment(name, 'parse_errors')
                buf, receipts, active_stamp = bytearray(), [], None
                expected_index, assembly_started_ns = 0, None
                continue
            if not is_final:
                continue

            processing_started_ns = time.perf_counter_ns()
            stamp = self._stamp_from_sec_nsec(sec, nsec, name)
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.format = "jpeg"
            m.data = bytes(buf)
            wire_sequence = publish_sensor(pub_c, m)
            self._publish_timing(name, pub_c, m, receipts, wire_sequence,
                                 sec=sec, nsec=nsec, reference='camera_frame')

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
            receipts = []
            completed_ns = time.perf_counter_ns()
            self._record_processing(name, processing_started_ns, completed_ns)
            if assembly_started_ns is not None:
                self.telemetry.sample(name, 'assembly',
                    (completed_ns - assembly_started_ns) / 1000000.0
                )
            assembly_started_ns = None
            self._mark_message(name)

    def _lidar_loop(self, sock, name, publisher, vertical,
                    packets_per_frame, frame_mode, value_field):
        """Velodyne UDP. 패킷을 모으되 타임아웃 시에도 부분 프레임을 살린다."""
        ch = vertical.shape[1]
        cos_v = np.cos(np.deg2rad(vertical))
        sin_v = np.sin(np.deg2rad(vertical))
        assembler = VelodyneFrameAssembler(frame_mode, packets_per_frame)
        raw_seen = 0

        while not rospy.is_shutdown():
            try:
                pkt, _, receipt = self.telemetry.receive_timed(name, sock, 2048)
            except socket.timeout:
                self.telemetry.increment(name, 'assembly_timeouts')
                previous_discarded = assembler.discarded_revolutions
                completed = assembler.flush()
                self.telemetry.increment(name, 'discarded_revolutions', assembler.discarded_revolutions - previous_discarded)
                if completed is not None:
                    chunks, device_times, assembly_started_ns = completed
                    self._lidar_publish(
                        chunks, device_times, name, publisher,
                        ch, cos_v, sin_v, value_field, assembly_started_ns,
                        assembler.completed_receipts, assembly_timeout=True
                    )
                continue

            raw_seen += 1
            if pkt is None:
                continue
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
                self.telemetry.increment(name, 'parse_errors')
                continue
            # 1200B 데이터부 (표준 1206 = 1200 + 6 timestamp)
            chunk = pkt[:1200] if len(pkt) >= 1200 else pkt.ljust(1200, b"\x00")[:1200]
            if len(pkt) >= 1204:
                device_time = struct.unpack("<I", pkt[1200:1204])[0]
            else:
                device_time = None
            previous_boundaries = assembler.boundaries
            previous_discarded = assembler.discarded_revolutions
            completed = assembler.push(
                chunk, device_time, time.perf_counter_ns(), receipt
            )
            if assembler.boundaries != previous_boundaries:
                self.telemetry.rotation(name, device_time)
            self.telemetry.increment(name, 'discarded_revolutions', assembler.discarded_revolutions - previous_discarded)
            if completed is not None:
                chunks, device_times, assembly_started_ns = completed
                self._lidar_publish(
                    chunks, device_times, name, publisher,
                    ch, cos_v, sin_v, value_field, assembly_started_ns,
                    assembler.completed_receipts
                )

    def _lidar_publish(self, chunks, device_times, name, publisher,
                       ch, cos_v, sin_v, value_field,
                       assembly_started_ns=None, receipts=(), assembly_timeout=False):
        processing_started_ns = time.perf_counter_ns()
        try:
            b = np.frombuffer(b"".join(chunks), dtype=np.uint8).reshape([-1, 100])
        except ValueError as e:
            self.telemetry.increment(name, 'parse_errors')
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
        cloud = self._make_cloud(
                x[valid], y[valid], z[valid], raw_value[valid],
                stamp, value_field
            )
        wire_sequence = publish_sensor(publisher, cloud)
        self._publish_timing(name, publisher, cloud, receipts, wire_sequence,
                             device_times=device_times, reference='lidar_sweep',
                             assembly_timeout=assembly_timeout)
        completed_ns = time.perf_counter_ns()
        self._record_processing(name, processing_started_ns, completed_ns)
        self.telemetry.sample(name, 'packets_per_frame', len(chunks))
        self.telemetry.sample(name, 'points_per_frame', int(np.count_nonzero(valid)))
        if assembly_started_ns is not None:
            self.telemetry.sample(name, 'assembly',
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
        if hasattr(self, '_header_fallback_reason'):
            self._header_fallback_reason[name] = 'source_regression' if regression else 'invalid_source'
        self.telemetry.increment(name, 'timestamp_fallbacks')
        if regression:
            self.telemetry.increment(name, 'timestamp_regressions')
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
        if hasattr(self, '_header_fallback_reason'):
            self._header_fallback_reason[name] = ''
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
        if hasattr(self, '_header_fallback_reason'):
            self._header_fallback_reason[name] = ''
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
                raw, _ = self.telemetry.receive(name, sock, 8196)
            except socket.timeout:
                continue
            if raw is None:
                continue
            processing_started_ns = time.perf_counter_ns()
            self._forward_to_planner(name, raw)
            try:
                line = raw.decode("ascii", "ignore").split("\r\n")[0]
            except Exception:
                continue
            f = line.split(",")
            if "GGA" not in f[0]:
                self.telemetry.increment(name, 'ignored_packets')
                continue
            if len(f) < 10:
                self.telemetry.increment(name, 'parse_errors')
                continue
            try:
                lat = self._nmea_deg(f[2], f[3])
                lon = self._nmea_deg(f[4], f[5])
                status = int(f[6]) if f[6] else 0
                alt = float(f[9]) if f[9] else 0.0
            except (ValueError, IndexError):
                self.telemetry.increment(name, 'parse_errors')
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
            self._broadcast_map_tf(stamp, lat, lon)
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
                raw, _, receipt = self.telemetry.receive_timed(name, sock, 65535)
            except socket.timeout:
                continue
            if raw is None:
                continue
            processing_started_ns = time.perf_counter_ns()
            self._forward_to_planner(name, raw)
            if raw[0:9] != b"#IMUData$" or len(raw) < 113:
                self.telemetry.increment(name, 'parse_errors')
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
            wire_sequence = publish_sensor(self.pub_imu, m)
            self._publish_timing(name, self.pub_imu, m, [receipt], wire_sequence,
                                 sec=sec, nsec=nsec, signature=d, reference='imu_sample')
            # map TF uses the latest IMU orientation. GPS and IMU arrive on
            # independent UDP streams, so no timestamp interpolation is
            # implied here.
            self._last_imu_q = (m.orientation.x, m.orientation.y,
                                m.orientation.z, m.orientation.w)
            self._record_processing(name, processing_started_ns)
            self._mark_message(name)

    def _publish_timing(self, name, publisher, sensor, receipts, wire_sequence,
                        sec=0, nsec=0, signature=None, device_times=None,
                        reference='', assembly_timeout=False):
        """One bounded sidecar per completed sensor frame; no payload copying.

        Metadata failures cannot kill a receiver. Missing sidecars leave sequence
        gaps, while diagnostic timing_errors distinguishes local failure.
        """
        self._timing_sequences[name] = self._timing_sequences.get(name, 0) + 1
        try:
            tracker = self._timing_trackers.get(name)
            if tracker is None:
                tracker = self._timing_trackers[name] = SourceClockTracker()
            flags = (tracker.device(device_times) if device_times is not None
                     else tracker.sec_nsec(sec, nsec, signature))
            for flag, counter in (('source_duplicate', 'source_duplicates'),
                                  ('source_conflict', 'source_conflicts'),
                                  ('source_regression', 'source_regressions'),
                                  ('device_hour_rollover', 'device_hour_rollovers')):
                if flags[flag]:
                    self.telemetry.increment(name, counter)
            if not flags['source_valid']:
                self.telemetry.increment(name, 'source_invalid')
            msg = SensorFrameTiming(**receipt_fields(receipts), **flags)
            msg.bridge_session_id = self.bridge_session_id
            msg.stream_id = name
            msg.frame_sequence = self._timing_sequences[name]
            msg.sensor_topic = publisher.resolved_name
            msg.sensor_frame_id = sensor.header.frame_id
            msg.published_stamp = sensor.header.stamp
            msg.sensor_wire_sequence, msg.sensor_wire_sequence_valid = wire_sequence
            msg.source_clock_domain = 'velodyne_device_hour' if device_times is not None else 'morai_sensor'
            msg.source_reference = reference
            msg.source_sec, msg.source_nsec = sec, nsec
            if device_times:
                msg.source_device_first_us = device_times[0] if device_times[0] is not None else 0
                msg.source_device_last_us = device_times[-1] if device_times[-1] is not None else 0
            msg.assembly_timeout = assembly_timeout
            msg.publish_ros_time = rospy.Time.now()
            msg.publish_monotonic_ns = time.monotonic_ns()
            msg.use_sim_time = self.use_sim_time
            msg.header_timestamp_policy = self.timestamp_mode
            msg.header_fallback_reason = self._header_fallback_reason.get(name, '')
            self.pub_timing.publish(msg)
            self.telemetry.increment(name, 'timing_messages')
        except Exception as error:
            self.telemetry.increment(name, 'timing_errors')
            rospy.logwarn_throttle(5., '%s timing metadata failed: %s', name, error)

    def _record_processing(self, name, started_ns, completed_ns=None):
        if completed_ns is None:
            completed_ns = time.perf_counter_ns()
        self.telemetry.sample(name, 'processing',
            (completed_ns - started_ns) / 1000000.0
        )

    def _mark_message(self, name):
        self.telemetry.message(name)

    def _broadcast_map_tf(self, stamp, lat, lon):
        """GPS 위치 + IMU 방위로 map -> base_link 를 발행한다.

        위경도를 UTM 으로 옮긴 뒤 gps_east/north_offset 을 뺀다 — GPSMessage 에
        싣는 offset 과 같은 값이라 TF 와 메시지의 map 좌표가 일치한다.

        한계: 이건 GPS 원시 위치를 그대로 쓰는 것이지 필터링된 측위가 아니다.
        GPS 가 튀면 TF 도 같이 튄다. IMU 와 시각 보간도 하지 않는다.
        """
        if not self.publish_map_tf:
            return
        if self._tf_bcast is None:
            self._tf_bcast = tf2_ros.TransformBroadcaster()
            if self.gps_east_offset == 0.0 and self.gps_north_offset == 0.0:
                rospy.logwarn(
                    "publish_map_tf 가 켜져 있는데 gps_east/north_offset 이 0 이다. "
                    "map 좌표가 UTM 절대값이 되어 RViz 원점에서 수백 km 떨어진다.")
        if self._utm is None:
            try:
                from pyproj import Transformer
                self._utm = Transformer.from_crs(
                    "EPSG:4326", "EPSG:%d" % self.utm_epsg, always_xy=True)
            except Exception as e:
                rospy.logerr_throttle(
                    30.0, "pyproj 없음 - map TF 를 발행할 수 없다: %s", e)
                self.publish_map_tf = False
                return
        if self._last_imu_q is None:
            # 방위 없이 위치만 내보내면 라이다 점군이 엉뚱한 방향으로 회전한다.
            rospy.logwarn_throttle(10.0, "IMU orientation 대기 중 - map TF 보류")
            return
        east, north = self._utm.transform(lon, lat)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_map
        t.child_frame_id = self.frame_base
        t.transform.translation.x = east - self.gps_east_offset
        t.transform.translation.y = north - self.gps_north_offset
        t.transform.translation.z = 0.0
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = self._last_imu_q
        self._tf_bcast.sendTransform(t)

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
        publish_report(self, 'morai_sensor_bridge')


def main():
    rospy.init_node("morai_sensor_bridge")
    SensorBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
