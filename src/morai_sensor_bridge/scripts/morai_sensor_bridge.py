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

import numpy as np
import rospy
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


class SensorBridge(object):

    def __init__(self):
        p = rospy.get_param
        self.frame_lidar = p("~lidar_frame", "velodyne")
        self.frame_imu = p("~imu_frame", "imu")
        self.frame_gps = p("~gps_frame", "gps")
        self.publish_raw = bool(p("~publish_raw_image", False))

        self.stats = {}
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
            if p("~enable_semantic_camera", True):
                sem = p("~semantic_cameras", DEFAULT_SEMANTIC_CAMERAS)
                if sem:
                    cams.extend(sem)
            for cam in cams:
                self._start_camera(cam)

        if p("~enable_lidar", True):
            model = self._norm_model(p("~lidar_model", "VLP-16"))
            if model is None:
                rospy.logerr("알 수 없는 lidar_model. 가능: %s",
                             list(VERTICAL_ANGLES))
            else:
                self.vert = np.array([VERTICAL_ANGLES[model]], dtype=np.float32)
                self.channel = self.vert.shape[1]
                n = int(p("~lidar_packets_per_frame", 0))
                if n <= 0:
                    rate = float(p("~lidar_rotation_rate", 10.0))
                    per_pkt = 24 if self.channel == 16 else 12
                    n = max(1, int(round((36000.0 / (rate * 2.0)) / per_pkt)))
                self.packets_per_frame = n
                self.pub_pc = rospy.Publisher("/velodyne_points", PointCloud2,
                                              queue_size=1)
                rospy.loginfo("lidar model=%s ch=%d packets_per_frame=%d",
                              model, self.channel, n)
                # Dest만 listen. Host 포트는 SIM bind — Docker에서 publish/listen 하지 말 것.
                ports = [int(p("~lidar_port", 9299))]
                host_p = int(p("~lidar_host_port", 0))
                if host_p > 0 and host_p not in ports:
                    ports.append(host_p)
                for i, port in enumerate(ports):
                    tag = "lidar" if i == 0 else "lidar_h"
                    self._spawn(tag, port, self._lidar)


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

    @staticmethod
    def _norm_model(name):
        key = str(name).upper().replace("_", "").replace("-", "").replace(" ", "")
        table = {"VLP16": "VLP-16", "VLP32C": "VLP-32C",
                 "HDL32E": "HDL-32E", "HDL32": "HDL-32E"}
        return table.get(key)

    def _spawn(self, name, port, fn):
        self.stats[name] = 0
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
        buf = b""
        while not rospy.is_shutdown():
            try:
                blk, _ = sock.recvfrom(65000)
            except socket.timeout:
                continue
            if len(blk) < 21:
                continue
            buf += blk[19:-2]
            if blk[-2:] != b"EI":
                continue

            stamp = rospy.Time.now()
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.format = "jpeg"
            m.data = buf
            pub_c.publish(m)

            if pub_r is not None:
                try:
                    import cv2
                    img = cv2.imdecode(np.frombuffer(buf, np.uint8),
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

            buf = b""
            self.stats[name] += 1

    def _lidar(self, sock, name):
        """Velodyne UDP. 패킷을 모으되 타임아웃 시에도 부분 프레임을 살린다."""
        ch = self.channel
        cos_v = np.cos(np.deg2rad(self.vert))
        sin_v = np.sin(np.deg2rad(self.vert))
        need = self.packets_per_frame
        min_pkts = max(12, need // 4)  # 최소 이만큼이면 한 프레임으로 발행
        chunks = []
        raw_seen = 0

        while not rospy.is_shutdown():
            try:
                pkt, _ = sock.recvfrom(2048)
            except socket.timeout:
                if len(chunks) >= min_pkts:
                    self._lidar_publish(chunks, name, ch, cos_v, sin_v)
                    chunks = []
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
                continue
            # 1200B 데이터부 (표준 1206 = 1200 + 6 timestamp)
            chunks.append(pkt[:1200] if len(pkt) >= 1200 else pkt.ljust(1200, b"\x00")[:1200])

            if len(chunks) >= need:
                self._lidar_publish(chunks, name, ch, cos_v, sin_v)
                chunks = []

    def _lidar_publish(self, chunks, name, ch, cos_v, sin_v):
        try:
            b = np.frombuffer(b"".join(chunks), dtype=np.uint8).reshape([-1, 100])
        except ValueError as e:
            rospy.logwarn_throttle(5.0, "%s reshape fail (%d pkts): %s",
                                   name, len(chunks), e)
            return

        if ch == 16:
            az = np.zeros((b.shape[0] * 2,), dtype=np.float32)
            base = b[:, 2].astype(np.float32) + 256.0 * b[:, 3]
            az[0::2] = base
            az[1::2] = base + 20.0
        else:
            az = (b[:, 2].astype(np.float32) + 256.0 * b[:, 3])

        dist = (b[:, 4::3].astype(np.float32)
                + 256.0 * b[:, 5::3].astype(np.float32)) * 2.0
        inten = b[:, 6::3].astype(np.float32).reshape(-1)

        az = az.reshape([-1, 1]) / 100.0
        dist = dist.reshape([-1, ch]) / 1000.0

        sin_a = np.sin(np.deg2rad(az))
        cos_a = np.cos(np.deg2rad(az))
        x = (dist * cos_v * sin_a).reshape(-1)
        y = (dist * cos_v * cos_a).reshape(-1)
        z = (dist * sin_v).reshape(-1)

        valid = dist.reshape(-1) > 0.01
        self.pub_pc.publish(
            self._make_cloud(x[valid], y[valid], z[valid], inten[valid]))
        self.stats[name] += 1

    def _make_cloud(self, x, y, z, i):
        n = int(x.shape[0])
        arr = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32),
                                 ("z", np.float32), ("intensity", np.float32)])
        arr["x"], arr["y"], arr["z"], arr["intensity"] = x, y, z, i

        m = PointCloud2()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame_lidar
        m.height = 1
        m.width = n
        m.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        m.is_bigendian = False
        m.point_step = 16
        m.row_step = 16 * n
        m.is_dense = True
        m.data = arr.tobytes()
        return m

    def _gps(self, sock, name):
        while not rospy.is_shutdown():
            try:
                raw, _ = sock.recvfrom(8196)
            except socket.timeout:
                continue
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
            self.pub_gps.publish(g)

            fx = NavSatFix()
            fx.header = g.header
            fx.status.status = (NavSatStatus.STATUS_FIX if status > 0
                                else NavSatStatus.STATUS_NO_FIX)
            fx.status.service = NavSatStatus.SERVICE_GPS
            fx.latitude, fx.longitude, fx.altitude = lat, lon, alt
            self.pub_fix.publish(fx)
            self.stats[name] += 1

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
            if raw[0:9] != b"#IMUData$" or len(raw) < 113:
                continue
            d = struct.unpack("<10d", raw[33:113])

            m = Imu()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = self.frame_imu
            m.orientation.w, m.orientation.x = d[0], d[1]
            m.orientation.y, m.orientation.z = d[2], d[3]
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z \
                = d[4], d[5], d[6]
            m.linear_acceleration.x, m.linear_acceleration.y, \
                m.linear_acceleration.z = d[7], d[8], d[9]
            self.pub_imu.publish(m)
            self.stats[name] += 1

    def _report(self, _):
        rospy.loginfo("rx " + "  ".join("%s=%d" % kv
                                        for kv in sorted(self.stats.items())))
        for k, v in self.stats.items():
            if v == 0:
                rospy.logwarn_throttle(
                    30.0, "%s 수신 0건 - MORAI Sensor Destination Port 확인.", k)


def main():
    rospy.init_node("morai_sensor_bridge")
    SensorBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
