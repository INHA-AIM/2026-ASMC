#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 카메라 추론 노드.

이 노드는 ROS Noetic의 시스템 Python 3.8이 아니라 CUDA 12.8 PyTorch가 설치된
전용 Python 3.9 환경(/opt/yolo-venv)에서 실행된다. 실행 경로는
`Main/scripts/yolo_detection_node` 래퍼가 고정하므로, 이 파일의 shebang이나
catkin이 생성하는 relay 스크립트의 shebang에 의존하지 않는다.

RTX 50 시리즈(sm_120)는 cu118 PyTorch 바이너리에 커널 이미지가 없어
`torch.cuda.is_available()`가 True여도 실제 커널 실행이 실패한다. 그래서 이
노드는 가용성 플래그만 믿지 않고 실제 CUDA 커널을 실행해 보는 preflight를
거친 뒤에만 추론을 시작한다.
"""
import statistics
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospkg
import rospy
import torch
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from Msg.msg import Yolo_bbox, Yolo_bbox_arr


def describe_torch_environment():
    """CUDA 실패 원인을 한눈에 가릴 수 있는 진단 항목을 모은다."""
    info = [
        "torch=%s" % torch.__version__,
        "torch_cuda=%s" % torch.version.cuda,
        "torch.cuda.is_available=%s" % torch.cuda.is_available(),
    ]
    try:
        info.append("torch_arch_list=%s" % ",".join(torch.cuda.get_arch_list()))
    except Exception as exc:  # 드라이버 없이 빌드된 바이너리 등
        info.append("torch_arch_list=<unavailable: %s>" % exc)
    try:
        info.append("device_count=%d" % torch.cuda.device_count())
    except Exception as exc:
        info.append("device_count=<unavailable: %s>" % exc)
    if torch.cuda.is_available():
        try:
            major, minor = torch.cuda.get_device_capability(0)
            info.append("gpu_name=%s" % torch.cuda.get_device_name(0))
            info.append("gpu_capability=sm_%d%d" % (major, minor))
        except Exception as exc:
            info.append("gpu_query=<failed: %s>" % exc)
    return info


def cuda_preflight(device):
    """실제 CUDA 커널을 실행해 장치가 쓸 수 있는 상태인지 확인한다.

    `torch.cuda.is_available()`만으로는 아키텍처 불일치를 걸러내지 못하므로,
    커널 실행 후 동기화까지 마쳐야 성공으로 본다.

    Returns:
        (ok, detail): ok가 False면 detail이 실패 사유 문자열이다.
    """
    if not torch.cuda.is_available():
        return False, "torch.cuda.is_available() is False"
    try:
        if torch.cuda.device_count() < 1:
            return False, "torch.cuda.device_count() == 0"
        # 실제 커널을 태우고 동기화해 비동기 오류까지 여기서 잡는다.
        probe = torch.randn(256, 256, device=device)
        product = probe @ probe
        torch.cuda.synchronize(device)
        value = float(product.sum().item())
        if not np.isfinite(value):
            return False, "CUDA matmul produced a non-finite result"
    except Exception as exc:
        return False, "%s: %s" % (type(exc).__name__, exc)
    return True, "CUDA kernel executed and synchronized"


class IntervalStats:
    """일정 구간 동안 표본을 모아 한 번만 로그로 내보낸다."""

    def __init__(self, interval_sec):
        self.interval_sec = interval_sec
        self.reset()

    def reset(self):
        self.samples = {}
        self.started_at = time.monotonic()
        self.frames = 0

    def add(self, name, value):
        self.samples.setdefault(name, []).append(value)

    def due(self):
        return time.monotonic() - self.started_at >= self.interval_sec

    def summary(self, name):
        values = self.samples.get(name)
        if not values:
            return "n/a"
        ordered = sorted(values)
        p95 = ordered[min(int(round(0.95 * (len(ordered) - 1))),
                          len(ordered) - 1)]
        return "%.1f/%.1f/%.1f" % (
            sum(ordered) / len(ordered), statistics.median(ordered), p95)

    def elapsed(self):
        return max(time.monotonic() - self.started_at, 1e-9)


class st_LaneDetection:
    def __init__(self):
        # --- 가중치 경로: launch 파라미터 우선, 기본값은 Main 패키지 내부 ---
        default_weights = Path(rospkg.RosPack().get_path("Main")) / "YOLOWeight" / "yolo11m.pt"
        weights_path = Path(rospy.get_param("~weights", str(default_weights)))
        if not weights_path.exists():
            rospy.logfatal(f"가중치 파일 없음: {weights_path}")
            raise SystemExit(1)

        # --- 파라미터화 ---
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.7))
        self.allowed_names = {s.strip().lower() for s in rospy.get_param("~allowed_names",
                                                                         ["bus","truck","person","car"])}
        self.stats = IntervalStats(
            float(rospy.get_param("~perf_log_interval_sec", 5.0)))

        # --- 장치 선택: 기본은 CUDA 강제, 명시적으로 꺼야만 CPU 허용 ---
        self.require_cuda = bool(rospy.get_param("~require_cuda", True))
        requested_device = str(rospy.get_param("~device", "cuda:0"))
        self.device = self._select_device(requested_device)
        self.use_cuda = self.device.startswith("cuda")

        # --- 모델 로드 ---
        self.model = YOLO(str(weights_path))
        try:
            self.model.fuse()
        except Exception:
            pass
        self.model.to(self.device)
        self._verify_model_device(weights_path)

        # Subscriber / Publisher
        self.st_Subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback_Image,
                                              queue_size=1, buff_size=2**24)
        self.st_Publisher  = rospy.Publisher("/YOLO/BBox", Yolo_bbox_arr, queue_size=5)

        self.st_BGRImg = None
        self.image_header = None
        self.frame_seq = 0
        self.decode_ms = 0.0
        self.frame_lock = threading.Lock()
        self.rate = rospy.Rate(30)

        # 모델 체크포인트에 들어있는 클래스명 사용 (id->name dict)
        self.names = self.model.names
        # 이름을 ID로 변환
        self.allowed_ids = {cid for cid, cname in self.names.items() if str(cname).lower() in self.allowed_names}
        if not self.allowed_ids:
            rospy.logwarn("[경고] allowed_ids가 비어 있음. 체크포인트 names 표기가 기대와 다를 수 있어, 루프에서 이름으로 재검증합니다.")

        self.SpeedSignDetection()

    def _select_device(self, requested):
        """preflight를 통과한 장치만 돌려준다. 조용한 CPU fallback은 없다."""
        if requested.startswith("cpu"):
            if self.require_cuda:
                rospy.logfatal(
                    "~device=%s 인데 ~require_cuda=true 입니다. CPU로 실행하려면 "
                    "~require_cuda=false 를 명시하십시오." % requested)
                raise SystemExit(1)
            rospy.logwarn("YOLO running on CPU by explicit request (~device=%s)"
                          % requested)
            return "cpu"

        ok, detail = cuda_preflight(requested)
        report = "\n  ".join(describe_torch_environment())
        if ok:
            rospy.loginfo("YOLO CUDA preflight OK:\n  device=%s\n  %s\n  %s"
                          % (requested, report, detail))
            return requested

        message = (
            "YOLO CUDA preflight FAILED: %s\n  requested_device=%s\n  %s\n"
            "  힌트: sm_120(RTX 50 시리즈)은 CUDA 12.8 이상 PyTorch가 필요합니다. "
            "torch_arch_list에 sm_120이 없으면 PyTorch 바이너리 문제이고, "
            "is_available=False 이면 드라이버 또는 Docker GPU 전달 문제입니다."
            % (detail, requested, report))
        if self.require_cuda:
            rospy.logfatal(message)
            raise SystemExit(1)
        rospy.logwarn(message + "\n  ~require_cuda=false 이므로 CPU로 계속합니다.")
        return "cpu"

    def _verify_model_device(self, weights_path):
        """모델 파라미터가 실제로 요청한 장치에 올라갔는지, warm-up 추론이
        그 장치에서 성공하는지까지 확인한다."""
        try:
            model_device = str(next(self.model.model.parameters()).device)
        except Exception as exc:
            model_device = "<unknown: %s>" % exc

        warmup_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        try:
            if self.use_cuda:
                torch.cuda.synchronize(self.device)
            started = time.perf_counter()
            list(self.model(warmup_image, device=self.device, stream=True,
                            verbose=False, conf=self.conf_thres))
            if self.use_cuda:
                torch.cuda.synchronize(self.device)
            warmup_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            rospy.logfatal(
                "YOLO warm-up inference failed on %s: %s: %s\n  weights=%s\n  %s"
                % (self.device, type(exc).__name__, exc, weights_path,
                   "\n  ".join(describe_torch_environment())))
            raise SystemExit(1)

        if self.use_cuda and not model_device.startswith("cuda"):
            rospy.logfatal(
                "YOLO model parameters are on %s but %s was requested; "
                "refusing to run a silent CPU fallback."
                % (model_device, self.device))
            raise SystemExit(1)

        rospy.loginfo(
            "YOLO model ready:\n  weights=%s\n  model_device=%s\n  "
            "warmup_inference=%.1f ms" % (weights_path, model_device, warmup_ms))

    def callback_Image(self, msg: CompressedImage):
        try:
            started = time.perf_counter()
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            decode_ms = (time.perf_counter() - started) * 1000.0
            if img_bgr is not None:
                with self.frame_lock:
                    self.st_BGRImg = img_bgr
                    # Keep the exact source-image timestamp. The detector may
                    # finish much later, so publish time is not usable for
                    # camera/LiDAR synchronization.
                    self.image_header = (msg.header.stamp, msg.header.frame_id)
                    self.frame_seq += 1
                    self.decode_ms = decode_ms
        except Exception as e:
            rospy.logwarn(f"CompressedImage 디코드 실패: {e}")

    def _log_performance(self):
        stats = self.stats
        fps = stats.frames / stats.elapsed()
        rospy.loginfo(
            "YOLO performance (mean/median/p95 ms over %.1fs, %d frames):\n"
            "  decode=%s\n  preprocess=%s\n  inference=%s\n  postprocess=%s\n"
            "  callback=%s\n  e2e=%s\n  fps=%.2f device=%s"
            % (stats.elapsed(), stats.frames,
               stats.summary("decode"), stats.summary("preprocess"),
               stats.summary("inference"), stats.summary("postprocess"),
               stats.summary("callback"), stats.summary("e2e"),
               fps, self.device))
        stats.reset()

    def SpeedSignDetection(self):
        while self.st_BGRImg is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        last_frame_seq = -1
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame_seq = self.frame_seq
                img = None if self.st_BGRImg is None else self.st_BGRImg.copy()
                image_header = self.image_header
                decode_ms = self.decode_ms

            # Run inference once per newly received camera frame.
            if img is None or frame_seq == last_frame_seq:
                self.rate.sleep()
                continue
            last_frame_seq = frame_seq
            callback_started = time.perf_counter()

            PublishMsg = Yolo_bbox_arr()
            if image_header is not None:
                PublishMsg.header.stamp = image_header[0]
                PublishMsg.header.frame_id = image_header[1]
            count = 0

            infer_kwargs = dict(stream=True, verbose=False, conf=self.conf_thres,
                                device=self.device)
            if self.allowed_ids:
                infer_kwargs["classes"] = list(self.allowed_ids)

            # CUDA 커널은 비동기라 동기화 없이는 추론 시간이 과소 측정된다.
            if self.use_cuda:
                torch.cuda.synchronize(self.device)
            inference_started = time.perf_counter()
            speed = {}

            for result in self.model(img, **infer_kwargs):
                speed = getattr(result, "speed", None) or speed
                boxes = getattr(result, "boxes", None)
                if boxes is None or len(boxes) == 0:
                    continue

                for box in boxes:
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    name = self.names.get(cls_id, str(cls_id))

                    # allowed_ids가 비어 있으면 이름으로 최종 필터(체크포인트 표기 차이 대비)
                    if not self.allowed_ids and str(name).lower() not in self.allowed_names:
                        continue
                    if conf < self.conf_thres:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    PublishMsg.yolo_bbox_arr.append(
                        Yolo_bbox(xMin=x1, yMin=y1, xMax=x2, yMax=y2, classes=name)
                    )
                    count += 1

                    # 디스플레이가 필요하면 주석 해제
                    # cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    # cv2.putText(img, f"{name} {conf:.2f}", (x1, max(0, y1-5)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1, cv2.LINE_AA)

            if self.use_cuda:
                torch.cuda.synchronize(self.device)
            inference_ms = (time.perf_counter() - inference_started) * 1000.0

            PublishMsg.objc = count
            self.st_Publisher.publish(PublishMsg)

            self.stats.frames += 1
            self.stats.add("decode", decode_ms)
            self.stats.add("preprocess", float(speed.get("preprocess", 0.0)))
            # ultralytics가 보고하는 순수 추론 시간과, 동기화까지 포함한
            # 실제 소요 시간을 함께 남긴다.
            self.stats.add("inference", inference_ms)
            self.stats.add("postprocess", float(speed.get("postprocess", 0.0)))
            self.stats.add(
                "callback", (time.perf_counter() - callback_started) * 1000.0)
            if image_header is not None and image_header[0].to_sec() > 0.0:
                self.stats.add(
                    "e2e", (rospy.Time.now() - image_header[0]).to_sec() * 1000.0)
            if self.stats.due():
                self._log_performance()

            # 필요 시 미리보기
            # cv2.imshow("YOLO Detection", img); cv2.waitKey(1)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('YoloDetection', anonymous=True)
    st_LaneDetection()
    rospy.spin()
