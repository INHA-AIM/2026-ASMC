#!/usr/bin/env python3
"""
ROS compressed image -> traffic-light inference -> ROS result topics.

This is the deployment entry point. It loads the model and performs inference
in one process.

Input:
  /cam_extra/image_jpeg/compressed  (sensor_msgs/CompressedImage)

Output:
  /traffic_light/state   (std_msgs/String: label only)
  /traffic_light/detail  (std_msgs/String: JSON with label/prob/top/ms)
"""

import argparse
import io
import json
import os
import sys
import threading
import time

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from model import IMAGENET_MEAN, IMAGENET_STD, decide, load_model  # noqa: E402


class LatestCompressedImage(object):
    """Keep only the newest JPEG received from a ROS CompressedImage topic."""

    def __init__(self, rospy, compressed_image_cls, topic):
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._sub = rospy.Subscriber(
            topic, compressed_image_cls, self._callback,
            queue_size=1, buff_size=16 * 1024 * 1024)

    def _callback(self, msg):
        with self._lock:
            self._jpeg = bytes(msg.data)
            self._seq += 1

    def latest(self):
        with self._lock:
            return self._jpeg, self._seq

    def close(self):
        self._sub.unregister()


def select_device(requested):
    """Select CUDA only when the installed PyTorch wheel supports this GPU."""
    if requested == "cpu":
        return torch.device("cpu")
    if not torch.cuda.is_available():
        if requested == "cuda":
            raise RuntimeError("--device cuda를 지정했지만 CUDA를 사용할 수 없다.")
        return torch.device("cpu")

    major, minor = torch.cuda.get_device_capability(0)
    gpu_arch = "sm_%d%d" % (major, minor)
    built_arches = torch.cuda.get_arch_list()
    # CUDA cubin은 같은 major compute capability 안에서 상위 minor GPU와
    # 호환된다. 따라서 sm_86 wheel은 RTX 40의 sm_89에서도 실행 가능하다.
    built_majors = set()
    for arch in built_arches:
        if arch.startswith("sm_") and len(arch) >= 5:
            try:
                built_majors.add(int(arch[3:-1]))
            except ValueError:
                pass
    if major not in built_majors:
        message = ("현재 PyTorch가 GPU 아키텍처 %s를 지원하지 않는다 "
                   "(지원 목록: %s). GPU에 맞는 PyTorch를 설치할 것."
                   % (gpu_arch, ", ".join(built_arches) or "없음"))
        if requested == "cuda":
            raise RuntimeError(message)
        print("[경고] %s CPU로 실행한다." % message)
        return torch.device("cpu")
    return torch.device("cuda")


def main(argv=None):
    try:
        import rospy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
    except ImportError as exc:
        print("ROS Python 모듈을 불러올 수 없다: %s" % exc)
        print("먼저 ROS 환경을 source 하거나 ./run.sh 를 사용할 것.")
        return 1

    ap = argparse.ArgumentParser(description="ROS 신호등 인지 노드")
    ap.add_argument("--ckpt", default=os.path.join(HERE, "best.pt"),
                    help="체크포인트 경로 (기본: traffic_light/best.pt)")
    ap.add_argument("--image-topic", default="/cam_extra/image_jpeg/compressed",
                    help="sensor_msgs/CompressedImage 입력 토픽")
    ap.add_argument("--state-topic", default="/traffic_light/state",
                    help="std_msgs/String 신호 라벨 출력 토픽")
    ap.add_argument("--detail-topic", default="/traffic_light/detail",
                    help="std_msgs/String JSON 상세 결과 출력 토픽")
    ap.add_argument("--hz", type=float, default=10.0, help="초당 추론 횟수 상한")
    ap.add_argument("--topk", type=int, default=3, help="상세 결과에 넣을 상위 후보 수")
    ap.add_argument("--min-detect", type=float, default=0.40,
                    help="unknown 제외 최고 확률이 이 값 이하면 not_detected")
    ap.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto",
                    help="추론 장치 (기본 auto; 미지원 CUDA wheel이면 CPU로 폴백)")
    ap.add_argument("--wait-first", type=float, default=10.0,
                    help="첫 이미지 대기 시간(초, 0이면 무한 대기)")
    ap.add_argument("--stale-sec", type=float, default=1.0,
                    help="이 시간 동안 새 결과가 없으면 not_detected 발행")
    ap.add_argument("--print-topk", action="store_true",
                    help="터미널에 2, 3순위 후보도 표시")
    # roslaunch appends __name:=... and __log:=... remapping arguments.
    # rospy.myargv removes them before argparse validates application options.
    args = ap.parse_args(rospy.myargv(argv=argv)[1:])

    rospy.init_node("traffic_light_infer", anonymous=False)

    try:
        device = select_device(args.device)
    except RuntimeError as exc:
        rospy.logfatal(str(exc))
        return 1
    model, tf, classes, size, val_acc = load_model(args.ckpt, device)

    out_h, out_w = size, int(round(size * 4 / 3))
    mean = torch.tensor(IMAGENET_MEAN, device=device).view(1, 3, 1, 1)
    std = torch.tensor(IMAGENET_STD, device=device).view(1, 3, 1, 1)

    def preprocess(img):
        x = torch.from_numpy(np.asarray(img, dtype=np.uint8)).to(device)
        x = x.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        x = F.interpolate(x, size=(out_h, out_w), mode="bilinear", align_corners=False)
        return (x - mean) / std

    source = LatestCompressedImage(rospy, CompressedImage, args.image_topic)
    state_pub = rospy.Publisher(args.state_topic, String, queue_size=1, latch=True)
    detail_pub = rospy.Publisher(args.detail_topic, String, queue_size=1, latch=True)

    print("=" * 68)
    print("입력 토픽 : %s" % args.image_topic)
    print("결과 토픽 : %s, %s" % (args.state_topic, args.detail_topic))
    print("모델      : %s | 클래스 %d개" % (os.path.basename(args.ckpt), len(classes)))
    print("입력 크기 : %dx%d" % (out_w, out_h))
    print("장치      : %s" % (torch.cuda.get_device_name(0)
                              if device.type == "cuda" else "CPU"))
    print("val_acc   : %.4f (체크포인트 저장 시점)" % val_acc)
    print("=" * 68)
    print("첫 이미지 대기 중...")

    wait_started = time.monotonic()
    while not rospy.is_shutdown():
        jpeg, _ = source.latest()
        if jpeg is not None:
            print("이미지 수신 시작 (%d bytes)" % len(jpeg))
            break
        if args.wait_first > 0 and time.monotonic() - wait_started > args.wait_first:
            rospy.logerr("%s 에서 이미지를 받지 못했다.", args.image_topic)
            source.close()
            return 1
        rospy.sleep(0.05)

    period = 1.0 / max(args.hz, 0.5)
    last_seq = -1
    count = 0
    last_result_at = 0.0
    stale_published = False
    try:
        while not rospy.is_shutdown():
            loop_started = time.perf_counter()
            jpeg, seq = source.latest()
            if jpeg is None or seq == last_seq:
                if (last_result_at and not stale_published and
                        time.monotonic() - last_result_at > args.stale_sec):
                    stale = {
                        "label": "not_detected", "prob": 0.0,
                        "top": [], "ms": 0.0, "stale": True,
                    }
                    state_pub.publish(String(data="not_detected"))
                    detail_pub.publish(String(data=json.dumps(
                        stale, separators=(",", ":"))))
                    stale_published = True
                rospy.sleep(0.005)
                continue
            last_seq = seq

            try:
                img = Image.open(io.BytesIO(jpeg)).convert("RGB")
                if device.type == "cuda":
                    x = preprocess(img)
                else:
                    x = tf(img).unsqueeze(0).to(device)
                with torch.no_grad():
                    with torch.autocast("cuda", dtype=torch.bfloat16,
                                        enabled=device.type == "cuda"):
                        probs = F.softmax(model(x).float(), dim=1)[0].cpu()
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "이미지 추론 실패: %s", exc)
                continue

            infer_ms = (time.perf_counter() - loop_started) * 1000
            label, prob = decide(probs, classes, args.min_detect)
            vals, idx = probs.topk(min(args.topk, len(classes)))
            result = {
                "label": label,
                "prob": round(float(prob), 4),
                "top": [[classes[int(i)], round(float(v), 4)]
                        for v, i in zip(vals, idx)],
                "ms": round(infer_ms, 2),
            }

            state_pub.publish(String(data=label))
            detail_pub.publish(String(data=json.dumps(result, separators=(",", ":"))))
            count += 1
            last_result_at = time.monotonic()
            stale_published = False

            line = "  ▶ %-16s %3.0f%%  (%.1fms)" % (
                label.upper(), result["prob"] * 100, result["ms"])
            if args.print_topk:
                line += "  [" + " ".join(
                    "%s %.0f%%" % (name, value * 100)
                    for name, value in result["top"][1:]) + "]"
            sys.stdout.write("\r\033[K" + line)
            sys.stdout.flush()

            remaining = period - (time.perf_counter() - loop_started)
            if remaining > 0:
                rospy.sleep(remaining)
    finally:
        source.close()
        print("\n종료. 총 %d장 추론." % count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
