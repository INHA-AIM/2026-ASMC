#!/usr/bin/env python3
"""Extra-camera traffic-light inference with typed and legacy outputs."""

from __future__ import annotations

import argparse
import copy
import hashlib
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

from asmc_camera.traffic_light_contract import (
    populate_detection,
)
from asmc_camera.traffic_light_model import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    decide,
    load_model,
)


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class LatestCompressedImage:
    """Keep bytes and the exact header of only the newest camera message."""

    def __init__(self, rospy, compressed_image_cls, topic, input_source=None):
        self._lock = threading.Lock()
        self._jpeg = None
        self._header = None
        self._seq = 0
        self.on_receive = None
        self._generation = None
        self._sub = (input_source.subscribe(topic, self._callback) if input_source
                     else rospy.Subscriber(topic, compressed_image_cls, self._callback,
                                           queue_size=1, buff_size=16 * 1024 * 1024))

    def _callback(self, message):
        generation = self.on_receive(message.header) if self.on_receive is not None else None
        with self._lock:
            self._jpeg = bytes(message.data)
            self._header = copy.deepcopy(message.header)
            self._seq += 1
            self._generation = generation

    def latest(self, include_generation=False):
        with self._lock:
            values = self._jpeg, self._seq, copy.deepcopy(self._header)
            return values + (self._generation,) if include_generation else values

    def close(self):
        self._sub.unregister()


def select_device(requested):
    if requested == "cpu":
        return torch.device("cpu")
    if not torch.cuda.is_available():
        if requested == "cuda":
            raise RuntimeError(
                "CUDA requested but unavailable in the asmc-ros-noetic runtime"
            )
        return torch.device("cpu")
    major, minor = torch.cuda.get_device_capability(0)
    built_majors = set()
    for architecture in torch.cuda.get_arch_list():
        if architecture.startswith("sm_") and len(architecture) >= 5:
            try:
                built_majors.add(int(architecture[3:-1]))
            except ValueError:
                pass
    if major not in built_majors:
        message = "PyTorch does not support GPU sm_%d%d" % (major, minor)
        if requested == "cuda":
            raise RuntimeError(message)
        return torch.device("cpu")
    return torch.device("cuda")


from asmc_camera.runtime.ros_control import RosInferenceControl
from asmc_camera.runtime.state import ACTIVE


def main(argv=None, parameters=None, input_source=None, init_node=True, on_ready=None, fault_handler=None):
    import rospy
    from asmc_msgs.msg import TrafficLightDetection
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ckpt", required=True)
    parser.add_argument("--image-topic", default="/cam_extra/image_jpeg/compressed")
    parser.add_argument("--state-topic", default="/traffic_light/state")
    parser.add_argument("--detail-topic", default="/traffic_light/detail")
    parser.add_argument("--typed-topic", default="/perception/traffic_light/detection")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--topk", type=int, default=3)
    parser.add_argument("--min-detect", type=float, default=0.40)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--wait-first", type=float, default=10.0)
    parser.add_argument("--stale-sec", type=float, default=1.0)
    parser.add_argument("--print-topk", action="store_true")
    args = parser.parse_args(rospy.myargv(argv=argv)[1:])

    if init_node:
        rospy.init_node("traffic_light_infer", anonymous=False)
    from asmc_camera.runtime.parameters import Parameters
    parameters = parameters or Parameters()
    for key, attribute in [('checkpoint','ckpt'), ('image_topic','image_topic'), ('device','device'),
                           ('target_hz','hz'), ('min_detect','min_detect'), ('stale_sec','stale_sec')]:
        if parameters.namespace or parameters.get('~runtime_profile', '') == 'camera_runtime_v1':
            setattr(args, attribute, parameters.get('~' + key, getattr(args, attribute)))
    try:
        device = select_device(args.device)
    except RuntimeError as exc:
        rospy.logfatal(str(exc))
        return 1

    checkpoint = os.path.abspath(os.path.expanduser(args.ckpt))
    load_started = time.perf_counter()
    model, transform, classes, size, validation_accuracy = load_model(
        checkpoint, device
    )
    model_load_ms = (time.perf_counter() - load_started) * 1000.
    model_id = "sha256:" + _sha256(checkpoint)
    output_height, output_width = size, int(round(size * 4 / 3))
    mean = torch.tensor(IMAGENET_MEAN, device=device).view(1, 3, 1, 1)
    std = torch.tensor(IMAGENET_STD, device=device).view(1, 3, 1, 1)

    def preprocess(image):
        value = torch.from_numpy(
            np.array(image, dtype=np.uint8, copy=True)
        ).to(device)
        value = value.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        value = F.interpolate(
            value,
            size=(output_height, output_width),
            mode="bilinear",
            align_corners=False,
        )
        return (value - mean) / std

    # Warm the exact runtime preprocessing, dtype and model before input handling.
    warmup_started = time.perf_counter()
    dummy = Image.new("RGB", (output_width, output_height))
    tensor = preprocess(dummy) if device.type == "cuda" else transform(dummy).unsqueeze(0).to(device)
    with torch.no_grad(), torch.autocast("cuda", dtype=torch.bfloat16, enabled=device.type == "cuda"):
        model(tensor)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    dummy_warmup_ms = (time.perf_counter() - warmup_started) * 1000.

    source = LatestCompressedImage(rospy, CompressedImage, args.image_topic, input_source)
    state_publisher = rospy.Publisher(
        args.state_topic, String, queue_size=1, latch=True
    )
    detail_publisher = rospy.Publisher(
        args.detail_topic, String, queue_size=1, latch=True
    )
    typed_publisher = rospy.Publisher(
        args.typed_topic, TrafficLightDetection, queue_size=1, latch=False
    )

    last_header = None

    def invalidate(mode):
        if mode == ACTIVE:
            return
        state_publisher.publish(String(data="not_detected"))
        detail_publisher.publish(String(data=json.dumps({"label": "not_detected", "stale": True})))
        if last_header is not None:
            typed_publisher.publish(populate_detection(
                TrafficLightDetection(), header=last_header, label="not_detected",
                confidence=0., stale=True, inference_latency_ms=0., model_id=model_id,
                candidate_labels=[], candidate_confidences=[]))

    control = RosInferenceControl("resnet18", ["camera4"], args.hz, on_transition=invalidate, parameters=parameters)
    control.state.record('startup', model_load_ms=model_load_ms, dummy_warmup_ms=dummy_warmup_ms)
    def observe_header(header):
        generation = control.observe("camera4", header.stamp.to_nsec())
        if control.state.trace is not None:
            control.state.record('received', view='camera4', source_stamp_ns=header.stamp.to_nsec())
        return generation
    source.on_receive = observe_header
    rospy.loginfo(
        "traffic-light ready input=%s legacy=%s,%s typed=%s model=%s device=%s val_acc=%.4f",
        args.image_topic,
        args.state_topic,
        args.detail_topic,
        args.typed_topic,
        model_id,
        device,
        validation_accuracy,
    )

    if on_ready:
        on_ready(control)
    wait_started = time.monotonic()
    while not rospy.is_shutdown():
        jpeg, _, header, source_generation = source.latest(True)
        if jpeg is not None and header is not None:
            break
        if not control.state.managed and args.wait_first > 0 and time.monotonic() - wait_started > args.wait_first:
            rospy.logerr("no image received from %s", args.image_topic)
            source.close()
            return 1
        rospy.sleep(0.05)

    period = 1.0 / max(args.hz, 0.5)
    last_seq = -1
    last_result_at = 0.0
    last_header = None
    stale_published = False
    count = 0
    try:
        while not rospy.is_shutdown():
            loop_started = time.perf_counter()
            jpeg, sequence, source_header, source_generation = source.latest(True)
            if jpeg is None or sequence == last_seq:
                if (
                    control.state.mode == ACTIVE
                    and last_result_at
                    and not stale_published
                    and time.monotonic() - last_result_at > args.stale_sec
                ):
                    detail = {
                        "label": "not_detected",
                        "prob": 0.0,
                        "top": [],
                        "ms": 0.0,
                        "stale": True,
                    }
                    state_publisher.publish(String(data="not_detected"))
                    detail_publisher.publish(
                        String(data=json.dumps(detail, separators=(",", ":")))
                    )
                    typed_publisher.publish(
                        populate_detection(
                            TrafficLightDetection(),
                            header=last_header,
                            label="not_detected",
                            confidence=0.0,
                            stale=True,
                            inference_latency_ms=0.0,
                            model_id=model_id,
                            candidate_labels=[],
                            candidate_confidences=[],
                        )
                    )
                    stale_published = True
                rospy.sleep(0.005)
                continue
            if source_generation is None:
                rospy.sleep(.005)
                continue
            stamps = {"camera4": source_header.stamp.to_nsec()}
            token = control.state.begin(stamps, expected_generation=source_generation)
            if token is None:
                rospy.sleep(.005)
                continue
            last_seq = sequence

            try:
                image = Image.open(io.BytesIO(jpeg)).convert("RGB")
                tensor = (
                    preprocess(image)
                    if device.type == "cuda"
                    else transform(image).unsqueeze(0).to(device)
                )
                with torch.no_grad():
                    with torch.autocast(
                        "cuda", dtype=torch.bfloat16, enabled=device.type == "cuda"
                    ):
                        probabilities = F.softmax(model(tensor).float(), dim=1)[0].cpu()
            except Exception as exc:  # pragma: no cover - hardware/runtime path
                if fault_handler:
                    fault_handler(exc)
                control.state.complete(token, stamps, False, "inference_error")
                rospy.logwarn_throttle(2.0, "traffic-light inference failed: %s", exc)
                continue

            inference_ms = (time.perf_counter() - loop_started) * 1000.0
            label, confidence = decide(probabilities, classes, args.min_detect)
            candidate_values, candidate_indices = probabilities.topk(
                min(args.topk, len(classes))
            )
            candidate_labels = [
                classes[int(index)] for index in candidate_indices
            ]
            candidate_confidences = [
                float(value) for value in candidate_values
            ]
            detail = {
                "label": label,
                "prob": round(float(confidence), 4),
                "top": [
                    [name, round(value, 4)]
                    for name, value in zip(candidate_labels, candidate_confidences)
                ],
                "ms": round(inference_ms, 2),
            }
            with control.state.output_guard(token) as allowed:
                if not allowed:
                    continue
                control.state.complete(token, stamps, label not in ("unknown", "not_detected"))
                if control.state.mode != ACTIVE:
                    continue
                state_publisher.publish(String(data=label))
                detail_publisher.publish(
                    String(data=json.dumps(detail, separators=(",", ":")))
                )
                typed_publisher.publish(
                    populate_detection(
                        TrafficLightDetection(),
                        header=source_header,
                        label=label,
                        confidence=confidence,
                        stale=False,
                        inference_latency_ms=inference_ms,
                        model_id=model_id,
                        candidate_labels=candidate_labels,
                        candidate_confidences=candidate_confidences,
                    )
                )
                last_header = source_header
                last_result_at = time.monotonic()
                stale_published = False
                count += 1
                control.state.record('published', stamps=stamps, inference_ms=inference_ms)

            if args.print_topk:
                rospy.loginfo(
                    "traffic-light label=%s confidence=%.3f latency=%.1fms candidates=%s",
                    label,
                    confidence,
                    inference_ms,
                    candidate_labels,
                )
            runtime_period = 1. / control.state.target_hz if control.state.managed else period
            remaining = runtime_period - (time.perf_counter() - loop_started)
            if remaining > 0:
                rospy.sleep(remaining)
    finally:
        control.close()
        source.close()
        rospy.loginfo("traffic-light stopped after %d frames", count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
