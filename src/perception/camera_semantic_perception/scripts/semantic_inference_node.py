#!/usr/bin/env python3
"""ROS1 live semantic inference with selectable MORAI camera views."""

from __future__ import annotations

import json
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from camera_semantic_perception.backends import create_backend
from camera_semantic_perception.frame_buffer import EncodedFrame, LatestFrameBuffer
from camera_semantic_perception.inference_engine import InferenceEngine


SUPPORTED_VIEWS = ("front", "left", "right")


class SemanticInferenceNode:
    def __init__(self) -> None:
        self.views = tuple(str(value) for value in rospy.get_param("~views", ["front"]))
        unknown = set(self.views) - set(SUPPORTED_VIEWS)
        if not self.views or unknown or len(set(self.views)) != len(self.views):
            raise ValueError(
                "views must be unique values from {}; got {}".format(
                    SUPPORTED_VIEWS, self.views
                )
            )
        camera_topics = dict(rospy.get_param("~camera_topics"))
        missing_topics = [view for view in self.views if view not in camera_topics]
        if missing_topics:
            raise ValueError("camera_topics missing views: {}".format(missing_topics))

        backend_name = str(rospy.get_param("~backend", "twinlite"))
        backend = create_backend(
            backend_name,
            checkpoint=str(rospy.get_param("~checkpoint")),
            model_config=str(rospy.get_param("~model_config", "medium")),
            twinlite_root=rospy.get_param("~twinlite_root", None),
            device=str(rospy.get_param("~device", "cuda")),
            weights=str(rospy.get_param("~weights", "auto")),
            amp=bool(rospy.get_param("~amp", True)),
            channels_last=bool(rospy.get_param("~channels_last", False)),
            input_hw=tuple(
                int(value)
                for value in rospy.get_param("~input_hw", [384, 640])
            ),
        )
        self.engine = InferenceEngine(
            backend,
            restore_original=bool(rospy.get_param("~restore_original", True)),
        )
        self.buffer = LatestFrameBuffer(
            self.views,
            batch_wait_ms=float(rospy.get_param("~batch_wait_ms", 4.0)),
        )
        self.log_interval = float(rospy.get_param("~log_interval_sec", 2.0))
        if self.log_interval <= 0:
            raise ValueError("log_interval_sec must be positive")
        self.categorical_heads = frozenset(backend.info.categorical_classes)
        self.output_topics = {
            view: {
                head: self._output_topic(view, head, head in self.categorical_heads)
                for head in backend.info.heads
            }
            for view in self.views
        }
        self.publishers = {
            view: {
                head: rospy.Publisher(
                    self.output_topics[view][head],
                    Image,
                    queue_size=1,
                )
                for head in backend.info.heads
            }
            for view in self.views
        }
        self.decode_errors = 0
        self.output_frames = 0
        self.output_batches = 0
        self.timing_totals = {
            "decode_ms": 0.0,
            "preprocess_ms": 0.0,
            "inference_ms": 0.0,
            "postprocess_ms": 0.0,
            "publish_ms": 0.0,
        }
        self._stats_lock = threading.Lock()
        self._last_log_wall = time.monotonic()
        self._last_output_frames = 0

        self.subscribers = []
        for view in self.views:
            subscriber = rospy.Subscriber(
                str(camera_topics[view]),
                CompressedImage,
                self._callback,
                callback_args=view,
                queue_size=1,
                buff_size=int(rospy.get_param("~subscriber_buffer_bytes", 16 << 20)),
            )
            self.subscribers.append(subscriber)
        self.worker = threading.Thread(target=self._worker, name="semantic-inference")
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "camera semantic backend: %s",
            json.dumps(backend.info.__dict__, sort_keys=True),
        )
        rospy.loginfo(
            "camera semantic views=%s inputs=%s outputs=%s",
            self.views,
            camera_topics,
            self.output_topics,
        )

    @staticmethod
    def _output_topic(view: str, head: str, categorical: bool) -> str:
        suffix = "class_id" if categorical else "probability"
        return "/perception/camera/{}/{}/{}".format(view, head, suffix)

    def _callback(self, message: CompressedImage, view: str) -> None:
        self.buffer.push(
            EncodedFrame(
                view=view,
                stamp=message.header.stamp,
                frame_id=message.header.frame_id,
                data=bytes(message.data),
                received_monotonic=time.monotonic(),
            )
        )

    @staticmethod
    def _image_message(
        value: np.ndarray, frame: EncodedFrame, categorical: bool = False
    ) -> Image:
        if categorical:
            value = np.ascontiguousarray(value, dtype=np.uint8)
        else:
            value = np.rint(np.clip(value, 0.0, 1.0) * 255.0).astype(np.uint8)
        message = Image()
        message.header.stamp = frame.stamp
        message.header.frame_id = frame.frame_id
        message.height, message.width = value.shape
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = message.width
        message.data = value.tobytes()
        return message

    def _worker(self) -> None:
        while not rospy.is_shutdown():
            frames = self.buffer.pop_batch(timeout=0.2)
            if not frames:
                continue
            decoded = {}
            usable_frames = {}
            started = time.perf_counter()
            for frame in frames:
                image = cv2.imdecode(
                    np.frombuffer(frame.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is None:
                    self.decode_errors += 1
                    rospy.logwarn_throttle(2.0, "failed to decode camera JPEG")
                    continue
                decoded[frame.view] = image
                usable_frames[frame.view] = frame
            decode_ms = (time.perf_counter() - started) * 1000.0
            if not decoded:
                continue
            try:
                prediction = self.engine.infer(decoded)
            except Exception as exc:
                rospy.logerr_throttle(2.0, "semantic inference failed: %s", exc)
                continue

            started = time.perf_counter()
            for view, value in prediction.views.items():
                frame = usable_frames[view]
                for head, output in value.probabilities.items():
                    self.publishers[view][head].publish(
                        self._image_message(
                            output, frame, head in self.categorical_heads
                        )
                    )
            publish_ms = (time.perf_counter() - started) * 1000.0
            with self._stats_lock:
                self.output_batches += 1
                self.output_frames += len(prediction.views)
                for name, value in (
                    ("decode_ms", decode_ms),
                    ("preprocess_ms", prediction.preprocess_ms),
                    ("inference_ms", prediction.inference_ms),
                    ("postprocess_ms", prediction.postprocess_ms),
                    ("publish_ms", publish_ms),
                ):
                    self.timing_totals[name] += value
                self._maybe_log()

    def _maybe_log(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_log_wall
        if elapsed < self.log_interval:
            return
        frame_delta = self.output_frames - self._last_output_frames
        divisor = max(1, self.output_batches)
        averages = {
            name: round(value / divisor, 2)
            for name, value in self.timing_totals.items()
        }
        rospy.loginfo(
            "semantic fps=%.2f output=%d batches=%d replaced=%s decode_errors=%d avg_ms=%s",
            frame_delta / elapsed,
            self.output_frames,
            self.output_batches,
            self.buffer.replaced,
            self.decode_errors,
            averages,
        )
        self._last_log_wall = now
        self._last_output_frames = self.output_frames
        self.output_batches = 0
        for name in self.timing_totals:
            self.timing_totals[name] = 0.0

    def _shutdown(self) -> None:
        self.buffer.close()


def main() -> None:
    rospy.init_node("camera_semantic_inference")
    SemanticInferenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
