#!/usr/bin/env python3
"""ROS1 live semantic inference with selectable MORAI camera views."""

from __future__ import annotations

import json
import threading
import time

import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage, Image

from asmc_camera.camera.backends import create_backend
from asmc_camera.camera.frame_buffer import EncodedFrame, LatestFrameBuffer
from asmc_camera.camera.inference_engine import InferenceEngine


from asmc_camera.runtime.cadence import CoherentFrames, downstream_gate
from asmc_camera.runtime.ros_control import RosInferenceControl
from asmc_msgs.msg import LaneBoundaryFamily


SUPPORTED_VIEWS = ("front", "left", "right")


class SemanticInferenceNode:
    def __init__(self, parameters=None, input_source=None, fault_handler=None) -> None:
        from asmc_camera.runtime.parameters import Parameters
        self.parameters = parameters or Parameters()
        self.input_source, self.fault_handler = input_source, fault_handler
        self.lazy_outputs = bool(self.parameters.get('~lazy_outputs', False))
        self.local_gpu_cache = None
        if bool(self.parameters.get('~gpu_preprocess', False)) and input_source is None:
            from asmc_camera.runtime.gpu_input import RawGpuCache
            self.local_gpu_cache = RawGpuCache(device=str(self.parameters.get('~device', 'cuda')))
        self.views = tuple(str(value) for value in self.parameters.get("~views", ["front"]))
        unknown = set(self.views) - set(SUPPORTED_VIEWS)
        if not self.views or unknown or len(set(self.views)) != len(self.views):
            raise ValueError(
                "views must be unique values from {}; got {}".format(
                    SUPPORTED_VIEWS, self.views
                )
            )
        camera_topics = dict(self.parameters.get("~camera_topics"))
        missing_topics = [view for view in self.views if view not in camera_topics]
        if missing_topics:
            raise ValueError("camera_topics missing views: {}".format(missing_topics))

        backend_name = str(self.parameters.get("~backend", "twinlite"))
        load_started = time.perf_counter()
        backend = create_backend(
            backend_name,
            **({'engine_manifest': str(self.parameters.get('~engine_manifest'))}
               if backend_name == 'twinlite_tensorrt' else {}),
            checkpoint=str(self.parameters.get("~checkpoint")),
            model_config=str(self.parameters.get("~model_config", "medium")),
            twinlite_root=self.parameters.get("~twinlite_root", None),
            device=str(self.parameters.get("~device", "cuda")),
            weights=str(self.parameters.get("~weights", "auto")),
            amp=bool(self.parameters.get("~amp", True)),
            channels_last=bool(self.parameters.get("~channels_last", False)),
            input_hw=tuple(
                int(value)
                for value in self.parameters.get("~input_hw", [384, 640])
            ),
            input_hw_by_view=self.parameters.get("~input_hw_by_view", {}),
        )
        self.engine = InferenceEngine(
            backend,
            restore_original=bool(self.parameters.get("~restore_original", True)),
            resize_interpolation=str(self.parameters.get("~resize_interpolation", "auto")),
        )
        if set(backend.info.input_hw_by_view) - set(SUPPORTED_VIEWS):
            raise ValueError("input_hw_by_view contains an unknown camera")
        model_load_ms = (time.perf_counter() - load_started) * 1000.
        dummy_warmup_ms = 0.
        warmup_iterations = int(self.parameters.get("~warmup_iterations", 1))
        if warmup_iterations < 0:
            raise ValueError("warmup_iterations must be non-negative")
        if warmup_iterations:
            warmup_image = np.zeros(
                (backend.info.input_hw[0], backend.info.input_hw[1], 3),
                dtype=np.uint8,
            )
            warmup_views = dict((view, warmup_image) for view in self.views)
            started = time.perf_counter()
            for _ in range(warmup_iterations):
                self.engine.infer(warmup_views)
            dummy_warmup_ms = (time.perf_counter() - started) * 1000.
            rospy.loginfo(
                "camera semantic warmup complete: iterations=%d views=%d elapsed_ms=%.1f",
                warmup_iterations,
                len(self.views),
                (time.perf_counter() - started) * 1000.0,
            )
        self.buffer = LatestFrameBuffer(
            self.views,
            batch_wait_ms=float(self.parameters.get("~batch_wait_ms", 4.0)),
        )
        self.log_interval = float(self.parameters.get("~log_interval_sec", 2.0))
        if self.log_interval <= 0:
            raise ValueError("log_interval_sec must be positive")
        self.categorical_heads = frozenset(backend.info.categorical_classes)
        self.backend_info = dict(backend.info.__dict__)
        self.latest_backend_stages = {}
        self.output_topics = {
            view: {
                head: self._output_topic(view, head, head in self.categorical_heads)
                for head in backend.info.heads
            }
            for view in self.views
        }
        self.confidence_topics = {
            view: {
                head: "/perception/camera/{}/{}/confidence".format(view, head)
                for head in self.categorical_heads
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
        self.confidence_publishers = {
            view: {
                head: rospy.Publisher(topic, Image, queue_size=1)
                for head, topic in self.confidence_topics[view].items()
            }
            for view in self.views
        }
        self.publish_model_grid_road_marking = bool(
            self.parameters.get("~publish_model_grid_road_marking", False)
        )
        self.model_grid_max_hz = float(self.parameters.get("~model_grid_max_hz", 10.0))
        if self.model_grid_max_hz <= 0.0:
            raise ValueError("model_grid_max_hz must be positive")
        self.model_grid_period_ns = int(round(1e9 / self.model_grid_max_hz))
        self.next_model_grid_stamp_ns = 0
        self.last_model_grid_stamp_ns = 0
        self.model_grid_timestamp_resets = 0
        self.model_grid_publishers = {}
        self.model_grid_confidence_publishers = {}
        self.grid_timing = None
        if (
            self.publish_model_grid_road_marking
            and "road_marking" in self.categorical_heads
        ):
            for view in self.views:
                namespace = "/perception/camera/{}/road_marking/model_grid".format(
                    view
                )
                self.model_grid_publishers[view] = rospy.Publisher(
                    namespace + "/class_id", Image, queue_size=1
                )
                self.model_grid_confidence_publishers[view] = rospy.Publisher(
                    namespace + "/confidence", Image, queue_size=1
                )
        self.diagnostics_publisher = rospy.Publisher(
            "/perception/camera/diagnostics", DiagnosticArray, queue_size=1
        )
        self.decode_errors = 0
        self.inference_errors = 0
        self.output_frames = 0
        self.output_by_view = {view: 0 for view in self.views}
        self.output_batches = 0
        self.total_output_batches = 0
        self.model_grid_output_batches = 0
        self.model_grid_throttled_batches = 0
        self.model_grid_incomplete_batches = 0
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
        self._last_decode_errors = 0
        self._last_inference_errors = 0
        self._queue_age_samples = []
        self._end_to_end_samples = []
        self._latest_input_stamp_ns = {}
        self._latest_receive_stamp_ns = {}
        self._latest_result_stamp_ns = 0
        self._timestamp_fallbacks_by_view = {view: None for view in self.views}
        self._timestamp_regressions_by_view = {view: None for view in self.views}

        self.control = RosInferenceControl("twinlite", self.views, 10., warmed_up=bool(warmup_iterations), parameters=self.parameters)
        if self.model_grid_publishers:
            from asmc_camera.runtime.grid_timing import GridTimingPublisher
            self.grid_timing = GridTimingPublisher(self.views, camera_topics, self.control)
            rospy.on_shutdown(self.grid_timing.close)
        self.control.state.record('startup', model_load_ms=model_load_ms,
                                  dummy_warmup_ms=dummy_warmup_ms)
        self.family_status_subscriber = rospy.Subscriber(
            "/perception/lane/boundary_family", LaneBoundaryFamily, self.control.pipeline_observation, queue_size=1)
        self.subscribers = []
        for view in self.views:
            subscriber = (self.input_source.subscribe(str(camera_topics[view]), self._callback, callback_args=view)
                          if self.input_source else rospy.Subscriber(str(camera_topics[view]), CompressedImage,
                          self._callback, callback_args=view, queue_size=1,
                          buff_size=int(self.parameters.get("~subscriber_buffer_bytes", 16 << 20))))
            self.subscribers.append(subscriber)
        self.subscribers.append(
            rospy.Subscriber(
                "/morai_sensor_bridge/diagnostics",
                DiagnosticArray,
                self._bridge_diagnostics_callback,
                queue_size=1,
            )
        )
        self._stop = threading.Event()
        self.worker = threading.Thread(target=self._worker, name="semantic-inference")
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "camera semantic backend: %s",
            json.dumps(backend.info.__dict__, sort_keys=True),
        )
        rospy.loginfo(
            "camera semantic views=%s inputs=%s outputs=%s confidences=%s "
            "model_grid_road_marking=%s model_grid_max_hz=%.2f",
            self.views,
            camera_topics,
            self.output_topics,
            self.confidence_topics,
            bool(self.model_grid_publishers),
            self.model_grid_max_hz,
        )

    @staticmethod
    def _output_topic(view: str, head: str, categorical: bool) -> str:
        suffix = "class_id" if categorical else "probability"
        return "/perception/camera/{}/{}/{}".format(view, head, suffix)

    def _callback(self, message: CompressedImage, view: str) -> None:
        generation = self.control.observe(view, message.header.stamp.to_nsec())
        self.buffer.push(
            EncodedFrame(
                view=view,
                stamp=message.header.stamp,
                frame_id=message.header.frame_id,
                data=bytes(message.data),
                received_monotonic=getattr(message, 'received_monotonic', time.monotonic()),
                received_ros_ns=getattr(message, 'received_ros_ns', int(rospy.Time.now().to_nsec())),
                decode_key=getattr(message, 'decode_key', None),
                decode_source=getattr(message, 'decode_source', None), generation=generation,
                source_sequence=int(getattr(message.header, 'seq', 0)),
            )
        )

    def _bridge_diagnostics_callback(self, message: DiagnosticArray) -> None:
        stream_to_view = {"cam1": "front", "cam2": "left", "cam3": "right"}
        for status in message.status:
            stream = status.name.rsplit("/", 1)[-1]
            view = stream_to_view.get(stream)
            if view not in self._timestamp_fallbacks_by_view:
                continue
            values = dict((item.key, item.value) for item in status.values)
            try:
                self._timestamp_fallbacks_by_view[view] = int(
                    values.get("timestamp_fallbacks", "0")
                )
                self._timestamp_regressions_by_view[view] = int(
                    values.get("timestamp_regressions", "0")
                )
            except ValueError:
                continue

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

    @staticmethod
    def _publish_unless_shutdown(publisher, message) -> bool:
        """Publish without reporting a false inference failure during shutdown."""
        if rospy.is_shutdown():
            return False
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return False
            raise
        return True

    def _publish_connected_image(self, publisher, value, frame, categorical=False):
        # Keep the topic advertised, but avoid full-resolution conversion and
        # serialization when nobody consumes it. A new subscriber gets the
        # next fresh observation; no cached output is replayed.
        if publisher.get_num_connections() == 0:
            return True
        return self._publish_unless_shutdown(
            publisher, self._image_message(value, frame, categorical=categorical))

    def _worker(self) -> None:
        pending = CoherentFrames(self.views, int(float(
            self.parameters.get("~managed_max_skew_ms", 35.)) * 1e6))
        while not self._stop.is_set() and not rospy.is_shutdown():
            frames = self.buffer.pop_batch(timeout=0.2)
            if not frames:
                continue
            generation = self.control.state.generation
            frames = [f for f in frames if f.generation is None or f.generation == generation]
            if not frames:
                continue
            if self.control.state.managed:
                if self.control.state.mode == 0:
                    pending.clear()
                    continue
                frames = pending.offer(frames, generation)
                if frames is None:
                    continue
            stamps = {frame.view: frame.stamp.to_nsec() for frame in frames}
            token = self.control.state.begin(stamps, expected_generation=generation)
            if token is None:
                continue
            pending.clear()
            from contextlib import ExitStack
            leases = ExitStack()
            decoded = {}
            local_gpu_cache = getattr(self, 'local_gpu_cache', None)
            gpu_inputs = {} if local_gpu_cache or (self.input_source and self.input_source.gpu_enabled) else None
            usable_frames = {}
            started = time.perf_counter()
            for frame in frames:
                try:
                    image = (leases.enter_context(frame.decode_source.lease(frame)) if frame.decode_source
                             else cv2.imdecode(np.frombuffer(frame.data, dtype=np.uint8), cv2.IMREAD_COLOR))
                except Exception:
                    image = None
                if image is None:
                    self.decode_errors += 1
                    rospy.logwarn_throttle(2.0, "failed to decode camera JPEG")
                    continue
                decoded[frame.view] = image
                usable_frames[frame.view] = frame
                if gpu_inputs is not None:
                    try:
                        if local_gpu_cache:
                            gpu_inputs[frame.view] = leases.enter_context(local_gpu_cache.lease(
                                frame.view, (generation, frame.stamp.to_nsec()), image))
                        else:
                            gpu_inputs[frame.view] = leases.enter_context(self.input_source.gpu_lease(frame, image))
                    except Exception as exc:
                        if self.fault_handler:
                            self.fault_handler(exc)
                        self.control.state.complete(token, stamps, False, 'gpu_input_error')
                        rospy.logerr_throttle(2., 'shared GPU input failed: %s', exc)
                        decoded.clear()
                        break
            decode_ms = (time.perf_counter() - started) * 1000.0
            if not decoded or (self.control.state.managed and len(decoded) != len(frames)):
                self.control.state.complete(token, stamps, False, "decode_error")
                leases.close()
                continue
            try:
                # Snapshot the lazy-publish decision for the complete batch.
                # Subscriber connections may change while GPU inference is in
                # progress; re-checking afterward could request arrays that
                # were intentionally not copied into this prediction.
                preserve_model_grid = self._model_grid_has_subscribers()
                native_outputs = self._native_output_requests() if self.lazy_outputs else None
                prediction = self.engine.infer(
                    decoded,
                    preserve_model_grid=preserve_model_grid,
                    gpu_inputs=gpu_inputs,
                    native_outputs=native_outputs,
                )
                self.latest_backend_stages = prediction.backend_stages
                if self._stop.is_set():
                    return
            except Exception as exc:
                if self.fault_handler:
                    self.fault_handler(exc)
                with self._stats_lock:
                    self.inference_errors += 1
                self.control.state.complete(token, stamps, False, "inference_error")
                rospy.logerr_throttle(2.0, "semantic inference failed: %s", exc)
                continue
            finally:
                leases.close()

            with self.control.state.output_guard(token) as allowed:
                if not allowed:
                    continue
                self.control.state.complete(token, {v: f.stamp.to_nsec() for v, f in usable_frames.items()})
                started = time.perf_counter()
                publish_model_grid = preserve_model_grid and self._should_publish_model_grid(
                    prediction.views, usable_frames
                )
                if not self._publish_predictions(prediction, usable_frames, publish_model_grid):
                    return
                publish_ms = (time.perf_counter() - started) * 1000.0
                self.control.state.record('published', stamps=stamps, decode_ms=decode_ms,
                    preprocess_ms=prediction.preprocess_ms, inference_ms=prediction.inference_ms,
                    postprocess_ms=prediction.postprocess_ms, publish_ms=publish_ms)
                completed_ros_ns = int(rospy.Time.now().to_nsec())
                completed_monotonic = time.monotonic()
                with self._stats_lock:
                    self.output_batches += 1
                    self.total_output_batches += 1
                    self.output_frames += len(prediction.views)
                    for view in prediction.views:
                        self.output_by_view[view] += 1
                    self._latest_result_stamp_ns = completed_ros_ns
                    for view, frame in usable_frames.items():
                        stamp_ns = int(frame.stamp.to_nsec())
                        self._latest_input_stamp_ns[view] = stamp_ns
                        self._latest_receive_stamp_ns[view] = frame.received_ros_ns
                        self._queue_age_samples.append(
                            max(0.0, (completed_monotonic - frame.received_monotonic) * 1000.0)
                        )
                        end_to_end_ms = (completed_ros_ns - stamp_ns) / 1000000.0
                        if 0.0 <= end_to_end_ms <= 60000.0:
                            self._end_to_end_samples.append(end_to_end_ms)
                    for name, value in (
                        ("decode_ms", decode_ms),
                        ("preprocess_ms", prediction.preprocess_ms),
                        ("inference_ms", prediction.inference_ms),
                        ("postprocess_ms", prediction.postprocess_ms),
                        ("publish_ms", publish_ms),
                    ):
                        self.timing_totals[name] += value
                    self._maybe_log()

    def _publish_predictions(self, prediction, usable_frames, publish_model_grid):
        categorical_outputs, other_outputs, model_grid_outputs = [], [], []
        for view, value in prediction.views.items():
            frame = usable_frames[view]
            for head, output in value.probabilities.items():
                if head in self.categorical_heads:
                    categorical_outputs.append((self.publishers[view][head], output, frame, True))
                else:
                    other_outputs.append((self.publishers[view][head], output, frame, False))
            for head, confidence in value.categorical_confidences.items():
                other_outputs.append((self.confidence_publishers[view][head], confidence, frame, False))
            if publish_model_grid and view in self.model_grid_publishers:
                model_grid_outputs.extend([
                    (self.model_grid_publishers[view],
                     value.model_grid_probabilities['road_marking'], frame, True),
                    (self.model_grid_confidence_publishers[view],
                     value.model_grid_categorical_confidences['road_marking'], frame, False)])
        # Lane fitting consumes this compact three-view grid. Deliver the whole
        # set before optional full-resolution debug probabilities serialize.
        grid_timing = getattr(self, 'grid_timing', None)
        if grid_timing is not None:
            # A tiny sidecar joins actual serialized output sequences back to
            # the original compressed input. Compact transport stays ahead of
            # expensive native visualization outputs.
            from std_msgs.msg import Header
            for view in prediction.views:
                frame = usable_frames[view]
                if not publish_model_grid or view not in self.model_grid_publishers:
                    continue
                value = prediction.views[view]
                headers = []
                for publisher, pixels, categorical in (
                    (self.model_grid_publishers[view], value.model_grid_probabilities['road_marking'], True),
                    (self.model_grid_confidence_publishers[view], value.model_grid_categorical_confidences['road_marking'], False)):
                    if publisher.get_num_connections() > 0:
                        message = self._image_message(pixels, frame, categorical)
                        headers.append(grid_timing.publish_image(publisher, message))
                    else:
                        headers.append((Header(stamp=frame.stamp, frame_id=frame.frame_id), False))
                grid_timing.publish(view, frame, headers)
            model_grid_outputs = []
        for publisher, value, frame, categorical in model_grid_outputs + categorical_outputs + other_outputs:
            if not self._publish_connected_image(publisher, value, frame, categorical):
                return False
        return True

    def _native_output_requests(self):
        requests = {}
        for view in self.views:
            requests[view] = {head for head, publisher in self.publishers[view].items()
                              if publisher.get_num_connections() > 0}
            requests[view].update(head + '_confidence'
                for head, publisher in self.confidence_publishers[view].items()
                if publisher.get_num_connections() > 0)
        return requests

    def _should_publish_model_grid(self, predictions, usable_frames):
        if not self.model_grid_publishers:
            return False
        if set(predictions) != set(self.views):
            self.model_grid_incomplete_batches += 1
            return False
        if not any(
            self.model_grid_publishers[view].get_num_connections() > 0
            or self.model_grid_confidence_publishers[view].get_num_connections() > 0
            for view in self.views
        ):
            return False
        stamp_ns = max(int(usable_frames[view].stamp.to_nsec()) for view in self.views)
        if stamp_ns <= 0:
            self.model_grid_incomplete_batches += 1
            return False
        control = getattr(self, "control", None)
        if control is not None and control.state.managed:
            publish, following, reset = downstream_gate(
                stamp_ns, self.last_model_grid_stamp_ns,
                self.next_model_grid_stamp_ns, self.model_grid_period_ns,
                control.state.target_hz)
            self.last_model_grid_stamp_ns = stamp_ns
            self.model_grid_timestamp_resets += int(reset)
            if not publish:
                self.model_grid_throttled_batches += 1
                return False
            self.next_model_grid_stamp_ns = following
            self.model_grid_output_batches += 1
            return True
        if (
            self.last_model_grid_stamp_ns
            and stamp_ns + self.model_grid_period_ns <= self.last_model_grid_stamp_ns
        ):
            # Sequential bags and simulator reloads may start a new source-time
            # epoch. Preserve the header but reset the rate deadline.
            self.next_model_grid_stamp_ns = 0
            self.model_grid_timestamp_resets += 1
        elif self.last_model_grid_stamp_ns and stamp_ns <= self.last_model_grid_stamp_ns:
            self.model_grid_throttled_batches += 1
            return False
        self.last_model_grid_stamp_ns = stamp_ns
        if self.next_model_grid_stamp_ns and stamp_ns < self.next_model_grid_stamp_ns:
            self.model_grid_throttled_batches += 1
            return False
        self.next_model_grid_stamp_ns = stamp_ns + self.model_grid_period_ns
        self.model_grid_output_batches += 1
        return True

    def _model_grid_has_subscribers(self):
        return bool(self.model_grid_publishers) and any(
            self.model_grid_publishers[view].get_num_connections() > 0
            or self.model_grid_confidence_publishers[view].get_num_connections() > 0
            for view in self.views
        )

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
        decode_delta = self.decode_errors - self._last_decode_errors
        inference_delta = self.inference_errors - self._last_inference_errors
        fps = frame_delta / elapsed
        rospy.loginfo(
            "semantic fps=%.2f output=%d batches=%d replaced=%s decode_errors=%d inference_errors=%d avg_ms=%s",
            fps,
            self.output_frames,
            self.output_batches,
            self.buffer.replaced,
            self.decode_errors,
            self.inference_errors,
            averages,
        )
        self._publish_diagnostics(
            fps, averages, decode_delta, inference_delta
        )
        self._last_log_wall = now
        self._last_output_frames = self.output_frames
        self._last_decode_errors = self.decode_errors
        self._last_inference_errors = self.inference_errors
        self.output_batches = 0
        for name in self.timing_totals:
            self.timing_totals[name] = 0.0
        self._queue_age_samples = []
        self._end_to_end_samples = []

    @staticmethod
    def _percentile(samples, value):
        if not samples:
            return None
        return float(np.percentile(np.asarray(samples, dtype=np.float64), value))

    @staticmethod
    def _diagnostic_value(key, value):
        if isinstance(value, (dict, list, tuple)):
            rendered = json.dumps(value, sort_keys=True)
        elif value is None:
            rendered = "unavailable"
        else:
            rendered = str(value)
        return KeyValue(key=str(key), value=rendered)

    def _publish_diagnostics(self, fps, averages, decode_delta, inference_delta):
        status = DiagnosticStatus()
        status.name = "camera_semantic_perception/inference"
        status.hardware_id = str(self.backend_info.get("device", "unknown"))
        if inference_delta:
            status.level = DiagnosticStatus.ERROR
            status.message = "inference errors"
        elif decode_delta:
            status.level = DiagnosticStatus.WARN
            status.message = "decode errors"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "running"

        values = {
            "backend": self.backend_info.get("name"),
            "model_config": self.backend_info.get("model_config"),
            "checkpoint_path": self.backend_info.get("checkpoint_path"),
            "checkpoint_sha256": self.backend_info.get("checkpoint_sha256"),
            "weights": self.backend_info.get("weights"),
            "views": self.views,
            "input_hw": self.backend_info.get("input_hw"),
            "input_hw_by_view": self.backend_info.get("input_hw_by_view"),
            "channels_last": self.backend_info.get("channels_last"),
            "received_by_view": self.buffer.received,
            "replaced_by_view": self.buffer.replaced,
            "output_frames": self.output_frames,
            "output_by_view": self.output_by_view,
            "output_batches": self.total_output_batches,
            "model_grid_output_batches": self.model_grid_output_batches,
            "model_grid_throttled_batches": self.model_grid_throttled_batches,
            "model_grid_incomplete_batches": self.model_grid_incomplete_batches,
            "model_grid_timestamp_resets": self.model_grid_timestamp_resets,
            "decode_errors": self.decode_errors,
            "inference_errors": self.inference_errors,
            "processing_fps": round(fps, 3),
            "average_stage_ms": averages,
            "latest_backend_stage_ms": self.latest_backend_stages,
            "queue_age_p50_ms": self._percentile(self._queue_age_samples, 50),
            "queue_age_p95_ms": self._percentile(self._queue_age_samples, 95),
            "end_to_end_p50_ms": self._percentile(self._end_to_end_samples, 50),
            "end_to_end_p95_ms": self._percentile(self._end_to_end_samples, 95),
            "latest_input_stamp_ns": self._latest_input_stamp_ns,
            "latest_receive_stamp_ns": self._latest_receive_stamp_ns,
            "latest_result_stamp_ns": self._latest_result_stamp_ns,
            "input_timestamp_fallbacks_by_view": self._timestamp_fallbacks_by_view,
            "input_timestamp_regressions_by_view": self._timestamp_regressions_by_view,
        }
        status.values = [
            self._diagnostic_value(key, value) for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def _shutdown(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self.buffer.close()
        if self.worker is not threading.current_thread():
            self.worker.join(timeout=2.)
        cache = getattr(self, 'local_gpu_cache', None)
        if cache is not None and not self.worker.is_alive():
            cache.close()
        self.control.close()


def main() -> None:
    rospy.init_node("camera_semantic_inference")
    SemanticInferenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
