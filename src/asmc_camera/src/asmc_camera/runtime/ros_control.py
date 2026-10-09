"""ROS adapter for the per-model execution API, without route policy."""
import threading
import rospy
from asmc_camera.msg import InferenceStatus
from asmc_camera.srv import SetInferenceMode, SetInferenceModeResponse
from .state import InferenceState, STANDBY, ACTIVE
from .trace import LifecycleTrace


class RosInferenceControl:
    def __init__(self, model, views, default_hz, warmed_up=True, on_transition=None):
        self.model = model
        self.trace = LifecycleTrace(rospy.get_param('~lifecycle_trace_path', ''))
        self.state = InferenceState(
            views, managed=bool(rospy.get_param('~managed', False)),
            initial_mode=int(rospy.get_param('~initial_mode', STANDBY)),
            target_hz=float(rospy.get_param('~target_hz', default_hz)), warmed_up=warmed_up,
            trace=self.trace if self.trace.path is not None else None,
            target_hz_by_view=rospy.get_param('~target_hz_by_view', {}))
        self.state.record('control_ready', loaded=True, warmed_up=bool(warmed_up))
        self.on_transition = on_transition
        self.pipeline = None
        self.publisher = rospy.Publisher('~inference_status', InferenceStatus, queue_size=1, latch=True)
        self.service = rospy.Service('~set_inference_mode', SetInferenceMode, self._request)
        self.stopped = threading.Event()
        self.publish()
        self.thread = threading.Thread(target=self._status_loop, daemon=True)
        self.thread.start()

    def _status_loop(self):
        # Wall time keeps stale readiness observable when /clock also stops.
        while not self.stopped.wait(.2) and not rospy.is_shutdown():
            try:
                self.publish()
            except rospy.ROSException:
                if not rospy.is_shutdown():
                    raise

    def _request(self, request):
        with self.state.lock:
            previous = self.state.generation
            accepted, generation, reason = self.state.request(request.mode, request.target_hz)
            if accepted and generation != previous:
                self.pipeline = None
                if self.on_transition:
                    self.on_transition(request.mode)
            self.publish()
        return SetInferenceModeResponse(accepted, generation, reason)

    def pipeline_observation(self, message):
        with self.state.lock:
            self.pipeline = (message.header.stamp.to_nsec(), bool(message.valid),
                             self.state.clock(), self.state.generation)
            self.state.record('pipeline_output', source_stamp_ns=message.header.stamp.to_nsec(),
                              valid=bool(message.valid))

    def publish(self, event=None):
        with self.state.lock:
            values = self.state.snapshot()
            source_age = (rospy.Time.now().to_nsec() - values["last_source_stamp"]) / 1e9
            if values["prepared"] and not (-.05 <= source_age <= self.state.freshness):
                values["prepared"] = False
                values["reason"] = "source_image_stale"
            msg = InferenceStatus()
            msg.header.stamp = rospy.Time.now()
            msg.model = self.model
            for key, value in values.items():
                if key in ('source_cutoff', 'last_source_stamp'):
                    value = rospy.Time(value // 1000000000, value % 1000000000)
                setattr(msg, key, value)
            if self.model == 'twinlite':
                p = self.pipeline
                msg.pipeline_ready = bool(values['prepared'] and p and p[1] and
                    p[0] > values['source_cutoff'] and p[3] == values['generation'] and
                    self.state.clock() - p[2] <= self.state.freshness and
                    -.05 <= (rospy.Time.now().to_nsec() - p[0]) / 1e9 <= self.state.freshness)
                msg.last_output_valid = bool(p and p[1])
            else:
                msg.pipeline_ready = values['prepared']
            self.publisher.publish(msg)
            self.state.record('status', prepared=msg.prepared, pipeline_ready=msg.pipeline_ready,
                              successful_batches=msg.successful_batches, reason=msg.reason)

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=1.)
        self.service.shutdown()
        self.trace.close()


class LaneEpochGate:
    """Reject queued lane inputs/results from earlier managed source epochs."""
    def __init__(self, on_standby=None):
        import threading
        self.lock = threading.RLock()
        self.enabled = bool(rospy.get_param('~managed', False))
        self.mode = STANDBY if self.enabled else ACTIVE
        self.cutoff = 0
        self.target_hz = 0
        self.on_standby = on_standby
        self.generation = None
        self.subscriber = rospy.Subscriber(
            '/camera_semantic_inference/inference_status', InferenceStatus,
            self._status, queue_size=1)

    def _status(self, status):
        with self.lock:
            changed = self.generation != status.generation or self.mode != status.mode
            self.generation = status.generation
            self.enabled = status.managed
            self.mode = status.mode
            self.cutoff = status.source_cutoff.to_nsec()
            self.target_hz = status.target_hz if status.managed else 0
            if changed and self.enabled and self.mode == STANDBY and self.on_standby:
                self.on_standby()

    def allows(self, stamp):
        with self.lock:
            return not self.enabled or (self.mode != STANDBY and stamp.to_nsec() > self.cutoff)
