"""ROS adapter for the per-model execution API, without route policy."""
import threading
import uuid
import time
import json
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from asmc_msgs.msg import InferenceStatus
from asmc_camera.msg import InferenceStatus as LegacyInferenceStatus
from asmc_camera.srv import SetInferenceMode as LegacySetInferenceMode, SetInferenceModeResponse as LegacyModeResponse
from asmc_msgs.srv import SetInferenceMode, SetInferenceModeResponse
from .state import InferenceState, STANDBY, ACTIVE
from .trace import LifecycleTrace
from .parameters import Parameters, control_paths


class RosInferenceControl:
    def __init__(self, model, views, default_hz, warmed_up=True, on_transition=None, parameters=None):
        self.model = model
        self.parameters = parameters or Parameters()
        get = self.parameters.get
        self.session_id = str(uuid.uuid4())
        self.fatal_reason = ''
        self.trace = LifecycleTrace(get('~lifecycle_trace_path', ''))
        self.state = InferenceState(
            views, managed=bool(get('~managed', False)),
            initial_mode=int(get('~initial_mode', STANDBY)),
            target_hz=float(get('~target_hz', default_hz)), warmed_up=warmed_up,
            trace=self.trace if self.trace.path is not None else None,
            target_hz_by_view=get('~target_hz_by_view', {}))
        self.state.record('control_ready', loaded=True, warmed_up=bool(warmed_up))
        self.on_transition = on_transition
        self.pipeline = None
        self.latest_receipts = {view:None for view in views}
        self.readiness_publisher = (rospy.Publisher('/perception/camera/diagnostics', DiagnosticArray, queue_size=1)
                                    if model == 'twinlite' else None)
        service, status, legacy = control_paths(model)
        self.publisher = rospy.Publisher(status, InferenceStatus, queue_size=1, latch=True)
        self.legacy_publisher = rospy.Publisher(legacy + '/inference_status', LegacyInferenceStatus, queue_size=1, latch=True)
        self.service = rospy.Service(service, SetInferenceMode, self._request)
        self.legacy_service = rospy.Service(legacy + '/set_inference_mode', LegacySetInferenceMode, self._legacy_request)
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
            if self.fatal_reason:
                return SetInferenceModeResponse(False, self.state.generation, self.fatal_reason)
            previous = self.state.generation
            accepted, generation, reason = self.state.request(request.mode, request.target_hz)
            if accepted and generation != previous:
                self.pipeline = None
                if self.on_transition:
                    self.on_transition(request.mode)
            self.publish()
        return SetInferenceModeResponse(accepted, generation, reason)

    def _legacy_request(self, request):
        response = self._request(request)
        return LegacyModeResponse(response.accepted, response.generation, response.reason)

    def observe(self, view, stamp):
        with self.state.lock:
            self.latest_receipts[view]=self.state.clock()
            previous = self.state.generation
            self.state.observe(view, stamp)
            if self.state.generation != previous:
                self.pipeline = None
                if self.on_transition:
                    self.on_transition(STANDBY)  # invalidate, keep requested mode
                self.publish()
            return self.state.generation

    def fault(self, reason):
        with self.state.lock:
            self.fatal_reason = 'runtime_restart_required: ' + str(reason)
            self.state.request(STANDBY)
            self.state.warmed_up = False
            self.state.errors += 1
            if self.on_transition:
                self.on_transition(STANDBY)
            self.publish()

    def pipeline_observation(self, message):
        generation = self.state.generation
        stamp = message.header.stamp.to_nsec()
        waiting = time.monotonic()
        with self.state.lock:
            lock_wait_ms = (time.monotonic()-waiting)*1000
            # Family has no generation field. Only accept stamps completed by
            # this producer generation; a delayed/restarted producer cannot
            # establish readiness using an unrecognized source image.
            if (generation != self.state.generation or self.state.mode == STANDBY or
                    stamp <= self.state.cutoff or stamp not in self.state.completed_sources):
                return
            self.pipeline = (stamp, bool(message.valid),
                             self.state.clock(), self.state.generation,
                             str(getattr(message,'status_reason','')))
            self.state.record('pipeline_output', source_stamp_ns=message.header.stamp.to_nsec(),
                              valid=bool(message.valid),lock_wait_ms=lock_wait_ms)

    def publish(self, event=None):
        with self.state.lock:
            values = self.state.snapshot()
            now = rospy.Time.now()
            source_age = (now.to_nsec() - values["last_source_stamp"]) / 1e9
            if values["prepared"] and not (-.05 <= source_age <= self.state.freshness):
                values["prepared"] = False
                values["reason"] = "source_image_stale"
            msg = InferenceStatus()
            msg.producer_session_id = self.session_id
            msg.header.stamp = now
            msg.model = self.model
            for key, value in values.items():
                if key in ('source_cutoff', 'last_source_stamp'):
                    value = rospy.Time(value // 1000000000, value % 1000000000)
                setattr(msg, key, value)
            if self.fatal_reason:
                msg.reason = self.fatal_reason
            if self.model == 'twinlite':
                p = self.pipeline
                detail = pipeline_readiness(values, p, now.to_nsec(), self.state.clock(), self.state.freshness)
                detail['latest_input_source_age_ms_by_view'] = json.dumps({
                    view:(now.to_nsec()-stamp)/1e6 if stamp else None
                    for view,stamp in self.state.latest_input.items()},sort_keys=True)
                detail['latest_input_receipt_age_ms_by_view'] = json.dumps({
                    view:1000*(self.state.clock()-received) if received is not None else None
                    for view,received in self.latest_receipts.items()},sort_keys=True)
                msg.pipeline_ready = detail['pipeline_ready']
                msg.last_output_valid = bool(p and p[1])
                diagnostic = DiagnosticStatus(name='camera_semantic_perception/readiness',
                    level=DiagnosticStatus.WARN if not msg.pipeline_ready and values['mode']==ACTIVE else DiagnosticStatus.OK,
                    message=detail['pipeline_ready_reason'],
                    values=[KeyValue(key=k, value=str(v)) for k,v in detail.items()])
                self.readiness_publisher.publish(DiagnosticArray(header=msg.header, status=[diagnostic]))
            else:
                msg.pipeline_ready = values['prepared']
            self.publisher.publish(msg)
            legacy = LegacyInferenceStatus()
            for field in legacy.__slots__:
                setattr(legacy, field, getattr(msg, field))
            self.legacy_publisher.publish(legacy)
            self.state.record('status', prepared=msg.prepared, pipeline_ready=msg.pipeline_ready,
                              successful_batches=msg.successful_batches, reason=msg.reason)

    def close(self):
        if self.stopped.is_set():
            return
        self.stopped.set()
        self.thread.join(timeout=1.)
        self.service.shutdown()
        self.legacy_service.shutdown()
        self.trace.close()


def pipeline_readiness(values, pipeline, now_ns, wall_now, freshness):
    """Explain the existing readiness predicates from the same status snapshot."""
    p = pipeline
    source_age = (now_ns-p[0])/1e9 if p else None
    receipt_age = wall_now-p[2] if p else None
    checks = [('model_not_prepared', bool(values['prepared'])),
              ('missing_family', p is not None), ('invalid_family', bool(p and p[1])),
              ('before_cutoff', bool(p and p[0] > values['source_cutoff'])),
              ('wrong_generation', bool(p and p[3] == values['generation'])),
              ('receipt_stale', bool(p and receipt_age <= freshness)),
              ('source_stale', bool(p and -.05 <= source_age <= freshness))]
    reason = next((name for name, passed in checks if not passed), 'ready')
    return dict(pipeline_ready=reason=='ready', pipeline_ready_reason=reason,
                input_prepared=bool(values['prepared']), generation=values['generation'],
                source_cutoff_ns=values['source_cutoff'], family_valid=bool(p and p[1]),
                family_source_ns=p[0] if p else None, family_generation=p[3] if p else None,
                family_status_reason=p[4] if p and len(p)>4 else None,
                family_source_age_ms=source_age*1000 if p else None,
                family_receipt_age_ms=receipt_age*1000 if p else None,
                input_source_age_ms=(now_ns-values['last_source_stamp'])/1e6,
                model_result_source_age_ms=(now_ns-values['last_source_stamp'])/1e6,
                freshness_ms=freshness*1000)


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
        self.epoch = 0
        self.successful_batches = None
        self.producer_session_id = None
        self._epoch_context = (self.enabled, self.generation, self.cutoff, self.epoch)
        self.subscriber = rospy.Subscriber(
            '/perception/camera/lane/status', InferenceStatus,
            self._status, queue_size=1)

    def _status(self, status):
        with self.lock:
            session = status.producer_session_id
            restarted = (self.producer_session_id is not None and session != self.producer_session_id) or (self.successful_batches is not None and
                         status.successful_batches < self.successful_batches)
            changed = (self.generation != status.generation or self.mode != status.mode or
                       self.cutoff != status.source_cutoff.to_nsec() or restarted)
            self.epoch += int(changed)
            self.producer_session_id = session
            self.successful_batches = status.successful_batches
            self.generation = status.generation
            self.enabled = status.managed
            self.mode = status.mode
            self.cutoff = status.source_cutoff.to_nsec()
            self.target_hz = status.target_hz if status.managed else 0
            self._epoch_context = (self.enabled, self.generation, self.cutoff, self.epoch)
            if changed and self.enabled and self.mode == STANDBY and self.on_standby:
                self.on_standby()

    def allows(self, stamp):
        with self.lock:
            return not self.enabled or (self.mode != STANDBY and stamp.to_nsec() > self.cutoff)

    def context(self):
        # Atomic immutable snapshot; subscription workers must not acquire the
        # gate lock while holding a transport/group lock.
        return self._epoch_context
