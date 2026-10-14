"""Experimental one-process runtime with independent, explicitly configured workers."""
import threading
import rospy
from sensor_msgs.msg import CompressedImage
from .parameters import Parameters, fatal_cuda_error
from .shared_input import SharedInput


class SharedRuntime:
    def __init__(self):
        self.input = SharedInput(rospy, CompressedImage)
        self.components, self.controls, self.threads = [], [], []
        self.fatal = ''
        self.closed = False
        self.lock = threading.RLock()
        self.models = tuple(rospy.get_param('~models'))
        initial = int(rospy.get_param('~initial_mode', 0))
        for role in self.models:
            rospy.set_param('/camera_runtime/components/' + role + '/initial_mode', 0)
            rospy.set_param('/camera_runtime/components/' + role + '/managed', True)
        from asmc_camera.nodes.object_detection_node import ObjectDetectionNode
        from asmc_camera.nodes.semantic_inference_node import SemanticInferenceNode
        for role, factory in [('objects', ObjectDetectionNode), ('lane', SemanticInferenceNode)]:
            if role in self.models:
                component = factory(parameters=Parameters('/camera_runtime/components/' + role),
                                    input_source=self.input, fault_handler=self.fault)
                self.components.append(component)
                self.controls.append(component.control)
        if 'traffic_light' in self.models:
            ready = threading.Event()
            def on_ready(control):
                self.controls.append(control)
                ready.set()
            def traffic():
                from asmc_camera.nodes.traffic_light_node import main
                params = Parameters('/camera_runtime/components/traffic_light')
                argv = ['traffic_light', '--ckpt', params.get('~checkpoint'), '--wait-first', '0']
                for param, arg, default in [('image_topic','image-topic','/cam_extra/image_jpeg/compressed'),
                                            ('target_hz','hz',10),('device','device','cuda'),
                                            ('min_detect','min-detect',.4),('stale_sec','stale-sec',1)]:
                    argv.extend(['--' + arg, str(params.get('~' + param, default))])
                try:
                    result = main(argv, parameters=params, input_source=self.input,
                                  init_node=False, on_ready=on_ready, fault_handler=self.fault)
                    if result:
                        self.fault('traffic-light worker exited', force=True)
                except Exception as exc:
                    self.fault(exc, force=True)
                finally:
                    ready.set()
            worker = threading.Thread(target=traffic, name='traffic-light', daemon=True)
            worker.start()
            self.threads.append(worker)
            if not ready.wait(120) or self.fatal:
                raise RuntimeError(self.fatal or 'traffic-light initialization timeout')
        if initial and not self.fatal:
            for control in self.controls:
                request = type('Request', (), dict(mode=initial, target_hz=0.))()
                control._request(request)
        rospy.on_shutdown(self.close)

    def fault(self, error, force=False):
        if not force and not fatal_cuda_error(error):
            return
        with self.lock:
            self.fatal = str(error)
            for control in self.controls:
                control.fault(self.fatal)
            rospy.logerr('shared camera runtime requires restart: %s', self.fatal)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
        for component in self.components:
            (component.shutdown if hasattr(component, 'shutdown') else component._shutdown)()
        self.input.close()
        import json
        cache = self.input.cache
        with cache.lock:
            values = dict(decodes=cache.decodes, hits=cache.hits, busy=cache.busy, errors=cache.errors,
                          slots={camera: len(rows) for camera, rows in cache.rows.items()})
        rospy.loginfo("shared input summary: %s", json.dumps(values, sort_keys=True))


def main():
    rospy.init_node('camera_runtime')
    runtime = None
    try:
        runtime = SharedRuntime()
        rospy.spin()
    finally:
        if runtime:
            runtime.close()
