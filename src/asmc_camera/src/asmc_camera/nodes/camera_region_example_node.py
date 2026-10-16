"""Optional HD-map example that calls the same API available to planning."""
import json
import threading
import time
import rospy
from asmc_hd_map.msg import HdMapContext
from asmc_msgs.msg import MissionZoneArray, VehicleState
from std_msgs.msg import String
from asmc_msgs.srv import SetInferenceMode
from asmc_camera.runtime.parameters import control_paths
from asmc_camera.runtime.region_example import RegionExample


class CameraRegionExampleNode:
    def __init__(self):
        if not rospy.get_param('~allow_example_control', False):
            raise RuntimeError('example disabled: explicitly set allow_example_control=true')
        self.config = rospy.get_param('~policy')
        self.policy = RegionExample(self.config)
        self.lock = threading.RLock()
        self.context = None
        self.zones = {}
        self.speed = 0.
        self.speed_at = 0.
        self.requested = {}
        self.pending = {}
        self.errors = {}
        self.publisher = rospy.Publisher('~requests', String, queue_size=1)
        self.context_sub = rospy.Subscriber('/hd_map/context', HdMapContext, self._context, queue_size=1)
        self.zone_sub = rospy.Subscriber('/hd_map/global/mission_zones', MissionZoneArray, self._zones, queue_size=1)
        self.speed_sub = rospy.Subscriber('/vehicle/state', VehicleState, self._speed, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.1), self._tick)

    def _context(self, value):
        with self.lock:
            self.context = value

    def _zones(self, value):
        with self.lock:
            self.zones = {z.id: (z.route_station_start_m, z.route_station_end_m)
                          for z in value.zones if z.status != z.STATUS_PROVISIONAL}

    def _speed(self, value):
        with self.lock:
            if value.valid:
                self.speed, self.speed_at = value.speed, time.monotonic()

    def _request(self, model, mode, hz):
        service = control_paths(model)[0]
        try:
            rospy.wait_for_service(service, timeout=1.)
            response = rospy.ServiceProxy(service, SetInferenceMode)(mode, hz)
            with self.lock:
                if response.accepted:
                    self.requested[model] = (mode, hz)
                    self.errors.pop(model, None)
                else:
                    self.errors[model] = response.reason
        except (rospy.ROSException, rospy.ServiceException) as exc:
            with self.lock:
                self.errors[model] = str(exc)
        finally:
            with self.lock:
                self.pending.pop(model, None)

    def _tick(self, event):
        with self.lock:
            now = time.monotonic()
            speed = self.speed if now - self.speed_at <= .5 else 0.
            modes, reason = self.policy.decide(self.context, self.zones, speed, now, rospy.Time.now().to_sec())
            for model, mode in modes.items():
                hz = float(self.config['models'][model]['target_hz'])
                if self.requested.get(model) != (mode, hz) and model not in self.pending:
                    self.pending[model] = now
                    threading.Thread(target=self._request, args=(model, mode, hz), daemon=True).start()
            pending = {model: now - start for model, start in self.pending.items()}
            self.publisher.publish(String(data=json.dumps(dict(
                development_only=True, requested_modes=modes, reason=reason,
                errors=self.errors, pending_sec=pending), sort_keys=True)))


def main():
    rospy.init_node('camera_region_example')
    CameraRegionExampleNode()
    rospy.spin()
