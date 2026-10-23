#!/usr/bin/env python3
"""Display published LiDAR observations; never run detection, tracking or fusion.

The public eight OBB corners already include the frame/mount/attitude transform.
Do not reconstruct a yaw-only box, flatten z, or apply IMU compensation again.
This RViz adapter is separate from Jaeho's future localhost web viewer.
"""
import math
import threading
import time

import rospy
from asmc_msgs.msg import LidarObjectArray, LidarObservation
from visualization_msgs.msg import Marker, MarkerArray


EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
         (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))
COLORS = {'vehicle': (0.2, 0.8, 1.0), 'dynamic_obstacle': (1.0, 0.45, 0.15),
          'static_obstacle': (0.3, 1.0, 0.45)}


class ObjectMarkers:
    """Bounded display state: current identities only, no point clouds/history."""
    def __init__(self, lifetime=0.35):
        self.lifetime = lifetime
        self.epoch = None
        self.sequence = None
        self.stamp = None
        self.keys = set()

    def marker(self, header, namespace, marker_id, action=Marker.ADD):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.action = action
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration.from_sec(self.lifetime)
        return marker

    def clear(self, header):
        result = MarkerArray()
        result.markers = [self.marker(header, 'objects', 0, Marker.DELETEALL)]
        self.keys.clear()
        return result

    def render(self, observation):
        objects = observation.objects
        epoch = (observation.producer_session_id, observation.generation)
        # Same producer's retired generation and duplicate results cannot put
        # old boxes back on screen. A new producer/epoch starts a new display.
        if self.epoch and epoch[0] == self.epoch[0] and epoch[1] < self.epoch[1]:
            return None
        changed = epoch != self.epoch
        if not changed and (observation.scan_sequence <= self.sequence or objects.header.stamp <= self.stamp):
            return None
        result = self.clear(objects.header) if changed else MarkerArray()
        self.epoch, self.sequence, self.stamp = epoch, observation.scan_sequence, objects.header.stamp
        keys = set()
        if objects.status == LidarObjectArray.OBSERVED:
            for obj in objects.objects:
                corners = obj.obb_corners
                if (obj.track_id < 0 or len(corners) != 8 or
                        not all(math.isfinite(v) for p in corners for v in (p.x, p.y, p.z)) or
                        not all(math.isfinite(v) for v in (obj.center.x, obj.center.y)) or
                        ('obb', obj.track_id) in keys):
                    continue
                box = self.marker(objects.header, 'obb', obj.track_id)
                box.type = Marker.LINE_LIST
                box.scale.x = 0.045
                box.color.r, box.color.g, box.color.b = COLORS.get(obj.classification, (1.0, 0.85, 0.2))
                box.color.a = 1.0
                box.points = [corners[index] for edge in EDGES for index in edge]
                label = self.marker(objects.header, 'label', obj.track_id)
                label.type = Marker.TEXT_VIEW_FACING
                label.scale.z = 0.28
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.pose.position.x, label.pose.position.y = obj.center.x, obj.center.y
                label.pose.position.z = max(point.z for point in corners) + 0.25
                label.text = '#{} {} / {}  n={}'.format(
                    obj.track_id, obj.classification or 'unknown', obj.camera_class or 'unknown', obj.source_point_count)
                result.markers.extend((box, label))
                keys.update((('obb', obj.track_id), ('label', obj.track_id)))
        # OBSERVED_EMPTY and invalid/pose-unavailable results remove old boxes
        # immediately. Markers also expire if input stops, including paused ROS time.
        result.markers.extend(self.marker(objects.header, namespace, marker_id, Marker.DELETE)
                              for namespace, marker_id in sorted(self.keys - keys))
        self.keys = keys
        return result


class MarkerNode:
    def __init__(self):
        self.timeout = 0.35
        self.builder = ObjectMarkers(self.timeout)
        self.lock = threading.Lock()
        self.last_receipt = None
        self.header = None
        self.stop = threading.Event()
        self.publisher = rospy.Publisher(rospy.get_param('~output', '/migration/lidar/markers'),
                                         MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber(rospy.get_param('~input', '/migration/lidar/observations_local'),
                                           LidarObservation, self.receive, queue_size=1, tcp_nodelay=True)
        self.worker = threading.Thread(target=self.expire, name='lidar_marker_expiry', daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

    def receive(self, observation):
        with self.lock:
            markers = self.builder.render(observation)
            if markers is None:
                return
            self.header = observation.objects.header
            self.last_receipt = time.monotonic()
            self.publisher.publish(markers)

    def expire(self):
        while not self.stop.wait(0.05):
            with self.lock:
                if (self.last_receipt is not None and
                        time.monotonic() - self.last_receipt > self.timeout):
                    self.publisher.publish(self.builder.clear(self.header))
                    self.last_receipt = None

    def shutdown(self):
        self.stop.set()
        self.worker.join(timeout=1.0)


if __name__ == '__main__':
    rospy.init_node('lidar_markers')
    node = MarkerNode()
    rospy.spin()
