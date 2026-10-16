"""Subscriber-lazy three-view object rendering with one final JPEG encode."""
import ast
from collections import Counter
import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from asmc_msgs.msg import InferenceStatus
from asmc_msgs.msg import CameraObjectObservation
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage

from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.visualization import render_object_overlay, render_object_mosaic
from asmc_camera.detection.view_cache import ObjectViewCache, VIEWS


class ObjectViewNode:
    TOPICS = dict(front='/image_jpeg/compressed', left='/cam_left/image_jpeg/compressed',
                  right='/cam_right/image_jpeg/compressed')
    OUTPUTS = dict(front='/perception/camera/objects/debug/image/compressed',
                   left='/perception/camera/objects/debug/left/image/compressed',
                   right='/perception/camera/objects/debug/right/image/compressed')

    def __init__(self):
        self.hz = float(rospy.get_param('~publish_hz', 10.))
        self.quality = int(rospy.get_param('~jpeg_quality', 80))
        self.width, self.height = int(rospy.get_param('~panel_width', 640)), int(rospy.get_param('~panel_height', 480))
        self.ttl = float(rospy.get_param('~stale_sec', .5))
        if not math.isfinite(self.hz) or self.hz <= 0 or not math.isfinite(self.ttl) or self.ttl <= 0 or not 1 <= self.quality <= 100 or min(self.width,self.height) <= 0:
            raise ValueError('invalid object view configuration')
        configs = rospy.get_param('~ego_mask_configs')
        enabled = rospy.get_param('~ego_mask_enabled', dict.fromkeys(VIEWS, True))
        self.masks = {v:HoodMask.load(enabled[v], configs[v]) for v in VIEWS}
        self.profiles = {}
        self.lock, self.stop = threading.RLock(), threading.Event()
        self.cache, self.rendered = ObjectViewCache(8, self.ttl), {}
        self.failed_keys, self.last_headers = {}, {}
        self.counts = Counter()
        self.render_by_view, self.last_fresh_views = Counter(), []
        self.last_mosaic_key = None
        self.last_individual_keys = {}
        self.publishers = {v:rospy.Publisher(t, CompressedImage, queue_size=1) for v,t in self.OUTPUTS.items()}
        self.mosaic = rospy.Publisher('/perception/camera/objects/debug/mosaic/compressed', CompressedImage, queue_size=1)
        self.diagnostics = rospy.Publisher('/perception/camera/objects/diagnostics', DiagnosticArray, queue_size=1)
        self.images = {}
        self.observations = rospy.Subscriber('/perception/camera/objects/observations', CameraObjectObservation, self._observation, queue_size=5)
        self.status = rospy.Subscriber('/perception/camera/objects/status', InferenceStatus, self._status, queue_size=1)
        self.producer_diagnostics = rospy.Subscriber('/perception/camera/objects/diagnostics', DiagnosticArray, self._profiles, queue_size=1)
        self.worker = threading.Thread(target=self._worker, name='object-view', daemon=True)
        rospy.on_shutdown(self.close)
        self.worker.start()

    def _observation(self, message):
        with self.lock:
            self.cache.observe(message, time.monotonic())

    def _status(self, message):
        with self.lock:
            self.cache.control(message)

    def _image(self, message, view):
        with self.lock:
            self.cache.image(view, message, time.monotonic())

    def _profiles(self, message):
        for status in message.status:
            if status.name != 'camera_objects/inference':
                continue
            values = {v.key:v.value for v in status.values}
            try:
                profiles = ast.literal_eval(values['ego_mask_by_view'])
            except (ValueError, SyntaxError, KeyError, TypeError):
                profiles = {}
            with self.lock:
                if profiles != self.profiles:
                    self.profiles = profiles
                    self.cache.epoch += 1

    def _mask_matches(self, view):
        profile = self.profiles.get(view)
        mask = self.masks[view]
        return bool(profile and bool(profile.get('enabled')) == mask.enabled and profile.get('config_sha256') == mask.config_sha256)

    def _subscriptions(self, wanted):
        for view in VIEWS:
            if view in wanted and view not in self.images:
                self.images[view] = rospy.Subscriber(self.TOPICS[view], CompressedImage, self._image,
                    callback_args=view, queue_size=1, buff_size=16 << 20)
            elif view not in wanted and view in self.images:
                self.images.pop(view).unregister()
                with self.lock:
                    self.cache.images[view].clear()
                    self.rendered.pop(view, None)
                    self.failed_keys.pop(view, None)
                    self.last_individual_keys.pop(view, None)

    def _tick(self):
        mosaic = self.mosaic.get_num_connections() > 0
        individual = {v for v,p in self.publishers.items() if p.get_num_connections() > 0}
        wanted = set(VIEWS) if mosaic else individual
        self._subscriptions(wanted)
        if not wanted:
            with self.lock:
                self.rendered.clear()
                self.cache.expire(time.monotonic())
            self.last_mosaic_key = None
            self.last_fresh_views = []
            return
        wall = time.monotonic()
        with self.lock:
            epoch, revisions = self.cache.epoch, dict(self.cache.revisions)
            for view in list(self.rendered):
                item = self.cache.observations.get(view)
                if (self.rendered[view][0][:2] != (epoch,revisions[view]) or not self.cache.active or
                        not item or not item[0].detections.valid or not self._mask_matches(view)):
                    del self.rendered[view]
            pairs = {v:self.cache.pair(v, wall) for v in wanted if self._mask_matches(v)}
        for view, pair in pairs.items():
            if pair is None:
                continue
            source, observation, received = pair
            d = observation.detections
            key = (epoch,revisions[view],d.header.stamp.to_nsec())
            if self.failed_keys.get(view) == key:
                continue
            if view in self.rendered and self.rendered[view][0] == key:
                continue
            try:
                image = cv2.imdecode(np.frombuffer(source.data,np.uint8),cv2.IMREAD_COLOR)
                self.counts['decode'] += 1
                if image is None or image.shape[:2] != (d.image_height,d.image_width):
                    self.counts['decode_or_shape_error'] += 1
                    self.failed_keys[view] = key
                    continue
                overlay = render_object_overlay(image,d,self.masks[view],
                    mask_label='hood mask' if view == 'front' else 'ego mask '+view)
                self.rendered[view] = (key, overlay, d.header, received)
                self.last_headers[view] = d.header
                self.counts['render'] += 1
                self.render_by_view[view] += 1
            except (cv2.error, ValueError) as exc:
                self.counts['render_error'] += 1
                self.failed_keys[view] = key
                rospy.logwarn_throttle(2.,'object view rejected frame: %s',exc)
        now, wall = rospy.Time.now(), time.monotonic()
        fresh = {v:item for v,item in self.rendered.items() if v in wanted and
                 -.05 <= now.to_sec()-item[2].stamp.to_sec() <= self.ttl and 0 <= wall-item[3] <= self.ttl}
        self.last_fresh_views = sorted(fresh)
        publications = []
        individual_keys = {}
        for view in individual:
            item = fresh.get(view)
            if item and self.last_individual_keys.get(view) != item[0]:
                encoded = self._encode(item[1],item[2],'individual_encode')
                if encoded is not None:
                    publications.append((self.publishers[view],encoded,'individual_publish'))
                    individual_keys[view] = item[0]
            elif not item and view in self.last_headers:
                key = ('stale',epoch,revisions[view])
                if self.last_individual_keys.get(view) != key:
                    blank = np.zeros((self.height,self.width,3),np.uint8)
                    cv2.putText(blank,view.upper()+' | STALE / WAITING',(12,30),0,.65,(0,190,255),1)
                    encoded = self._encode(blank,self.last_headers[view],'individual_encode')
                    if encoded is not None:
                        publications.append((self.publishers[view],encoded,'individual_publish'))
                        individual_keys[view] = key
        mosaic_key = (epoch,tuple((v,fresh[v][0] if v in fresh else None) for v in VIEWS))
        if mosaic and mosaic_key != self.last_mosaic_key:
            panels = {v:(item[1],item[2].stamp.to_sec(),item[3]) for v,item in fresh.items()}
            output = render_object_mosaic(panels,now.to_sec(),wall,self.width,self.height,self.ttl)
            encoded = self._encode(output,None,'mosaic_encode')
            if encoded is not None:
                encoded.header.stamp, encoded.header.frame_id = now,'camera_objects_debug_mosaic'
                publications.append((self.mosaic,encoded,'mosaic_publish'))
        with self.lock:
            # A mode/session/invalid-result transition during rendering cancels
            # the entire in-flight debug output, including encoded old boxes.
            if self.stop.is_set() or epoch != self.cache.epoch or revisions != self.cache.revisions:
                self.counts['invalidated_render'] += 1
                self.last_fresh_views = []
                return
            for publisher,message,counter in publications:
                publisher.publish(message)
                self.counts[counter] += 1
            self.last_individual_keys.update(individual_keys)
            if any(counter=='mosaic_publish' for _,_,counter in publications):
                self.last_mosaic_key = mosaic_key

    def _encode(self, image, header, counter):
        ok, encoded = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,self.quality])
        self.counts[counter] += 1
        if not ok:
            self.counts['encode_error'] += 1
            return None
        message = CompressedImage(format='jpeg',data=encoded.tobytes())
        if header is not None:
            message.header = header
        return message

    def _worker(self):
        diagnostic_at = 0.
        while not self.stop.is_set() and not rospy.is_shutdown():
            start = time.monotonic()
            try:
                self._tick()
            except Exception as exc:
                self.counts['worker_error'] += 1
                rospy.logerr_throttle(2.,'object view worker failed: %s',exc)
            if start-diagnostic_at >= 1.:
                with self.lock:
                    values = dict(self.counts, cache=dict(self.cache.counts),
                        render_by_view=dict(self.render_by_view), fresh_views=self.last_fresh_views,
                        image_cache_sizes={v:len(q) for v,q in self.cache.images.items()},
                        source_subscriptions=sorted(self.images), pending_capacity=1, image_capacity_per_view=8)
                error = self.counts['worker_error'] or self.counts['encode_error']
                warning = self.counts['decode_or_shape_error'] or self.counts['render_error']
                status = DiagnosticStatus(name='camera_objects/view', level=2 if error else 1 if warning else 0,
                    message='worker_error' if error else 'render_error' if warning else 'running',
                    values=[KeyValue(key=k,value=json.dumps(v)) for k,v in values.items()])
                self.diagnostics.publish(DiagnosticArray(status=[status]))
                diagnostic_at = start
            self.stop.wait(max(0,1./self.hz-(time.monotonic()-start)))
        self._subscriptions(set())

    def close(self):
        self.stop.set()
        if threading.current_thread() is not self.worker:
            self.worker.join(timeout=2.)


def main():
    rospy.init_node('camera_object_view')
    ObjectViewNode()
    rospy.spin()


if __name__ == '__main__':
    main()
