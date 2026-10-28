#!/usr/bin/env python3
"""Experimental probability/coverage producer; no inference or vehicle control.

Explicit scenario launch only. Uses the same source-exact family plane context
as the comparison BEV. The operational fixed-plane raster is not repurposed.
"""
import copy
import threading
import time
from functools import partial
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from asmc_msgs.msg import CameraGroundProjection
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.comparison_projection import ComparisonProjection
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.runtime.comparison_frames import ComparisonFrames
from asmc_camera.runtime.ros_control import LaneEpochGate
from asmc_camera.runtime.render_worker import RenderWorker


class SemanticSpaceTrial:
    def __init__(self):
        cv2.setNumThreads(1)
        self.grid = BevGridSpec.from_mapping(rospy.get_param('~'))
        self.cameras = calibrations_from_dict(dict(ground_plane=rospy.get_param('~ground_plane'),
                                                 cameras=rospy.get_param('~cameras')))
        self.renderer = ComparisonProjection(self.cameras, self.grid,
            projector_options=dict(max_ground_range_m=22., min_camera_depth_m=.1))
        self.masks = {v: HoodMask.load(True, rospy.get_param('~ego_masks/'+v)).geometry(c.height,c.width)[0]
                      for v,c in self.cameras.items()}
        # r04 RGB/plane replay found hood-edge non-road interpolation2-3
        # original pixels outside the object mask. Keep this4px margin only
        # in experimental visibility; neither model output nor YOLO changes.
        self.visibility_margin_px=int(rospy.get_param('~ego_visibility_margin_px',4))
        self.gate = LaneEpochGate()
        self.lock = threading.Lock()
        self.epoch = self.gate.context()
        self.frames = ComparisonFrames(self.cameras, self.renderer.calibration_ids, max_age_sec=.35)
        self.sequence = 0
        self.counts = dict(outputs=0, rejected=0, stale=0, m0=0)
        self.publishers = {k: rospy.Publisher('~'+k, Image, queue_size=1)
                           for k in ('drivable_probability','coverage')}
        self.context_pub = rospy.Publisher('~context', CameraGroundProjection, queue_size=1)
        self.worker = RenderWorker(self.render, max_hz=10., on_error=self.error)
        self.subs = [rospy.Subscriber('/perception/lane/ground_projection', CameraGroundProjection,
                                     self.context, queue_size=1)]
        self.subs += [rospy.Subscriber(c.drivable_topic, Image, partial(self.semantic,v),
                                     queue_size=1, buff_size=4*1024*1024) for v,c in self.cameras.items()]
        rospy.on_shutdown(self.worker.close)

    def error(self, exc):
        self.counts['rejected'] += 1
        rospy.logwarn_throttle(2., 'semantic space rejected: %s', exc)

    def reset_epoch(self):
        epoch = self.gate.context()
        if epoch != self.epoch:
            self.frames.reset()
            self.epoch = epoch

    def context(self, msg):
        with self.gate.lock, self.lock:
            self.reset_epoch()
            if (not self.gate.allows(msg.header.stamp) or
                    msg.producer_session_id != self.gate.producer_session_id or msg.generation != self.gate.generation):
                return
            if not msg.imu_corrected:
                self.counts['m0'] += 1
                return  # This experiment is explicitly IMU-only, no silent M0 fallback.
            if max(h.stamp.to_nsec() for h in msg.image_headers)-min(h.stamp.to_nsec() for h in msg.image_headers)>35000000:
                self.error('view source skew exceeds35ms');return
            try: self.frames.add_context(msg)
            except ValueError as exc: self.error(exc);return
        self.worker.offer()

    def semantic(self, view, msg):
        with self.gate.lock, self.lock:
            self.reset_epoch()
            if not self.gate.allows(msg.header.stamp):return
            self.frames.add_semantic(view, (msg,))
        self.worker.offer()

    def render(self):
        with self.gate.lock, self.lock:
            self.reset_epoch()
            epoch = self.epoch
            selected = self.frames.latest(require_raw=False)
        if selected is None:return False
        context, _, inputs = selected
        age = (rospy.Time.now()-context.header.stamp).to_sec()
        if age<-.05 or age>.35:self.counts['stale']+=1;return False
        start=time.monotonic()
        arrays={}
        for view, messages in inputs.items():
            msg=messages[0];c=self.cameras[view]
            if msg.encoding!='mono8' or (msg.height,msg.width)!=(c.height,c.width) or msg.step<msg.width or len(msg.data)!=msg.step*msg.height:
                raise ValueError('wrong original semantic image shape/encoding')
            arrays[view]=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
        planes={v:context.plane_coefficients[3*i:3*i+3] for i,v in enumerate(context.views)}
        layers=self.renderer.drivable(planes,arrays,self.masks,self.visibility_margin_px)
        with self.gate.lock:
            if epoch!=self.gate.context() or not self.gate.allows(context.header.stamp):return False
            if (rospy.Time.now()-context.header.stamp).to_sec()>.35:self.counts['stale']+=1;return False
            self.sequence+=1
            context=copy.deepcopy(context);context.header.seq=self.sequence
            self.context_pub.publish(context)
            for topic,key in [('drivable_probability','drivable'),('coverage','coverage')]:
                arr=layers[key]
                m=Image(header=context.header,height=arr.shape[0],width=arr.shape[1],encoding='mono8',step=arr.shape[1],data=arr.tobytes())
                self.publishers[topic].publish(m)
        self.counts['outputs']+=1
        rospy.loginfo_throttle(5.,'semantic space %s projection=%.1fms source_age=%.1fms',self.counts,
            1000*(time.monotonic()-start),1000*(rospy.Time.now()-context.header.stamp).to_sec())
        return True

if __name__=='__main__':
    rospy.init_node('semantic_space_trial')
    SemanticSpaceTrial()
    rospy.spin()
