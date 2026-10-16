"""Real projection/fusion and ROS images on an in-process transport fixture."""
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image
from asmc_camera.nodes.camera_bev_node import CameraBevNode
from asmc_camera.nodes.camera_bev_fusion_node import CameraBevFusionNode


class Bus:
    def __init__(self):
        self.subscribers, self.publishers = [], {}

    def subscribe(self, topic, kind, callback, **kwargs):
        item = NS(topic=topic,callback=callback,active=True)
        item.unregister=lambda: setattr(item,'active',False)
        self.subscribers.append(item)
        return item

    def publisher(self, topic, kind, **kwargs):
        item=NS(messages=[])
        item.get_num_connections=lambda: sum(s.active and s.topic==topic for s in self.subscribers)
        def publish(message):
            item.messages.append(message)
            for s in list(self.subscribers):
                if s.active and s.topic==topic: s.callback(message)
        item.publish=publish
        self.publishers[topic]=item
        return item


class BevDemandChainTest(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[2]/'config'
        values={}
        for name in ('cameras_v3.yaml','bev/bev_grid.yaml','bev/fusion.yaml'):
            values.update(yaml.safe_load((root/name).read_text()))
        values.update(managed=False,verify_sensor_set=False,enabled_views=['front','left','right'])
        self.bus=Bus()
        def get(key, *default):
            key=key.lstrip('~')
            if key in values: return values[key]
            if default: return default[0]
            raise KeyError(key)
        patches=[patch('rospy.get_param',side_effect=get),patch('rospy.Publisher',self.bus.publisher),
            patch('rospy.Subscriber',self.bus.subscribe),patch('rospy.Timer'),patch('rospy.loginfo'),
            patch('rospy.Time.now',return_value=rospy.Time(100)),
            patch('asmc_camera.nodes.camera_bev_node.DemandInputs'),
            patch('asmc_camera.nodes.camera_bev_fusion_node.DemandInputs')]
        for p in patches: p.start(); self.addCleanup(p.stop)
        self.bev=CameraBevNode(); self.fusion=CameraBevFusionNode()
        for node in (self.bev,self.fusion):
            for g in node.subscribers: self.addCleanup(g.unregister)

    def refresh(self):
        # Demand propagates downstream -> upstream, as the independent wall
        # pollers do eventually; no graph/master introspection is required.
        for node in (self.fusion,self.bev):
            for g in node.subscribers: g.refresh()

    def inject_view(self,view,stamp):
        camera=self.bev.cameras[view]
        for topic in (camera.drivable_topic,camera.road_marking_topic,camera.road_marking_confidence_topic):
            image=Image(height=camera.height,width=camera.width,encoding='mono8',step=camera.width,
                        data=np.zeros((camera.height,camera.width),np.uint8).tobytes())
            image.header.stamp=rospy.Time(stamp)
            for s in list(self.bus.subscribers):
                if s.active and s.topic==topic: s.callback(image)

    def test_no_viewer_cascades_to_zero_native_inputs_then_recorder_restores_fusion(self):
        self.refresh()
        self.assertFalse(any(g.active for g in self.bev.subscribers+self.fusion.subscribers))
        topic='/perception/bev/camera/road_marking/class_id'
        received=[]
        viewer=self.bus.subscribe(topic,Image,received.append)
        self.refresh()
        self.assertTrue(all(g.active for g in self.bev.subscribers+self.fusion.subscribers))
        for view in self.bev.views: self.inject_view(view,10)
        self.assertEqual(len(received),1)
        self.assertEqual(received[0].header.stamp,rospy.Time(10))
        self.assertEqual((received[0].height,received[0].width),self.fusion.grid.shape)
        # No connected consumer means no serialization of other fused outputs.
        self.assertFalse(self.bus.publishers['/perception/bev/camera/drivable_probability'].messages)
        viewer.unregister(); self.refresh()
        self.assertFalse(any(g.active for g in self.bev.subscribers+self.fusion.subscribers))
        self.assertTrue(all(not q for q in self.fusion.buffers.values()))
        viewer=self.bus.subscribe(topic,Image,received.append); self.refresh()
        for view in self.bev.views: self.inject_view(view,20)
        self.assertEqual([m.header.stamp.to_sec() for m in received],[10,20])

    def test_standby_detaches_inputs_even_with_a_recorder(self):
        self.bus.subscribe('/perception/bev/camera/coverage',Image,Mock())
        self.refresh()
        for node in (self.bev,self.fusion):
            node.epoch_gate.enabled=True; node.epoch_gate.mode=0; node.epoch_gate.generation=2
        self.refresh()
        self.assertFalse(any(g.active for g in self.bev.subscribers+self.fusion.subscribers))


if __name__=='__main__': unittest.main()
