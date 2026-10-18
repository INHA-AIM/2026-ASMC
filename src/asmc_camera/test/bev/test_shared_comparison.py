"""Exact frame lineage and one shared raster geometry for both BEV panels."""
from dataclasses import replace
import io
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from asmc_msgs.msg import CameraGroundProjection, CameraModelGridTiming
from asmc_camera.bev.calibration import load_calibration_snapshot, calibration_id
from asmc_camera.bev.comparison_projection import ComparisonProjection
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.imu_projection import attitude_camera
from asmc_camera.bev.fusion import CameraBevFusion
from asmc_camera.bev.visualization import fuse_projected_rgb
from asmc_camera.nodes.camera_bev_visualizer_node import CameraBevVisualizerNode
from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode
from asmc_camera.runtime.comparison_frames import ComparisonFrames


class SharedComparisonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.cameras = load_calibration_snapshot(Path(__file__).resolve().parents[2]/'config/cameras_v3.yaml')
        cls.views = tuple(cls.cameras)
        cls.ids = {v: calibration_id(c) for v, c in cls.cameras.items()}

    def header(self, view, stamp=10., seq=42):
        return Header(seq=seq, stamp=rospy.Time.from_sec(stamp), frame_id='camera_'+view)

    def context(self, stamp=10., projection_session='family', corrected=False):
        return CameraGroundProjection(header=Header(stamp=rospy.Time.from_sec(stamp), frame_id='base_link'),
            views=list(self.views), image_headers=[self.header(v, stamp) for v in self.views],
            calibration_ids=[self.ids[v] for v in self.views],
            plane_coefficients=[x for v in self.views for x in self.cameras[v].ground_plane.coefficients],
            producer_session_id='inference', generation=2, projection_session_id=projection_session,
            config_id='fixed', imu_corrected=corrected, reason='disabled')

    def cache(self):
        self.clock = 1.
        return ComparisonFrames(self.views, self.ids, clock=lambda: self.clock)

    def feed(self, cache, stamp=10., seq=42):
        for v in self.views:
            cache.add_raw(v, CompressedImage(header=self.header(v, stamp, seq)))
            cache.add_semantic(v, tuple(Image(header=self.header(v, stamp, seq+100*i)) for i in range(3)))

    def test_exact_raw_sequence_and_complete_three_views_required(self):
        cache = self.cache(); cache.add_context(self.context()); self.feed(cache, seq=41)
        self.assertIsNone(cache.latest())
        for v in self.views[:-1]:
            cache.add_raw(v, CompressedImage(header=self.header(v)))
        self.assertIsNone(cache.latest())
        cache.add_raw(self.views[-1], CompressedImage(header=self.header(self.views[-1])))
        self.assertEqual(cache.latest()[0].header.stamp.to_sec(), 10.)
        self.assertIsNone(cache.latest(), 'same frame must not be rendered repeatedly')

    def test_latest_complete_bundle_wins_without_mixing_neighboring_images(self):
        cache = self.cache()
        for stamp in (10., 10.05, 10.1):
            self.feed(cache, stamp); cache.add_context(self.context(stamp))
        context, rgb, semantic = cache.latest()
        self.assertAlmostEqual(context.header.stamp.to_sec(), 10.1, places=8)
        self.assertTrue(all(m.header.stamp==context.header.stamp for m in rgb.values()))
        self.assertTrue(all(ms[0].header.stamp==context.header.stamp for ms in semantic.values()))

    def test_duplicate_native_stamp_is_ambiguous_not_a_guess(self):
        cache = self.cache(); self.feed(cache); cache.add_context(self.context())
        cache.add_semantic('front', tuple(Image(header=self.header('front', seq=900+i)) for i in range(3)))
        self.assertIsNone(cache.latest())

    def test_bounds_expiry_and_reset(self):
        cache = self.cache()
        for i in range(25):
            self.feed(cache, 10.+i); cache.add_context(self.context(10.+i))
        self.assertEqual(len(cache.contexts), 8)
        self.assertTrue(all(len(q)==8 for q in cache.raw.values()))
        self.clock += .501
        self.assertIsNone(cache.latest())
        cache.reset(); self.assertIsNone(cache.session)
        self.assertTrue(all(not q for q in cache.raw.values()))

    def test_calibration_invalid_plane_and_family_restart(self):
        cache = self.cache(); bad = self.context(); bad.calibration_ids[0] = 'wrong'
        with self.assertRaises(ValueError): cache.add_context(bad)
        bad = self.context(); bad.plane_coefficients[0] = float('nan')
        with self.assertRaises(ValueError): cache.add_context(bad)
        cache.add_context(self.context()); self.feed(cache); cache.latest()
        cache.add_context(self.context(10.1, projection_session='family2'))
        with self.assertRaises(ValueError): cache.add_context(self.context(10.2))
        self.assertIsNone(cache.latest(), 'restart clears images from previous context')

    def test_family_sidecar_uses_existing_planes_and_no_second_imu_lookup(self):
        node = LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.views = self.views; node.supply_session_id = 'family'; node.boundary_family_config_sha256 = 'config'
        node.projection_calibration_ids = self.ids
        node.projection_publisher = NS(get_num_connections=lambda:1)
        node._attitude_cameras = Mock(side_effect=AssertionError('second IMU lookup'))
        frames = {v: NS(timing=CameraModelGridTiming(producer_session_id='inference', generation=2,
            grid_wire_sequences_valid=True, image_header=self.header(v))) for v in self.views}
        audit = dict(projection_planes={v:self.cameras[v].ground_plane.coefficients for v in self.views},
                     projection_corrected=True, projection_reason='corrected')
        message = node._projection_message(frames, audit, self.context().header)
        wire = io.BytesIO(); message.serialize(wire)
        copied = CameraGroundProjection().deserialize(wire.getvalue())
        self.assertEqual(tuple(copied.plane_coefficients), tuple(self.context().plane_coefficients))
        self.assertEqual(copied.image_headers, self.context().image_headers)
        node._attitude_cameras.assert_not_called()
        frames['right'].timing = None
        self.assertIsNone(node._projection_message(frames, audit, self.context().header))
        node.projection_publisher.get_num_connections=lambda:0
        self.assertIsNone(node._projection_message({}, {}, Header()))

    def renderer(self):
        return ComparisonProjection(self.cameras, BevGridSpec(-5.,20.,-8.,8.,.1),
                                    projector_options=dict(max_ground_range_m=22.))

    def test_projection_sidecar_does_no_work_without_demand_and_gates_publication(self):
        node=LaneVectorizerNode.__new__(LaneVectorizerNode)
        node.projection_publisher=NS(get_num_connections=lambda:0,publish=Mock())
        node._publish_frame_projection({}, {})  # no frame/header allocation
        node.projection_publisher.get_num_connections=lambda:1
        node.frame_id='base_link'; node._projection_message=Mock(return_value=self.context())
        node.epoch_gate=NS(lock=threading.RLock(),context=lambda:'new')
        frames={'front':NS(stamp_ns=10**10,header=self.header('front'))}
        node._context_allows=Mock(return_value=False)
        node._publish_frame_projection(frames,{})
        node.projection_publisher.publish.assert_not_called()
        node._context_allows.return_value=True
        node._publish_frame_projection(frames,{})
        node.projection_publisher.publish.assert_called_once()

    def images(self):
        rgb, sem = {}, {}
        for v, c in self.cameras.items():
            a = np.zeros((c.height,c.width),np.uint8)
            a[c.height//2+30:,c.width//2:c.width//2+9] = 1
            rgb[v] = np.repeat((a*255)[:,:,None],3,axis=2)
            sem[v] = (a*200,a,np.full_like(a,255))
        return rgb, sem

    def test_nominal_comparison_matches_existing_static_projection_exactly(self):
        renderer=self.renderer(); rgb,sem=self.images()
        planes = {v:c.ground_plane.coefficients for v,c in self.cameras.items()}
        actual_rgb, actual = renderer.render(planes,rgb,sem)
        projectors, quality = renderer.nominal
        expected = CameraBevFusion(quality).fuse({v:p.project(*sem[v]) for v,p in projectors.items()})
        expected_rgb = fuse_projected_rgb({v:cv2.warpPerspective(rgb[v],p.homography.bev_from_image,
            (renderer.grid.width_px,renderer.grid.height_px)) for v,p in projectors.items()},quality)
        np.testing.assert_array_equal(actual_rgb, expected_rgb)
        np.testing.assert_array_equal(actual['road_marking'], expected.road_marking_class_id)
        np.testing.assert_array_equal(actual['drivable'], expected.drivable_probability)

    def test_corrected_panels_share_three_projectors_and_preserve_input(self):
        renderer=self.renderer(); rgb,sem=self.images()
        before={v:im.copy() for v,im in rgb.items()}
        corrected = {v:attitude_camera(c,(0.,np.sin(.02),0.,np.cos(.02))) for v,c in self.cameras.items()}
        planes={v:c.ground_plane.coefficients for v,c in corrected.items()}
        from asmc_camera.bev.projector import CameraBevProjector
        with patch('asmc_camera.bev.comparison_projection.CameraBevProjector', wraps=CameraBevProjector) as make:
            actual_rgb, actual=renderer.render(planes,rgb,sem)
        self.assertEqual(make.call_count, 3, 'one geometry per view, shared by both panels')
        for v in self.views: np.testing.assert_array_equal(rgb[v],before[v])
        # White RGB and white marking originate in exactly the same pixels.
        marking=actual['road_marking']>0
        self.assertGreater(np.count_nonzero(marking), 0)
        self.assertGreater(np.mean(actual_rgb[marking]), 100.)
        self.assertEqual(renderer.cameras['front'], self.cameras['front'])

    def test_comparison_does_not_demand_old_static_bev_chain(self):
        node=CameraBevVisualizerNode.__new__(CameraBevVisualizerNode)
        node.shared_projection=True
        node.comparison_publisher=NS(get_num_connections=lambda:1)
        node.fused_publisher=node.quality_publisher=None
        self.assertFalse(node._fused_requested())
        node.shared_projection=False
        self.assertTrue(node._fused_requested())

    def test_epoch_change_during_encoding_discards_completed_render(self):
        node=CameraBevVisualizerNode.__new__(CameraBevVisualizerNode)
        node.shared_projection=True; node.lock=threading.Lock()
        gate=NS(lock=threading.RLock(), epoch=1, allows=lambda stamp:True)
        gate.context=lambda:gate.epoch; node.gate=gate
        node.cache_epoch=1; node.views=self.views; node.cameras=self.cameras
        node.grid=node.display_grid=BevGridSpec(-5.,20.,-8.,8.,.1)
        node.fused_size=(80,100); node.comparison_drivable_threshold=.5
        node.comparison_publisher=NS(get_num_connections=lambda:1,publish=Mock())
        rgb, sem=self.images(); raw={}; messages={}
        for v in self.views:
            ok, encoded=cv2.imencode('.jpg',rgb[v]); self.assertTrue(ok)
            raw[v]=CompressedImage(data=encoded.tobytes())
            messages[v]=[Image(height=a.shape[0],width=a.shape[1],step=a.shape[1],encoding='mono8',data=a.tobytes()) for a in sem[v]]
        node.comparison_frames=NS(latest=lambda:(self.context(),raw,messages))
        node.projection_renderer=self.renderer()
        def encode(*args):
            # A newer epoch may arrive while expensive rendering is outside locks.
            gate.epoch += 1
            return CompressedImage()
        node._compressed=encode
        with patch('rospy.Time.now',return_value=rospy.Time.from_sec(10.1)):
            self.assertFalse(node._publish_shared_comparison())
        node.comparison_publisher.publish.assert_not_called()


if __name__=='__main__': unittest.main()
