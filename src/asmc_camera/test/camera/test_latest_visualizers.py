"""Fresh display cadence, exact input pairing, and render/epoch concurrency."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from asmc_camera.nodes.semantic_visualizer_node import SemanticVisualizerNode
from asmc_camera.nodes.family_view_node import FamilyViewNode
from asmc_camera.bev.grid import BevGridSpec


class Gate:
    def __init__(self):
        self.lock = threading.RLock()
        self.epoch = 1
    def context(self): return self.epoch
    def allows(self, stamp): return True


def publisher():
    return NS(get_num_connections=lambda: 1, publish=Mock())


class LatestVisualizersTest(unittest.TestCase):
    def setUp(self):
        self.clock = 20.
        for target in ('rospy.Time.now',):
            p = patch(target, side_effect=lambda: rospy.Time.from_sec(self.clock))
            p.start(); self.addCleanup(p.stop)
        p = patch('time.monotonic', side_effect=lambda: self.clock)
        p.start(); self.addCleanup(p.stop)

    def semantic(self):
        n = SemanticVisualizerNode.__new__(SemanticVisualizerNode)
        n.views = ('front', 'left', 'right')
        n._lock = threading.RLock(); n.gate = Gate()
        n.latest_event = threading.Event(); n.last_display_stamp = None; n.last_display_key = None
        n.samples = {}; n.latest_panels = {}; n.latest_headers = {}; n.panel_keys = {}
        n.callback_counts = dict.fromkeys(n.views, 0); n.render_counts = dict.fromkeys(n.views, 0)
        n.mosaic_count = n.shape_errors = n.decode_errors = 0
        n.panel_publishers = {}; n.mosaic_publisher = publisher()
        n.panel_width = n.mosaic_view_width = 320
        n._render = Mock(return_value=np.zeros((30, 320, 3), np.uint8))
        n._maybe_log = Mock()
        n._compressed_message = lambda image, header, frame_id=None: NS(header=header or Header())
        return n

    def feed(self, node, stamp, views=None):
        msg = CompressedImage(header=Header(stamp=rospy.Time.from_sec(stamp)))
        for view in views or node.views:
            node._callback(msg, Image(), Image(), view)

    def test_fifteen_hz_input_produces_ten_distinct_timer_outputs(self):
        n = self.semantic()
        inputs = 0
        for tick in range(10):
            self.clock = 20.+tick*.1
            while 20.+inputs/15 <= self.clock+1e-8:
                self.feed(n, 20.+inputs/15); inputs += 1
            n._publish(None)
        stamps = [call.args[0].header.stamp.to_nsec()
                  for call in n.mosaic_publisher.publish.call_args_list]
        self.assertEqual(len(stamps), 10)
        self.assertEqual(len(set(stamps)), 10)
        self.assertEqual(n.render_counts, dict.fromkeys(n.views, 10))

    def test_encoding_does_not_hold_callback_lock_and_epoch_change_drops_result(self):
        n = self.semantic(); self.feed(n, self.clock)
        def encode(*args):
            t = threading.Thread(target=lambda: self.feed(n, self.clock+.01))
            t.start(); t.join(timeout=.5)
            self.assertFalse(t.is_alive(), 'encoding blocked the latest input callback')
            n.gate.epoch += 1
            return NS(header=args[1])
        n._compressed_message = encode
        n._publish(None)
        n.mosaic_publisher.publish.assert_not_called()

    def test_paused_view_is_blanked_and_not_rendered_as_fresh(self):
        n = self.semantic(); self.feed(n, self.clock)
        n._publish(None)
        self.clock += .4; self.feed(n, self.clock, ('front', 'left'))
        with patch('asmc_camera.nodes.semantic_visualizer_node.render_mosaic',
                   return_value=np.zeros((30, 960, 3), np.uint8)) as mosaic:
            n._publish(None)
            right = mosaic.call_args.args[0]['right']
            self.assertEqual(int(right[-1].sum()), 0)
            self.assertEqual(n.render_counts['right'], 1)

    def test_old_subscription_context_cannot_be_relabelled_by_new_status(self):
        n = self.semantic(); n.gate.epoch = 2
        msg = CompressedImage(header=Header(stamp=rospy.Time.from_sec(self.clock)))
        for view in n.views:
            n._callback(msg, Image(), Image(), view, context=1)
        n._publish(None)
        n._render.assert_not_called()
        self.assertEqual(n.mosaic_publisher.publish.call_args.args[0].header.stamp.to_nsec(), 0)

    def test_repeated_source_is_not_counted_as_a_new_display_frame(self):
        n = self.semantic(); self.feed(n, self.clock)
        n._publish(None)
        n.latest_event.wait = Mock()
        self.clock += .1
        n._publish(None)
        self.assertEqual(n.mosaic_publisher.publish.call_count, 1)
        n.latest_event.wait.assert_called_once_with(.04)

    def test_individual_panel_clears_stale_result_with_zero_stamp(self):
        n = self.semantic(); n.panel_publishers = {'front': publisher()}
        n.mosaic_publisher.get_num_connections = lambda: 0
        self.feed(n, self.clock, ('front',)); n._publish(None)
        self.clock += .4; n._publish(None)
        messages = n.panel_publishers['front'].publish.call_args_list
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[-1].args[0].header.stamp.to_nsec(), 0)

    def test_zero_subscribers_clears_pending_and_does_no_rendering(self):
        n = self.semantic(); self.feed(n, self.clock)
        n.mosaic_publisher.get_num_connections = lambda: 0
        n._publish(None)
        n._render.assert_not_called(); self.assertFalse(n.samples)

    def test_mono8_is_read_only_view_and_rendering_matches_old_float_path(self):
        import cv2
        from asmc_camera.camera.visualization import render_panel
        image = np.full((40, 64, 3), 100, np.uint8)
        raw = np.arange(40*64, dtype=np.uint8).reshape(40, 64)
        msg = Image(height=40, width=64, step=64, encoding='mono8', data=raw.tobytes())
        actual = SemanticVisualizerNode._probability_array(msg)
        before = render_panel(image, raw.astype(np.float32)/255, raw % 4,
                              'front', secondary_mode='road_marking', panel_width=320)
        after = render_panel(image, actual, raw % 4, 'front',
                             secondary_mode='road_marking', panel_width=320)
        np.testing.assert_array_equal(before, after)
        self.assertFalse(actual.flags.writeable)

    def test_family_jpeg_is_outside_receive_lock_and_epoch_checked(self):
        n = FamilyViewNode.__new__(FamilyViewNode)
        n.lock = threading.RLock(); n.gate = Gate(); n.publisher = publisher()
        n.latest_event = threading.Event(); n.last_display_stamp = None
        n.grid = BevGridSpec(-2, 10, -4, 4, .1)
        n.background = np.zeros((*n.grid.shape, 3), np.uint8)
        header = Header(stamp=rospy.Time.from_sec(self.clock))
        n.sample = (header, [], '', 1, self.clock)
        def encode(*args):
            t = threading.Thread(target=n._clear)
            t.start(); t.join(timeout=.5)
            self.assertFalse(t.is_alive())
            n.gate.epoch += 1
            return True, np.zeros(3, np.uint8)
        with patch('asmc_camera.nodes.family_view_node.cv2.imencode', side_effect=encode):
            n._publish(None)
        n.publisher.publish.assert_not_called()


if __name__ == '__main__': unittest.main()
