"""ROS client shutdown hooks run before rospy.is_shutdown becomes true."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from asmc_camera.camera.frame_buffer import LatestFrameBuffer
from asmc_camera.nodes.semantic_inference_node import SemanticInferenceNode


class SemanticShutdownTest(unittest.TestCase):
    def test_managed_fifteen_hz_model_grid_keeps_each_fresh_batch(self):
        node = SemanticInferenceNode.__new__(SemanticInferenceNode)
        node.views = ('front', 'left', 'right')
        node.model_grid_publishers = {v: Mock() for v in node.views}
        for publisher in node.model_grid_publishers.values():
            publisher.get_num_connections.return_value = 1
        node.model_grid_confidence_publishers = dict(node.model_grid_publishers)
        node.control = SimpleNamespace(state=SimpleNamespace(managed=True, target_hz=15.))
        node.model_grid_period_ns = int(round(1e9/15.))
        node.last_model_grid_stamp_ns = node.next_model_grid_stamp_ns = 0
        node.model_grid_timestamp_resets = node.model_grid_incomplete_batches = 0
        node.model_grid_throttled_batches = node.model_grid_output_batches = 0
        predictions = dict.fromkeys(node.views)
        for stamp in (1000000000, 1050000000, 1100000000, 1200000000):
            frames = {v: SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda t=stamp:t)) for v in node.views}
            self.assertTrue(node._should_publish_model_grid(predictions, frames))
        self.assertEqual(node.model_grid_output_batches, 4)
        self.assertEqual(node.model_grid_throttled_batches, 0)
        self.assertFalse(node._should_publish_model_grid(predictions, frames))

    def test_unconsumed_images_do_not_materialize_and_new_subscriber_gets_fresh_header(self):
        node = SemanticInferenceNode.__new__(SemanticInferenceNode)
        publisher = Mock()
        publisher.get_num_connections.return_value = 0
        node._image_message = Mock(return_value='fresh-source-image')
        with patch('rospy.is_shutdown', return_value=False):
            self.assertTrue(node._publish_connected_image(publisher, 'pixels', 'source-header', True))
            node._image_message.assert_not_called()
            publisher.publish.assert_not_called()
            publisher.get_num_connections.return_value = 1
            self.assertTrue(node._publish_connected_image(publisher, 'pixels', 'source-header', True))
            node._image_message.assert_called_once_with('pixels', 'source-header', categorical=True)
            publisher.publish.assert_called_once_with('fresh-source-image')

    def test_shutdown_terminates_waiting_worker_before_trace_close(self):
        node = SemanticInferenceNode.__new__(SemanticInferenceNode)
        from asmc_camera.runtime.parameters import Parameters
        node.parameters = Parameters(getter=lambda name, default: default)
        node.views = ['front']
        node.buffer = LatestFrameBuffer(node.views, batch_wait_ms=0)
        node._stop = threading.Event()
        closed = []
        node.control = Mock()
        node.control.close.side_effect = lambda: closed.append(not node.worker.is_alive())
        with patch('rospy.is_shutdown', return_value=False) as shutting_down, \
                patch('rospy.get_param', return_value=35.):
            node.worker = threading.Thread(target=node._worker, daemon=True)
            node.worker.start()
            try:
                node._shutdown()
                self.assertFalse(node.worker.is_alive())
                self.assertEqual([True], closed)
            finally:
                shutting_down.return_value = True
                node.buffer.close()
                node.worker.join(timeout=1.)


if __name__ == '__main__':
    unittest.main()
