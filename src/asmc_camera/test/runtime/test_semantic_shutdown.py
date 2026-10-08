"""ROS client shutdown hooks run before rospy.is_shutdown becomes true."""
import threading
import unittest
from unittest.mock import Mock, patch

from asmc_camera.camera.frame_buffer import LatestFrameBuffer
from asmc_camera.nodes.semantic_inference_node import SemanticInferenceNode


class SemanticShutdownTest(unittest.TestCase):
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
