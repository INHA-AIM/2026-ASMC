import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from asmc_camera.nodes.object_visualizer_node import ObjectVisualizerNode


class OverlayViewTest(unittest.TestCase):
    def test_equal_stamp_side_result_cannot_consume_front_sync_input(self):
        node=ObjectVisualizerNode.__new__(ObjectVisualizerNode);node.expected_frame_id='camera_front'
        target=Mock()
        for view in ['left','right','front']:
            message=SimpleNamespace(header=SimpleNamespace(stamp=42,frame_id='camera_'+view))
            node._accept_view(target,message)
        self.assertEqual(1,target.signalMessage.call_count)
        self.assertEqual('camera_front',target.signalMessage.call_args.args[0].header.frame_id)


if __name__=='__main__':unittest.main()
