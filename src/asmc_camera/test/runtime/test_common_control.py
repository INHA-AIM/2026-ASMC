import unittest
from unittest.mock import Mock, patch
import rospy
from asmc_camera.msg import InferenceStatus as LegacyStatus
from asmc_camera.srv import SetInferenceModeResponse as LegacyResponse
from asmc_msgs.msg import InferenceStatus
from asmc_msgs.srv import SetInferenceModeRequest, SetInferenceModeResponse
from asmc_camera.runtime.ros_control import RosInferenceControl
from asmc_camera.runtime.state import InferenceState
from asmc_camera.runtime.parameters import canonical_control_path


class CommonControlTest(unittest.TestCase):
    def control(self):
        control = RosInferenceControl.__new__(RosInferenceControl)
        control.state = InferenceState(['front'], managed=True)
        control.session_id = 'same-session-as-observations'
        control.model = 'yolo'
        control.fatal_reason = ''
        control.on_transition = Mock()
        control.publisher = Mock()
        control.legacy_publisher = Mock()
        return control

    def test_each_publication_has_identical_legacy_fields(self):
        control = self.control()
        with patch('rospy.Time.now', return_value=rospy.Time(10)):
            for mode in (0, 1, 2, 0):
                control._request(SetInferenceModeRequest(mode, 15.))
        canonical = control.publisher.publish.call_args_list
        legacy = control.legacy_publisher.publish.call_args_list
        self.assertEqual(len(canonical), 4)
        self.assertEqual(len(legacy), 4)
        for a, b in zip(canonical, legacy):
            a, b = a.args[0], b.args[0]
            self.assertIsInstance(a, InferenceStatus)
            self.assertIsInstance(b, LegacyStatus)
            self.assertEqual(a.producer_session_id, control.session_id)
            for field in LegacyStatus.__slots__:
                self.assertEqual(getattr(a, field), getattr(b, field), field)

    def test_services_share_one_state_and_return_their_own_generated_type(self):
        control = self.control()
        control.publish = Mock()
        request = SetInferenceModeRequest(2, 15.)
        first = control._request(request)
        second = control._legacy_request(request)
        self.assertIsInstance(first, SetInferenceModeResponse)
        self.assertIsInstance(second, LegacyResponse)
        self.assertEqual(first.generation, second.generation)
        self.assertEqual(control.on_transition.call_count, 1)

    def test_process_names_resolve_to_public_api_without_changing_ownership(self):
        self.assertEqual(canonical_control_path('/camera_object_detector', 'status'), '/perception/camera/objects/status')
        self.assertEqual(canonical_control_path('/camera_semantic_inference', 'set_mode'), '/perception/camera/lane/set_mode')
        with self.assertRaises(ValueError): canonical_control_path('/other', 'status')


if __name__ == '__main__': unittest.main()
