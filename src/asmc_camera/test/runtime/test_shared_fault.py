import threading
import unittest
from unittest.mock import Mock
from asmc_camera.runtime.shared_runtime import SharedRuntime
from asmc_camera.runtime.ros_control import RosInferenceControl
from asmc_camera.runtime.state import InferenceState, ACTIVE, STANDBY
from asmc_msgs.srv import SetInferenceModeRequest


class SharedFaultTest(unittest.TestCase):
    def test_source_epoch_cannot_readmit_an_old_pending_frame(self):
        control = RosInferenceControl.__new__(RosInferenceControl)
        control.state = InferenceState(['front','left'], managed=True, initial_mode=ACTIVE)
        control.state.observe('front',100)
        control.state.observe('left',100)
        control.pipeline = ('old',)
        control.latest_receipts = {'front':None,'left':None}
        control.publish, control.on_transition = Mock(), Mock()
        self.assertEqual(control.observe('front',1),1)
        self.assertIsNone(control.pipeline)
        self.assertEqual(control.state.mode,ACTIVE)
        control.on_transition.assert_called_once_with(STANDBY)
        self.assertIsNone(control.state.begin({'left':100},per_view=True,expected_generation=0))
        self.assertEqual(control.state.begin({'front':1},per_view=True,expected_generation=1),1)

    def test_fatal_fault_invalidates_all_workers_and_requires_restart(self):
        controls = []
        for _ in range(3):
            control = RosInferenceControl.__new__(RosInferenceControl)
            control.state = InferenceState(['front'], managed=True, initial_mode=ACTIVE)
            control.fatal_reason = ''
            control.publish = Mock()
            control.on_transition = Mock()
            controls.append(control)
        runtime = SharedRuntime.__new__(SharedRuntime)
        runtime.lock, runtime.controls, runtime.fatal = threading.RLock(), controls, ''
        runtime.fault('malformed JPEG')
        self.assertTrue(all(c.state.mode == ACTIVE for c in controls))
        runtime.fault('CUDA error: illegal memory access')
        for control in controls:
            self.assertEqual(control.state.mode, STANDBY)
            self.assertFalse(control.state.warmed_up)
            self.assertEqual(control.state.generation, 1)
            control.on_transition.assert_called_once_with(STANDBY)
            reply = control._request(SetInferenceModeRequest(1, 0.))
            self.assertFalse(reply.accepted)
            self.assertIn('runtime_restart_required', reply.reason)
            with control.state.output_guard(0) as allowed:
                self.assertFalse(allowed)


if __name__ == '__main__':
    unittest.main()
