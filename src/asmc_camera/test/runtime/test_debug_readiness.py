import unittest
from unittest.mock import Mock, patch

from asmc_camera.runtime.ros_control import pipeline_readiness, RosInferenceControl, LaneEpochGate
from asmc_camera.runtime.state import InferenceState, ACTIVE, STANDBY
from types import SimpleNamespace as NS
from asmc_camera.runtime.demand_inputs import DemandSubscriber


class ReadinessTest(unittest.TestCase):
    def test_restart_with_reused_generation_invalidates_the_local_input_epoch(self):
        with patch('rospy.get_param',return_value=True),patch('rospy.Subscriber'):
            gate=LaneEpochGate()
        status=NS(producer_session_id="session1",generation=0,mode=ACTIVE,managed=True,target_hz=15.,successful_batches=20,
                  source_cutoff=NS(to_nsec=lambda:0))
        gate._status(status); old=gate.context()
        gate._status(status); self.assertEqual(gate.context(),old)
        status.producer_session_id="session2"
        gate._status(status); self.assertNotEqual(gate.context(),old)
        old=gate.context()
        status.successful_batches=0
        gate._status(status); self.assertNotEqual(gate.context(),old)
        status.source_cutoff=NS(to_nsec=lambda:100)
        previous=gate.context(); gate._status(status)
        self.assertNotEqual(gate.context(),previous)

    def test_family_feedback_must_belong_to_this_completed_producer_epoch(self):
        control = RosInferenceControl.__new__(RosInferenceControl)
        control.state = InferenceState(['front'], managed=True, initial_mode=ACTIVE)
        control.pipeline = None
        def family(stamp):
            return NS(header=NS(stamp=NS(to_nsec=lambda: stamp)),valid=True)
        control.pipeline_observation(family(10))
        self.assertIsNone(control.pipeline)
        control.state.complete(0, {'front':10})
        control.pipeline_observation(family(10))
        self.assertTrue(control.pipeline[1])
        control.state.request(STANDBY)
        control.pipeline = None
        control.pipeline_observation(family(10))
        self.assertIsNone(control.pipeline)
        control.state.request(ACTIVE)
        control.pipeline_observation(family(10))
        self.assertIsNone(control.pipeline)
        token=control.state.generation
        control.state.complete(token, {'front':20})
        control.pipeline_observation(family(20))
        self.assertEqual(control.pipeline[3],token)
        for stamp in range(21, 600):
            control.state.complete(token, {'front':stamp})
        self.assertEqual(len(control.state.completed_sources),256)

    def test_existing_predicates_explain_independent_failure_causes(self):
        values = dict(prepared=True,generation=2,source_cutoff=1,last_source_stamp=1000000000)
        p = (1000000000,True,10.,2)
        def reason(values=values,p=p,now=1100000000,wall=10.1):
            return pipeline_readiness(values,p,now,wall,.35)['pipeline_ready_reason']
        self.assertEqual(reason(),'ready')
        self.assertEqual(reason(dict(values,prepared=False)),'model_not_prepared')
        self.assertEqual(reason(p=None),'missing_family')
        self.assertEqual(reason(p=(p[0],False,p[2],p[3])),'invalid_family')
        self.assertEqual(reason(dict(values,source_cutoff=p[0])),'before_cutoff')
        self.assertEqual(reason(p=(p[0],True,p[2],1)),'wrong_generation')
        self.assertEqual(reason(wall=10.351),'receipt_stale')
        self.assertEqual(reason(now=1351000000),'source_stale')
        self.assertEqual(reason(now=949000000),'source_stale')
        self.assertEqual(reason(now=1350000000,wall=10.35),'ready')

    def test_idle_debug_inputs_subscribe_only_while_consumed(self):
        demand = Mock(return_value=False)
        with patch('rospy.Subscriber') as subscribe:
            source=DemandSubscriber('/large/image',object,demand,queue_size=1)
            source.refresh();subscribe.assert_not_called()
            demand.return_value=True;source.refresh();source.refresh()
            self.assertEqual(subscribe.call_count,1)
            demand.return_value=False;source.refresh()
            subscribe.return_value.unregister.assert_called_once()
            demand.return_value=True;source.refresh();self.assertEqual(subscribe.call_count,2)
            source.unregister();source.refresh();self.assertEqual(subscribe.call_count,2)


if __name__=='__main__':unittest.main()
