import unittest
from unittest.mock import Mock, patch

from asmc_camera.runtime.ros_control import pipeline_readiness
from asmc_camera.runtime.demand_inputs import DemandSubscriber


class ReadinessTest(unittest.TestCase):
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
