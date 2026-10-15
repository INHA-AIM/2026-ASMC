import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import rospy
from asmc_camera.runtime.demand_inputs import DemandSynchronizer


class DemandSynchronizerTest(unittest.TestCase):
    def setUp(self):
        clock = patch('rospy.Time.now', return_value=rospy.Time(100))
        clock.start(); self.addCleanup(clock.stop)
        self.connections = []
        def subscribe(topic, kind, callback, **kwargs):
            item = NS(topic=topic, callback=callback, unregister=Mock())
            self.connections.append(item)
            return item
        self.patch = patch('rospy.Subscriber', side_effect=subscribe)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.wanted, self.generation = False, 1
        self.delivered, self.reset = Mock(), Mock()
        self.group = DemandSynchronizer(['/id','/confidence'], object,
            lambda: self.wanted, self.delivered, context=lambda: self.generation,
            on_reset=self.reset)
        self.addCleanup(self.group.unregister)

    def image(self, stamp):
        return NS(header=NS(stamp=rospy.Time(stamp)))

    def test_no_demand_means_no_subscriptions_and_one_pair_is_delivered_once(self):
        self.group.refresh(); self.assertFalse(self.connections)
        self.wanted = True
        self.group.refresh(); self.group.refresh()
        self.assertEqual(len(self.connections), 2)
        self.connections[0].callback(self.image(10))
        self.connections[1].callback(self.image(11))
        self.delivered.assert_not_called()
        self.connections[1].callback(self.image(10))
        self.delivered.assert_called_once()

    def test_disconnect_clears_partial_pair_and_late_transport_callback(self):
        self.wanted = True; self.group.refresh()
        old = list(self.connections)
        old[0].callback(self.image(10))
        self.wanted = False; self.group.refresh()
        for c in old: c.unregister.assert_called_once()
        self.wanted = True; self.group.refresh()
        old[1].callback(self.image(10))
        self.connections[-1].callback(self.image(10))
        self.delivered.assert_not_called()
        self.connections[-2].callback(self.image(10))
        self.delivered.assert_called_once()

    def test_generation_change_rejects_old_callbacks_before_periodic_refresh(self):
        self.wanted = True; self.group.refresh()
        old = list(self.connections)
        old[0].callback(self.image(10))
        self.generation = 2
        old[1].callback(self.image(10))
        self.delivered.assert_not_called()
        self.group.refresh()
        self.reset.assert_called_once()
        self.connections[-2].callback(self.image(20))
        self.connections[-1].callback(self.image(20))
        self.delivered.assert_called_once()

    def test_closed_group_does_not_reconnect_or_deliver(self):
        self.wanted = True; self.group.refresh()
        self.group.unregister(); self.group.refresh()
        self.assertEqual(len(self.connections),2)
        for c in self.connections: c.callback(self.image(10))
        self.delivered.assert_not_called()


if __name__ == '__main__':
    unittest.main()
