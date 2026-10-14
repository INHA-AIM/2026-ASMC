import threading
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
import numpy as np

from asmc_camera.runtime.shared_input import CacheBusy, DecodedCache, SharedInput
from asmc_camera.runtime.parameters import Parameters, control_paths
from asmc_camera.runtime.observation import ObservationIdentity


class SharedInputTest(unittest.TestCase):
    def test_concurrent_consumers_decode_once_and_cannot_modify_input(self):
        cache = DecodedCache()
        started, release = threading.Event(), threading.Event()
        seen, calls = [], []
        def decode(data):
            calls.append(data)
            started.set()
            self.assertTrue(release.wait(2))
            return np.zeros((4, 4, 3), np.uint8)
        def consume():
            with cache.lease('front', 1, b'jpeg', decode) as value:
                seen.append(value)
        first = threading.Thread(target=consume)
        second = threading.Thread(target=consume)
        first.start()
        self.assertTrue(started.wait(2))
        second.start()
        release.set()
        first.join(2); second.join(2)
        self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(len(calls), 1)
        self.assertIs(seen[0], seen[1])
        with self.assertRaises(ValueError):
            seen[0][0, 0] = 1
        masked = seen[0].copy()
        masked[0, 0] = 1
        self.assertFalse(seen[0].any())

    def test_two_pinned_slots_drop_third_without_wait_and_recover(self):
        cache = DecodedCache()
        decode = lambda _: np.zeros((4, 4, 3), np.uint8)
        with ExitStack() as leases:
            for key in (1, 2):
                leases.enter_context(cache.lease('front', key, b'', decode))
            with self.assertRaises(CacheBusy):
                with cache.lease('front', 3, b'', decode):
                    pass
            # Another camera is independent of the pinned slots.
            leases.enter_context(cache.lease('left', 3, b'', decode))
        with cache.lease('front', 3, b'', decode):
            pass
        self.assertEqual(len(cache.rows['front']), 2)
        self.assertEqual(cache.busy, 1)

    def test_decoder_error_releases_slot(self):
        cache = DecodedCache()
        with self.assertRaises(ValueError):
            with cache.lease('front', 1, b'', lambda _: None):
                pass
        self.assertEqual(cache.rows['front'][0]['refs'], 0)

    def test_one_subscription_demand_decode_and_source_epoch(self):
        subscriptions, delivered = [], []
        class ROS:
            Time = NS(now=lambda: NS(to_nsec=lambda: 123))
            @staticmethod
            def Subscriber(*args, **kwargs):
                subscriptions.append(args)
                return NS(unregister=lambda: None)
        broker = SharedInput(ROS, object)
        a = broker.subscribe('/front', delivered.append)
        broker.subscribe('/front', delivered.append)
        self.assertEqual(len(subscriptions), 1)
        def receive(stamp, data):
            broker._receive('/front', NS(header=NS(stamp=NS(to_nsec=lambda: stamp), frame_id='camera_front'), data=data))
        receive(5, b'a'); receive(5, b'b'); receive(4, b'b')
        self.assertEqual(broker.cache.decodes, 0)
        self.assertEqual(delivered[0].decode_key[1][1], 0)
        self.assertEqual(delivered[2].decode_key[1][1], 1)
        self.assertEqual(delivered[4].decode_key[1][1], 2)
        a.unregister(); receive(6, b'c')
        self.assertEqual(len(delivered), 7)

    def test_explicit_parameters_and_alias_names(self):
        params = {'/runtime/objects/target_hz': 15, '/runtime/lane/target_hz': 10}
        self.assertEqual(Parameters('/runtime/objects', params.get).get('~target_hz'), 15)
        self.assertEqual(Parameters('/runtime/lane', params.get).get('~target_hz'), 10)
        self.assertEqual(control_paths('yolo'), ('/perception/camera/objects/set_mode',
                         '/perception/camera/objects/status', '/camera_object_detector'))

    def test_observation_retains_source_header_empty_invalid_and_restart(self):
        calibration = {'cameras': {'front': {'width': 1280, 'height': 720}}}
        identity = ObservationIdentity(calibration, 'model-sha', {'confidence': .4})
        result = NS(header=NS(frame_id='camera_front', stamp=123), valid=True, boxes=[])
        wrapped = identity.wrap(result, 7, NS)
        self.assertIsNot(wrapped.detections.header, result.header)
        self.assertEqual(wrapped.detections.header.stamp, result.header.stamp)
        self.assertEqual(wrapped.generation, 7)
        self.assertTrue(wrapped.calibration_id)
        self.assertNotEqual(identity.session_id, ObservationIdentity(calibration, 'model-sha', {}).session_id)
        self.assertNotEqual(identity.config_id, ObservationIdentity(calibration, 'model-sha', {'confidence': .3}).config_id)
        result.valid = False
        self.assertFalse(identity.wrap(result, 8, NS).detections.valid)
        result.valid, result.header.frame_id = True, 'unknown'
        with self.assertRaises(ValueError):
            identity.wrap(result, 8, NS)


if __name__ == '__main__':
    unittest.main()
