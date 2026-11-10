from types import SimpleNamespace
import unittest
from scen_runner.runtime.trial_command_gate import TrialCommandGate


class TrialGateTest(unittest.TestCase):
    def test_preparation_cannot_apply_the_upstream_throttle_or_steering(self):
        sent=[]
        def hold():return SimpleNamespace(accel=0,brake=1,steering=0)
        gate=TrialCommandGate(sent.append,hold)
        request=SimpleNamespace(accel=.4,brake=0,steering=.2)
        gate.command(request);gate.tick()
        self.assertEqual(len(sent),2)
        self.assertTrue(all(m.brake==1 and m.accel==0 and m.steering==0 for m in sent))
        self.assertEqual(request.accel,.4)
        gate.release()
        self.assertEqual(len(sent),2) # Releasing never replays a cached command.
        gate.tick();self.assertEqual(len(sent),2)
        gate.command(request);self.assertIs(sent[-1],request)
        gate.close();gate.command(request)
        self.assertEqual(sent[-1].brake,1)
        self.assertEqual(gate.forwarded,1)


if __name__=='__main__':unittest.main()
