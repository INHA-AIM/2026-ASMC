"""Integrated contact transport and independent short-transition sampling."""
import os
import sys
import time
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from PyQt5 import QtCore,QtWidgets
from scen_runner.config import load_config
from scen_runner.morai.episode_link import SceneLink
from scen_runner.morai.grpc_collision import CollisionMonitor


class GrpcCollisionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_query_retains_all_actor_evidence_and_excludes_self(self):
        link=SceneLink(load_config());stub=Mock();link._sa=S(_actor_stub=stub)
        ego=S(actor_info=S(id=S(value='Ego')),vehicle_state=S(collision_objects=['Ego','1']))
        other=S(actor_info=S(id=S(value='1')))
        stub.GetAllActorsState.return_value=S(states=[ego,other],timestamp=12,sync_mode_tick_count=34)
        self.assertEqual(link.ego_contacts(),('1',))
        request=stub.GetAllActorsState.call_args.args[0]
        self.assertTrue(request.vehicle and request.pedestrian and request.obstacle)
        self.assertEqual(link.contact_states,[ego,other])
        self.assertEqual((link.contact_native_timestamp,link.contact_tick_count),(12,34))
        self.assertEqual(stub.GetAllActorsState.call_args.kwargs['timeout'],.5)
        stub.GetActorState.assert_not_called()

    def test_sampling_keeps_short_contacts_while_generation_is_blocked(self):
        entered=threading.Event();release=threading.Event();samples=[];actor_samples=[]
        def blocked_generation():
            entered.set();release.wait(2)
        producer=threading.Thread(target=blocked_generation)
        reader=Mock(connected=True)
        reader.contact_states=[S(actor_info=S(id=S(value='1')),
            transform=S(location=S(x=1.,y=2.)),velocity=S(x=60.,y=0.,z=0.),
            vehicle_state=S(collision_objects=[]))]
        reader.contact_native_timestamp=reader.contact_tick_count=None
        transitions=[(),('1',),('1','2'),()];calls=[0]
        def read():
            n=calls[0];calls[0]+=1;return transitions[min(n,3)]
        reader.ego_contacts.side_effect=read
        monitor=CollisionMonitor(load_config(),'scope',link_factory=lambda _:reader)
        monitor.publish_actors=True
        monitor.sample.connect(samples.append,QtCore.Qt.DirectConnection)
        monitor.actors.connect(actor_samples.append,QtCore.Qt.DirectConnection)
        try:
            producer.start();self.assertTrue(entered.wait(1));monitor.start()
            deadline=time.monotonic()+1
            while len(samples)<4 and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual([s['contacts'] for s in samples[:4]],transitions)
            self.assertFalse(release.is_set())
            self.assertEqual(actor_samples[0]['states'][0]['speed'],60.)
        finally:
            monitor.stopping.set();monitor.wait(2000);release.set();producer.join(2)
        self.assertFalse(monitor.isRunning());self.assertFalse(producer.is_alive())


if __name__=='__main__':unittest.main()
