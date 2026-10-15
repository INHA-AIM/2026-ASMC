"""Optimization contracts: current state, geometry reload and same diagnostics."""
import json
import io
import math
import os
import random
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config
from scen_runner.runtime.episode_traffic import RoundaboutSection
from scen_runner.records.episode_telemetry import SpeedTrace,vehicle_records
from scen_runner.planning.traffic_geometry import load_links


class PerformanceContractTest(unittest.TestCase):
    def test_lazy_speed_sample_keeps_cadence_latest_values_and_unthrottled_contacts(self):
        trace=SpeedTrace.__new__(SpeedTrace)
        trace.stream=io.StringIO();trace.lock=threading.RLock()
        trace.last_record=trace.last_console=float('-inf')
        live={'x':1.}
        supplier=Mock(side_effect=lambda:[dict(live)])
        environment=dict(weather='Foggy',hour=11)
        with patch('scen_runner.records.episode_telemetry.time.monotonic') as clock:
            clock.return_value=0.
            trace.sample([],1,'running',seed=42,actor_supplier=supplier,environment=environment)
            for index in range(1,5):
                clock.return_value=index/5;live['x']=index+1.
                trace.sample([],1,'running',actor_supplier=supplier)
            evidence=dict(episode=1,seed=42,phase='running',contacts=['3'])
            trace.record_contact(evidence,[dict(live)])
            clock.return_value=1.;live['x']=6.
            trace.sample([],1,'running',seed=42,actor_supplier=supplier,environment=environment)
        self.assertEqual(supplier.call_count,2)
        records=[json.loads(line) for line in trace.stream.getvalue().splitlines()]
        self.assertEqual([r['event'] for r in records],['sample','contact','sample'])
        self.assertEqual([r['actors'][0]['x'] for r in records],[1.,5.,6.])
        self.assertEqual(records[1]['contact'],evidence)
        for record in (records[0],records[2]):
            self.assertEqual(record['environment'],environment)
            self.assertEqual((record['episode'],record['seed'],record['phase']),(1,42,'running'))
        trace.close()
        trace.sample([],1,'running',actor_supplier=supplier)
        self.assertEqual(supplier.call_count,2)

    def test_map_cache_is_immutable_and_reloads_same_size_same_mtime_edit(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'links.json'
            path.write_text(json.dumps([dict(idx='a',points=[[1,2,3],[4,5,6]])]))
            first=load_links(path);old=path.stat()
            self.assertIs(first,load_links(path))
            with self.assertRaises(TypeError):first['a']['points'][0][0]=9
            path.write_text(json.dumps([dict(idx='a',points=[[9,2,3],[4,5,6]])]))
            os.utime(path,ns=(old.st_atime_ns,old.st_mtime_ns))
            second=load_links(path)
            self.assertIsNot(first,second);self.assertEqual(second['a']['points'][0][0],9)
            self.assertEqual(first['a']['points'][0][0],1)

    def test_roundabout_uses_fresh_state_only_when_spawn_decision_is_due(self):
        now=[0.];link=Mock(actor_scopes={});link.states.return_value=[]
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=RoundaboutSection('roundabout',42,'scope',load_config(),lambda:False,lambda *_:None,clock=lambda:now[0])
        section.next_due=5.
        section.tick();link.states.assert_not_called();link.spawn.assert_not_called()
        now[0]=5.;section.tick()
        link.states.assert_called_once();link.spawn.assert_called_once()
        # Spawn-clear rejection is still checked every due tick.
        section.next_due=now[0]
        with patch('scen_runner.runtime.episode_traffic.spawn_clear',return_value=False):
            section.tick();section.tick()
        self.assertEqual(link.states.call_count,3);self.assertEqual(link.spawn.call_count,1)

    def test_dense_front_distance_matches_scalar_rules_at_lateral_boundaries(self):
        rng=random.Random(4);states=[];details={}
        for index in range(90):
            aid=str(index);x,y=rng.uniform(-50,50),rng.uniform(-50,50)
            if index<4:x,y=[(0,0),(5,2),(6,1.999999999),(7,-2)][index]
            angle=0. if index<4 else rng.uniform(-180,180)
            state=S(actor_info=S(id=S(value=aid),object_type=1),
                transform=S(location=S(x=x,y=y,z=0.),rotation=S(z=angle)),velocity=S(x=50,y=0,z=0),
                vehicle_state=S(throttle=.5,brake=0.,gear_mode=4,current_link_info=S(id=S(value='a')),
                                remaining_distance=100.,tl_id=S(value='none'),tl_color=-2))
            states.append(state);details[aid]=dict(kind=1,section='high_speed',model='K5',target=70,born_at=0.,source=6)
        actual=vehicle_records(states,details,20.)
        for state,record in zip(states,actual):
            theta=math.radians(state.transform.rotation.z);p=state.transform.location
            distances=[]
            for other in states:
                if other.actor_info.id.value==state.actor_info.id.value:continue
                q=other.transform.location;dx,dy=q.x-p.x,q.y-p.y
                along=dx*math.cos(theta)+dy*math.sin(theta)
                across=-dx*math.sin(theta)+dy*math.cos(theta)
                if along>0 and abs(across)<2.:distances.append(along)
            expected=min(distances) if distances else None
            if expected is None:self.assertIsNone(record['front_center_m'])
            else:self.assertAlmostEqual(record['front_center_m'],expected,places=12)
            self.assertEqual(record['target'],70);self.assertEqual(record['speed'],50)
        self.assertEqual(actual[0]['front_center_m'],6.)
