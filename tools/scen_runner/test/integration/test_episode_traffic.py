"""Independent runtime contracts: target/cap, ownership, reuse and partial failure."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config
from scen_runner.runtime.episode_traffic import HighSpeedSection, RoundaboutSection
from scen_runner.morai.native_vehicle_source import SourcePlacementDeferred
from scen_runner.records.episode_telemetry import vehicle_records


class EpisodeTrafficTest(unittest.TestCase):
    def section(self, kind=HighSpeedSection):
        self.now=0.
        link=Mock(actor_scopes={},ego_id='Ego')
        link.states.return_value=[];link.destroy.return_value=True
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=kind('high_speed' if kind==HighSpeedSection else 'roundabout',42,'scope',
                         load_config(),lambda:False,lambda *_:None,clock=lambda:self.now)
        return section,link

    @staticmethod
    def ticket(link,source,model,target):
        return dict(info=S(client_key=f'scope_{source.source_id}',id=S(value=str(source.source_id))),
                    lane=source,model=model,target=target,actor=None,missing_since=None)

    @staticmethod
    def state(source,aid):
        cfg=source.plan.cfg;p=cfg['spawn_pose']
        return S(actor_info=S(id=S(value=aid),object_type=1),
            transform=S(location=S(x=p['x'],y=p['y'],z=p['z']),rotation=S(z=-90)),
            vehicle_state=S(current_link_info=S(id=S(value=cfg['npc_links'][0])),
                throttle=1.,brake=0.,gear_mode=4,remaining_distance=-1.,tl_id=S(value='none'),tl_color=-2),
            velocity=S(x=0.,y=-20.,z=0.))

    def test_configs_routes_and_catalogue_match_verified_section_settings(self):
        section,link=self.section()
        self.assertEqual([s.source_id for s in section.sources],[6,8,10])
        self.assertIn(section.period,[6,7]);self.assertEqual(len(section.cfg['models']),16)
        self.assertEqual(section.cfg['speeds_kmh'],list(range(50,81)))
        self.assertEqual(section.cfg['maximum_spawn_vehicle'],50)
        for source in section.sources:
            for a,b in zip(source.plan.cfg['npc_links'],source.plan.cfg['npc_links'][1:]):
                self.assertEqual(section.links[a]['to_node_idx'],section.links[b]['from_node_idx'])

    def test_native_target_also_resets_physics_ceiling_for_every_birth(self):
        section,link=self.section()
        with patch('scen_runner.runtime.episode_traffic.create_ticket',side_effect=self.ticket), \
             patch('scen_runner.runtime.episode_traffic.enable_ticket'):
            section.advance([])
            states=[self.state(s,str(s.source_id+100)) for s in section.sources]
            section.advance(states)
        self.assertEqual(len(section.actors),3)
        self.assertEqual(link.set_speed_limit.call_count,3)
        for call in link.set_speed_limit.call_args_list:
            aid,target=call.args
            self.assertEqual(section.owned()[aid][1],target)
            self.assertGreaterEqual(target,40);self.assertLessEqual(target,60)
        self.assertTrue(section.ready_live(states))
        link.spawn.assert_not_called()

    def test_failed_cap_rpc_retains_created_actor_for_cleanup(self):
        section,link=self.section();source=section.sources[0]
        ticket=self.ticket(link,source,'2015_Kia_K5',80)
        ticket['baseline']=set();section.tickets=[ticket];source.pending=ticket
        state=self.state(source,'3')
        link.set_speed_limit.side_effect=RuntimeError('setter failed')
        with patch('scen_runner.runtime.episode_traffic.enable_ticket'):
            with self.assertRaisesRegex(RuntimeError,'setter failed'):
                section.bind(ticket,state)
        self.assertEqual(ticket['actor'],'3');self.assertIn('3',section.owned())
        link.states.return_value=[state]
        def retire(*args,**kwargs):
            link.states.return_value=[]
            return True
        link.destroy.side_effect=retire
        with patch('scen_runner.runtime.episode_traffic.enable_ticket'),patch('scen_runner.runtime.episode_traffic.destroy_ticket'):
            section.cleanup()
        link.destroy.assert_called_once_with('3',object_type=1)

    def test_reused_id_retains_new_car_and_replaces_old_cap(self):
        section,link=self.section();section.initial_ids={'Ego'}
        source=section.sources[0]
        old=self.ticket(link,source,'2015_Kia_K5',50);old['actor']='3'
        new=self.ticket(link,source,'2015_Kia_K5',80);new.update(baseline={'3'},requested_at=0.)
        section.register('3',1,old['model'],50)
        section.tickets=[old,new];source.pending=new
        for s in section.sources:s.next_due=float('inf')
        section.samples={'3':(0.,-100.,-200.)};self.now=.2
        with patch('scen_runner.runtime.episode_traffic.destroy_ticket') as destroy,patch('scen_runner.runtime.episode_traffic.enable_ticket'):
            section.advance([self.state(source,'3')])
        destroy.assert_called_once_with(link,old);link.destroy.assert_not_called()
        self.assertEqual(section.owned()['3'][1],80)
        link.set_speed_limit.assert_called_once_with('3',80)

    def test_generation_matching_only_runs_for_jumps_and_rejects_ambiguous_reuse(self):
        section,link=self.section();source=section.sources[0]
        state=self.state(source,'3');p=state.transform.location
        ticket=self.ticket(link,source,'2015_Kia_K5',80);ticket['baseline']={'3'}
        source.pending=ticket;section.initial_ids={'3'}
        with patch.object(section,'at_start',wraps=section.at_start) as at_start,patch.object(
                section,'forget_reused_id') as forget:
            for offset,elapsed in [(0.,.2),(15.,.2),(300.,10.)]:
                section.samples={'3':(0.,p.x-offset,p.y)}
                section.track_generations({'3':state},elapsed)
            at_start.assert_not_called();forget.assert_not_called()
            # Two pending sources matching a jumped body are still ambiguous.
            section.sources[1].pending=dict(ticket,baseline={'3'})
            section.samples={'3':(0.,p.x-300.,p.y)}
            section.track_generations({'3':state},.2)
            self.assertEqual(at_start.call_count,2);forget.assert_not_called()
            self.assertIn('3',section.initial_ids);self.assertIn('3',ticket['baseline'])
        self.assertEqual(section.samples['3'],(.2,p.x,p.y))

    def test_deferred_placement_does_not_block_other_sources_or_redraw_choice(self):
        section,link=self.section();choices=[]
        def create(link,source,model,target):
            choices.append((source.source_id,model,target))
            if len(choices)==1:raise SourcePlacementDeferred('False')
            return self.ticket(link,source,model,target)
        with patch('scen_runner.runtime.episode_traffic.create_ticket',side_effect=create),patch('scen_runner.runtime.episode_traffic.enable_ticket'):
            section.advance([]);self.assertEqual(len(choices),3)
            self.now=2.;section.advance([])
        self.assertEqual(choices[0],choices[-1])

    def test_ambiguous_birth_disables_producer_and_does_not_claim_foreign_car(self):
        section,link=self.section()
        with patch('scen_runner.runtime.episode_traffic.create_ticket',side_effect=self.ticket),patch('scen_runner.runtime.episode_traffic.enable_ticket') as enable:
            section.advance([])
            source=section.sources[0]
            with self.assertRaisesRegex(RuntimeError,'대응 모호'):
                section.advance([self.state(source,'3'),self.state(source,'4')])
            self.assertFalse(enable.call_args.args[2])
        self.assertFalse(section.owned())

    def test_roundabout_next_success_after_fifteen_is_twentyone(self):
        section,link=self.section(RoundaboutSection)
        section.pending=('2015_Kia_K5',15)
        serial=[0]
        def spawn(model,speed,plan,on_created):
            serial[0]+=1;on_created(str(serial[0]))
        link.spawn.side_effect=spawn
        section.advance([]);self.now=section.period;section.advance([])
        self.assertEqual([c.args[1] for c in link.spawn.call_args_list],[15,21])

    def test_speed_diagnostics_preserve_units_target_and_native_controls(self):
        section,link=self.section();source=section.sources[0]
        section.register('3',1,'2015_Kia_K5',80,source=6);self.now=15.
        records=vehicle_records([self.state(source,'3')],section.actor_details(),self.now)
        self.assertEqual(records[0]['target'],80);self.assertEqual(records[0]['speed'],20)
        self.assertEqual(records[0]['age_s'],15);self.assertEqual(records[0]['throttle'],1.)
        self.assertEqual(records[0]['brake'],0.);self.assertIsNone(records[0]['front_center_m'])


if __name__=='__main__':unittest.main()
