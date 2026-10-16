"""Independent episode traffic: per-vehicle native targets and explicit ownership."""
import copy
import math
import random
from dataclasses import dataclass
from types import SimpleNamespace

from scen_runner.config import TOOL_ROOT, load_yaml, repo_path
from scen_runner.runtime.episode_actors import Section
from scen_runner.morai.native_vehicle_source import (create_ticket, enable_ticket, destroy_ticket,
                                   ticket_candidates, SourcePlacementDeferred)
from scen_runner.planning.traffic_geometry import route_between, spawn_clear,load_links
from scen_runner.records.replay_spec import vehicle_spec


class RoundaboutSection(Section):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cfg = load_yaml(TOOL_ROOT/'config/roundabout.yaml')
        self.rng = random.Random(self.seed)
        self.period = self.rng.choice(self.cfg['periods_s'])
        self.links = load_links(repo_path(self.cfg['link_map']))
        self.plan = SimpleNamespace(cfg=self.cfg, destination=list(self.links[self.cfg['npc_links'][-1]]['points'][self.cfg['end_point_index']]))
        self.count, self.previous_speed, self.pending = 0, None, None
        self.next_due = self.clock()

    def advance(self, states):
        if self.cancelled():
            return
        if self.count >= self.cfg['maximum_spawn_vehicle'] or self.clock() < self.next_due:
            return
        if self.pending is None:
            speed = 21 if self.previous_speed == 15 else self.rng.choice(self.cfg['speeds_kmh'])
            self.pending = self.rng.choice(list(self.cfg['models'])), speed
        model, speed = self.pending
        if not spawn_clear(self.cfg, model, states):
            self.status = '생성 위치 안전거리/TTC 대기'
            return
        # Register immediately, before any route/destination RPC can fail.
        self.link.spawn(model, speed, self.plan,
                        on_created=lambda aid: self.register(aid, 1, model, speed,
                            replay=dict(vehicle_spec(model,speed,self.plan,'direct'),period_s=self.period)))
        self.count += 1
        self.previous_speed, self.pending = speed, None
        self.next_due = self.clock()+self.period
        self.status = f'누적 {self.count}대 · 주기 {self.period}초'
        print(f'[통합 회전교차로 생성 {self.count}] {model} · 상한 {speed}km/h', flush=True)

    def poll_prepare(self):
        self.advance(self.link.states())
        self.prepared = self.count >= 1
        self.progress(self.status)
        return self.prepared

    def resume_schedule(self, now):
        self.next_due = now+self.period

    def tick(self):
        if self.cancelled() or self.count>=self.cfg['maximum_spawn_vehicle'] or self.clock()<self.next_due:
            return
        self.advance(self.link.states())


@dataclass
class SourceSchedule:
    source_id: int
    plan: object
    next_due: float
    count: int = 0
    pending: object = None
    retry_choice: object = None


class HighSpeedSection(Section):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        common = load_yaml(TOOL_ROOT/'config/roundabout.yaml')
        high = load_yaml(TOOL_ROOT/'config/high_speed.yaml')
        self.cfg = dict(common, **{k:v for k,v in high.items() if k != 'points'})
        self.cfg['models'] = dict(common['models'], **high['extra_models'])
        self.links = load_links(repo_path(common['link_map']))
        self.rng = random.Random(self.seed)
        self.period = self.rng.choice(high['periods_s'])
        self.sources = []
        for point in high['points']:
            cfg = copy.deepcopy(self.cfg)
            cfg.update(spawn_pose=point['spawn_pose'],
                       npc_links=route_between(self.links, point['start_link'], point['end_link']),
                       start_point_index=point['start_point_index'], end_point_index=point['end_point_index'])
            self.sources.append(SourceSchedule(point['id'], SimpleNamespace(cfg=cfg, period=self.period), self.clock()))
        self.tickets, self.initial_ids, self.samples = [], None, {}

    @staticmethod
    def at_start(ticket, state):
        cfg = ticket['lane'].plan.cfg
        p, spawn = state.transform.location, cfg['spawn_pose']
        return (state.actor_info.object_type == 1 and
                state.vehicle_state.current_link_info.id.value == cfg['npc_links'][0] and
                math.hypot(p.x-spawn['x'], p.y-spawn['y']) <= 8.)

    def forget_reused_id(self, aid):
        for ticket in list(self.tickets):
            if ticket['actor'] == aid:
                destroy_ticket(self.link, ticket)
                self.tickets.remove(ticket)
        super().forget_reused_id(aid)

    def track_generations(self, live, now):
        pending = [s.pending for s in self.sources if s.pending is not None]
        maximum_speed = max(self.cfg['speeds_kmh'])
        for aid, state in live.items():
            previous = self.samples.get(aid)
            if aid == self.link.ego_id or previous is None:
                continue
            t, x, y = previous
            p = state.transform.location
            bound = max(15., 2*maximum_speed/3.6*max(0.,now-t)+5.)
            if not math.hypot(p.x-x,p.y-y) > bound:
                continue
            matches = [ticket for ticket in pending if self.at_start(ticket, state)]
            if len(matches) == 1:
                self.forget_reused_id(aid)
                self.initial_ids.discard(aid)
                for ticket in pending:
                    ticket['baseline'].discard(aid)
        for aid, state in live.items():
            p = state.transform.location
            self.samples[aid] = now, p.x, p.y

    def bind(self, ticket, state):
        aid = state.actor_info.id.value
        enable_ticket(self.link, ticket, False)
        ticket['actor'] = aid
        self.link.actor_scopes[aid] = ticket['info'].client_key
        self.register(aid, 1, ticket['model'], ticket['target'], source=ticket['lane'].source_id,
                      replay=vehicle_spec(ticket['model'],ticket['target'],ticket['lane'].plan,'native',ticket['lane'].source_id))
        # Native Custom target is separate from the physical dynamics ceiling.
        # A reused body can carry a low ceiling; always overwrite it for this
        # generation. Register first so a failed setter remains cleanable.
        self.link.set_speed_limit(aid, ticket['target'])
        source = ticket['lane']
        source.count += 1
        source.pending = None
        print(f'[통합 고주로 {source.source_id} 생성 {source.count}] {ticket["model"]} · '
              f'목표 {ticket["target"]}km/h · 주기 {self.period}초', flush=True)

    def advance(self, states):
        now = self.clock()
        live = {s.actor_info.id.value: s for s in states}
        if self.initial_ids is None:
            self.initial_ids = set(live)
        else:
            self.initial_ids.intersection_update(live)
        for source in self.sources:
            if source.pending:
                source.pending['baseline'].intersection_update(live)
        self.track_generations(live, now)
        for ticket in list(self.tickets):
            aid = ticket['actor']
            if aid is None or aid in live:
                ticket['missing_since'] = None
            elif ticket['missing_since'] is None:
                ticket['missing_since'] = now
            elif now-ticket['missing_since'] >= 1.:
                destroy_ticket(self.link, ticket)
                self.tickets.remove(ticket)
                super().forget_reused_id(aid)
        messages = []
        for source in self.sources:
            if self.cancelled():
                break
            if source.pending:
                matches = ticket_candidates(source.pending, states, set(self.owned())|self.initial_ids)
                if len(matches) > 1:
                    enable_ticket(self.link, source.pending, False)
                    raise RuntimeError('고주로 신규 차량 ID 대응 모호: 생성 중단')
                if not matches:
                    elapsed = now-source.pending['requested_at']
                    messages.append(f'{source.source_id}: MORAI 생성/ID 대응 대기 {elapsed:.1f}초')
                    continue
                self.bind(source.pending, matches[0])
            if source.count >= self.cfg['maximum_spawn_vehicle'] or now < source.next_due:
                continue
            model, speed = source.retry_choice or (self.rng.choice(list(self.cfg['models'])), self.rng.choice(self.cfg['speeds_kmh']))
            try:
                ticket = create_ticket(self.link, source, model, speed)
            except SourcePlacementDeferred:
                source.retry_choice = model, speed
                source.next_due = now+2.
                messages.append(f'{source.source_id}: 생성점 배치 거부 · 재시도 대기')
                continue
            source.retry_choice = None
            ticket.update(baseline=set(live), requested_at=now)
            self.tickets.append(ticket)
            source.pending = ticket
            source.next_due = now+self.period
            enable_ticket(self.link, ticket, True)
        self.status = ' / '.join(messages) or f'누적 {sum(s.count for s in self.sources)}대 · 주기 {self.period}초'

    def poll_prepare(self):
        self.advance(self.link.states())
        self.prepared = all(s.count >= 1 for s in self.sources)
        self.progress(self.status)
        return self.prepared

    def resume_schedule(self, now):
        for source in self.sources:
            source.next_due = now+self.period

    def tick(self):
        self.advance(self.link.states())

    def cleanup(self):
        for ticket in self.tickets:
            enable_ticket(self.link, ticket, False)
        states = self.link.states()
        if self.initial_ids is not None:
            self.track_generations({s.actor_info.id.value:s for s in states}, self.clock())
        for ticket in self.tickets:
            if ticket['actor'] is not None:
                continue
            matches = ticket_candidates(ticket, states, set(self.owned())|(self.initial_ids or set()))
            if len(matches) > 1:
                raise RuntimeError('고주로 종료 중 신규 차량 소유권 미확정')
            if matches:
                self.bind(ticket, matches[0])
        self.delete_actors()
        for ticket in list(self.tickets):
            destroy_ticket(self.link, ticket)
            self.tickets.remove(ticket)
        self.link.disconnect()
