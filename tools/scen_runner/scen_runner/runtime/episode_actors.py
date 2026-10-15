"""Typed ownership for one episode section, independent of old runner sessions."""
from dataclasses import dataclass
import copy
import time
import uuid

from scen_runner.morai.episode_link import SceneLink


@dataclass
class OwnedActor:
    request_id: str
    native_id: str
    kind: int
    model: str
    target_kmh: float = 0.
    born_at: float = 0.
    source: str = ''
    replay_key: str = ''


class Section:
    def __init__(self, name, seed, scope, base, cancelled, progress, clock=time.monotonic):
        self.name, self.seed, self.scope = name, seed, scope
        self.base, self.cancelled, self.progress, self.clock = base, cancelled, progress, clock
        self.link = SceneLink(base)
        self.link.client_key = scope
        self.link.connect()
        self.actors = []
        self.replay_history = []
        self.prepared = False
        self.known_ids = set()
        self.status = '준비 중'

    def register(self, aid, kind, model, target=0., native=None, source='', replay=None):
        actor = OwnedActor(aid, aid if native is None and kind != 3 else native,
                           kind, model, target, self.clock(), str(source), uuid.uuid4().hex[:16])
        if replay is not None:
            self.replay_history.append(dict(key=actor.replay_key, native_id=actor.native_id,
                born_at=actor.born_at, kind=kind, model=model, target_kmh=target, source=str(source),
                definition=copy.deepcopy(replay)))
        self.actors.append(actor)
        return actor

    def map_statics(self):
        for actor in self.actors:
            if actor.kind == 3 and actor.native_id is None:
                actor.native_id = self.link.static_native_id(actor.request_id)
                for record in self.replay_history:
                    if record['key']==actor.replay_key:record['native_id']=actor.native_id

    def owned(self):
        return {a.native_id: (a.model, a.target_kmh) for a in self.actors if a.native_id}

    def actor_details(self):
        return {a.native_id: dict(section=self.name, model=a.model, target=a.target_kmh,
                born_at=a.born_at, source=a.source, kind=a.kind) for a in self.actors if a.native_id}

    def crossing_details(self):
        return {}

    def ready_live(self, states):
        live = {s.actor_info.id.value: s.actor_info.object_type for s in states}
        return bool(self.actors) and all(a.native_id and live.get(a.native_id) == a.kind for a in self.actors)

    def forget_reused_id(self, aid):
        self.actors = [a for a in self.actors if a.native_id != aid]
        self.link.actor_scopes.pop(aid, None)
        self.known_ids.discard(aid)

    def resume_schedule(self, now):
        pass

    def tick(self):
        pass

    def delete_actors(self):
        if not self.actors:return

        native_pending=set()
        def present_ids(track=False):
            # Static deletion uses the registered alias, not the collision ID
            # in the global cache. Missing/reused numeric IDs do not establish
            # that an alias-owned obstacle has been deleted.
            scoped = {s.actor_info.id.value for s in self.link.scoped_statics()} if any(
                a.kind == 3 for a in self.actors) else set()
            live = {(s.actor_info.id.value,s.actor_info.object_type) for s in self.link.states()} if any(
                a.native_id for a in self.actors) else set()
            if track:
                native_pending.update(a.request_id for a in self.actors if a.kind==3 and
                    a.request_id in scoped and (a.native_id,a.kind) in live)
            return {a.request_id for a in self.actors if
                    (a.request_id in scoped or a.request_id in native_pending and (a.native_id,a.kind) in live
                     if a.kind == 3 else (a.native_id,a.kind) in live)}

        present = present_ids(track=True)
        scopes = dict(self.link.actor_scopes)
        for actor in self.actors:
            if actor.request_id in present:
                # An acknowledgement alone does not prove physical deletion.
                # Even a refused request may race native destination deletion;
                # decide from the following successful state query.
                try:self.link.destroy(actor.request_id,object_type=actor.kind)
                finally:
                    # Retain the registration scope through partial failure or
                    # an acknowledgement whose deletion is not yet visible.
                    if actor.request_id in scopes:
                        self.link.actor_scopes[actor.request_id]=scopes[actor.request_id]
        deadline = time.monotonic()+1.
        while True:
            pending = present_ids()
            for actor in list(self.actors):
                if actor.request_id not in pending:
                    self.actors.remove(actor)
                    self.known_ids.discard(actor.native_id)
                    self.link.actor_scopes.pop(actor.request_id,None)
                elif actor.request_id in scopes:
                    self.link.actor_scopes[actor.request_id]=scopes[actor.request_id]
            if not pending:
                print(f'[객체 정리 확인] {self.name} · 삭제/이미 삭제 확인',flush=True)
                return
            if time.monotonic()>=deadline:
                raise RuntimeError(f'{self.name}: 객체 삭제 미확인 ID={",".join(sorted(pending))} · '
                                   '소유권 유지 · 다음 회차 생성 중단')
            time.sleep(.05)

    def cleanup(self):
        self.delete_actors()
        self.link.disconnect()
