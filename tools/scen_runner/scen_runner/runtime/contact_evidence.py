"""Validate native contact IDs against the actors in the same RPC snapshot.

Distance cannot create a collision. It only rejects a reported ID whose live
body is clearly elsewhere, including IDs reused after a stale contact record.
"""
import math


def body_envelope(state):
    box=state.bounding_box
    dimensions=(box.extent.x,box.extent.y,box.extent.z)
    if not all(math.isfinite(v) and v>0 for v in dimensions):
        return None
    p=box.location
    if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
        return None
    # Circumscribed sphere is independent of the box's orientation.
    radius=math.sqrt(sum(v*v for v in dimensions))
    return (p.x,p.y,p.z),radius


def inspect_contacts(sample,owned_ids,ego_id='Ego'):
    hits=set(sample.get('contacts',())) & set(sample.get('owned_ids',())) & set(owned_ids)
    live={s.actor_info.id.value:s for s in sample.get('actor_states',())}
    ego=live.get(ego_id)
    records=[dict(id=aid,verdict='ignored',unowned=True,
                  reason='이번 회차의 관리 객체가 아닌 접촉 ID',
                  owned_at_query=aid in sample.get('owned_ids',()),owned_now=aid in owned_ids)
             for aid in sorted(set(sample.get('contacts',()))-hits)
             if aid.isdigit() and int(aid)>0]
    for aid in sorted(hits):
        actor=live.get(aid)
        row=dict(id=aid,verdict='unknown',reason='동일 조회의 ego/상대 상태 미수신')
        if ego is not None and actor is not None:
            own_box,other_box=body_envelope(ego),body_envelope(actor)
            if own_box is None or other_box is None:
                row['reason']='차체 BoundingBox 미수신'
            else:
                position,radius=own_box;other_position,other_radius=other_box
                separation=math.dist(position,other_position)
                speed=lambda s:math.sqrt(s.velocity.x*s.velocity.x+s.velocity.y*s.velocity.y+s.velocity.z*s.velocity.z)/3.6
                travel=(speed(ego)+speed(actor))*max(0.,sample.get('query_s',0.))
                # Deliberately broad: one additional body radius per actor
                # accommodates the native trigger footprint and cached pose.
                # This is an impossibility check, not an overlap-based detector.
                envelope=2*(radius+other_radius)+travel
                row.update(separation_m=separation,envelope_m=envelope,
                           ego_position=list(position),actor_position=list(other_position),
                           kind=actor.actor_info.object_type)
                if separation>envelope:
                    row.update(verdict='rejected',reason='접촉 ID의 실제 차체가 ego에서 멀리 떨어져 있음')
                else:
                    row.update(verdict='near',reason='gRPC 접촉 ID의 차체 주변 범위 확인 · 새 접촉 여부 미확인')
        records.append(row)
    return records


class ContactTransitions:
    """A persistent old ID becoming nearby is not a new native contact event.

    Quarantined IDs re-arm only after a successful query omits them. Do not
    re-arm on query failures or on distance changes. If the simulator never
    clears an old ID, genuine later contact with it is unconfirmable here.
    """
    def __init__(self):
        self.previous=set()
        self.quarantined=set()
        self.confirmed=set()
        self.latest_query=float('-inf')

    def observe(self,sample,owned_ids,ego_id,eligible):
        if sample.get('error'):return []
        started=sample.get('query_started_at')
        records=inspect_contacts(sample,owned_ids,ego_id)
        if started is None or started<self.latest_query:
            for row in records:
                row.update(verdict='ignored',reason='접촉 조회 시각 미수신 또는 순서가 뒤바뀐 응답')
            return records
        self.latest_query=started
        contacts=set(sample.get('contacts',()))-{ego_id}
        disappeared=self.previous-contacts
        self.quarantined.difference_update(disappeared)
        self.confirmed.difference_update(disappeared)
        entered=contacts-self.previous
        if not eligible:
            self.quarantined.update(contacts)
        for row in records:
            if row.get('unowned'):continue
            aid=row['id']
            row['native_enter']=aid in entered
            if row['verdict'] in ('rejected','unknown'):
                self.quarantined.add(aid)
                self.confirmed.discard(aid)
            elif not eligible:
                row.update(verdict='stale',reason='주행 시작 전부터 관찰된 접촉 기록 · 새 접촉 미확인')
            elif aid in self.quarantined:
                row.update(verdict='stale',reason='기존 접촉 ID가 계속 남아 있음 · 목록 해제 후 새 접촉 대기')
            elif aid in entered:
                self.confirmed.add(aid)
                row.update(verdict='accepted',reason='주행 중 새 gRPC 접촉 ID 진입 및 차체 주변 범위 확인')
            elif aid in self.confirmed:
                row.update(verdict='ongoing',reason='확인된 접촉 유지')
            else:
                self.quarantined.add(aid)
                row.update(verdict='stale',reason='새 접촉 진입을 확인하지 못한 기존 기록')
        self.previous=contacts
        return records


def state_evidence(state):
    p,v,box=state.transform.location,state.velocity,state.bounding_box
    return dict(id=state.actor_info.id.value,kind=state.actor_info.object_type,
                position=[p.x,p.y,p.z],velocity=[v.x,v.y,v.z],
                box_center=[box.location.x,box.location.y,box.location.z],
                box_extents=[box.extent.x,box.extent.y,box.extent.z],
                rotation=[state.transform.rotation.x,state.transform.rotation.y,state.transform.rotation.z],
                box_rotation=[box.rotation.x,box.rotation.y,box.rotation.z])
