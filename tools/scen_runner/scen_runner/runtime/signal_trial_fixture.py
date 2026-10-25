"""Explicit, reversible stationary-prop relocation for isolated signal trials.

Never deletes, respawns or changes the classification of an existing actor.
The original snapshot is emitted before the first write and retained if either
relocation or restoration fails. Only explicitly named stationary props qualify.
"""
import math
import time


class StaticFixture:
    def __init__(self,link,emit):
        self.link=link;self.emit=emit;self.original={}

    def state(self,aid):
        matches=[s for s in self.link.states() if s.actor_info.id.value==aid]
        if len(matches)!=1 or matches[0].actor_info.object_type!=3:
            raise RuntimeError('fixture requires one existing static prop: '+aid)
        state=matches[0]
        if math.sqrt(sum(getattr(state.velocity,k)**2 for k in ('x','y','z')))>.1:
            raise RuntimeError('fixture refuses a moving actor: '+aid)
        t=state.transform
        return dict(pose=dict(x=t.location.x,y=t.location.y,z=t.location.z,
            roll=t.rotation.x,pitch=t.rotation.y,yaw=t.rotation.z))

    def verify(self,aid,expected):
        actual=self.state(aid)['pose'];wanted=expected['pose']
        if math.dist([actual[k] for k in ('x','y','z')],[wanted[k] for k in ('x','y','z')])>.05 or \
           any(abs(math.remainder(actual[k]-wanted[k],360))>1 for k in ('roll','pitch','yaw')):
            raise RuntimeError('fixture pose readback mismatch: '+aid)
        return actual

    def place(self,aid,expected):
        # This build's SetTransform yaw and ActorState yaw differ by local
        # ENU/grid convergence. Close the loop against actual readback, as the
        # existing Ego reset does; never accumulate that offset across trials.
        command=dict(pose=dict(expected['pose']))
        for attempt in range(4):
            self.link.restore_actor(aid,3,command);time.sleep(.1)
            actual=self.state(aid)['pose']
            error=math.remainder(actual['yaw']-expected['pose']['yaw'],360)
            if abs(error)<=.1:return self.verify(aid,expected)
            command['pose']['yaw']-=error
        return self.verify(aid,expected)

    def park(self,aid,dx,dy):
        if aid in self.original or not math.isfinite(dx+dy) or math.hypot(dx,dy)<10:
            raise ValueError('fixture requires a unique actor and explicit parking displacement >=10m')
        original=self.state(aid);self.original[aid]=original
        self.emit(dict(event='static_fixture_snapshot',actor=aid,original=original))
        parked=dict(pose=dict(original['pose']))
        parked['pose']['x']+=dx;parked['pose']['y']+=dy
        actual=self.place(aid,parked)
        self.emit(dict(event='static_fixture_parked',actor=aid,actual=actual))

    def restore(self):
        errors=[]
        for aid,original in list(self.original.items()):
            try:
                actual=self.place(aid,original)
                self.emit(dict(event='static_fixture_restored',actor=aid,actual=actual))
                del self.original[aid]
            except Exception as error:errors.append(str(error))
        if errors:raise RuntimeError('static fixture restoration incomplete: '+'; '.join(errors))
