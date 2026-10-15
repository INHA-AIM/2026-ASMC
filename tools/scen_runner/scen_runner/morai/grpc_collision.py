"""Read ego contacts independently of vehicle generation RPCs."""
import threading
import time
import math
from datetime import datetime, timezone

from PyQt5 import QtCore



class CollisionMonitor(QtCore.QThread):
    sample = QtCore.pyqtSignal(object)
    actors = QtCore.pyqtSignal(object)

    def __init__(self, base, client_key, link_factory=None):
        super().__init__()
        if link_factory is None:
            from scen_runner.runtime.validation_episode import EpisodeLink
            link_factory = EpisodeLink
        self.link = link_factory(base)
        self.link.client_key = client_key
        self.stopping = threading.Event()
        self.context_lock = threading.Lock()
        self.epoch = 0
        self.owned_provider = lambda: {}
        self.publish_actors = False
        self.publish_contact_evidence = False

    def reset_context(self):
        with self.context_lock:
            self.epoch += 1
            return self.epoch

    def context(self):
        with self.context_lock:
            return self.epoch

    def run(self):
        previous = None
        last_sample = None
        last_actors = None
        try:
            while not self.stopping.is_set():
                epoch = self.context()
                owned = self.owned_provider()
                owned_ids = frozenset(owned)
                started = time.monotonic()
                started_utc = datetime.now(timezone.utc).isoformat()
                try:
                    if not self.link.connected:
                        self.link.connect(timeout=1)
                    contacts = tuple(sorted(set(self.link.ego_contacts())))
                    error = None
                except Exception as exc:
                    contacts = ()
                    error = exc.details() if callable(getattr(exc,'details',None)) else str(exc)
                finished = time.monotonic()
                if epoch != self.context():
                    continue  # A response from the previous scene must not enter the next.
                if self.publish_actors and (last_actors is None or finished-last_actors>=.2):
                    records=[]
                    if error is None:
                        for state in getattr(self.link,'contact_states',[]):
                            p=state.transform.location;v=state.velocity
                            records.append(dict(id=state.actor_info.id.value,x=p.x,y=p.y,
                                                speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                                                collisions=list(state.vehicle_state.collision_objects)))
                    self.actors.emit(dict(epoch=epoch,states=records,owned=dict(owned),error=error,
                                          received_at=finished,query_s=finished-started))
                    last_actors=finished
                key = (epoch,contacts,error,owned_ids)
                # Queue each contact transition, including a short contact and its release.
                if key != previous or last_sample is None or finished-last_sample >= 1:
                    self.sample.emit(dict(epoch=epoch,contacts=contacts,error=error,
                                          query_started_at=started,received_at=finished,
                                          query_s=finished-started,owned_ids=owned_ids,
                                          query_started_utc=started_utc,received_at_utc=datetime.now(timezone.utc).isoformat(),
                                          native_timestamp=getattr(self.link,'contact_native_timestamp',None),
                                          native_tick_count=getattr(self.link,'contact_tick_count',None),
                                          actor_states=tuple(getattr(self.link,'contact_states',()))
                                          if self.publish_contact_evidence and error is None else ()))
                    previous = key
                    last_sample = finished
                self.stopping.wait(max(0.,.05-(finished-started)))
        finally:
            self.link.disconnect()
