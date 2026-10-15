"""Bounded consumer journals and timings; large reports are loaded after ACTIVE."""
import gc
import json
import sys
import threading
import time
from camera_runtime_probe import measurement_journal


class RecordedRows:
    def __init__(self, path):
        self.path = path
        self.journal = measurement_journal(path)

    def append(self, row):
        self.journal.append(dict(event='consumer_record', record=row))

    def finish(self):
        self.journal.close()
        with self.path.open() as stream:
            return [row['record'] for row in map(json.loads, stream) if row['event']=='consumer_record']


def instrument_consumer(path, family_class):
    journal = measurement_journal(path)
    original = family_class.deserialize
    def deserialize(self, data):
        began, cpu = time.monotonic(), time.thread_time()
        result = original(self, data)
        journal.append(dict(event='family_deserialize',at=time.monotonic(),began=began,
                            stamp=self.header.stamp.to_nsec(),wall_ms=1000*(time.monotonic()-began),
                            cpu_ms=1000*(time.thread_time()-cpu),bytes=len(data)))
        return result
    family_class.deserialize = deserialize
    def observe_gc(phase, info):
        if info['generation']!=2:
            return
        caller=sys._getframe(1)
        event=dict(event='gc',phase=phase,generation=2,at=time.monotonic(),
                   thread=threading.current_thread().name,caller_file=caller.f_code.co_filename,
                   caller_function=caller.f_code.co_name,collected=info.get('collected'))
        del caller
        journal.append(event)
    gc.callbacks.append(observe_gc)
    def close():
        gc.callbacks.remove(observe_gc)
        family_class.deserialize=original
        journal.close()
    return close
