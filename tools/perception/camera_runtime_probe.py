#!/usr/bin/env python3
"""Benchmark-only launch prefix: in-memory callback-to-publish timing, no trace IO.

The same prefix instruments the frozen R0 and both candidates. Input receipt is
the entry to the actual model (or shared broker) callback, not a separate ROS
subscriber and not the sensor header's clock. Production launch never uses it.
"""
import argparse
import atexit
import gc
from collections import OrderedDict
import json
import hashlib
from pathlib import Path
import runpy
import sys
import threading
import time


def install(output):
    import rospy
    subscriber, publisher = rospy.Subscriber, rospy.Publisher
    receipts, rows, lock = OrderedDict(), [], threading.RLock()

    def gc_event(phase, info):
        # Diagnose rare whole-process pauses without changing GC policy. Only
        # generation2 is recorded; no lock, payload copy, or IO in this hook.
        if info['generation']==2:
            rows.append(dict(event='gc',phase=phase,generation=2,at=time.monotonic()))
    gc.callbacks.append(gc_event)

    def subscribe(name, data_class, callback=None, callback_args=None, **kwargs):
        if data_class.__name__ != 'CompressedImage' or callback is None:
            return subscriber(name, data_class, callback, callback_args, **kwargs)
        def receive(message):
            now = time.monotonic()
            with lock:
                key = (message.header.frame_id, message.header.stamp.to_nsec())
                receipts[key] = (now, message.header.seq)
                if len(receipts) > 20000:
                    receipts.popitem(last=False)
                rows.append(dict(event='input', at=now, frame=key[0], stamp=key[1], topic=name))
            return callback(message, callback_args) if callback_args is not None else callback(message)
        return subscriber(name, data_class, receive, **kwargs)

    class MeasuredPublisher:
        def __init__(self, original, name):
            self.original, self.name = original, name
        def __getattr__(self, key):
            return getattr(self.original, key)
        def publish(self, message):
            result = self.original.publish(message)
            now = time.monotonic()
            if hasattr(message, 'header'):
                key = (message.header.frame_id, message.header.stamp.to_nsec())
                with lock:
                    receipt = receipts.get(key)
                    received = receipt[0] if receipt else None
                    rows.append(dict(event='output', at=now, frame=key[0], stamp=key[1], topic=self.name,
                        valid=bool(getattr(message, 'valid', True)), stale=bool(getattr(message, 'stale', False)),
                        classification=(dict(state=message.state,label=message.label,confidence=message.confidence,
                            candidate_labels=list(message.candidate_labels),candidate_confidences=list(message.candidate_confidences))
                            if self.name == '/perception/traffic_light/detection' else None),
                        received=received, source_sequence=receipt[1] if receipt else None,
                        payload_sha256=hashlib.sha256(bytes(message.data)).hexdigest() if hasattr(message, 'encoding') else None,
                        wall_ms=(now-received)*1000 if received is not None else None))
            return result

    def advertise(name, data_class, **kwargs):
        original = publisher(name, data_class, **kwargs)
        measured = name == '/perception/camera/objects' or name == '/perception/traffic_light/detection' or (
            name.startswith('/perception/camera/') and name.endswith('/road_marking/class_id'))
        return MeasuredPublisher(original, name) if measured else original

    def save():
        gc.callbacks.remove(gc_event)
        output.parent.mkdir(parents=True, exist_ok=True)
        with lock, output.open('w') as stream:
            for row in rows:
                stream.write(json.dumps(row, separators=(',', ':')) + '\n')
    rospy.Subscriber, rospy.Publisher = subscribe, advertise
    atexit.register(save)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--visual-codecs', action='store_true', help='Count debug JPEG calls without instrumenting model outputs')
    parser.add_argument('--lane-fit-profile', action='store_true', help='Diagnostic cProfile of the fitting worker; not a timing acceptance run')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('node executable required')
    if args.lane_fit_profile:
        import cProfile
        from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode
        original = LaneVectorizerNode._process
        profile = cProfile.Profile()
        LaneVectorizerNode._process = lambda self,frames:profile.runcall(original,self,frames)
        atexit.register(lambda:profile.dump_stats(str(args.output)))
    elif args.visual_codecs:
        install_codecs(args.output)
    else:
        install(args.output)
    sys.argv = command
    runpy.run_path(command[0], run_name='__main__')


def install_codecs(output):
    import cv2
    rows, lock = [], threading.Lock()
    for name, event in (('imdecode','decode'),('imencode','encode')):
        original = getattr(cv2,name)
        def measured(*args, _original=original, _event=event, **kwargs):
            start = time.monotonic()
            try:
                return _original(*args,**kwargs)
            finally:
                end = time.monotonic()
                with lock:
                    rows.append(dict(event=_event,at=end,duration_ms=(end-start)*1000))
        setattr(cv2,name,measured)
    def save():
        output.parent.mkdir(parents=True,exist_ok=True)
        with lock,output.open('w') as stream:
            for row in rows:
                stream.write(json.dumps(row,separators=(',',':'))+'\n')
    atexit.register(save)


if __name__ == '__main__':
    main()
