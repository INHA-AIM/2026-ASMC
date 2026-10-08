#!/usr/bin/env python3
"""Check fresh three-view raw-label outputs independently of the legacy front contract."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import threading
import time

from b0_measurements import output_report

LABELS = frozenset(('person', 'dog', 'bicycle', 'car', 'motorcycle', 'bus', 'truck'))
VIEWS = ('front', 'left', 'right')


def check_view(rows, begin, end, minimum):
    report = output_report(rows, begin, end, expected_seconds=end-begin)
    failures = []
    if not report['complete'] or not report['five_second_hz']:
        failures.append('incomplete measurement')
    elif any(hz < minimum for hz in report['five_second_hz']):
        failures.append('new output rate')
    for key in ('duplicate_stamps', 'reversed_stamps', 'input_anchor_missing'):
        if report[key]: failures.append(key)
    if report['valid_messages'] != report['messages']: failures.append('invalid output')
    if report['max_gap_sec'] > .25: failures.append('output gap')
    latency = report['input_to_output_wall_ms']
    if latency['p95'] is None or latency['p95'] > 150 or latency['p99'] > 200:
        failures.append('input-to-output wall latency')
    return dict(passed=not failures, failures=failures, measurements=report)


def main():
    import rosgraph
    import rospy
    from asmc_msgs.msg import CameraObjectBoxArray
    from sensor_msgs.msg import CompressedImage
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--minimum-hz', type=float, default=9.5)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    if a.seconds < 5 or a.minimum_hz <= 0: p.error('at least five seconds and a positive rate required')
    rospy.init_node('verify_coco7_runtime', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    anchors = {}; rows = defaultdict(list); bad = []; lock = threading.RLock()
    def receive_input(view, msg):
        with lock:
            anchors[view, msg.header.stamp.to_nsec()] = time.monotonic()
            if len(anchors) > 12000:
                for key in list(anchors)[:3000]: anchors.pop(key)
    def receive_output(msg):
        now = time.monotonic(); stamp = msg.header.stamp.to_nsec()
        view = msg.header.frame_id.replace('camera_', '', 1)
        with lock:
            if msg.header.frame_id not in tuple('camera_'+v for v in VIEWS):
                bad.append('unexpected frame_id '+msg.header.frame_id); return
            if stamp == 0: bad.append('zero source stamp')
            if msg.image_width <= 0 or msg.image_height <= 0: bad.append('missing image dimensions')
            if any(b.class_name != b.source_class_name or b.source_class_name not in LABELS for b in msg.boxes):
                bad.append('output does not preserve raw COCO7 labels')
            anchor = anchors.get((view, stamp))
            rows[view].append(dict(arrival=now, stamp_ns=stamp, valid=msg.valid,
                source_age_ms=(rospy.Time.now().to_nsec()-stamp)/1e6,
                wall_latency_ms=(now-anchor)*1000 if anchor is not None else None,
                input_anchor='matched' if anchor is not None else 'missing'))
    subs = [rospy.Subscriber(topic, CompressedImage, lambda m,v=view:receive_input(v,m),
            queue_size=100, buff_size=2**24) for view,topic in zip(VIEWS,
            ('/image_jpeg/compressed','/cam_left/image_jpeg/compressed','/cam_right/image_jpeg/compressed'))]
    subs.append(rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray, receive_output, queue_size=100))
    # Exclude ROS subscription handshakes and anchors collected after a result.
    time.sleep(1.)
    begin = time.monotonic(); end = begin+a.seconds
    with lock: rows.clear(); bad.clear()
    while time.monotonic() < end and not rospy.is_shutdown(): time.sleep(.05)
    actual_end = min(time.monotonic(), end)
    with lock:
        result = dict(views={v:check_view(rows[v],begin,actual_end,a.minimum_hz) for v in VIEWS},
                      interface_failures=sorted(set(bad)), expected_seconds=a.seconds,
                      elapsed_seconds=actual_end-begin)
    publishers = next((names for topic,names in master.getSystemState()[0]
                       if topic=='/perception/camera/objects'), [])
    result['publishers'] = publishers
    result['passed'] = (len(publishers)==1 and not bad and actual_end-begin >= a.seconds
                        and all(r['passed'] for r in result['views'].values()))
    for sub in subs: sub.unregister()
    text = json.dumps(result, indent=2)+'\n'
    if a.output: a.output.write_text(text)
    print(text)
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
