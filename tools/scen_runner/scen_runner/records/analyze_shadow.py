"""Stream an S1 bag into decision evidence, without replaying control logic.

All correlations use recorded receipt/header ROS times. Selector diagnostics
are sampled (1 Hz), so this report does not assert exact command arbitration.
Run under the ROS Python environment: python3 -m scen_runner.records.analyze_shadow.
"""
import argparse
from collections import Counter, OrderedDict
import html
import json
from pathlib import Path


class TopicStats:
    def __init__(self):
        self.count = self.regressions = self.duplicate_stamps = 0
        self.first = self.last = self.stamp = None
        self.maximum_gap_s = 0.

    def observe(self, receipt, stamp=None):
        self.count += 1
        if self.first is None:
            self.first = receipt
        if self.last is not None:
            self.maximum_gap_s = max(self.maximum_gap_s, receipt-self.last)
        self.last = receipt
        if stamp is not None and stamp > 0:
            if self.stamp is not None:
                self.duplicate_stamps += stamp == self.stamp
                self.regressions += stamp < self.stamp
            self.stamp = stamp

    def summary(self):
        elapsed = (self.last-self.first) if self.count > 1 else 0.
        return dict(count=self.count, receipt_hz=(self.count-1)/elapsed if elapsed > 0 else None,
                    maximum_receipt_gap_s=self.maximum_gap_s,
                    duplicate_source_stamps=self.duplicate_stamps, regressing_source_stamps=self.regressions)


def parse_selector(value):
    return dict(token.split('=', 1) for token in value.split() if '=' in token)


def analyze(directory):
    import rosbag
    from scen_runner.records.sensor_recording import REQUIRED_TOPICS, write_json
    directory = Path(directory)
    output = directory/'analysis'
    output.mkdir(exist_ok=False)
    stats, reasons, box_reasons, selectors = {}, Counter(), Counter(), Counter()
    images, pending, cases = OrderedDict(), OrderedDict(), []
    case_counts = Counter()
    last_selector = None
    latest_selector = {}
    logs = output/'decisions.jsonl'

    def emit(stream, record):
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')

    def gallery(stamp, result):
        reason = result.reason
        if result.valid or case_counts[reason] >= 2 or len(cases) >= 32 or stamp not in images:
            return
        frame = images[stamp]
        # Exact source identity only; no nearest-frame substitution.
        extension = '.png' if 'png' in frame.image.format.lower() else '.jpg'
        name = 'case-%02d%s' % (len(cases)+1, extension)
        (output/name).write_bytes(bytes(frame.image.data))
        boxes = [dict(label=b.label, bbox=[b.xmin,b.ymin,b.xmax,b.ymax],
                      assignment=int(result.assignments[i]), reason=result.box_reasons[i])
                 for i,b in enumerate(result.boxes)]
        cases.append(dict(image=name, source_stamp_ns=stamp, reason=reason, boxes=boxes))
        case_counts[reason] += 1

    with logs.open('x', encoding='utf-8') as stream:
        # rosbag split files are timestamped; process in file order, never join
        # unrelated runs or select future sensor state for an earlier decision.
        for path in sorted((directory/'ros').glob('*.bag')):
            with rosbag.Bag(str(path)) as bag:
                for topic, msg, receipt in bag.read_messages():
                    t = receipt.to_sec()
                    stamp = msg.header.stamp if hasattr(msg, 'header') else None
                    stats.setdefault(topic, TopicStats()).observe(t, stamp.to_sec() if stamp else None)
                    if topic == '/planning/control_selector/diagnostics':
                        latest_selector = parse_selector(msg.data)
                        key = (latest_selector.get('mode'), latest_selector.get('selected_source'), latest_selector.get('command_fresh'))
                        selectors[str(key)] += 1
                        if key != last_selector:
                            emit(stream, dict(receipt=t, topic=topic, selector=latest_selector))
                            last_selector = key
                    elif topic == '/planning/camera_bbox_lane_change/diagnostics':
                        for status in msg.status:
                            values = {v.key:v.value for v in status.values}
                            reasons[status.message] += 1
                            emit(stream, dict(receipt=t, topic=topic, reason=status.message, values=values,
                                              last_selector_sample=latest_selector))
                    elif topic == '/planning/camera_bbox_lane/result':
                        key = msg.header.stamp.to_nsec()
                        boxes = [dict(label=b.label, bbox=[b.xmin,b.ymin,b.xmax,b.ymax],
                                      assignment=int(msg.assignments[i]), reason=msg.box_reasons[i])
                                 for i,b in enumerate(msg.boxes)]
                        for b in boxes:
                            box_reasons[b['label']+' | '+b['reason'].split(':')[0]] += 1
                        emit(stream, dict(receipt=t, topic=topic, source_stamp_ns=key, valid=msg.valid,
                                          reason=msg.reason, lane_stamp_ns=msg.lane_stamp.to_nsec(), boxes=boxes))
                        if not msg.valid:
                            if key in images:gallery(key,msg)
                            else:pending[key]=msg
                        while len(pending)>64:pending.popitem(last=False)
                    elif topic == '/planning/camera_bbox_lane/frame':
                        key = msg.header.stamp.to_nsec()
                        images[key] = msg
                        if key in pending:gallery(key,pending.pop(key))
                        while len(images)>64:images.popitem(last=False)
                    elif topic == '/rosout_agg' and msg.level >= 4:
                        emit(stream, dict(receipt=t, topic=topic, node=msg.name, message=msg.msg))

    summary = dict(schema='asmc.shadow_trial_analysis.v1', topics={k:v.summary() for k,v in stats.items()},
                   missing_required=[k for k in REQUIRED_TOPICS if k not in stats],
                   follower_reason_samples=dict(reasons), bbox_reason_samples=dict(box_reasons),
                   selector_samples=dict(selectors), gallery_cases=cases,
                   interpretation='Counts are message samples, not independent failures. Receipt gaps include startup/reset. '
                   'Last selector sample is diagnostic context, not per-command arbitration proof. '
                   'Gallery uses exact bbox/frame stamp; no independently measured geometry GT.')
    write_json(output/'summary.json', summary)
    cards=''.join('<article><h2>'+html.escape(c['reason'])+'</h2><img width="960" src="'+c['image']+'"><pre>'+
                  html.escape(json.dumps(c['boxes'],ensure_ascii=False,indent=2))+'</pre></article>' for c in cases)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Shadow trial evidence</title>'
        '<style>body{background:#161b22;color:#eee;font:16px sans-serif;margin:24px}img{max-width:100%}'
        'pre{white-space:pre-wrap}article{border-top:1px solid #555;padding:16px 0}</style>'
        '<h1>음영구간 판단 기록</h1><p>동일 stamp의 원본 영상과 실제 bbox 판정입니다. '
        '시나리오 재현은 동일 센서 프레임의 재생을 보장하지 않습니다.</p><a href="summary.json">요약 JSON</a>'+cards,
        encoding='utf-8')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    args=parser.parse_args()
    result=analyze(args.run_directory)
    print(json.dumps({k:result[k] for k in ('missing_required','follower_reason_samples','bbox_reason_samples')},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
