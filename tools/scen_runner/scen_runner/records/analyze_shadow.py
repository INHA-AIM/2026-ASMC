"""Stream an S1 bag into decision evidence, without replaying control logic.

All correlations use recorded receipt/header ROS times. Selector diagnostics
are sampled (1 Hz), so this report does not assert exact command arbitration.
Run under the ROS Python environment: python3 -m scen_runner.records.analyze_shadow.
"""
import argparse
from collections import Counter, OrderedDict
import html
import math
import json
from pathlib import Path


class TopicStats:
    def __init__(self):
        self.count = self.regressions = self.duplicate_stamps = 0
        self.first = self.last = None
        self.stamps = {}
        self.untracked_stream_samples = 0
        self.maximum_gap_s = 0.

    def observe(self, receipt, stamp=None, stream="default"):
        self.count += 1
        if self.first is None:
            self.first = receipt
        if self.last is not None:
            self.maximum_gap_s = max(self.maximum_gap_s, receipt-self.last)
        self.last = receipt
        if stamp is not None and stamp > 0:
            # Three views interleave on one topic. Compare source order only
            # within a view/session/generation, never left against front.
            if stream not in self.stamps and len(self.stamps)>=128:
                self.untracked_stream_samples+=1
                return
            previous=self.stamps.get(stream)
            if previous is not None:
                self.duplicate_stamps += stamp == previous
                self.regressions += stamp < previous
            self.stamps[stream] = stamp

    def summary(self):
        elapsed = (self.last-self.first) if self.count > 1 else 0.
        return dict(count=self.count, receipt_hz=(self.count-1)/elapsed if elapsed > 0 else None,
                    maximum_receipt_gap_s=self.maximum_gap_s,
                    duplicate_source_stamps=self.duplicate_stamps, regressing_source_stamps=self.regressions, source_stream_count=len(self.stamps),
                    untracked_source_stream_samples=self.untracked_stream_samples)


def parse_selector(value):
    return dict(token.split('=', 1) for token in value.split() if '=' in token)


def completed_bags(directory):
    """Never report only the finalized first split while recording is active."""
    directory=Path(directory)
    manifest=json.loads((directory/'ros/manifest.json').read_text())
    if manifest.get('status')!='complete' or list((directory/'ros').glob('*.active')):
        raise ValueError('recording must be closed and complete before analysis')
    bags=sorted((directory/'ros').glob('*.bag'))
    expected={f['file']:f['bytes'] for f in manifest.get('files',[])}
    if not bags or {p.name:p.stat().st_size for p in bags}!=expected:
        raise ValueError('finalized bag inventory differs from recording manifest')
    return bags


def analyze(directory, output_name='analysis'):
    import rosbag
    from scen_runner.records.sensor_recording import REQUIRED_TOPICS, write_json
    directory = Path(directory)
    if Path(output_name).name != output_name or output_name in ('.','..'):
        raise ValueError('analysis output must be a new subdirectory name')
    bags=completed_bags(directory)
    output = directory/output_name
    output.mkdir(exist_ok=False)
    stats, reasons, box_reasons, selectors = {}, Counter(), Counter(), Counter()
    images, pending, cases = OrderedDict(), OrderedDict(), []
    case_counts = Counter()
    collision_samples, collision_ids = 0, Counter()
    unresolved_collision_samples = 0
    contact_decisions = Counter()
    modes = []
    latest_ego = None
    last_selector = None
    latest_selector = {}
    logs = output/'decisions.jsonl'

    def emit(stream, record):
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')

    def gallery(stamp, result):
        reason = result.reason+' | '+','.join(sorted(set(result.box_reasons)))
        kind = result.reason+' | '+','.join(sorted(set(r.split(':')[0] for r in result.box_reasons)))
        if case_counts[kind] >= 2 or len(cases) >= 32 or stamp not in images:
            return
        frame = images[stamp]
        # Exact source identity only; no nearest-frame substitution.
        extension = '.png' if 'png' in frame.image.format.lower() else '.jpg'
        name = 'case-%02d%s' % (len(cases)+1, extension)
        (output/name).write_bytes(bytes(frame.image.data))
        import cv2
        import numpy as np
        decoded=cv2.imdecode(np.frombuffer(bytes(frame.image.data),np.uint8),cv2.IMREAD_COLOR)
        if decoded is None:
            return
        height,width=decoded.shape[:2]
        boxes = [dict(label=b.label, bbox=[b.xmin,b.ymin,b.xmax,b.ymax],
                      assignment=int(result.assignments[i]), reason=result.box_reasons[i])
                 for i,b in enumerate(result.boxes)]
        cases.append(dict(image=name, width=width, height=height, source_stamp_ns=stamp, reason=reason, boxes=boxes))
        case_counts[kind] += 1

    with logs.open('x', encoding='utf-8') as stream:
        # rosbag split files are timestamped; process in file order, never join
        # unrelated runs or select future sensor state for an earlier decision.
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                for topic, msg, receipt in bag.read_messages():
                    t = receipt.to_sec()
                    header=getattr(msg,'header',None)
                    identity=msg
                    if hasattr(msg,'detections'):header=msg.detections.header
                    if hasattr(msg,'observation'):
                        identity=msg.observation;header=identity.objects.header
                    stamp=header.stamp if header else None
                    stream_id=(header.frame_id if header else '',getattr(identity,'producer_session_id',''),getattr(identity,'generation',0))
                    stats.setdefault(topic, TopicStats()).observe(t, stamp.to_sec() if stamp else None,stream_id)
                    if topic == '/Ego_topic':
                        latest_ego = dict(receipt=t, x=msg.position.x, y=msg.position.y,
                            heading_deg=msg.heading, velocity_xyz_mps=[msg.velocity.x,msg.velocity.y,msg.velocity.z])
                    elif topic == '/CollisionData':
                        # UDP repeats contact state: count samples and IDs, not unique collisions.
                        reports = list(msg.collision_object) + list(msg.collision_objecta)
                        # Local MORAI emits type=-1/id=0 ego-location sentinel
                        # continuously. Keep it separate; not 4000 collisions.
                        unresolved_collision_samples += any(o.type<0 or o.unique_id<=0 for o in reports)
                        hits = [o for o in reports if o.type>=0 and o.unique_id>0]
                        if hits:
                            collision_samples += 1
                            collision_ids.update(str(o.unique_id) for o in hits)
                            emit(stream,dict(receipt=t,topic=topic,ids=[o.unique_id for o in hits]))
                    elif topic == '/planning/mode':
                        if not modes or modes[-1]['mode'] != int(msg.data):
                            modes.append(dict(receipt=t, mode=int(msg.data), last_ego=latest_ego))
                    elif topic == '/planning/control_selector/diagnostics':
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
                            contact_decisions[values.get("contact_decision", "not_in_this_revision")] += 1
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
                                          reason=msg.reason, lane_stamp_ns=msg.lane_stamp.to_nsec(), boxes=boxes,
                                          contact_projection_valid=getattr(msg,'contact_projection_valid',None),
                                          contact_support_x_max_m=getattr(msg,'contact_support_x_max_m',None),
                                          ground_contact_valid=list(getattr(msg,'ground_contact_valid',[])),
                                          ground_contact_sources=list(getattr(msg,'ground_contact_sources',[])),
                                          local_obstacles_valid=getattr(msg,'local_obstacles_valid',False),
                                          local_obstacles_stamp_ns=msg.local_obstacles_stamp.to_nsec() if hasattr(msg,'local_obstacles_stamp') else None,
                                          local_obstacle_footprints=[[[p.x,p.y,p.z] for p in poly.points] for poly in getattr(msg,'local_obstacle_footprints',[])],
                                          ground_contacts=[[[p.x,p.y,p.z] for p in poly.points]
                                              for poly in getattr(msg,'ground_contacts',[])]))
                        if not msg.valid or 5 in msg.assignments:
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

    summary = dict(schema='asmc.shadow_trial_analysis.v1', input_bags=[dict(file=p.name,bytes=p.stat().st_size) for p in bags], topics={k:v.summary() for k,v in stats.items()},
                   missing_required=[k for k in REQUIRED_TOPICS if k not in stats],
                   follower_reason_samples=dict(reasons), bbox_reason_samples=dict(box_reasons),
                   selector_samples=dict(selectors), gallery_cases=cases,
                   contact_decision_samples=dict(contact_decisions), mode_transitions=modes,
                   unresolved_collision_report_samples=unresolved_collision_samples,
                   collision_report_samples=collision_samples, collision_report_ids=dict(collision_ids),
                   interpretation='Counts are message samples, not independent failures. Receipt gaps include startup/reset. '
                   'Last selector sample is diagnostic context, not per-command arbitration proof. '
                   'Gallery uses exact bbox/frame stamp; no independently measured geometry GT.')
    write_json(output/'summary.json', summary)
    def card(c):
        rectangles=[]
        names=('UNKNOWN','OWN','ADJACENT','BOTH','OUTSIDE','DEFERRED_FAR')
        for box in c['boxes']:
            x,y,x2,y2=box['bbox'];assignment=box['assignment']
            label=html.escape(box['label']+' / '+(names[assignment] if assignment<len(names) else str(assignment)))
            color='#ffb454' if assignment in (0,5) else '#71f7a2'
            rectangles.append(f'<rect x="{x}" y="{y}" width="{max(0,x2-x)}" height="{max(0,y2-y)}" '
                              f'fill="none" stroke="{color}" stroke-width="2"/><text x="{x}" y="{max(14,y-4)}" '
                              f'fill="{color}" stroke="#111" stroke-width=".3" font-size="15">{label}</text>')
        return '<article><h2>'+html.escape(c['reason'])+f'</h2><svg viewBox="0 0 {c["width"]} {c["height"]}"><image href="{c["image"]}" '+\
               f'width="{c["width"]}" height="{c["height"]}"/>'+''.join(rectangles)+'</svg><pre>'+\
               html.escape(json.dumps(c['boxes'],ensure_ascii=False,indent=2))+'</pre></article>'
    cards=''.join(card(c) for c in cases)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Shadow trial evidence</title>'
        '<style>body{background:#161b22;color:#eee;font:16px sans-serif;margin:24px}svg{width:100%;max-width:1280px}'
        'pre{white-space:pre-wrap}article{border-top:1px solid #555;padding:16px 0}</style>'
        '<h1>음영구간 판단 기록</h1><p>동일 stamp의 원본 영상과 실제 bbox 판정입니다. '
        '시나리오 재현은 동일 센서 프레임의 재생을 보장하지 않습니다.</p><a href="summary.json">요약 JSON</a>'+cards,
        encoding='utf-8')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output-name', default='analysis', help='new run subdirectory; never overwrites earlier analysis')
    args=parser.parse_args()
    result=analyze(args.run_directory,args.output_name)
    print(json.dumps({k:result[k] for k in ('missing_required','follower_reason_samples','bbox_reason_samples')},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
