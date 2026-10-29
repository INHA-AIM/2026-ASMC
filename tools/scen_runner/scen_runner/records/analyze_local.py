"""Bounded LOCAL-1 trace metrics. No closed-loop success inferred from a bag."""

import argparse
from collections import Counter
import json
from pathlib import Path


def read_rows(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def quantiles(values):
    x = sorted(values)
    return {
        k: x[min(len(x) - 1, round((len(x) - 1) * q))] if x else None
        for k, q in [("p50", 0.5), ("p95", 0.95), ("p99", 0.99), ("max", 1)]
    }


def analyze(root, output_name="summary.json"):
    root = Path(root)
    reasons = Counter()
    reject = Counter()
    speed = Counter()
    timing = []
    caps = []
    gaps = []
    valid_gaps = []
    last_valid = None
    last_valid_frame = None
    count = valid = assisted = dropped = discontinuities = 0
    last = None
    last_frame = None
    examples = {}
    previous = None
    for row in read_rows(root / "decisions.jsonl"):
        count += 1
        valid += bool(row["valid"])
        assisted += bool(row.get("assisted"))
        dropped = max(dropped, row.get("journal_dropped", 0))
        reasons[row["reason"]] += 1
        reject.update(row.get("rejected", {}))
        speed[row.get("speed_reason", "unavailable")] += 1
        if row["processing_ms"] > 0:
            timing.append(row["processing_ms"])
        if row["valid"]:
            caps.append(row["selected_kph"])
            if last_valid_frame == row['frame'] and last_valid is not None and row['decision_stamp'] >= last_valid:
                valid_gaps.append((row['decision_stamp']-last_valid)*1000)
            last_valid, last_valid_frame = row['decision_stamp'], row['frame']
        t = row["decision_stamp"]
        frame = row["frame"]
        if frame == last_frame and last is not None:
            if last > 0 and 0 <= t - last <= 10:
                gaps.append((t - last) * 1000)
            else:
                discontinuities += 1
        last = t
        last_frame = frame
        key = (row["reason"], next(iter(row.get("rejected", {})), ""))
        if key not in examples:
            examples[key] = row
    control = []
    if (root / "control.jsonl").exists():
        for row in read_rows(root / "control.jsonl"):
            if row.get("valid", True):
                control.append(row["processing_ms"])
    result = dict(
        plans=count,
        valid=valid,
        assisted=assisted,
        reasons=dict(reasons),
        rejections=dict(reject),
        speed_reasons=dict(speed),
        planning_ms=quantiles(timing),
        selected_kph=quantiles(caps),
        plan_gap_ms=quantiles(gaps),
        plan_decision_gap_ms=quantiles(gaps),
        valid_plan_gap_ms=quantiles(valid_gaps),
        plan_gap_definition="consecutive decision records, including invalid; not actuator command gap",
        clock_discontinuities=discontinuities,
        mpc_ms=quantiles(control),
        journal_dropped=dropped,
        examples=list(examples.values()),
        acceptance="analysis only; closed-loop and physical contact require simulator evidence",
    )
    # Detail-OFF diagnostics are sampled at1Hz. Supply metrics use every
    # LocalTrajectory delivery, not the number of detailed JSON records.
    result['decision_records'] = count
    deliveries = root / 'deliveries.jsonl'
    if deliveries.exists():
        delivered = [json.loads(line) for line in deliveries.read_text().splitlines()]
        result['plans'] = len(delivered)
        result['valid'] = sum(bool(row['valid']) for row in delivered)
        result['reasons'] = dict(Counter(row['reason'] for row in delivered))
        def gaps_of(rows):
            times = [row['received_steady'] for row in rows]
            return quantiles([(b-a)*1000 for a,b in zip(times,times[1:]) if b>=a])
        result['plan_decision_gap_ms'] = gaps_of(delivered)
        result['valid_plan_gap_ms'] = gaps_of([row for row in delivered if row['valid']])
        result['delivery_metric_clock'] = 'monotonic subscriber receipt; command delivery is measured separately'
        events = root / 'events.jsonl'
        if events.exists():
            markers = {r['event']:r['monotonic'] for r in read_rows(events)
                       if r['event'] in ('driving_started','driving_ended')}
            if len(markers)==2:
                begin,end=markers['driving_started'],markers['driving_ended']
                valid_times=[r['received_steady'] for r in delivered if r['valid'] and begin<=r['received_steady']<=end]
                edges=[begin,*valid_times,end]
                result['driving_valid_plan_max_silence_ms']=max((b-a)*1000 for a,b in zip(edges,edges[1:]))
                result['driving_valid_plan_hz']=len(valid_times)/max(1e-6,end-begin)
                # NORMAL intentionally publishes no LOCAL plan. Keep the full
                # trial silence above, but never score NORMAL idle as a LOCAL
                # outage in an integrated transition experiment.
                spans=[];active=None
                for event in read_rows(events):
                    if event['event']!='mode': continue
                    if event['value']==1 and active is None:
                        active=max(begin,event['monotonic'])
                    elif event['value']==0 and active is not None:
                        if min(end,event['monotonic'])>active:
                            spans.append((active,min(end,event['monotonic'])))
                        active=None
                if active is not None and end>active: spans.append((active,end))
                silences=[]
                for a,b in spans:
                    edges=[a,*[t for t in valid_times if a<=t<=b],b]
                    silences.extend((r-l)*1000 for l,r in zip(edges,edges[1:]))
                result['active_local_valid_plan_max_silence_ms']=max(silences) if silences else None
                result['active_local_seconds']=sum(b-a for a,b in spans)
    timings = root / 'timings.jsonl'
    if timings.exists():
        rows = [json.loads(line) for line in timings.read_text().splitlines()]
        valid_rows = [row for row in rows if row[1]]
        result['timing_sample_count'] = len(rows)
        for name, index in [('input_lock_wait_ms',2),('input_lock_hold_ms',3),
                ('critical_path_ms',4),('debug_geometry_ms',5),('serialization_ms',6),('index_ms',7)]:
            result[name] = quantiles([row[index] for row in valid_rows])
    for name, filename in [('mpc_command_receive_gap_ms','mpc-commands.jsonl'),
                            ('wire_command_receive_gap_ms','wire.jsonl')]:
        file = root / filename
        rows = [json.loads(line) for line in file.read_text().splitlines()] if file.exists() else []
        result[name] = quantiles([(b['received_steady']-a['received_steady'])*1000 for a,b in zip(rows,rows[1:])])
        # Startup/cleanup are not steady driving. Preserve the full metric above
        # and report the explicitly marked drive window separately.
        events = root / 'events.jsonl'
        if events.exists():
            markers = {r['event']:r['monotonic'] for r in map(json.loads,events.read_text().splitlines())
                       if r['event'] in ('driving_started','driving_ended')}
            if len(markers) == 2:
                active = [r for r in rows if markers['driving_started'] <= r['received_steady'] <= markers['driving_ended']]
                result['driving_'+name] = quantiles([(b['received_steady']-a['received_steady'])*1000 for a,b in zip(active,active[1:])])
    resources = root / 'resources.jsonl'
    if resources.exists():
        by_process = {}
        for row in map(json.loads, resources.read_text().splitlines()):
            for proc in row['processes']:
                by_process.setdefault(proc['name'], []).append(proc['rss_kib'])
        result['rss_kib'] = {name:dict(first=values[0], last=values[-1], maximum=max(values), samples=len(values))
                             for name,values in by_process.items()}
    target = root / output_name
    with target.open("x") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    return {k: v for k, v in result.items() if k != "examples"}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    print(json.dumps(analyze(p.parse_args().root), ensure_ascii=False, indent=2))
