"""Read-only handoff evidence; no simulated pose is fed back to the planner."""
import json
from pathlib import Path


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def analyze(root):
    root=Path(root)
    events=rows(root/'events.jsonl')
    wire=rows(root/'wire.jsonl')
    native=[r for r in events if r['event']=='native_sample']
    sources={0:rows(root/'normal-commands.jsonl'),1:rows(root/'mpc-commands.jsonl')}
    results=[]; previous=None
    for event in events:
        if event['event']!='mode': continue
        mode=event['value']
        if previous is None or mode==previous:
            previous=mode;continue
        previous=mode;t=event['monotonic']
        after=next((r for r in sources[mode] if r['received_steady']>=t),None)
        before=next((r for r in reversed(wire) if r['received_steady']<t),None)
        actual=next((r for r in wire if r['received_steady']>=t),None)
        state=min(native,key=lambda r:abs(r['monotonic']-t)) if native else None
        delta=actual['steering']-before['steering'] if actual and before else None
        dt=actual['received_steady']-before['received_steady'] if actual and before else None
        results.append(dict(to_mode='LOCAL' if mode else 'NORMAL',stamp=event['ros_time'],
            first_source_command_ms=(after['received_steady']-t)*1000 if after else None,
            full_stop_ticks_first150ms=sum(r['brake']>=.95 for r in wire if t<=r['received_steady']<=t+.15),
            steering_change_rad=delta,command_interval_s=dt,
            steering_rate_rad_s=abs(delta)/dt if dt and dt>0 else None,
            native_speed_kph=state['ego']['speed_kmh'] if state else None))
    result=dict(transitions=results,
                evidence='subscriber receipt; startup mode excluded; native speed is evaluation only; missing commands remain null')
    with (root/'handoff-analysis.json').open('x') as f:json.dump(result,f,indent=2)
    return result
