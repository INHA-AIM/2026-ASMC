"""Offline receive-clock audit of the existing LOCAL bicycle/actuator model.

No tuning is applied. Effective delay includes receipt scheduling and actuator
response; it is not a measurement of pure steering delay or a source-clock fix.
"""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


def pairs(imus, speeds, commands, delay):
    """Causal lookups only; no future speed/command is borrowed into a sample."""
    vs = [r[0] for r in speeds]
    cs = [r[0] for r in commands]
    out = []
    rejected = {}
    for stamp, yaw_rate in imus:
        vi = bisect_right(vs, stamp) - 1
        ci = bisect_right(cs, stamp - delay) - 1
        reason = None
        if vi < 0 or ci < 0:
            reason = 'no_causal_history'
        elif stamp - vs[vi] > .10 or stamp - delay - cs[ci] > .15:
            reason = 'history_stale'
        elif speeds[vi][1] < .5 or abs(commands[ci][1]) < .02:
            reason = 'insufficient_excitation'
        elif not math.isfinite(yaw_rate + speeds[vi][1] + commands[ci][1]):
            reason = 'nonfinite'
        if reason:
            rejected[reason] = rejected.get(reason, 0) + 1
            continue
        v, steering = speeds[vi][1], commands[ci][1]
        out.append((stamp, v / 3. * math.tan(steering), yaw_rate, v))
    return out, rejected


def fit(rows):
    import numpy as np
    if len(rows) < 30:
        return None
    x, y = np.array([(r[1], r[2]) for r in rows]).T
    gain = float(x @ y / max(1e-12, x @ x))
    for _ in range(5):
        residual = y - gain * x
        scale = max(.003, float(np.median(abs(residual))) * 1.4826)
        weight = np.minimum(1., 1.5 * scale / np.maximum(abs(residual), 1e-12))
        gain = float((weight * x) @ y / max(1e-12, (weight * x) @ x))
    return gain


def error(rows, gain):
    import numpy as np
    if not rows or gain is None:
        return {'count': len(rows), 'p95_rad_s': None}
    e = [abs(r[2] - gain * r[1]) for r in rows]
    return dict(count=len(rows), p50_rad_s=float(np.percentile(e, 50)),
                p95_rad_s=float(np.percentile(e, 95)), max_rad_s=max(e))


def audit(imus, speeds, commands):
    split = .5 * (imus[0][0] + imus[-1][0]) if imus else 0
    choices = []
    for delay in (0., .05, .10, .15, .20, .25, .30):
        rows, rejected = pairs(imus, speeds, commands, delay)
        train = [r for r in rows if r[0] < split]
        test = [r for r in rows if r[0] >= split]
        gain = fit(train)
        choices.append(dict(delay_s=delay, train_gain=gain,
                            train=error(train, gain), held_out=error(test, gain), rejected=rejected))
    usable = [r for r in choices if r['train_gain'] is not None and
              r['train']['p95_rad_s'] is not None]
    best = min(usable, key=lambda r: r['train']['p95_rad_s']) if usable else None
    rows, rejected = pairs(imus, speeds, commands, .15)
    return dict(configured=dict(wheelbase_m=3., gain=.844, delay_s=.15,
                                residual=error(rows, .844), rejected=rejected),
                train_selected_candidate=best, candidates=choices,
                changed_runtime=False,
                limitation='Receive-clock effective-delay sensitivity; steering command is not a measured wheel angle. '
                'Not independent vehicle-parameter GT. No model/timing setting is changed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if not any((p/'registration.json').is_file() for p in args.output.parents):
        raise ValueError('output requires a new registered run')
    import rosbag
    commands = [(r['ros_time'], r['steering']) for r in
                (json.loads(line) for line in (args.run/'wire.jsonl').open())]
    speeds, imus = [], []
    for bag in sorted(args.run.glob('sensors*.bag')):
        with rosbag.Bag(str(bag)) as stream:
            for topic, m, _ in stream.read_messages(topics=['/imu', '/Ego_topic']):
                stamp = m.header.stamp.to_sec()
                if topic == '/Ego_topic':
                    speeds.append((stamp, m.velocity.x))
                else:
                    q = m.orientation
                    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
                    if not math.isfinite(norm) or abs(norm-1.) > .05:
                        continue
                    roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
                    if abs(pitch) > .5:
                        continue
                    yaw = (math.sin(roll)*m.angular_velocity.y + math.cos(roll)*m.angular_velocity.z)/math.cos(pitch)
                    imus.append((stamp, yaw))
    result = audit(sorted(imus), sorted(speeds), sorted(commands))
    result['input_run'] = args.run.name
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)


if __name__ == '__main__':
    main()
