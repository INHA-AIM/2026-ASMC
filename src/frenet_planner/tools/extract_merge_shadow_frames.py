#!/usr/bin/env python3
"""Convert an external topic-indexed decoded ROS bag to offline shadow inputs.

The decoder's JSON schema is {topic: [{t: receive_time, msg: message}, ...]}.
No raw observations are checked into the repository. All joins are causal.
"""
import argparse
import bisect
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('decoded_bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start-s', type=float, default=780)
    parser.add_argument('--end-s', type=float, default=935)
    args = parser.parse_args()
    bag = json.loads(args.decoded_bag.read_text())
    timestamps = {topic: [row['t'] for row in rows]
                  for topic, rows in bag.items()}
    origin = min(rows[0]['t'] for rows in bag.values() if rows)

    def prior(topic, time):
        index = bisect.bisect_right(timestamps[topic], time) - 1
        return bag[topic][index] if index >= 0 else None

    frames = []
    for row in bag['/planning/trajectory']:
        time = row['t']
        feature = prior('/behavior/feature_debug', time)
        objects = prior('/perception/objects', time)
        if not feature or not objects:
            continue
        if not args.start_s < feature['msg']['ego_s'] < args.end_s:
            continue
        ego = row['msg']['ego_at_plan']
        frames.append({
            't': time - origin,
            'ego': ego,
            'objects': objects['msg']['objects'],
            'object_age': time - objects['t'],
            'object_stamp_age': (ego['header']['stamp']
                                 - objects['msg']['header']['stamp']),
        })
    args.output.write_text(json.dumps(frames))
    print(f'{len(frames)} frames written')


if __name__ == '__main__':
    main()
