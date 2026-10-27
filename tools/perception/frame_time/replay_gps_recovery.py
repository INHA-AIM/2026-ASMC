#!/usr/bin/env python3
"""Compare before/fixed bridge cores using an already-recorded GPS-zero trial.

VehicleState receipt events trigger offline queries, not a replayed ROS timer.
GPS/IMU messages and bag arrival order are preserved. No ROS master/control.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_frame_time import ROOT, read, sha, write_json


def compare(input_bag, baseline, fixed, output):
    import rosbag
    from sensor_msgs.msg import PointCloud2
    output.mkdir(parents=True, exist_ok=False)
    phases = []
    zero = False
    changed = None
    with tempfile.TemporaryDirectory(prefix='c0-gps-replay-') as scratch:
        fixture = Path(scratch) / 'query.bag'
        with rosbag.Bag(str(input_bag)) as source, rosbag.Bag(str(fixture), 'w') as target:
            for topic, message, receipt in source.read_messages(topics=['/gps', '/imu', '/vehicle/state']):
                now = receipt.to_nsec()
                if topic == '/gps':
                    current = message.latitude == 0 and message.longitude == 0
                    if changed is None or current != zero:
                        changed = now
                    zero = current
                if topic in ('/gps', '/imu'):
                    target.write(topic, message, receipt)
                else:
                    query = PointCloud2()
                    query.header.stamp = receipt
                    query.header.frame_id = 'offline_query_only'
                    target.write('/velodyne_points', query, receipt)
                    phases.append((zero, changed is not None and now - changed > 100000000))
        result = {'input_sha256': sha(input_bag), 'query_fixture_sha256': sha(fixture), 'cores': {}}
        for name, probe in [('before', baseline), ('fixed', fixed)]:
            rows_path = output / (name + '.jsonl')
            subprocess.run([str(probe), str(fixture), str(ROOT / 'src/asmc_bridge/config/vehicle_state.yaml'), str(rows_path)], check=True)
            rows = list(read(rows_path))
            if len(rows) != len(phases):
                raise ValueError('query count mismatch')
            result['cores'][name] = {
                'binary_sha256': sha(probe), 'queries': len(rows),
                'gps_zero_valid': sum(bool(row['valid']) for row, (is_zero, stable) in zip(rows, phases) if is_zero and stable),
                'gps_zero_queries': sum(is_zero and stable for is_zero, stable in phases),
                'gps_nonzero_valid': sum(bool(row['valid']) for row, (is_zero, stable) in zip(rows, phases) if not is_zero and stable),
                'output_sha256': sha(rows_path),
            }
    result['passed'] = (result['cores']['before']['gps_zero_valid'] > 0 and
                        result['cores']['fixed']['gps_zero_queries'] > 0 and
                        result['cores']['fixed']['gps_zero_valid'] == 0 and
                        result['cores']['fixed']['gps_nonzero_valid'] > 0)
    result['limits'] = ['Recorded arrival-order query, not exact original node callback scheduling.',
                        'First 100ms after GPS-zero transitions excluded, no cross-recording joins.',
                        'No sensor math is implemented in Python; both native cores read the same fixture.']
    write_json(output / 'summary.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'baseline-probe', 'fixed-probe', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.input, args.baseline_probe, args.fixed_probe, args.output)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result['passed'] else 1)
