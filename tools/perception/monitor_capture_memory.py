#!/usr/bin/env python3
"""Record Windows memory and request an owned collection session to stop.

Never terminates Windows processes. The collector observes the files and stops
its own controller, preserving the failed/partial run and restoration evidence.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import time

GIB = 1024 ** 3
QUERY = r"""
$m = Get-CimInstance Win32_PerfFormattedData_PerfOS_Memory
$p = @(Get-Process -Name Simulator -ErrorAction SilentlyContinue)
@{ committed_bytes=[double]$m.CommittedBytes; commit_limit_bytes=[double]$m.CommitLimit;
   available_mb=[double]$m.AvailableMBytes;
   simulators=@($p | ForEach-Object { @{pid=$_.Id; private_bytes=[double]$_.PrivateMemorySize64;
       working_set_bytes=[double]$_.WorkingSet64} }) } | ConvertTo-Json -Depth 4 -Compress
"""


def decision(sample):
    processes = sample.get('simulators', [])
    limit = sample.get('commit_limit_bytes', 0)
    if not processes or limit <= 0:
        return 'abort', 'simulator absent or invalid memory telemetry'
    private = sum(p['private_bytes'] for p in processes)
    commit = sample['committed_bytes'] / limit
    if private >= 32 * GIB or commit >= .85 or sample['available_mb'] < 512:
        return 'abort', 'critical memory pressure'
    if private >= 16 * GIB or commit >= .65:
        return 'stop_between_laps', 'memory headroom declining'
    return 'continue', 'within collection memory limits'


def check_render(sensor_root, record):
    """A successful RPC may still have written constant/black textures."""
    from PIL import Image
    custom = record['custom_name']
    if Path(custom).name != custom:
        raise ValueError('invalid Capture name')
    bad = []
    for sensor in ('CAMERA_1', 'CAMERA_2', 'CAMERA_3'):
        path = sensor_root / sensor / (custom + '_Intensity.png')
        with Image.open(path) as im:
            extrema = im.convert('RGB').getextrema()
        if all(lo == hi for lo, hi in extrema):
            bad.append(sensor)
    return dict(custom_name=custom, constant_views=bad)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop-after-current-file', type=Path, required=True)
    p.add_argument('--abort-file', type=Path, required=True)
    p.add_argument('--max-seconds', type=float, default=3600)
    p.add_argument('--interval', type=float, default=15)
    p.add_argument('--sensor-root', type=Path, help='Windows SensorData path visible on this host')
    p.add_argument('--capture-runs-root', type=Path, help='Inspect only newly modified route_pp_* manifests')
    a = p.parse_args()
    if a.output.exists() or a.stop_after_current_file.exists() or a.abort_file.exists():
        raise ValueError('use fresh telemetry and stop files for each session')
    if a.max_seconds <= 0 or a.interval < 5:
        raise ValueError('finite positive duration and interval >=5 required')
    if bool(a.sensor_root) != bool(a.capture_runs_root):
        raise ValueError('both render-check paths must be supplied')
    for path in (a.output, a.stop_after_current_file, a.abort_file):
        path.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + a.max_seconds
    started_wall = time.time()
    errors = 0
    with a.output.open('x') as log:
        while time.monotonic() < deadline:
            row = {'utc': datetime.now(timezone.utc).isoformat()}
            try:
                result = subprocess.run(['powershell.exe', '-NoProfile', '-Command', QUERY],
                    capture_output=True, text=True, timeout=8, check=True)
                row['sample'] = json.loads(result.stdout.lstrip('\ufeff'))
                row['action'], row['reason'] = decision(row['sample'])
                if a.sensor_root:
                    manifests = [f for f in a.capture_runs_root.glob('route_pp_*/capture_manifest.jsonl')
                                 if f.stat().st_mtime >= started_wall]
                    if manifests:
                        latest = max(manifests, key=lambda f: f.stat().st_mtime)
                        # A record two captures behind has had time to finish its files.
                        records = [json.loads(line) for line in latest.read_text().splitlines()]
                        if len(records) >= 3 and records[-3].get('success'):
                            row['render'] = check_render(a.sensor_root, records[-3])
                            if row['render']['constant_views']:
                                row.update(action='abort',reason='constant RGB texture after successful Capture RPC')
                errors = 0
            except (OSError, subprocess.SubprocessError, ValueError, KeyError) as exc:
                errors += 1
                row.update(action='abort' if errors >= 2 else 'stop_between_laps',
                           reason='memory telemetry or Capture RGB integrity unavailable', error=str(exc))
            log.write(json.dumps(row) + '\n'); log.flush()
            if row['action'] != 'continue':
                a.stop_after_current_file.write_text(json.dumps(row) + '\n')
            if row['action'] == 'abort':
                a.abort_file.write_text(json.dumps(row) + '\n')
                return 2
            time.sleep(min(a.interval, max(0, deadline-time.monotonic())))
    # A collector must not continue after its monitor expires.
    a.abort_file.write_text(json.dumps({'reason': 'memory monitor duration expired'})+'\n')
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
