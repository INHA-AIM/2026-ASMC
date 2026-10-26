#!/usr/bin/env python3
"""Run every isolated CI group with bounded concurrency and preserve failures."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[2]


def stop(process):
    for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue


def run_groups(groups, output, workers=2, root=ROOT):
    if not groups or workers not in (1, 2):
        raise ValueError('non-empty groups and one or two workers required')
    names = [g['name'] for g in groups]
    if len(names) != len(set(names)):
        raise ValueError('duplicate group name')
    for g in groups:
        if (not g['name'].replace('-', '').isalnum() or
                not (root/g['script']).is_file() or g['timeout_s'] <= 0):
            raise ValueError('invalid group: '+str(g))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)  # prior evidence is never replaced
    active, lock, cancelled = {}, threading.Lock(), threading.Event()

    def cancel(signum, frame):
        cancelled.set()
        with lock:
            processes = list(active.values())
        for process in processes:
            stop(process)

    handlers = {}
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, cancel)

    def execute(group):
        name = group['name']
        started = time.monotonic()
        result = dict(name=name, script=group['script'], returncode=None,
                      script_sha256=hashlib.sha256((root/group['script']).read_bytes()).hexdigest())
        if cancelled.is_set():
            return dict(result, status='cancelled', duration_s=0)
        print('START '+name, flush=True)
        env = dict(os.environ, ASMC_CI_OUTPUT=str(output))
        with (output/(name+'.log')).open('xb') as log:
            process = subprocess.Popen(['bash', '-e', '-o', 'pipefail', group['script']],
                                       cwd=str(root), env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            with lock:
                active[name] = process
            try:
                if cancelled.is_set():
                    stop(process)
                process.wait(timeout=group['timeout_s'])
                result.update(returncode=process.returncode,
                              status='passed' if process.returncode == 0 and not cancelled.is_set() else 'failed')
            except subprocess.TimeoutExpired:
                stop(process)
                result.update(returncode=process.returncode, status='timeout')
            finally:
                with lock:
                    active.pop(name, None)
        result['duration_s'] = round(time.monotonic()-started, 3)
        print('%s %s %.3fs' % (result['status'].upper(), name, result['duration_s']), flush=True)
        if result['status'] != 'passed':
            print('\n'.join((output/(name+'.log')).read_text(errors='replace').splitlines()[-60:]), flush=True)
        return result

    started = time.monotonic()
    try:
        # Deadline/HTTP admission tests measure real wall time. Do not add
        # unrelated test load while these groups execute; keep their limits.
        results = []
        for group in groups:
            if group.get('exclusive', False):
                try:
                    results.append(execute(group))
                except Exception as error:
                    results.append(dict(name=group['name'], status='error', error=str(error)))
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
            # No fail-fast: another failing group never silently skips coverage.
            parallel = [group for group in groups if not group.get('exclusive', False)]
            futures = [pool.submit(execute, group) for group in parallel]
            for group, future in zip(parallel, futures):
                try:
                    results.append(future.result())
                except Exception as error:
                    results.append(dict(name=group['name'], status='error', error=str(error)))
        summary = dict(workers=workers, elapsed_s=round(time.monotonic()-started, 3),
                       groups=results, passed=all(r['status'] == 'passed' for r in results),
                       source_revision=os.environ.get('GITHUB_SHA'))
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        return summary
    finally:
        for sig, previous in handlers.items():
            signal.signal(sig, previous)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups', type=Path, default=ROOT/'.github/ci/groups.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    groups = json.loads(args.groups.read_text())['groups']
    return 0 if run_groups(groups, args.output, args.workers)['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
