#!/usr/bin/env python3
"""Request a graceful stop after an atomic, fully evaluated epoch checkpoint.

This external guard leaves the running trainer and its LR schedule untouched.
It uses the existing STOP protocol; it never kills the training process. The
trainer writes epochNNN.pt only after validation, before latest/best/history.
STOP is checked after those writes, during preview or at an optimizer boundary.
Any late optimizer update is reported explicitly, never counted as target epoch.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time


def read(path):
    return json.loads(path.read_text()) if path.exists() else None


def write(path, value):
    temporary = path.with_suffix(path.suffix + '.part')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def request_if_ready(output, target, progress, run_id):
    if not progress or progress.get('run_id') != run_id:
        raise ValueError('training run_id mismatch or missing progress')
    checkpoint = output / ('epoch{:03d}.pt'.format(target))
    if not checkpoint.is_file() or checkpoint.stat().st_size == 0:
        return False
    # Only the atomically renamed final file counts, never epochNNN.pt.part.
    (output / 'STOP').touch(exist_ok=True)
    return True


def check_result(result, history, target):
    row = next((item for item in history if item['epoch'] == target), None)
    if row is None:
        return dict(state='ended_before_target', completed_epochs=result.get('completed_epochs'))
    extra = result.get('optimizer_steps', 0) - row['optimizer_steps']
    exact = result.get('completed_epochs') == target and extra == 0 and result.get('state') == 'stopped'
    return dict(state='stopped_at_target' if exact else 'needs_review',
                completed_epochs=result.get('completed_epochs'),
                optimizer_updates_after_target=extra, training_state=result.get('state'),
                stop_reason=result.get('stop_reason'))


def run(output, monitor, target, run_id):
    if target < 1 or output.name != run_id:
        raise ValueError('invalid target epoch or output/run_id mismatch')
    config = read(output / 'config.json')
    if not config or config['arguments']['output'] != str(output):
        raise ValueError('output does not match the active training configuration')
    policy = dict(run_id=run_id, stop_after_epoch=target, lr_schedule_epochs=config['arguments']['epochs'],
                  requested_at_utc=datetime.now(timezone.utc).isoformat(), state='armed',
                  implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  training_code_modified=False)

    def report(**values):
        policy.update(values, updated_at_utc=datetime.now(timezone.utc).isoformat())
        write(monitor / 'stop-policy.json', policy)
        write(output / 'stop-policy.json', policy)

    requested = False
    try:
        progress = read(monitor / 'progress.json')
        if not progress or progress.get('run_id') != run_id:
            raise ValueError('active run_id mismatch')
        report()
        last_report = time.monotonic()
        while True:
            result = read(output / 'result.json')
            if result:
                history = read(output / 'history.json') or []
                report(**check_result(result, history, target))
                return
            # A final epoch checkpoint is a stronger boundary than the UI's
            # periodically updated progress; no large JSON is polled at 20 Hz.
            if not requested and (output / ('epoch{:03d}.pt'.format(target))).is_file():
                requested = request_if_ready(output, target, read(monitor / 'progress.json'), run_id)
                if requested:
                    report(state='stop_requested', checkpoint='epoch{:03d}.pt'.format(target))
            if time.monotonic() - last_report >= 30:
                report()
                last_report = time.monotonic()
            time.sleep(.05 if not requested else .5)
    except Exception as error:
        report(state='guard_error', error=str(error))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--monitor', required=True, type=Path)
    parser.add_argument('--epoch', required=True, type=int)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    run(args.output, args.monitor, args.epoch, args.run_id)
