#!/usr/bin/env python3
"""Evaluate an already bounded trainer after its supervisor has been suspended.

Uses the run's frozen evaluator. Does not stop or resume a trainer, mutate its
weights, or start a replacement run. A completion marker is emitted only after
the full checkpoint and both requested evaluations have been verified.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace


def completed(record, epoch):
    if record.get('errors') or record.get('status') == 'failed':
        raise RuntimeError('Trainer failed; preserve evidence and do not restart')
    n = record.get('completed_epochs')
    if n is not None and n > epoch:
        raise RuntimeError('Trainer crossed the requested boundary')
    return n == epoch and record.get('epoch_boundary_pause') is True


def finish(job, epoch, timeout, config_path=None, control_name='operator-closeout'):
    sys.path.insert(0, str(job/'code/tools/perception'))
    from run_camera_bbox_main import evaluate, compare_metrics, gallery, page, save, sha
    if Path(control_name).name != control_name or control_name in ('.', '..'):
        raise ValueError('control directory must be one local name')
    control = job/control_name
    control.mkdir(exist_ok=True)
    marker = control/'_EVALUATED'
    if marker.exists():
        return
    def state(status, **kwargs):
        save(control/'state.json', dict(status=status, target_epoch=epoch,
             updated_utc=datetime.now(timezone.utc).isoformat(), **kwargs))
    deadline = time.monotonic()+timeout
    state('waiting_for_full_epoch')
    while True:
        record = json.loads((job/'main/run.json').read_text())
        if completed(record, epoch):
            break
        if time.monotonic() > deadline:
            raise TimeoutError('Checkpoint boundary wait expired')
        time.sleep(15)
    weights = job/'main/fit/weights'/f'epoch{epoch-1}.pt'
    import torch
    checkpoint = torch.load(weights, map_location='cpu')
    if checkpoint.get('epoch') != epoch-1 or checkpoint.get('optimizer') is None:
        raise ValueError('Not a complete, resumable epoch checkpoint')
    del checkpoint
    cfg = config_path or job/'code/config/experiments/camera_bbox_main_v9.json'
    config = json.loads(cfg.read_text())
    args = SimpleNamespace(config=cfg, output=job)
    old_state = json.loads((job/'supervisor.json').read_text())
    if old_state['config_sha256'] != sha(cfg):
        raise ValueError('Frozen configuration identity changed')
    save(control/'supervisor-before.json', old_state)
    state('evaluating', checkpoint=str(weights), checkpoint_sha256=sha(weights))
    folder = job/f'epoch-{epoch:03d}-1280'
    result = evaluate(args, weights, folder)
    baseline = json.loads((job/'baseline-1280/evaluation.json').read_text())
    metrics = compare_metrics(baseline, result, config['evaluation'].get('minimum_new_gt', 20),
        config['evaluation'].get('existing_class_quality')) if config_path else compare_metrics(
        baseline, result, config['evaluation'].get('minimum_new_gt', 20))
    if config.get('regression_dataset'):
        evaluate(args, weights, job/f'regression-{epoch:03d}',
                 dataset=Path(config['regression_dataset']))
    if (job/'gallery').exists() and not (control/'previous-gallery').exists():
        shutil.copytree(job/'gallery', control/'previous-gallery')
    gallery(job, folder)
    entry = dict(epoch=epoch, metrics=metrics, checkpoint=str(weights), sha256=sha(weights))
    old_state['evaluations'] = [e for e in old_state['evaluations'] if e['epoch'] != epoch]+[entry]
    old_state.update(status='stopped_at_requested_epoch', message=f'Epoch {epoch} evaluated; awaiting operator analysis before further training',
                     updated_utc=datetime.now(timezone.utc).isoformat())
    save(job/'supervisor.json', old_state)
    page(job, old_state)
    save(control/'result.json', dict(**entry, evaluator_sha256=sha(job/'code/tools/perception/run_camera_bbox_main.py'),
         config_sha256=sha(cfg), dataset_metadata_sha256=old_state['dataset_metadata_sha256'],
         evaluation_sha256=sha(folder/'evaluation.json'), production_approved=False))
    state('evaluated', checkpoint=str(weights), result_sha256=sha(control/'result.json'))
    marker.write_text(sha(control/'result.json')+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--job', required=True, type=Path)
    p.add_argument('--epoch', type=int, default=8)
    p.add_argument('--timeout', type=float, default=21600)
    p.add_argument('--config', type=Path, help='Pinned run configuration; omitted preserves legacy v9 path')
    p.add_argument('--control-name', default='operator-closeout', help='Separate completion record when observing another boundary')
    a = p.parse_args()
    try:
        finish(a.job.resolve(), a.epoch, a.timeout, a.config.resolve() if a.config else None, a.control_name)
    except BaseException as exc:
        out = a.job/a.control_name/'error.json'
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(dict(type=type(exc).__name__, error=str(exc)), indent=2))
        raise
