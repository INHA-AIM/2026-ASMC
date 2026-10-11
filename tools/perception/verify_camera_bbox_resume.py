#!/usr/bin/env python3
"""Verify real epoch-boundary resume using locally produced trusted checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path

from train_camera_bbox_pilot import save, sha


def verify(root):
    import torch
    checks = {}
    for name in ('yolo26l', 'yoloe26l'):
        folder = root / name
        run = json.loads((folder / 'run.json').read_text())
        checkpoint = folder / 'fit/weights/epoch0.pt'
        before = torch.load(checkpoint, map_location='cpu')
        after = torch.load(folder / 'fit/weights/epoch1.pt', map_location='cpu')
        state = before['optimizer']['state']
        steps = [float(v['step']) for v in state.values() if 'step' in v]
        moment = next(v['exp_avg'] for v in state.values() if 'exp_avg' in v)
        moment_sha = hashlib.sha256(moment.cpu().float().numpy().tobytes()).hexdigest()
        restored = run['optimizer_at_start']
        assert run['status'] == 'complete' and run['completed_epochs'] == 2
        assert before['epoch'] == 0 and after['epoch'] == 1
        assert before['updates'] == 16 and after['updates'] == 32
        assert min(steps) > 0 and restored['state_slots'] == len(state)
        assert restored['step_min'] == min(steps) and restored['step_max'] == max(steps)
        assert restored['first_moment_sha256'] == moment_sha
        assert restored['ema_updates'] == before['updates']
        assert restored['scaler_scale'] == before['scaler']['scale']
        assert run['resume_checkpoint_sha256'] == sha(checkpoint)
        names = after['ema'].names
        canonical = list(names.values()) if isinstance(names, dict) else list(names)
        assert canonical == run['canonical_names']
        if name == 'yoloe26l':
            assert restored['head_fused'] is True
            assert type(after['ema'].model[-1]).__name__ == 'YOLOEDetect'
        histories = list((folder / 'resume-history').glob('*.json'))
        assert len(histories) == 1
        paused = json.loads(histories[0].read_text())
        assert paused['status'] == 'paused_at_epoch' and paused['completed_epochs'] == 1
        new_steps = [float(v['step']) for v in after['optimizer']['state'].values() if 'step' in v]
        assert min(new_steps) > max(steps)
        checks[name] = dict(status='pass', paused_epoch=1, resumed_complete_epoch=2,
            fixture_manifest_sha256=run['manifest_sha256'], batch=run['batch'],
            optimizer_state_slots=len(state), adamw_steps_before=min(steps),
            adamw_steps_after=min(new_steps), restored_first_moment_sha256=moment_sha,
            restored_scaler_scale=restored['scaler_scale'], ema_attempts_before=16,
            ema_attempts_after=32, head_fused=restored['head_fused'],
            before_checkpoint_sha256=sha(checkpoint),
            after_checkpoint_sha256=sha(folder/'fit/weights/epoch1.pt'),
            run_sha256=sha(folder/'run.json'), history_sha256=sha(histories[0]),
            trainer_source_sha256=run['trainer_source_sha256'])
    return dict(status='pass', models=checks,
        scope='actual optimizer/EMA/scaler/fused-head restore and continued updates; not bitwise data/augmentation RNG continuity')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.root)
    save(args.output, result)
    print(json.dumps(result, indent=2))
