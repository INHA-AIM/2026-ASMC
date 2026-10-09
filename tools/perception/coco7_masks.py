"""Explicit experiment mask profiles; historical black profiles remain reproducible."""
from pathlib import Path
from asmc_camera.detection.hood_mask import HoodMask,load_view_masks

ROOT=Path(__file__).resolve().parents[2]/'src/asmc_camera/config/camera'


def add_mask_profile(parser):
    parser.add_argument('--mask-profile',choices=['black-v1','neutral-v2'],default='black-v1')


def load_masks(profile,enabled=True):
    suffix='v1' if profile=='black-v1' else 'neutral_v2' if profile=='neutral-v2' else None
    if suffix is None:raise ValueError('unknown mask profile')
    return load_view_masks(['front','left','right'],HoodMask.load(enabled,ROOT/f'yolo_hood_mask_{suffix}.yaml'),enabled,
        {v:ROOT/f'yolo_ego_mask_{v}_{suffix}.yaml' for v in ('left','right')})
