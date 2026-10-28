#!/usr/bin/env python3
"""Build TwinLite Large v7 E8 once for the current GPU; camera.launch finds it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/asmc_camera/src'))
from asmc_camera.camera.backends.engine_selection import local_engine_directory


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate(directory, checkpoint_sha, gpu, trt):
    m = json.loads((directory / 'manifest.json').read_text())
    if (m.get('completed') is not True or m.get('checkpoint_sha256') != checkpoint_sha
            or m.get('gpu_name') != gpu or m.get('tensorrt') != trt
            or m.get('model_config') != 'large'):
        raise RuntimeError('local engine manifest does not match this model/GPU/runtime')
    if len(m.get('engines', [])) != 2:
        raise RuntimeError('both front and side engines are required')
    for row in m['engines']:
        engine = (directory / row['file']).resolve()
        if engine.parent != directory.resolve() or sha(engine) != row['sha256']:
            raise RuntimeError('local engine file/SHA mismatch: ' + str(engine))
    return m


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, default=ROOT / 'src/asmc_camera/models/lane_segmentation/twinlite-large-v7-e8.pt')
    p.add_argument('--cache-root', type=Path)
    p.add_argument('--check', action='store_true', help='validate only; never build or restart nodes')
    a = p.parse_args()
    import torch
    import tensorrt
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is required to prepare the local lane engine')
    gpu, trt = torch.cuda.get_device_name(0), tensorrt.__version__
    checkpoint_sha = sha(a.checkpoint)
    directory = local_engine_directory(checkpoint_sha, gpu, trt, a.cache_root)
    print('GPU: {} | TensorRT: {}'.format(gpu, trt), flush=True)
    print('Local engine: {}'.format(directory), flush=True)
    if directory.exists():
        validate(directory, checkpoint_sha, gpu, trt)
        print('LOCAL LANE ENGINE OK; existing build retained')
        return
    if a.check:
        raise RuntimeError('local lane engine is absent; rerun without --check to build it')
    directory.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='lane-build-', dir=str(directory.parent)))
    try:
        output = work / 'output'
        subprocess.run([sys.executable, '-u', str(ROOT / 'tools/perception/prepare_twinlite_tensorrt.py'),
            '--checkpoint', str(a.checkpoint), '--output', str(output), '--model-config', 'large',
            '--model-prefix', 'twinlite_large_v7_e8', '--output-mode', 'reduced', '--precision', 'fp16'], check=True)
        validate(output, checkpoint_sha, gpu, trt)
        output.rename(directory)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print('LOCAL LANE ENGINE OK; restart the existing camera.launch when ready')


if __name__ == '__main__':
    main()
