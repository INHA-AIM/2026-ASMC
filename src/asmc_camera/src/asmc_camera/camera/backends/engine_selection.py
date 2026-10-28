"""Select a local GPU/runtime build without weakening TensorRT validation."""
import hashlib
import json
import os
from pathlib import Path


def local_engine_directory(checkpoint_sha256, gpu_name, tensorrt_version, cache_root=None):
    if len(checkpoint_sha256) != 64 or any(c not in '0123456789abcdef' for c in checkpoint_sha256):
        raise ValueError('invalid checkpoint SHA256')
    key = hashlib.sha256(json.dumps([gpu_name, tensorrt_version]).encode()).hexdigest()[:20]
    source_root = Path(__file__).resolve().parents[6]
    default = (source_root / 'diagnostics/camera_models/lane_engines'
               if (source_root / 'src/asmc_camera/package.xml').is_file()
               else Path('~/.cache/asmc_camera/engines'))
    root = Path(cache_root or os.environ.get('ASMC_CAMERA_ENGINE_CACHE', str(default))).expanduser()
    return root / checkpoint_sha256 / key


def select_engine_manifest(requested, gpu_name, tensorrt_version, cache_root=None):
    requested = Path(requested).expanduser().resolve()
    original = json.loads(requested.read_text())
    if original.get('gpu_name') == gpu_name and original.get('tensorrt') == tensorrt_version:
        return requested
    local = local_engine_directory(original['checkpoint_sha256'], gpu_name,
                                   tensorrt_version, cache_root) / 'manifest.json'
    if local.is_file():
        candidate = json.loads(local.read_text())
        if (candidate.get('completed') is True
                and candidate.get('gpu_name') == gpu_name
                and candidate.get('tensorrt') == tensorrt_version
                and candidate.get('checkpoint_sha256') == original['checkpoint_sha256']):
            return local
    raise RuntimeError(
        'TwinLite engine environment mismatch: packaged GPU={!r}, current GPU={!r}; '
        'packaged TensorRT={!r}, current TensorRT={!r}. '
        'Build once from the repository root: '
        'python3 tools/perception/prepare_local_lane_engine.py; '
        'Then restart the existing camera.launch. Expected local manifest: {}'.format(
            original.get('gpu_name'), gpu_name, original.get('tensorrt'),
            tensorrt_version, local))
