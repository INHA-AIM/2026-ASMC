"""Use the verified local export with the same source model in plain roslaunch."""
import hashlib
import json
import os
from pathlib import Path


def select_object_engine(requested, source_sha, gpu, tensorrt, head_profile, profile_path=None):
    root = Path(__file__).resolve().parents[6]
    profile_path = Path(profile_path or os.environ.get(
        'ASMC_CAMERA_RUNTIME_PROFILE', str(root / 'tools/scen_runner/.runtime/camera_runtime.json')))
    if not profile_path.is_file():
        return Path(requested), source_sha
    profile = json.loads(profile_path.read_text())
    # A local E3 export never overrides an explicit E8/pretrained choice.
    if profile.get('source_engine_sha256') != source_sha:
        return Path(requested), source_sha
    validation = profile.get('validation', {})
    engine = Path(profile['engine_path'])
    if not engine.is_absolute():
        engine = profile_path.parent.parent / engine
    digest = hashlib.sha256(engine.read_bytes()).hexdigest()
    provenance = json.loads(engine.with_suffix('.engine.provenance.json').read_text())
    if (profile.get('schema_version') != 1 or profile.get('gpu_name') != gpu
            or profile.get('tensorrt') != tensorrt
            or profile.get('engine_sha256') != digest
            or validation.get('passed') is not True
            or validation.get('engine_sha256') != digest
            or provenance.get('engine_sha256') != digest
            or provenance.get('tensorrt') != tensorrt
            or provenance.get('head_profile') != head_profile):
        raise RuntimeError('Local object engine validation/model/GPU/runtime mismatch; rebuild with '
                           'bash tools/scen_runner/prepare_camera_engines.sh')
    return engine, digest
