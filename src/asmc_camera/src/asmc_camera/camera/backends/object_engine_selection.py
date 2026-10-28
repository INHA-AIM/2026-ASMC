"""Use the verified local export with the same source model in plain roslaunch."""
import hashlib
import json
import os
from pathlib import Path

TOOL_ROOT = Path(__file__).resolve().parents[6] / 'tools/scen_runner'


def select_object_engine(requested, source_sha, gpu, tensorrt, head_profile, profile_path=None):
    choice = profile_path if profile_path is not None else os.environ.get('ASMC_CAMERA_RUNTIME_PROFILE', 'auto')
    if str(choice) == 'off':
        return Path(requested), source_sha
    profile_path = (TOOL_ROOT / '.runtime/camera_runtime.json' if str(choice) == 'auto'
                    else Path(choice).expanduser())
    if not profile_path.is_absolute():
        profile_path = TOOL_ROOT / profile_path
    if not profile_path.is_file():
        if str(choice) != 'auto':
            raise RuntimeError('Explicit local object engine profile is missing: '+str(profile_path))
        return Path(requested), source_sha
    profile = json.loads(profile_path.read_text())
    # A local E3 export never overrides an explicit E8/pretrained choice.
    # Also revalidate an engine already selected by scen_runner: its checksum
    # differs from the packaged source, but GPU/runtime checks still apply.
    if source_sha not in (profile.get('source_engine_sha256'), profile.get('engine_sha256')):
        return Path(requested), source_sha
    validation = profile.get('validation', {})
    engine = Path(profile['engine_path'])
    if not engine.is_absolute():
        engine = TOOL_ROOT / engine
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
