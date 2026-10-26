"""Select a locally verified camera engine without copying the team's model settings."""
import json
import hashlib
import os
from pathlib import Path
import re
import sys

from scen_runner.config import TOOL_ROOT


def lane_override(profile, launch, tool_root):
    """Validate the optional local lane engine before changing any launch params."""
    lane = profile.get('lane')
    if lane is None:
        return None
    nodes = [n for n in launch.nodes if n.type == 'semantic_inference_node.py']
    if len(nodes) != 1:
        raise ValueError('로컬 차선 엔진에는 차선 노드 하나가 필요합니다.')
    prefix = nodes[0].namespace+nodes[0].name+'/'
    params = launch.params
    manifest = Path(lane['engine_manifest'])
    if not manifest.is_absolute():
        manifest = tool_root/manifest
    def sha(path):
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    if (params[prefix+'backend'].value != 'twinlite_tensorrt' or
            sha(params[prefix+'engine_manifest'].value) != lane.get('source_manifest_sha256') or
            sha(params[prefix+'checkpoint'].value) != lane.get('checkpoint_sha256')):
        raise ValueError('팀 차선 모델 설정이 변경됐습니다. 로컬 엔진을 다시 변환하세요.')
    validation = lane.get('validation', {})
    if (validation.get('passed') is not True or
            validation.get('manifest_sha256') != sha(manifest)):
        raise ValueError('차선 엔진 기본 동작 검사 기록이 일치하지 않습니다.')
    # Runtime checks the manifest's engine hashes, GPU, TRT, shapes and classes.
    return prefix+'engine_manifest', str(manifest)


def configure_camera_runtime(config, launch, tool_root=TOOL_ROOT):
    choice = config.get('camera_runtime', 'auto')
    if choice in (None, False, 'off'):
        return
    tool_root = Path(tool_root)
    path = tool_root/'.runtime/camera_runtime.json' if choice == 'auto' else Path(choice)
    if not path.is_absolute():
        path = tool_root/path
    if choice == 'auto' and not path.exists():
        return
    profile = json.loads(path.read_text(encoding='utf-8'))
    if profile.get('schema_version') != 1:
        raise ValueError('카메라 실행 환경 형식 오류: '+str(path))
    nodes = [n for n in launch.nodes if n.type == 'object_detection_node.py']
    if not nodes:
        return
    if len(nodes) != 1:
        raise ValueError('카메라 객체 인식 노드가 여러 개입니다.')
    node = nodes[0]
    prefix = node.namespace+node.name+'/'
    params = launch.params
    source_sha = params[prefix+'expected_sha256'].value
    if (source_sha != profile.get('source_engine_sha256') or
            params[prefix+'runtime_backend'].value != 'tensorrt'):
        raise ValueError('팀 카메라 모델 설정이 변경됐습니다. 로컬 엔진을 다시 변환·검증하세요: '+str(path))
    engine_sha = profile.get('engine_sha256', '')
    validation = profile.get('validation', {})
    if (not re.fullmatch(r'[0-9a-f]{64}', engine_sha) or
            validation.get('passed') is not True or validation.get('engine_sha256') != engine_sha):
        raise ValueError('추론 검증을 통과한 카메라 엔진 기록이 없습니다: '+str(path))
    engine = Path(profile['engine_path'])
    dependencies = Path(profile['runtime_pythonpath'])
    if not engine.is_absolute():
        engine = tool_root/engine
    if not dependencies.is_absolute():
        dependencies = tool_root/dependencies
    if not engine.is_file() or not dependencies.is_dir():
        raise ValueError('로컬 카메라 엔진 또는 실행 환경이 없습니다: '+str(path))
    lane = lane_override(profile, launch, tool_root)
    # The original detector verifies the engine SHA and class/head metadata
    # before inference. All other parameters remain owned by the team launch.
    params[prefix+'weights'].value = str(engine)
    params[prefix+'expected_sha256'].value = engine_sha
    env = dict(node.env_args)
    existing = env.get('PYTHONPATH', os.environ.get('PYTHONPATH', ''))
    env['PYTHONPATH'] = str(dependencies)+(os.pathsep+existing if existing else '')
    env['YOLO_AUTOINSTALL'] = 'false'
    node.env_args = list(env.items())
    if lane:
        params[lane[0]].value = lane[1]
        print('[차선 엔진] '+lane[1], file=sys.stderr)
    print('[카메라 엔진] '+str(engine)+' · SHA256='+engine_sha[:12], file=sys.stderr)
