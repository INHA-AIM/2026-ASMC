#!/usr/bin/env python3
"""Build local camera engines, smoke-test them, then atomically select them.

Run in the ROS container after stopping MORAI and perception. No ROS nodes or
vehicle commands are started. The smoke test is not a driving/accuracy test.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT/'tools/scen_runner'
RUNTIME = TOOL/'.runtime'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n')


def nested_params(flat):
    """Reconstruct the dictionaries rospy sees from roslaunch's slash keys."""
    result = {}
    for key, value in sorted(flat.items(), key=lambda item: item[0].count('/')):
        parts = key.split('/')
        current = result
        for part in parts[:-1]:
            current = current.setdefault(part, {})
        current[parts[-1]] = value
    return result


def camera_params():
    import roslaunch.config
    launch = roslaunch.config.load_config_default(
        [str(ROOT/'src/asmc_camera/launch/camera.launch')], None)
    def params(node_type):
        node, = [n for n in launch.nodes if n.type == node_type]
        prefix = node.namespace+node.name+'/'
        return nested_params({k[len(prefix):]: v.value
                              for k, v in launch.params.items() if k.startswith(prefix)})
    return params('object_detection_node.py'), params('semantic_inference_node.py')


def smoke(kind, directory):
    import numpy as np
    obj, lane = camera_params()
    if kind == 'lane':
        from asmc_camera.camera.backends.twinlite_tensorrt import TwinLiteTensorRTBackend
        lane['engine_manifest'] = str(directory/'lane/manifest.json')
        backend = TwinLiteTensorRTBackend(**lane)
        for batch, height, width in [(1, 544, 960), (1, 480, 640), (2, 480, 640)]:
            result = backend.infer(np.zeros((batch, 3, height, width), np.float32))
            for values in (result['drivable'], result['road_marking'].confidence):
                if not np.isfinite(values).all():
                    raise RuntimeError('nonfinite lane inference')
        digest = sha(directory/'lane/manifest.json')
    else:
        from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
        engine = directory/'objects/yolo26l.engine'
        obj.update(weights=str(engine), expected_sha256=sha(engine))
        # Production constructor executes front, side B1/B2 and all-view warmup.
        backend = BatchedYoloDetector(**obj)
        digest = sha(engine)
    write_json(directory/(kind+'-smoke.json'), dict(passed=True, sha256=digest,
        scope='synthetic startup/inference smoke only; accuracy and driving not verified'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='read-only prerequisites; no GPU build')
    parser.add_argument('--resume', type=Path,
                        help='reuse completed engines in this artifact directory; repeat smoke and selection only')
    parser.add_argument('--smoke', choices=['lane', 'objects'], help=argparse.SUPPRESS)
    parser.add_argument('--directory', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.smoke:
        smoke(args.smoke, args.directory)
        return
    obj, lane = camera_params()
    model_manifest = json.loads((ROOT/'config/models/camera_object_models_v2.json').read_text())
    source = next(row for row in model_manifest['files']
                  if row['profile'] == 'finetuned_e8' and row['path'].endswith('.pt'))
    checkpoint = ROOT/'src/asmc_camera/models'/source['path']
    if sha(checkpoint) != source['sha256'] or obj['deployment_profile'] != 'finetuned_e8':
        raise RuntimeError('Expected the pinned E8 object model; no other model is substituted')
    if (obj['front_width'] != 960 or obj['batch_mode'] != '2plus1' or
            lane['model_config'] != 'large' or lane['backend'] != 'twinlite_tensorrt'):
        raise RuntimeError('Camera deployment profile changed; review export shapes/settings first')
    if sha(obj['weights']) != obj['expected_sha256']:
        raise RuntimeError('Team object engine checksum mismatch')
    original_lane = json.loads(Path(lane['engine_manifest']).read_text())
    if sha(lane['checkpoint']) != original_lane['checkpoint_sha256']:
        raise RuntimeError('Team lane checkpoint checksum mismatch')
    import torch
    import tensorrt as trt
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is unavailable')
    print('GPU:', torch.cuda.get_device_name(0), 'TensorRT:', trt.__version__, flush=True)
    required = ['onnx', 'onnxruntime', 'onnxslim', 'ultralytics']
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    if args.check:
        print('Missing build modules:', ', '.join(missing) or 'none')
        print('Run without --check after stopping MORAI/perception; nothing changed.')
        return
    if missing:
        raise RuntimeError('Missing modules: '+', '.join(missing)+
                           '. Use tools/scen_runner/prepare_camera_engines.sh')
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    directory = args.resume.resolve() if args.resume else RUNTIME/'camera_engines'/stamp
    if args.resume:
        if directory.parent != (RUNTIME/'camera_engines').resolve():
            raise RuntimeError('Resume directory must be inside .runtime/camera_engines')
        provenance = json.loads((directory/'objects/yolo26l.engine.provenance.json').read_text())
        if (provenance['checkpoint_sha256'] != sha(checkpoint) or
                provenance['engine_sha256'] != sha(directory/'objects/yolo26l.engine') or
                provenance['head_profile'] != obj['head_profile'] or
                provenance['tensorrt'] != trt.__version__):
            raise RuntimeError('Existing object engine provenance differs from current deployment')
        completed_lane = json.loads((directory/'lane/manifest.json').read_text())
        if (completed_lane.get('completed') is not True or
                completed_lane['checkpoint_sha256'] != sha(lane['checkpoint'])):
            raise RuntimeError('Existing lane engines are incomplete or use another checkpoint')
    else:
        directory.mkdir(parents=True, exist_ok=False)
    def run(*arguments):
        subprocess.run([sys.executable, *map(str, arguments)], cwd=ROOT, check=True)
    print('Artifacts:', directory, flush=True)
    converter = ROOT/'tools/perception/prepare_coco7_models.py'
    if not args.resume:
        run(converter, '--directory', directory/'objects', '--model', 'yolo26l',
            '--head-profile', 'mission16_one2one_nms', '--stage', 'onnx', '--checkpoint', checkpoint)
        run(converter, '--directory', directory/'objects', '--model', 'yolo26l',
            '--head-profile', 'mission16_one2one_nms', '--stage', 'engine')
    rows = original_lane['engines']
    if len({(r['output_mode'], r['precision'], r['fp32_sensitive']) for r in rows}) != 1:
        raise RuntimeError('Mixed lane export settings require review')
    options = ['--fp32-sensitive'] if rows[0]['fp32_sensitive'] else []
    if not args.resume:
        run(ROOT/'tools/perception/prepare_twinlite_tensorrt.py',
            '--checkpoint', lane['checkpoint'], '--output', directory/'lane',
            '--model-config', lane['model_config'], '--output-mode', rows[0]['output_mode'],
            '--precision', rows[0]['precision'], *options)
    for kind in ('objects', 'lane'):
        run(__file__, '--smoke', kind, '--directory', directory)
    object_smoke = json.loads((directory/'objects-smoke.json').read_text())
    lane_smoke = json.loads((directory/'lane-smoke.json').read_text())
    # Empty path leaves production dependencies unchanged; export-only libraries
    # are NOT injected into the camera's inference process.
    dependencies = directory/'runtime_python'
    dependencies.mkdir(exist_ok=True)
    profile = dict(schema_version=1, source_engine_sha256=obj['expected_sha256'],
        engine_path=str(directory/'objects/yolo26l.engine'),
        engine_sha256=object_smoke['sha256'], runtime_pythonpath=str(dependencies),
        gpu_name=torch.cuda.get_device_name(0), tensorrt=trt.__version__,
        validation=dict(object_smoke, engine_sha256=object_smoke['sha256']),
        lane=dict(engine_manifest=str(directory/'lane/manifest.json'),
            source_manifest_sha256=sha(lane['engine_manifest']),
            checkpoint_sha256=sha(lane['checkpoint']),
            validation=dict(lane_smoke, manifest_sha256=lane_smoke['sha256'])))
    target = RUNTIME/'camera_runtime.json'
    if target.exists():
        shutil.copy2(target, directory/'previous-camera_runtime.json')
    temporary = directory/'camera_runtime.json'
    write_json(temporary, profile)
    os.replace(temporary, target)
    print('READY: local object + lane engines selected. Run run_perception.sh.')
    print('Basic inference passed. Real camera accuracy, rate and LiDAR validity still require verification.')


if __name__ == '__main__':
    main()
