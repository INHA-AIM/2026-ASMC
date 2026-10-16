"""Stable producer/configuration identity without rewriting the image header."""
import hashlib
import copy
import json
import uuid
from pathlib import Path


def digest(value):
    return 'sha256:' + hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                               allow_nan=False).encode()).hexdigest()


class ObservationIdentity:
    def __init__(self, calibration, model_id, config, session_id=None):
        self.session_id = session_id or str(uuid.uuid4())
        self.model_id = model_id
        self.config_id = digest(config)
        self.calibrations = {'camera_' + view: digest(dict(camera=values,
                             source_sha256=calibration.get('source_sha256', '')))
                             for view, values in calibration['cameras'].items()}

    @classmethod
    def from_parameters(cls, parameters, backend, session_id):
        import yaml
        import rospkg
        package = Path(rospkg.RosPack().get_path('asmc_camera'))
        path = parameters.get('~calibration_config', str(package / 'config/cameras_v3.yaml'))
        calibration = yaml.safe_load(Path(path).read_text())
        keys = ('views', 'front_width', 'batch_mode', 'batch_wait_ms', 'target_hz',
                'conf_thres', 'nms_iou', 'head_profile', 'class_groups', 'label_mode',
                'hood_mask', 'side_ego_mask', 'deployment_profile', 'target_hz_by_view',
                'image_topic', 'image_topics', 'predict_overrides', 'require_cuda',
                'ego_hood_filter', 'model_kind', 'task', 'runtime_backend', 'allowed_names')
        config = {key: parameters.get('~' + key, None) for key in keys}
        config.update(runtime_layout=backend.get('runtime_layout','separate'), tensor_shapes=backend.get('shapes',{}))
        for key in ('hood_mask_config', 'ego_hood_filter_config', 'calibration_config'):
            configured = parameters.get('~' + key, str(path) if key == 'calibration_config' else '')
            if configured and Path(configured).is_file():
                config[key + '_sha256'] = hashlib.sha256(Path(configured).read_bytes()).hexdigest()
        config['side_ego_mask_sha256'] = {view: hashlib.sha256(Path(filename).read_bytes()).hexdigest()
            for view, filename in parameters.get('~side_ego_mask_configs', {}).items()}
        return cls(calibration, backend.get('weights_sha256') or parameters.get('~expected_sha256', ''),
                   config, session_id)

    def wrap(self, detections, generation, message_type):
        result = message_type()
        result.detections = copy.copy(detections)
        result.detections.header = copy.deepcopy(detections.header)
        result.producer_session_id = self.session_id
        result.generation = generation
        result.calibration_id = self.calibrations.get(detections.header.frame_id, '')
        result.model_id, result.config_id = self.model_id, self.config_id
        if detections.valid and not result.calibration_id:
            raise ValueError('unknown camera calibration: ' + detections.header.frame_id)
        return result
