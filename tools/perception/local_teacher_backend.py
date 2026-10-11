"""Offline bbox teachers. Imports heavy libraries only inside their own process.

Every backend returns original-image xyxy and the unmodified detector phrase.
Unknown phrases are proposals, never invented ground truth. No SAM/ROS imports.
"""
from pathlib import Path


class LocalTeacher:
    def __init__(self, name, config, models, source, contrast=False):
        import torch
        self.torch = torch
        self.name = name
        self.prompts = config['prompts']
        self.spec = dict(config['models'][name])
        if contrast:
            self.spec.update(config['oom_contrast'])
        self.minimum = min(config['confidence_sweep'])
        self.device = 'cuda:0'
        if not torch.cuda.is_available():
            raise RuntimeError('GPU unavailable; no silent CPU fallback')
        if name == 'yoloe_x':
            from asmc_camera.detection.segmentation import SegmentationDetector, load_prompt_embeddings
            from camera_data_audit import sha
            weights = models / self.spec['checkpoint']
            names, embeddings = load_prompt_embeddings(models / self.spec['embeddings'], sha(weights))
            if names != self.prompts:
                raise ValueError('checkpoint-bound prompt cache differs from experiment')
            self.model = SegmentationDetector(weights=str(weights), device='cuda:0',
                conf_thres=self.minimum, allowed_names=names,
                model_kind='yoloe', prompt_embeddings=str(models / self.spec['embeddings']),
                predict_overrides=dict(imgsz=self.spec.get('yolo_front_width', self.spec['front_width']),
                                       half=True, iou=.7, max_det=300, agnostic_nms=False, end2end=False))
        elif name == 'gdino_b':
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
            directory = models / 'teacher_v6' / 'grounding-dino-base'
            self.processor = AutoProcessor.from_pretrained(str(directory), local_files_only=True)
            self.processor.image_processor.size = {'shortest_edge': self.spec['short_edge'],
                                                    'longest_edge': self.spec['long_edge']}
            self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
                str(directory), local_files_only=True).to(self.device).eval()
        elif name == 'mm_gdino_l':
            from mmengine.config import Config
            from mmdet.apis import init_detector
            cfg = Config.fromfile(str(source / 'mmdetection' / self.spec['config']))
            cfg.model.backbone.init_cfg = None
            # These training-only wrappers do not change no-grad inference or
            # checkpoint parameters; avoid requiring fairscale for evaluation.
            cfg.model.backbone.with_cp = False
            cfg.model.encoder.num_cp = 0
            cfg.model.language_model.name = str(models / 'teacher_v6' / 'bert-base-uncased')
            cfg.model.use_autocast = True
            # ndarray pipeline keeps original shape metadata for rescale=True.
            self.pipeline = [dict(type='mmdet.LoadImageFromNDArray'),
                dict(type='mmdet.FixScaleResize', scale=(self.spec['long_edge'], self.spec['short_edge']), keep_ratio=True),
                dict(type='mmdet.PackDetInputs', meta_keys=('img_id', 'img_path', 'ori_shape', 'img_shape', 'scale_factor', 'text', 'custom_entities'))]
            # MM's sentence-block mask is 3-D. New Transformers defaults to
            # SDPA's 2-D padding-mask path; use the reference eager BERT path.
            import mmdet.models.language_models.bert as mm_bert
            original_bert=mm_bert.HFBertModel
            class EagerBert:
                @staticmethod
                def from_pretrained(*args,**kwargs):
                    return original_bert.from_pretrained(*args,attn_implementation='eager',**kwargs)
            mm_bert.HFBertModel=EagerBert
            try:
                self.model = init_detector(cfg, str(models / 'teacher_v6' / self.spec['checkpoint']), device=self.device)
            finally:
                mm_bert.HFBertModel=original_bert
        else:
            raise ValueError('unknown local teacher: ' + name)

    def predict(self, bgr, view):
        torch = self.torch
        with torch.inference_mode():
            if self.name == 'yoloe_x':
                width = self.spec.get('yolo_front_width', self.spec['front_width']) if view == 'front' else self.spec['side_width']
                self.model.backend.predict_overrides['imgsz'] = width
                ds, _ = self.model.detect(bgr)
                return [dict(raw_phrase=d.source_class_name, confidence=float(d.confidence),
                             xyxy=[float(d.xmin), float(d.ymin), float(d.xmax), float(d.ymax)]) for d in ds]
            if self.name == 'gdino_b':
                from PIL import Image
                text = '. '.join(self.prompts) + '.'
                inputs = self.processor(images=Image.fromarray(bgr[:, :, ::-1]), text=text, return_tensors='pt').to(self.device)
                with torch.autocast('cuda', dtype=torch.float16):
                    output = self.model(**inputs)
                result = self.processor.post_process_grounded_object_detection(output, inputs.input_ids,
                    box_threshold=self.minimum, text_threshold=self.spec['text_threshold'], target_sizes=[bgr.shape[:2]])[0]
                labels = result.get('text_labels', result.get('labels'))
                return [dict(raw_phrase=str(label), confidence=float(score), xyxy=[float(x) for x in box])
                        for box, score, label in zip(result['boxes'].cpu(), result['scores'].cpu(), labels)]
            from mmcv.transforms import Compose
            from mmdet.apis import inference_detector
            result = inference_detector(self.model, bgr, test_pipeline=Compose(self.pipeline),
                                        text_prompt=self.prompts, custom_entities=True)
            instances = result.pred_instances.cpu()
            return [dict(raw_phrase=self.prompts[int(label)], confidence=float(score), xyxy=[float(x) for x in box])
                    for box, score, label in zip(instances.bboxes, instances.scores, instances.labels)
                    if float(score) >= self.minimum and 0 <= int(label) < len(self.prompts)]
