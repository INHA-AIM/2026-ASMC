#!/usr/bin/env python3
"""Pinned, serial bbox pilot training with explicit smoke, budget and resume records.

Runs in the isolated training image, not the shared ROS Python environment.
Smoke results are discarded; main training starts from the original weights.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import time
from copy import deepcopy


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def save(path, data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=str)+'\n');tmp.replace(path)


def common_epochs(seconds_per_epoch, budget=10800, margin=1.2, maximum=20):
    if len(seconds_per_epoch)!=2 or any(not math.isfinite(v) or v<=0 for v in seconds_per_epoch):
        raise ValueError('two valid timing measurements required')
    return min(maximum,int(budget/(max(seconds_per_epoch)*margin)))


def validate_resume(record, current):
    if record.get('stage')=='smoke':raise ValueError('smoke checkpoint is not a full epoch')
    for key in ('model_key','config_sha256','manifest_sha256','initial_weights_sha256','batch','epochs'):
        if record.get(key)!=current.get(key):raise ValueError('resume identity mismatch: '+key)


def run(args):
    import torch
    import ultralytics
    import yaml
    from ultralytics import YOLO,YOLOE
    from ultralytics.models.yolo.yoloe import YOLOEPETrainer
    from ultralytics.models.yolo.detect import DetectionTrainer
    import ultralytics.utils.checks as checks
    import ultralytics.utils as utils
    checks.WEIGHTS_DIR=args.data_root/'models/camera_bbox_pilot_v1/amp-check'
    utils.WEIGHTS_DIR=checks.WEIGHTS_DIR
    if not (checks.WEIGHTS_DIR/'yolo26n.pt').is_file():
        raise ValueError('pre-stage official AMP-check checkpoint; no in-run downloads')

    cfg=json.loads(args.config.read_text()); settings=cfg['training']
    if args.batch not in settings['batch_candidates'] or settings['nbs']%args.batch:
        raise ValueError('batch must preserve the common effective batch exactly')
    data=yaml.safe_load((args.dataset/'data.yaml').read_text());names=data['names']
    if len(names)!=16 or len(cfg['text_names'])!=16:raise ValueError('fixed 16-class contract required')
    weights=args.data_root/cfg['models'][args.model]
    identity=dict(model_key=args.model,config_sha256=sha(args.config),
        manifest_sha256=sha(args.dataset/'manifest.jsonl'),initial_weights_sha256=sha(weights),
        batch=args.batch,epochs=args.epochs)
    args.output.mkdir(parents=True,exist_ok=True)
    state=args.output/'run.json'
    if state.exists() and not args.resume:raise ValueError('existing run requires explicit --resume')
    if args.resume:
        previous=json.loads(state.read_text())
        validate_resume(previous,identity)
        ckpt=torch.load(args.resume,map_location='cpu')
        if ckpt.get('epoch',-1)<0 or ckpt.get('optimizer') is None:
            raise ValueError('checkpoint has no resumable optimizer/epoch')
        if previous.get('budget_exceeded') and ckpt['epoch']>=previous.get('last_epoch',0):
            raise ValueError('budget-stopped last epoch is partial; resume preceding complete epoch checkpoint')
        history=args.output/'resume-history';history.mkdir(exist_ok=True)
        save(history/(sha(state)+'.json'),previous)
        del ckpt
    started=time.monotonic(); batches=[];epoch_times=[];order=hashlib.sha256()
    record=dict(**identity,stage='smoke' if args.smoke_batches else 'training',status='starting',
        started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        python=platform.python_version(),torch=torch.__version__,ultralytics=ultralytics.__version__,
        cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0),text_names=cfg['text_names'],
        canonical_names=names,resume_checkpoint=str(args.resume) if args.resume else None,
        resume_checkpoint_sha256=sha(args.resume) if args.resume else None,
        resume_policy='epoch-boundary optimizer restore; unfinished epoch is replayed',
        intended_effective_batch=settings['nbs'],sample_order_sha256=None,
        errors=[],runtime_changes=False)
    save(state,record)

    class TextBBoxTrainer(YOLOEPETrainer):
        def get_model(self,*a,**kw):
            weights=kw.get('weights')
            if weights is not None and getattr(weights.model[-1],'is_fused',False):
                # PE checkpoints already contain the learned class convolutions.
                # Re-embedding would discard these and reprta was removed at fuse.
                if weights.model[-1].nc!=len(names) or list(weights.names.values())!=names:
                    raise ValueError('fused resume vocabulary differs from dataset')
                return deepcopy(weights).float()
            original=self.data['names']
            self.data['names']=dict(enumerate(cfg['text_names']))
            try:model=super().get_model(*a,**kw)
            finally:self.data['names']=original
            model.names=original
            return model

    def train_start(t):
        from ultralytics.utils.torch_utils import init_seeds
        init_seeds(cfg['seed'], deterministic=True)
        record.update(status='running',actual_batch=t.batch_size,
            train_images=len(t.train_loader.dataset),batches_per_epoch=len(t.train_loader),
            trainable_parameters=sum(p.numel() for p in t.model.parameters() if p.requires_grad),
            parameters=sum(p.numel() for p in t.model.parameters()),amp_enabled=t.amp,
            actual_task=t.args.task,actual_head=type(t.model.model[-1]).__name__,
            actual_args=vars(t.args),dataset_cache_sha256=None)
        if not t.amp:raise RuntimeError('AMP silently disabled')
        if t.batch_size!=args.batch:raise RuntimeError('silent batch change')
        if t.args.task!='detect' or 'Segment' in type(t.model.model[-1]).__name__:
            raise RuntimeError('bbox candidate still has segmentation head')
        save(state,record)
        if not args.smoke_batches and not args.resume:
            initial=args.output/'initial.pt'
            torch.save(dict(model=deepcopy(t.model).cpu().half(),ema=None,
                train_args=vars(t.args),epoch=-1,version=ultralytics.__version__),initial)
            record['before_training_checkpoint']=dict(path=str(initial),sha256=sha(initial))
            save(state,record)
    def batch_start(t):
        t._pilot_batch_start=time.monotonic()
    def batch_end(t):
        torch.cuda.synchronize()
        duration=time.monotonic()-t._pilot_batch_start
        losses={k:float(v.detach().cpu()) for k,v in t.loss_items.items()}
        if not all(math.isfinite(v) for v in losses.values()):raise RuntimeError('nonfinite loss')
        if t.batch_size!=args.batch:raise RuntimeError('automatic OOM batch reduction is forbidden')
        batches.append(duration)
        order.update(('\n'.join(map(str,t.batch.get('im_file',[]))) if hasattr(t,'batch') else '').encode())
        if args.smoke_batches and len(batches)>=args.smoke_batches:t.stop=True
        if time.monotonic()-started>args.max_seconds:
            record['budget_exceeded']=True;t.stop=True
        if len(batches)%25==0 or t.stop:
            record.update(completed_batches=len(batches),last_loss=losses,
                batch_seconds=sum(batches[-100:])/len(batches[-100:]),
                gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
                gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved(),
                wall_seconds=time.monotonic()-started)
            save(state,record)
    def epoch_end(t):
        epoch_times.append(t.epoch_time or 0)
        record.update(last_epoch=t.epoch,epoch_seconds=epoch_times,
            resumable_checkpoint=str(t.last),completed_batches=len(batches),
            wall_seconds=time.monotonic()-started)
        save(state,record)
        if args.stop_after_epoch and t.epoch+1>=args.stop_after_epoch:
            record['epoch_boundary_pause']=True;t.stop=True;save(state,record)
    model=YOLO(str(args.resume or weights)) if args.model=='yolo26l' else (
        YOLOE(str(args.resume)) if args.resume else YOLOE('yoloe-26l.yaml').load(str(weights)))
    for event,cb in [('on_train_start',train_start),('on_train_batch_start',batch_start),
                     ('on_train_batch_end',batch_end),('on_fit_epoch_end',epoch_end)]:
        model.add_callback(event,cb)
    try:
        # nbs controls accumulation/weight decay. Warmup accumulation is fixed by
        # preprocess_batch, after Ultralytics' LR warmup interpolation.
        Base=TextBBoxTrainer if args.model=='yoloe26l' else DetectionTrainer
        class PilotTrainer(Base):
            def validate(self):
                # Partial labels must not create false FP rankings. External
                # evaluator separates complete GT from recall-only annotations.
                return self.metrics,0.0
            def preprocess_batch(self,batch):
                self.accumulate=max(1,settings['nbs']//args.batch)
                self.batch=batch
                order.update(('\n'.join(map(str,batch.get('im_file',[])))+'\n').encode())
                return super().preprocess_batch(batch)
            def final_eval(self):
                # Preserve optimizer in last.pt for epoch-boundary resume.
                # Final evaluation is performed once by the external comparator.
                return
        kwargs=dict(data=str(args.dataset/'data.yaml'),imgsz=settings['imgsz'],batch=args.batch,
            nbs=settings['nbs'],epochs=args.epochs,amp=True,optimizer='AdamW',lr0=settings['lr0'],
            weight_decay=settings['weight_decay'],freeze=None,seed=cfg['seed'],deterministic=True,
            device=0,workers=2,cache=False,plots=False,val=False,patience=0,
            project=str(args.output),name='fit',exist_ok=bool(args.resume),save=True,save_period=1,
            close_mosaic=0,mosaic=0.5,mixup=0,copy_paste=0,degrees=0,translate=.1,scale=.5,
            fliplr=.5,flipud=0,hsv_h=.015,hsv_s=.7,hsv_v=.4,
            warmup_epochs=1.0,warmup_bias_lr=0.01,verbose=False)
        if args.resume:kwargs['resume']=str(args.resume)
        model.train(trainer=PilotTrainer,**kwargs)
        settled=batches[20:] or batches
        record.update(status='budget_stopped' if record.get('budget_exceeded') else
            'paused_at_epoch' if record.get('epoch_boundary_pause') else 'complete',
            completed_batches=len(batches),completed_epochs=0 if args.smoke_batches else
                record.get('last_epoch',-1)+1-int(bool(record.get('budget_exceeded'))),
            smoke_complete=bool(args.smoke_batches and len(batches)==args.smoke_batches),
            batch_seconds=sum(settled)/len(settled),wall_seconds=time.monotonic()-started,
            sample_order_sha256=order.hexdigest(),gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
            gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved())
        record['epoch_seconds_estimate']=record['batch_seconds']*record['batches_per_epoch']
        for f in (args.output/'fit/weights').glob('*.pt'):
            record.setdefault('checkpoints',{})[f.name]=dict(path=str(f),sha256=sha(f))
        save(state,record)
    except BaseException as e:
        record.update(status='failed',completed_batches=len(batches),wall_seconds=time.monotonic()-started)
        record['errors'].append(dict(type=type(e).__name__,message=str(e)))
        save(state,record);raise
    return record


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--data-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',choices=['yolo26l','yoloe26l'],required=True)
    p.add_argument('--batch',type=int,required=True);p.add_argument('--epochs',type=int,default=20)
    p.add_argument('--smoke-batches',type=int,default=0);p.add_argument('--max-seconds',type=float,default=10800)
    p.add_argument('--resume',type=Path)
    p.add_argument('--stop-after-epoch',type=int,help='explicit epoch-boundary pause for recovery testing')
    print(json.dumps(run(p.parse_args()),ensure_ascii=False,indent=2))
