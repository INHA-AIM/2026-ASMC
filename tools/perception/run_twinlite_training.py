#!/usr/bin/env python3
"""Bounded, resumable TwinLite training using the project's dataset/loss/evaluator.

Checkpoints are committed only at optimizer boundaries. Full epochs alone enter
best-model selection. A fixed wall deadline survives process restarts.
"""
import argparse
from collections import deque
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import sys
import time
import traceback

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO/'scripts'), str(REPO/'src/asmc_camera/src')]
import train_twinlite as base
from asmc_camera.training.twinlite.batching import ShapeBatchSampler


def utc():
    return datetime.now(timezone.utc)


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.part')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


class StopTraining(Exception):
    pass


class OffsetBatches:
    def __init__(self, sampler):
        self.sampler, self.start = sampler, 0

    def __len__(self):
        return max(0, len(self.sampler)-self.start)

    def __iter__(self):
        for index, batch in enumerate(self.sampler):
            if index >= self.start:
                yield batch


class Reporter:
    def __init__(self, directory, initial):
        self.directory = directory
        self.data = initial
        self.trace = deque(maxlen=1500)
        self.last_write = self.last_trace = -math.inf
        directory.mkdir(parents=True, exist_ok=True)

    def update(self, force=False, **values):
        self.data.update(values)
        now = time.monotonic()
        if not force and now-self.last_write < 5:
            return
        self.last_write = now
        self.data['updated_at_utc'] = utc().isoformat()
        if self.data.get('state') == 'training' and now-self.last_trace >= 60:
            self.last_trace = now
            self.trace.append({key:self.data.get(key) for key in
                ('updated_at_utc','elapsed_seconds','epoch','batch','loss','lr','optimizer_steps')})
            write_json(self.directory/'trace.json',list(self.trace))
        write_json(self.directory/'progress.json',self.data)


class Session:
    def __init__(self, args, reporter, limit):
        self.args, self.reporter, self.limit = args, reporter, limit
        self.deadline = datetime.fromisoformat(limit['deadline_utc'])
        self.requested_stop = False
        self.stop_reason = None
        self.next_batch = 0
        self.optimizer_steps = self.optimizer_attempts = self.amp_skips = 0
        self.epoch_samples = 0
        self.totals = {}
        self.epoch = 1
        self.completed_epochs = 0
        self.history = []
        self.best = {'mean':-1.,'drivable':-1.,'road_marking':-1.}
        self.epoch_started = time.monotonic()
        self.prior_epoch_seconds = 0.

    def remaining(self):
        return (self.deadline-utc()).total_seconds()

    def check_stop(self):
        if self.requested_stop or (self.output/'STOP').exists():
            self.stop_reason = 'stop_requested'
            raise StopTraining()
        if self.remaining() <= 120:
            self.stop_reason = 'time_budget'
            raise StopTraining()

    def report(self, force=False, **values):
        values.setdefault('training_performed',self.optimizer_steps>0)
        self.reporter.update(force=force,
            elapsed_seconds=(utc()-datetime.fromisoformat(self.limit['started_at_utc'])).total_seconds(),
            remaining_seconds=max(0,self.remaining()),epoch=self.epoch,
            completed_epochs=self.completed_epochs,optimizer_steps=self.optimizer_steps,
            optimizer_attempts=self.optimizer_attempts,amp_skipped_updates=self.amp_skips,**values)

    def state(self, complete=False, validation=None):
        return dict(schema_version='twinlite-managed-checkpoint-1.0.0',
            epoch=self.epoch,epoch_complete=complete,next_batch=self.next_batch,
            completed_epochs=self.completed_epochs,args=vars(self.args),
            upstream_commit=self.upstream,task_spec=self.task.as_dict(),
            model_state_dict=self.model.state_dict(),ema_state_dict=self.ema.model.state_dict(),
            ema_updates=self.ema.updates,optimizer_state_dict=self.optimizer.state_dict(),
            scheduler_state_dict=self.scheduler.state_dict(),scaler_state_dict=self.scaler.state_dict(),
            optimizer_steps=self.optimizer_steps,optimizer_attempts=self.optimizer_attempts,
            amp_skips=self.amp_skips,epoch_samples=self.epoch_samples,
            epoch_totals={k:float(v) for k,v in self.totals.items()},
            epoch_elapsed_seconds=self.prior_epoch_seconds+time.monotonic()-self.epoch_started,
            history=self.history,best=self.best,validation=validation,
            initialization=self.initialization,dataset_provenance=self.provenance,
            rng=dict(python=random.getstate(),numpy=np.random.get_state(),
                     torch=torch.get_rng_state(),cuda=torch.cuda.get_rng_state_all()),
            limit=self.limit,training_performed=self.optimizer_steps>0)

    def save_progress(self):
        # This method is called only after step/update and zero_grad, never with
        # unsaved accumulated gradients. A partial epoch is explicitly marked.
        complete = self.completed_epochs==self.epoch and bool(self.history)
        base._atomic_torch_save(self.state(complete=complete,
            validation=self.history[-1]['validation'] if complete else None),self.output/'latest-progress.pt')

    def restore(self, checkpoint):
        if checkpoint['dataset_provenance'] != self.provenance:
            raise RuntimeError('resume dataset SHA mismatch')
        for key in ('epochs','batch_size','accumulate','lr','weight_decay','seed','config'):
            if checkpoint['args'][key] != getattr(self.args,key):
                raise RuntimeError('resume argument mismatch: '+key)
        if checkpoint['upstream_commit'] != self.upstream:
            raise RuntimeError('resume upstream mismatch')
        if checkpoint['limit'] != self.limit:
            raise RuntimeError('resume deadline mismatch')
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.ema.model.load_state_dict(checkpoint['ema_state_dict'])
        self.ema.updates = checkpoint['ema_updates']
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        self.scaler.load_state_dict(checkpoint['scaler_state_dict'])
        self.optimizer_steps = checkpoint['optimizer_steps']
        self.optimizer_attempts = checkpoint['optimizer_attempts']
        self.amp_skips = checkpoint['amp_skips']
        self.completed_epochs = checkpoint['completed_epochs']
        self.history, self.best = checkpoint['history'], checkpoint['best']
        self.epoch = checkpoint['epoch'] + int(checkpoint['epoch_complete'])
        self.next_batch = 0 if checkpoint['epoch_complete'] else checkpoint['next_batch']
        self.epoch_samples = 0 if checkpoint['epoch_complete'] else checkpoint['epoch_samples']
        self.totals = {} if checkpoint['epoch_complete'] else checkpoint['epoch_totals']
        self.prior_epoch_seconds = 0. if checkpoint['epoch_complete'] else checkpoint['epoch_elapsed_seconds']
        self.initialization = checkpoint['initialization']
        random.setstate(checkpoint['rng']['python']); np.random.set_state(checkpoint['rng']['numpy'])
        torch.set_rng_state(checkpoint['rng']['torch'])
        torch.cuda.set_rng_state_all(checkpoint['rng']['cuda'])

    def evaluate(self):
        session = self
        class Tracked:
            def __len__(self):return len(session.val_loader)
            def __iter__(self):
                for index,batch in enumerate(session.val_loader,1):
                    session.check_stop()
                    session.report(state='validating',validation_batch=index,
                                   validation_batches=len(self),phase='full_validation')
                    yield batch
        return base.validate(self.ema.model,Tracked(),self.criterion,self.task,
                             self.device,True,log_interval=0)

    @torch.no_grad()
    def previews(self, label):
        self.report(state='preview',force=True)
        self.ema.model.eval()
        folder = self.monitor/'previews'/label
        folder.mkdir(parents=True,exist_ok=True)
        cards = []
        for index in self.preview_indices:
            self.check_stop()
            sample = self.val[index]
            image = sample['image'].unsqueeze(0).to(self.device)
            with torch.cuda.amp.autocast():
                outputs = base.twinlite_training_step(self.ema.model,
                    {'image':image,'targets':{k:v.unsqueeze(0).to(self.device) for k,v in sample['targets'].items()},
                     'valid_masks':{k:v.unsqueeze(0).to(self.device) for k,v in sample['valid_masks'].items()},
                     'valid_mask':sample['valid_mask'].unsqueeze(0).to(self.device)},self.criterion)[0]
            rgb = (sample['image'].permute(1,2,0).numpy()*255).round().astype(np.uint8)
            panels = [cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)]
            for da,mark in [(sample['targets']['drivable'].numpy(),sample['targets']['road_marking'].numpy()),
                            (outputs['drivable'][0].argmax(0).cpu().numpy(),outputs['road_marking'][0].argmax(0).cpu().numpy())]:
                color = np.zeros_like(panels[0]); color[da>0] = (40,190,40)
                for c,value in [(1,(255,255,255)),(2,(0,230,255)),(3,(0,0,255))]:color[mark==c] = value
                canvas = panels[0].copy(); active = ((da>0)|(mark>0))&sample['valid_mask'].numpy()
                canvas[active] = cv2.addWeighted(panels[0],.5,color,.5,0)[active]
                panels.append(canvas)
            name = str(len(cards)).zfill(2)+'.jpg'
            if not cv2.imwrite(str(folder/name),np.hstack(panels)):
                raise RuntimeError('preview encoding failed')
            row = self.val.rows[index]
            cards.append(dict(image='previews/'+label+'/'+name,sample_id=row['sample_id'],
                              weather=row['weather'],hour=row['hour'],view=row['view']))
        write_json(self.monitor/'previews.json',dict(checkpoint=label,panels=['RGB','GT','prediction'],samples=cards))

    def train_epoch(self):
        self.model.train()
        self.sampler.set_epoch(self.epoch-1)
        self.offset.start = self.next_batch
        total = len(self.sampler)
        self.totals = {k:torch.tensor(v,device=self.device) for k,v in self.totals.items()}
        self.optimizer.zero_grad(set_to_none=True)
        started = time.monotonic(); new_samples = 0; last_report = -math.inf
        group_size = 1
        consecutive_skips = 0
        for number,host in enumerate(self.train_loader,self.next_batch+1):
            if (number-1)%self.args.accumulate == 0:
                self.check_stop()
                group_size = min(self.args.accumulate,total-number+1)
                group_samples = sum(len(b) for b in self.sampler._batches()[number-1:number-1+group_size])
            batch = base._prepare_batch(host,self.device,False)
            with torch.cuda.amp.autocast(enabled=self.device.type=='cuda'):
                _,losses = base.twinlite_training_step(self.model,batch,self.criterion)
            if not torch.isfinite(losses['loss']):
                raise RuntimeError('non-finite training loss at batch '+str(number))
            size = int(batch['image'].shape[0]); self.epoch_samples += size; new_samples += size
            self.scaler.scale(losses['loss']*size/group_samples).backward()
            for key,value in losses.items():
                self.totals[key] = self.totals.get(key,0)+value.detach()*size
            boundary = number%self.args.accumulate == 0 or number==total
            if boundary:
                previous_scale = self.scaler.get_scale()
                self.scaler.step(self.optimizer); self.scaler.update()
                self.optimizer_attempts += 1
                skipped = self.scaler.get_scale() < previous_scale
                if skipped:
                    self.amp_skips += 1; consecutive_skips += 1
                else:
                    self.optimizer_steps += 1; consecutive_skips = 0
                    self.ema.update(self.model)
                self.optimizer.zero_grad(set_to_none=True)
                self.next_batch = number
                if consecutive_skips >= 20:
                    raise RuntimeError('20 consecutive AMP-skipped optimizer updates')
                if number%self.args.checkpoint_batches == 0 or (self.optimizer_steps==1 and not skipped):
                    self.save_progress()
            if time.monotonic()-last_report >= 5 or number==total:
                last_report = time.monotonic()
                elapsed = max(.001,last_report-started)
                self.report(state='training',batch=number,batches=total,
                    epoch_samples=self.epoch_samples,epoch_exposures=self.epoch_exposures,
                    loss=float(self.totals['loss']/self.epoch_samples),
                    losses={k:float(v/self.epoch_samples) for k,v in self.totals.items()},
                    lr=self.optimizer.param_groups[0]['lr'],images_per_second=new_samples/elapsed,
                    epoch_eta_seconds=(self.epoch_exposures-self.epoch_samples)/max(.001,new_samples/elapsed),
                    cuda_allocated_mib=torch.cuda.memory_allocated()/1024**2,
                    cuda_reserved_mib=torch.cuda.memory_reserved()/1024**2,
                    cuda_peak_mib=torch.cuda.max_memory_allocated()/1024**2)
                print('epoch={} batch={}/{} loss={:.5f} optimizer_steps={} skipped={}'.format(
                    self.epoch,number,total,float(self.totals['loss']/self.epoch_samples),
                    self.optimizer_steps,self.amp_skips),flush=True)
        return {k:float(v/self.epoch_samples) for k,v in self.totals.items()}


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',required=True)
    p.add_argument('--dataset-version',required=True)
    p.add_argument('--init-checkpoint',required=True)
    p.add_argument('--twinlite-root',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--monitor',required=True)
    p.add_argument('--resume')
    p.add_argument('--config',default='large')
    p.add_argument('--epochs',type=int,default=40)
    p.add_argument('--max-hours',type=float,default=24.)
    p.add_argument('--batch-size',type=int,default=4)
    p.add_argument('--accumulate',type=int,default=3)
    p.add_argument('--num-workers',type=int,default=2)
    p.add_argument('--checkpoint-batches',type=int,default=1500)
    p.add_argument('--lr',type=float,default=.0002)
    p.add_argument('--weight-decay',type=float,default=.0005)
    p.add_argument('--seed',type=int,default=20261017)
    return p.parse_args()


def run(a):
    if min(a.epochs,a.max_hours,a.batch_size,a.accumulate,a.checkpoint_batches)<=0 or a.num_workers<0:
        raise ValueError('invalid training limits')
    if a.checkpoint_batches%a.accumulate:
        raise ValueError('checkpoint interval must end at an optimizer boundary')
    output,monitor = Path(a.output),Path(a.monitor)
    if output.exists() and not a.resume:
        raise FileExistsError('new run cannot overwrite an existing output')
    output.mkdir(parents=True,exist_ok=True); monitor.mkdir(parents=True,exist_ok=True)
    limit_path = output/'limit.json'
    if a.resume:
        limit = json.loads(limit_path.read_text())
    else:
        start = utc()
        limit = dict(started_at_utc=start.isoformat(),deadline_utc=(start+timedelta(hours=a.max_hours)).isoformat(),max_hours=a.max_hours)
        write_json(limit_path,limit)
    reporter = Reporter(monitor,dict(run_id=output.name,state='loading',epochs_planned=a.epochs,
        **limit,training_performed=False,configuration=vars(a),test_used=False))
    session = Session(a,reporter,limit); session.output=output; session.monitor=monitor
    def stop(signum,frame):session.requested_stop=True
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    session.report(force=True)
    try:
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; CPU training is not an automatic fallback')
        cv2.setNumThreads(1);torch.set_num_threads(2);base.seed_everything(a.seed)
        torch.backends.cudnn.benchmark=True
        session.device=torch.device('cuda')
        root=Path(a.data_root)/'dataset_versions'/a.dataset_version
        meta=json.loads((root/'dataset.json').read_text())
        if base._sha256(root/'dataset.json') != json.loads((root/'_SUCCESS').read_text())['dataset_sha256']:
            raise RuntimeError('dataset metadata SHA mismatch')
        for split in ('train','val','test'):
            if base._sha256(root/(split+'.jsonl')) != meta['split_manifest_sha256'][split]:
                raise RuntimeError('split SHA mismatch: '+split)
        session.provenance=dict(dataset_version=a.dataset_version,metadata_sha256=base._sha256(root/'dataset.json'),
                                split_sha256=meta['split_manifest_sha256'])
        session.train=base.MoraiTwinLiteDataset(a.data_root,a.dataset_version,'train',verify_files=False)
        session.val=base.MoraiTwinLiteDataset(a.data_root,a.dataset_version,'val',verify_files=False)
        session.task=session.train.task_spec
        session.sampler=ShapeBatchSampler(session.train.rows,a.batch_size,a.seed,balance_conditions=True)
        session.offset=OffsetBatches(session.sampler)
        session.epoch_exposures=sum(len(b) for b in session.sampler)
        session.train_loader=DataLoader(session.train,batch_sampler=session.offset,num_workers=a.num_workers,
            pin_memory=True,persistent_workers=a.num_workers>0,generator=torch.Generator().manual_seed(a.seed))
        session.val_loader=DataLoader(session.val,batch_sampler=ShapeBatchSampler(session.val.rows,a.batch_size,shuffle=False),
            num_workers=a.num_workers,pin_memory=True,persistent_workers=a.num_workers>0,
            generator=torch.Generator().manual_seed(a.seed+1))
        session.model,session.upstream=base.load_external_twinlite(config=a.config,root=a.twinlite_root)
        reset=base.configure_twinlite_task(session.model,session.task)
        source=torch.load(a.init_checkpoint,map_location='cpu')
        if source['upstream_commit']!=session.upstream:raise RuntimeError('initial checkpoint upstream mismatch')
        transfer=base.load_task_compatible_state_dict(session.model,source['model_state_dict'],
            base.task_from_checkpoint(source),session.task,reset)
        session.initialization=dict(checkpoint_sha256=base._sha256(Path(a.init_checkpoint)),**transfer)
        session.model=session.model.to(session.device)
        session.ema=base.ModelEMA(session.model)
        session.optimizer=torch.optim.AdamW(session.model.parameters(),lr=a.lr,weight_decay=a.weight_decay)
        session.scheduler=torch.optim.lr_scheduler.LambdaLR(session.optimizer,
            lambda epoch:max(0.,1.-epoch/a.epochs)**1.5)
        session.scaler=torch.cuda.amp.GradScaler()
        session.criterion=base.MaskedTwinLiteLoss(task_spec=session.task).to(session.device)
        cells={}
        for index,row in enumerate(session.val.rows):
            key=(row['weather'],str(row['hour']),row['view'])
            if key not in cells:cells[key]=[]
            cells[key].append(index)
        session.preview_indices=[indices[len(indices)//2] for key,indices in sorted(cells.items())]
        configuration=dict(arguments=vars(a),dataset_provenance=session.provenance,
            train_images=len(session.train),val_images=len(session.val),epoch_exposures=session.epoch_exposures,
            train_batches=len(session.sampler),initialization=session.initialization,
            upstream_commit=session.upstream,torch_version=torch.__version__,gpu=torch.cuda.get_device_name(0),
            task=session.task.as_dict(),effective_batch=a.batch_size*a.accumulate,
            selection_metric=meta['selection_metric'],augmentation='none; native semantic target policy retained',
            source_files_sha256={str(p.relative_to(REPO)):base._sha256(p) for p in
                [Path(__file__).resolve(),REPO/'scripts/train_twinlite.py',
                 REPO/'src/asmc_camera/src/asmc_camera/training/twinlite/dataset.py',
                 REPO/'src/asmc_camera/src/asmc_camera/training/twinlite/batching.py']},**limit)
        write_json(output/'config.json',configuration);write_json(monitor/'config.json',configuration)
        if a.resume:
            session.restore(torch.load(a.resume,map_location='cpu'))
            write_json(monitor/'history.json',session.history)
        else:
            base._atomic_torch_save(session.state(),output/'initial.pt')
            initial=session.evaluate()
            write_json(output/'initial-validation.json',initial);write_json(monitor/'initial-validation.json',initial)
            session.previews('initial')
        torch.cuda.reset_peak_memory_stats()
        while session.epoch<=a.epochs:
            session.check_stop()
            if session.history and session.next_batch==0:
                estimate=float(np.mean([h['epoch_seconds'] for h in session.history[-3:]]))
                if session.remaining()<1.2*estimate+120:
                    session.stop_reason='insufficient_time_for_complete_epoch';raise StopTraining()
            session.epoch_started=time.monotonic()
            session.report(state='training',force=True,batch=session.next_batch,batches=len(session.sampler),
                           epoch_exposures=session.epoch_exposures,training_performed=session.optimizer_steps>0)
            losses=session.train_epoch()
            session.save_progress()
            validation=session.evaluate()
            scores=validation['six_condition_macro_head_scores']
            if len(scores)!=2:raise RuntimeError('six-condition validation coverage is incomplete')
            mean=validation['six_condition_macro_mean_head_iou']
            row=dict(epoch=session.epoch,train_losses=losses,validation=validation,
                lr=session.optimizer.param_groups[0]['lr'],optimizer_steps=session.optimizer_steps,
                amp_skips=session.amp_skips,epoch_seconds=session.prior_epoch_seconds+time.monotonic()-session.epoch_started)
            session.scheduler.step();session.completed_epochs=session.epoch
            session.history.append(row)
            improved=[]
            for name,value in dict(mean=mean,**scores).items():
                if value>session.best[name]:session.best[name]=value;improved.append(name)
            state=session.state(complete=True,validation=validation)
            base._atomic_torch_save(state,output/('epoch{:03d}.pt'.format(session.epoch)))
            base._atomic_torch_save(state,output/'latest.pt')
            for name in improved:base._atomic_torch_save(state,output/('best_'+name+'.pt'))
            session.save_progress()
            write_json(output/'history.json',session.history);write_json(monitor/'history.json',session.history)
            session.report(state='epoch_complete',force=True,best=session.best,latest_validation=validation,
                           last_complete_checkpoint='epoch{:03d}.pt'.format(session.epoch),training_performed=True)
            print('COMPLETED EPOCH {} six_condition_macro={:.6f}'.format(session.epoch,mean),flush=True)
            session.previews('epoch{:03d}'.format(session.epoch))
            session.epoch+=1;session.next_batch=0;session.epoch_samples=0;session.totals={};session.prior_epoch_seconds=0.
        session.epoch=a.epochs
        session.report(state='completed',force=True,training_performed=True,best=session.best)
    except StopTraining:
        if hasattr(session,'model'):
            # Never save an in-progress accumulation: stops are checked only at
            # group starts, or inside no-grad validation/preview.
            session.save_progress()
        session.report(state='stopped',force=True,stop_reason=session.stop_reason,
                       training_performed=session.optimizer_steps>0,best=session.best)
    except Exception as error:
        session.report(state='error',force=True,error=str(error),traceback=traceback.format_exc(),
                       training_performed=session.optimizer_steps>0)
        raise
    finally:
        write_json(output/'result.json',dict(session.reporter.data))


if __name__=='__main__':
    run(arguments())
