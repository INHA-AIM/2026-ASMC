#!/usr/bin/env python3
"""Serial pilot orchestration using two completed, same-batch smoke runs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from train_camera_bbox_pilot import common_epochs,save,sha


def validate_equal_training(records,epoch,minimum=5):
    if len(records)!=2 or epoch<minimum:raise ValueError('two sufficiently trained candidates required')
    reference=records[0]
    for r in records:
        if r['status'] not in ('complete','budget_stopped'):
            raise ValueError('failed/OOM training is not an equal-checkpoint comparison')
        if r.get('completed_epochs',0)<epoch:raise ValueError('requested epoch is not complete in both runs')
        for key in ('config_sha256','manifest_sha256','batch','epochs'):
            if r[key]!=reference[key]:raise ValueError('candidate training identity differs: '+key)


def evaluate_equal_checkpoint(args):
    """Preserve a failed budget run, compare only equal fully saved epochs."""
    repo=Path(__file__).resolve().parents[2];root=args.output
    original=json.loads((root/'comparison.json').read_text())
    if original['status']!='failed' or original['stage']!='training':
        raise ValueError('equal-checkpoint recovery is only for a preserved failed training budget')
    records=[json.loads((root/m/'run.json').read_text()) for m in ('yolo26l','yoloe26l')]
    cfg=json.loads(args.config.read_text());epoch=args.common_epoch
    validate_equal_training(records,epoch,cfg['training']['epochs_min'])
    if records[0]['config_sha256']!=sha(args.config) or records[0]['manifest_sha256']!=sha(args.dataset/'manifest.jsonl'):
        raise ValueError('evaluation config/dataset differs from actual training')
    if not any(r['status']=='budget_stopped' for r in records):raise ValueError('no wall-budget stop to recover')
    proof=json.loads(args.order_proof.read_text())
    if proof['status']!='complete' or len(proof['checks'])!=2 or not all(c['matched'] for c in proof['checks']):
        raise ValueError('actual complete/partial training order is not independently verified')
    if proof['manifest_sha256']!=sha(args.dataset/'manifest.jsonl'):raise ValueError('sampler proof dataset mismatch')
    for model,r in zip(('yolo26l','yoloe26l'),records):
        check=next((c for c in proof['checks'] if Path(c['run'])==root/model/'run.json'),None)
        if not check or check['actual_sha256']!=r['sample_order_sha256'] or check['batches']!=r['completed_batches']:
            raise ValueError('sampler proof is stale')
    state=root/'comparison-equal-epoch.json'
    if state.exists():raise ValueError('recovery record already exists; preserve failed evaluation')
    record=dict(status='running',stage='equal_checkpoint_verification',epochs=epoch,batch=records[0]['batch'],
        original_comparison=str(root/'comparison.json'),original_requested_epochs=records[0]['epochs'],
        actual_completed_epochs=[r['completed_epochs'] for r in records],
        frozen_training_schedule='original identical requested epoch schedule; later updates excluded from selected checkpoints',
        original_failure_preserved=True,manifest_sha256=sha(args.dataset/'manifest.jsonl'),
        config_sha256=sha(args.config),order_proof_sha256=sha(args.order_proof),
        common_sample_order_sha256=proof['prefix_sha256']['epoch:'+str(epoch)],same_sample_order=True,
        jobs=[],runtime_promoted=False,model_selection_status='pending evaluation')
    save(state,record);started=time.monotonic()
    def docker(script):return ['docker','exec',args.container,'/usr/bin/python3',str(repo/'tools/perception'/script)]
    try:
        for model,r in zip(('yolo26l','yoloe26l'),records):
            checkpoint=root/model/'fit/weights'/f'epoch{epoch-1}.pt'
            code='import sys,json,torch;d=torch.load(sys.argv[1],map_location="cpu");print(json.dumps(dict(epoch=d["epoch"],updates=d["updates"],optimizer_present=d.get("optimizer") is not None,head=type(d["ema"].model[-1]).__name__)))'
            command=['docker','exec',args.container,'/usr/bin/python3','-c',code,str(checkpoint)]
            metadata=json.loads(subprocess.check_output(command,text=True))
            expected=epoch*r['batches_per_epoch']*r['batch']//cfg['training']['nbs']
            if metadata['epoch']!=epoch-1 or metadata['updates']!=expected or not metadata['optimizer_present']:
                raise ValueError('checkpoint epoch/optimizer update amount differs')
            if metadata['head'] not in ('Detect','YOLOEDetect'):raise ValueError('not a bbox checkpoint')
            record.setdefault('checkpoints',{})[model]=dict(path=str(checkpoint),sha256=sha(checkpoint),**metadata)
            save(state,record)
        for model in ('yolo26l','yoloe26l'):
            for width in (960,1280):
                for phase in ('before','after'):
                    weights=root/model/'initial.pt' if phase=='before' else Path(record['checkpoints'][model]['path'])
                    output=root/f'{model}-{phase}-{width}'
                    command=docker('evaluate_camera_bbox_pilot.py')+[
                        '--config',str(args.config),'--dataset',str(args.dataset),'--weights',str(weights),
                        '--front-width',str(width),'--output',str(output)]
                    entry=dict(stage='evaluation',model=f'{model}-{phase}-{width}',status='running',command=command,
                        log=str(root/f'{model}-{phase}-{width}.log'))
                    record['jobs'].append(entry);record['stage']='evaluation';save(state,record)
                    with Path(entry['log']).open('w') as stream:
                        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
                    entry.update(exit_code=result.returncode,status='complete' if result.returncode==0 else 'failed')
                    save(state,record)
                    if result.returncode:raise RuntimeError('recovery evaluation failed; original output preserved')
        record.update(status='complete',stage='evaluation_complete',wall_seconds=time.monotonic()-started,
            model_selection_status='requires coverage and regression review; no automatic runtime promotion')
        save(state,record)
    except BaseException as e:
        record.update(status='failed',error=repr(e),wall_seconds=time.monotonic()-started);save(state,record);raise


def run(args):
    repo=Path(__file__).resolve().parents[2];root=repo.parent
    cfg=json.loads(args.config.read_text());settings=cfg['training']
    a=json.loads(args.smoke_a.read_text());b=json.loads(args.smoke_b.read_text())
    for record in (a,b):
        if record['status']!='complete' or not record['smoke_complete'] or record['completed_batches']!=200:
            raise ValueError('both 200-batch smoke runs must pass')
        if record['manifest_sha256']!=sha(args.dataset/'manifest.jsonl'):
            raise ValueError('smoke dataset mismatch')
    if a['batch']!=b['batch']:raise ValueError('common batch not established')
    epochs=common_epochs([a['epoch_seconds_estimate'],b['epoch_seconds_estimate']],
        settings['per_model_budget_seconds'],settings['epoch_margin'],settings['epochs_max'])
    if epochs<settings['epochs_min']:raise ValueError('prepare the policy-identical 4000-image fallback first')
    args.output.mkdir(parents=True,exist_ok=True)
    state=args.output/'comparison.json'
    if state.exists():raise ValueError('comparison output already exists; do not silently retrain')
    record=dict(status='running',stage='training',batch=a['batch'],epochs=epochs,
        config_sha256=sha(args.config),manifest_sha256=sha(args.dataset/'manifest.jsonl'),
        dataset=str(args.dataset),smoke_files=[str(args.smoke_a),str(args.smoke_b)],
        estimated_epoch_seconds=[a['epoch_seconds_estimate'],b['epoch_seconds_estimate']],
        estimate_margin=settings['epoch_margin'],budget_seconds=settings['budget_seconds'],
        started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),jobs=[],
        runtime_promoted=False,model_selection_status='pending evaluation')
    save(state,record);start=time.monotonic()
    def job(stage,model,command,log):
        entry=dict(stage=stage,model=model,status='running',command=command,log=str(log))
        record['jobs'].append(entry);record['stage']=stage;save(state,record)
        with log.open('w') as stream:result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
        entry.update(status='complete' if result.returncode==0 else 'failed',exit_code=result.returncode)
        record['wall_seconds']=time.monotonic()-start;save(state,record)
        if result.returncode:raise RuntimeError(f'{stage} {model} failed; see {log}')
    def docker(script):
        return ['docker','exec',args.container,'/usr/bin/python3',str(repo/'tools/perception'/script)]
    try:
        for model in ('yolo26l','yoloe26l'):
            output=args.output/model
            command=docker('train_camera_bbox_pilot.py')+[
                '--config',str(args.config),'--dataset',str(args.dataset),'--data-root',str(root/'data'),
                '--model',model,'--batch',str(a['batch']),'--epochs',str(epochs),
                '--output',str(output),'--max-seconds',str(settings['per_model_budget_seconds'])]
            job('training',model,command,args.output/(model+'-train.log'))
            status=json.loads((output/'run.json').read_text())
            if status['status']!='complete' or status['completed_epochs']!=epochs:
                raise RuntimeError('common training amount incomplete; no model ranking')
        ra=json.loads((args.output/'yolo26l/run.json').read_text())
        rb=json.loads((args.output/'yoloe26l/run.json').read_text())
        record['same_sample_order']=ra['sample_order_sha256']==rb['sample_order_sha256']
        if not record['same_sample_order']:raise RuntimeError('candidate training order differs')
        for model in ('yolo26l','yoloe26l'):
            for width in (960,1280):
                for phase in ('before','after'):
                    weights=(args.output/model/'initial.pt' if phase=='before' else args.output/model/'fit/weights/last.pt')
                    output=args.output/f'{model}-{phase}-{width}'
                    command=docker('evaluate_camera_bbox_pilot.py')+[
                        '--config',str(args.config),'--dataset',str(args.dataset),'--weights',str(weights),
                        '--front-width',str(width),'--output',str(output)]
                    job('evaluation',f'{model}-{phase}-{width}',command,args.output/f'{model}-{phase}-{width}.log')
        record.update(status='complete',stage='evaluation_complete',wall_seconds=time.monotonic()-start,
            model_selection_status='requires coverage and regression review; no automatic runtime promotion')
        save(state,record)
    except BaseException as e:
        record.update(status='failed',error=repr(e),wall_seconds=time.monotonic()-start)
        save(state,record);raise


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--smoke-a',type=Path,required=True);p.add_argument('--smoke-b',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--container',default='asmc-camera-bbox-pilot')
    p.add_argument('--common-epoch',type=int,help='explicit equal-checkpoint evaluation after a preserved wall-budget stop')
    p.add_argument('--order-proof',type=Path,help='CPU sampler proof matching both settled actual runs')
    args=p.parse_args()
    if args.common_epoch:
        if not args.order_proof:p.error('--common-epoch requires --order-proof')
        evaluate_equal_checkpoint(args)
    else:run(args)
