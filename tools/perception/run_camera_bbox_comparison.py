#!/usr/bin/env python3
"""Serial pilot orchestration using two completed, same-batch smoke runs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from train_camera_bbox_pilot import common_epochs,save,sha


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
    run(p.parse_args())
