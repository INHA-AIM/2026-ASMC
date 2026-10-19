#!/usr/bin/env python3
"""Explicit E8 optimizer-preserving LR-only branch; gated by E14 visual review.

Original trainer/config/dataset/loss stay frozen. Scale all optimizer learning
rates and scheduler bases together, preserving the original decay shape. No
implicit warm restart, optimizer reset, or overwrite of a parent checkpoint.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_status_page(out):
    """Read durable evidence, without treating an unfinished branch as a result."""
    (out/'index.html').write_text('''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>2026-10-14 · v11 E8 LR 분기 현황</title>
<style>body{font:17px/1.7 system-ui;background:#101827;color:#eee;max-width:1100px;margin:30px auto;padding:0 20px}a{color:#89d3ff}progress{width:100%;height:24px}table{border-collapse:collapse;width:100%}td,th{padding:10px;border:1px solid #607080}pre{white-space:pre-wrap}.card{background:#243346;padding:20px;margin:20px 0}</style>
<h1>v11 E8 → E10 · LR만 낮춘 분기</h1>
<p><a href="../../index.html">ASMC 실험 목록</a> · <a href="../../camera-v11-epoch14-review-20261014-r1/index.html">E14 loss·LR·실패 장면 검토</a> · <a href="branch.json">분기 증거 JSON</a></p>
<p>동일 E8의 모델·optimizer·EMA를 복원합니다. 데이터·loss·증강은 유지하며 E9의 LR은0.0002, E10은 약0.0001938입니다. 원래 LR로 수행한 E10과 비교합니다.</p>
<section class="card"><h2 id="status">상태 확인 중</h2><progress id="bar" max="1" value="0"></progress><p id="detail"></p><p id="stamp"></p></section>
<h2>동일 평가본 · confidence 0.40</h2><table><thead><tr><th>구성</th><th>16종 macro F1</th><th>bus TP</th><th>truck TP</th><th>car TP</th></tr></thead><tbody id="metrics"></tbody></table>
<p>표본이 적고 같은 장면이 반복된 클래스가 있습니다. 이 결과만으로 일반화·배포를 승인하지 않습니다. 평가 완료 전에는 빈 점수를 성능 0으로 표시하지 않습니다.</p>
<p id="gallery"></p><pre id="failure"></pre><script>
async function load(path){const r=await fetch(path+'?t='+Date.now(),{cache:'no-store'});if(!r.ok)throw Error(path+': '+r.status);return r.json()}
async function refresh(){try{const b=await load('branch.json');let r={};try{r=await load('main/run.json')}catch(e){};
const n=r.completed_batches||0,total=(r.batches_per_epoch||10976)*2;
document.getElementById('status').textContent=({starting:'준비 중',training:'LR 분기 학습 중',evaluating:'E9·E10 평가 중',completed:'두 epoch와 평가 완료',failed:'실패 — 원본 보존'})[b.status]||b.status;
document.getElementById('bar').value=Math.min(n/total,1);
document.getElementById('detail').textContent=`${n.toLocaleString()} / ${total.toLocaleString()} batch · ${Math.min(n/total*100,100).toFixed(1)}% · `+(b.checks||[]).map(x=>'E'+x.epoch+' LR '+x.actual_lrs[0].toPrecision(5)).join(' → ');
document.getElementById('stamp').textContent='화면 갱신 '+new Date().toLocaleString()+' · 10초 간격';
document.getElementById('failure').textContent=b.error||'';
if(b.status==='completed'){const box=document.getElementById('gallery');box.replaceChildren();const a=document.createElement('a');a.href='paired-comparison/gallery/index.html';a.textContent='원래 E10 / 낮춘 LR E10 · 같은 영상 비교';box.append(a)}
const rows=[...(b.control||[]).map(x=>({...x,label:'원래 LR E'+x.epoch})),...(b.evaluations||[]).map(x=>({...x,label:'낮춘 LR E'+x.epoch}))];
const body=document.getElementById('metrics');body.replaceChildren();for(const x of rows){const tr=document.createElement('tr');for(const v of [x.label,(x.metrics.macro_f1_fixed16*100).toFixed(2)+'%',...['bus','truck','car'].map(c=>x.metrics.classes[c].tp)]){const td=document.createElement('td');td.textContent=v;tr.append(td)}body.append(tr)}
}catch(e){document.getElementById('stamp').textContent='상태 갱신 실패: '+e.message}}refresh();setInterval(refresh,10000);
</script></html>''', encoding='utf-8')


def scale_learning_rate(trainer, first_lr):
    if first_lr<=0 or trainer.start_epoch<1:raise ValueError('positive LR and resumed epoch required')
    scheduler=trainer.scheduler;groups=trainer.optimizer.param_groups
    if len(groups)!=len(scheduler.base_lrs):raise ValueError('scheduler group mismatch')
    next_lrs=[base*fn(trainer.start_epoch) for base,fn in zip(scheduler.base_lrs,scheduler.lr_lambdas)]
    if not next_lrs or min(next_lrs)<=0 or max(next_lrs)-min(next_lrs)>1e-12:
        raise ValueError('unequal/invalid parent group LR')
    factor=first_lr/next_lrs[0]
    before=[dict(lr=g['lr'],initial_lr=g['initial_lr']) for g in groups]
    scheduler.base_lrs=[x*factor for x in scheduler.base_lrs]
    scheduler._last_lr=[x*factor for x in scheduler.get_last_lr()]
    for g in groups:g['lr']*=factor;g['initial_lr']*=factor
    trainer.args.lr0*=factor
    return dict(first_epoch=trainer.start_epoch+1,target_first_lr=first_lr,
        parent_next_lrs=next_lrs,factor=factor,before=before,
        scheduler_base_lrs=list(scheduler.base_lrs),effective_lr0=trainer.args.lr0,
        optimizer_moments_reset=False,decay_shape='parent linear decay retained')


def run(a):
    parent=a.parent.resolve();out=a.output.resolve();gate=json.loads(a.review_gate.read_text())
    if not gate.get('passed') or gate.get('decision')!='run_lr_only_branch':raise ValueError('review not complete')
    for name,digest in gate['required_files'].items():
        if sha(name)!=digest:raise ValueError('review pin changed: '+name)
    if out.exists():raise ValueError('branch output exists; no implicit resume or overwrite')
    cfg=parent/'code/config/experiments/camera_bbox_main_v11.json';weights=parent/'main/fit/weights/epoch7.pt'
    if gate['branch_checkpoint_sha256']!=sha(weights):raise ValueError('branch checkpoint differs')
    close=parent/'operator-closeout/result.json'
    if json.loads(close.read_text())['epoch']!=14:raise ValueError('E14 not evaluated')
    if (parent/'operator-closeout/_EVALUATED').read_text().strip()!=sha(close):raise ValueError('closeout pin mismatch')
    sys.path.insert(0,str(parent/'code/tools/perception'))
    import torch
    import train_camera_bbox_pilot as training
    from ultralytics import YOLO
    from run_camera_bbox_main import evaluate,compare_metrics,save,gallery
    checkpoint=torch.load(weights,map_location='cpu')
    if checkpoint.get('epoch')!=7 or checkpoint.get('training_model') is None or checkpoint.get('optimizer') is None:
        raise ValueError('not complete E8 live model with optimizer')
    del checkpoint
    records=[];controls=[]
    for f in (parent/'main/resume-history').glob('*.json'):
        r=json.loads(f.read_text())
        if r.get('completed_epochs')==8 and r.get('status')=='paused_at_epoch':records.append((f,r))
        if r.get('completed_epochs')==10 and r.get('resume_checkpoint_sha256')==gate['branch_checkpoint_sha256']:
            controls.append((f,r))
    if len(records)!=1:raise ValueError('E8 provenance record must be unique')
    if len(controls)!=1:raise ValueError('original E8-to-E10 control record must be unique')
    out.mkdir(parents=True);main=out/'main';main.mkdir();shutil.copy2(records[0][0],main/'run.json')
    (out/'evaluation').symlink_to(parent/'evaluation',target_is_directory=True)
    (out/'baseline-1280').symlink_to(parent/'baseline-1280',target_is_directory=True)
    config=json.loads(cfg.read_text());dataset=Path(gate['dataset']);data_root=dataset.parents[1]
    proof=dict(status='starting',parent=str(parent),source_epoch=8,stop_epoch=10,
        target_first_lr=a.first_lr,checkpoint=str(weights),checkpoint_sha256=sha(weights),
        config_sha256=sha(cfg),dataset_metadata_sha256=sha(dataset/'pretrain.json'),
        review_gate_sha256=sha(a.review_gate),wrapper_sha256=sha(__file__),only_change='optimizer LR scale',checks=[])
    proof['control_training_record']=dict(path=str(controls[0][0]),sha256=sha(controls[0][0]))
    control=json.loads((parent/'supervisor.json').read_text())
    proof['control']=[e for e in control['evaluations'] if e['epoch'] in (8,10)]
    save(out/'branch.json',proof)
    write_status_page(out)
    original_train=YOLO.train
    def wrapped_train(model,*args,**kwargs):
        # Native resume restores the checkpoint's project/name. save_dir is an
        # explicit supported override in pinned 8.4.150; never write the parent.
        kwargs['save_dir']=str(main/'fit')
        def apply(t):
            if t.start_epoch!=8:raise ValueError('wrong resume epoch')
            if Path(t.save_dir).resolve()!=(main/'fit').resolve():raise ValueError('branch output escaped isolation')
            current=json.loads((main/'run.json').read_text())
            for k in ('optimizer_at_start','e2e_loss_schedule_at_start'):
                if current[k]!=controls[0][1][k]:raise ValueError('branch/control resume state differs: '+k)
            proof['control_resume_state_verified']=True
            proof['lr_change']=scale_learning_rate(t,a.first_lr);proof['status']='training'
            proof['e2e_at_start']={k:getattr(t.model.criterion,k) for k in ('updates','o2m','o2o')}
            save(out/'branch.json',proof)
        def check(t):
            expected=a.first_lr*t.lf(t.epoch)/t.lf(8)
            rates=[g['lr'] for g in t.optimizer.param_groups]
            if any(abs(v-expected)>1e-12 for v in rates):raise ValueError('branch LR overwritten')
            if not proof['checks'] or proof['checks'][-1]['epoch']!=t.epoch+1:
                proof['checks'].append(dict(epoch=t.epoch+1,actual_lrs=rates));save(out/'branch.json',proof)
        model.add_callback('on_train_start',apply)
        model.add_callback('on_train_batch_start',check)
        return original_train(model,*args,**kwargs)
    YOLO.train=wrapped_train
    args=SimpleNamespace(config=cfg,dataset=dataset,data_root=data_root,output=main,
        model='yolo26l',batch=4,epochs=40,smoke_batches=0,max_seconds=9000,
        pretrain=True,resume=weights,stop_after_epoch=10,epoch_budget=True,reserve_seconds=600)
    try:
        training.run(args)
        record=json.loads((main/'run.json').read_text())
        if record.get('completed_epochs')!=10:raise ValueError('branch did not finish two complete epochs')
        proof['data_order_matches_control']=record['sample_order_sha256']==controls[0][1]['sample_order_sha256']
        if not proof['data_order_matches_control']:raise ValueError('branch/control input order differs')
        proof['status']='evaluating';save(out/'branch.json',proof)
        ea=SimpleNamespace(config=cfg,output=out)
        baseline=json.loads((parent/'baseline-1280/evaluation.json').read_text());evaluations=[]
        for epoch in (9,10):
            w=main/f'fit/weights/epoch{epoch-1}.pt';folder=out/f'epoch-{epoch:03d}-1280'
            result=evaluate(ea,w,folder)
            evaluations.append(dict(epoch=epoch,metrics=compare_metrics(baseline,result,20,config['evaluation']['existing_class_quality']),checkpoint_sha256=sha(w)))
            proof['evaluations']=evaluations;save(out/'branch.json',proof)
        gallery(out,out/'epoch-010-1280')
        paired=out/'paired-comparison';paired.mkdir()
        (paired/'evaluation').symlink_to(parent/'evaluation',target_is_directory=True)
        (paired/'baseline-1280').symlink_to(parent/'epoch-010-1280',target_is_directory=True)
        (paired/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>v11 LR 분기 E10 동등 학습량 비교</title><a href="../index.html">분기 결과</a><p>Before는 원래 LR E10, After는 낮춘 LR E10입니다. 동일 E8에서 출발한 두 epoch 결과이며 평가 설정은 같습니다.</p><a href="gallery/index.html">동일 영상 비교</a>')
        gallery(paired,out/'epoch-010-1280')
        paired_page=paired/'gallery/index.html'
        paired_page.write_text(paired_page.read_text().replace('YOLO26l v8 failures','v11 E10 original LR vs reduced LR').replace('GT / before / after.','GT / original LR E10 / reduced LR E10.'))
        control=json.loads((parent/'supervisor.json').read_text())
        proof.update(status='completed',evaluations=evaluations,control=[e for e in control['evaluations'] if e['epoch'] in (8,10)],production_approved=False)
        save(out/'branch.json',proof)
    except BaseException as e:
        proof.update(status='failed',error=type(e).__name__+': '+str(e));save(out/'branch.json',proof);raise
    finally:YOLO.train=original_train


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','output','review-gate'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--first-lr',type=float,default=.0002)
    run(p.parse_args())
