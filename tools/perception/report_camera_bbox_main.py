#!/usr/bin/env python3
"""Publish a read-only, offline live dashboard without modifying the training job."""
import argparse
from datetime import datetime, timezone, timedelta
import html
import json
from pathlib import Path


def build(run, dataset=None):
    state=json.loads((run/'supervisor.json').read_text())
    version=state.get('dataset_version','unknown')
    root=Path(__file__).resolve().parents[3]
    dataset=dataset or root/'data/dataset_versions'/version
    summary=json.loads((dataset/'summary.json').read_text()) if (dataset/'summary.json').exists() else {}
    split=summary.get('splits',{})
    def count(role,key):
        part=split.get(role,{})
        if isinstance(part,int):return part if key=='images' else None
        if key=='images':return part.get('images')
        return sum(c.get(key,0) for c in part.get('classes',{}).values()) if part.get('classes') else None
    date=datetime.fromisoformat(state['started_utc']).astimezone(timezone(timedelta(hours=9))).strftime('%Y-%m-%d')
    cfgs=list((run/'code/config/experiments').glob('camera_bbox_main_'+version.split('_bbox_')[-1].split('-')[0]+'.json'))
    cfg=json.loads(cfgs[0].read_text()) if cfgs else {}
    config=dict(evaluation_interval=cfg.get('evaluation',{}).get('interval_after_four',4),version=version,date=date,model='YOLO26l detection',
        train=count('train','images') or summary.get('balance',{}).get('images'),
        positives=count('train','positive'),ignore=count('train','ignore'),
        selection=count('selection','images'),heldout=count('heldout','images'),
        exposures=summary.get('balance',{}).get('exposures'),
        classes=summary.get('classes',[]))
    template=Path(__file__).with_name('templates')/'camera_training_live.html'
    content=template.read_text().replace('__DASHBOARD_CONFIG__',json.dumps(config,ensure_ascii=False).replace('<','\\u003c'))
    content=content.replace('__TITLE__',html.escape(date+' · YOLO26l '+version.split('_bbox_')[-1]+' 학습 현황'))
    output=run/'live.html';output.write_text(content,encoding='utf-8')
    # This is an index of views, not a replacement for the trainer's evaluation page.
    links=''.join('<li><a href="'+html.escape(p.name,quote=True)+'/live.html">'+html.escape(p.name)+'</a></li>'
        for p in sorted(run.parent.iterdir()) if p.is_dir() and (p/'supervisor.json').is_file())
    (run.parent/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>'+date+' · 카메라 본학습</title><a href="../index.html">전체 실험 목차</a><h1>카메라 본학습 실행</h1><ul>'+links+'</ul></html>')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--dataset',type=Path)
    a=p.parse_args();print(build(a.run.resolve(),a.dataset.resolve() if a.dataset else None))
