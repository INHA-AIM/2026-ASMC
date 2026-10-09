#!/usr/bin/env python3
"""Interactive identical-image backend/resolution comparison from saved predictions."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    gt={r['image']:r for r in map(json.loads,a.gt.read_text().splitlines())}
    report=json.loads(a.report.read_text());images={}
    for row in report['parity']:
        name=row['image']
        if name not in images:
            source=a.datasets/name; expected=gt[name]['source_sha256']['intensity']
            if hashlib.sha256(source.read_bytes()).hexdigest()!=expected:raise RuntimeError('RGB SHA mismatch')
            local=str(len(images))+source.suffix;shutil.copyfile(source,a.output/local)
            images[name]=dict(image=local,gt=gt[name]['annotations'],models={})
        images[name]['models'][f'{row["model"]}/{row["width"]}']=dict(
            pytorch=row['pytorch'],tensorrt=row['tensorrt'],parity_passed=row['passed'])
    # Only saved predictions above the run's inference threshold are available.
    data=dict(images=images,minimum_confidence=report['confidence'])
    (a.output/'data.json').write_text(json.dumps(data,indent=2)+'\n')
    page='''<!doctype html><meta charset="utf-8"><title>COCO7 near-object comparison</title>
<style>body{font:16px system-ui;margin:24px}canvas{max-width:100%;height:auto}select,input{margin:8px}section{border:1px solid #bbb;padding:12px;margin:12px 0}</style>
<h1>같은 RGB · 해상도 / backend 대조</h1>
<p>파랑은 GT, 초록은 저장된 검출입니다. 검출 점수 slider는 저장한 결과만 필터링합니다.
재추론·새 GT·실제 거리 평가가 아닙니다. 차체 mask를 적용해 추론한 결과이며 영상은 원본입니다.</p>
<label>영상 <select id="image"></select></label><label>모델 <select id="model"></select></label>
<label>confidence <input id="conf" type="range" min="0.25" max="0.7" step="0.05" value="0.65"><span id="value"></span></label>
<div id="panels"></div><script>
const data=DATA;
const names=Object.keys(data.images);const imageSelect=document.querySelector('#image');
for(const name of names){const o=document.createElement('option');o.textContent=name;o.value=name;imageSelect.append(o)}
const modelSelect=document.querySelector('#model');
for(const name of [...new Set(Object.values(data.images).flatMap(v=>Object.keys(v.models).map(k=>k.split('/')[0])))]){
 const o=document.createElement('option');o.textContent=name;o.value=name;modelSelect.append(o)}
const slider=document.querySelector('#conf');slider.min=data.minimum_confidence;
async function draw(){
 const row=data.images[imageSelect.value],threshold=Number(slider.value);document.querySelector('#value').textContent=threshold.toFixed(2);
 const img=new Image();img.src=row.image;await img.decode();const container=document.querySelector('#panels');container.replaceChildren();
 for(const width of [960,1280])for(const backend of ['pytorch','tensorrt']){
  const result=row.models[modelSelect.value+'/'+width];if(!result)continue;
  const section=document.createElement('section');const title=document.createElement('p');
  const boxes=result[backend].filter(b=>b.confidence>=threshold);
  title.textContent=modelSelect.value+' · '+width+' · '+backend+' · '+boxes.length+' boxes · parity '+result.parity_passed;section.append(title);
  const c=document.createElement('canvas');c.width=img.naturalWidth;c.height=img.naturalHeight;section.append(c);container.append(section);
  const ctx=c.getContext('2d');ctx.drawImage(img,0,0);ctx.lineWidth=2;ctx.font='16px sans-serif';
  for(const [objects,color] of [[row.gt,'#009dff'],[boxes,'#00ff30']])for(const box of objects){
   const [x,y,r,b]=box.xyxy;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.strokeRect(x,y,r-x,b-y);
   ctx.fillText(box.source_class_name+(box.confidence!==undefined?' '+box.confidence.toFixed(3):' GT'),x,Math.max(16,y-4))}
 }
}
for(const element of [imageSelect,modelSelect,slider])element.addEventListener('input',draw);draw();
</script>'''
    page=page.replace('DATA',json.dumps(data).replace('</','<\\/'))
    (a.output/'index.html').write_text(page)


if __name__=='__main__':main()
