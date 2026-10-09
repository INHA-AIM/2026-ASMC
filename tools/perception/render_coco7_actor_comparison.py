#!/usr/bin/env python3
"""Paired browser inspection of saved predictions; never infer pixel GT from actors."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from PIL import Image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args(); root=args.directory
    report=json.loads((root/'report.json').read_text())
    examples={}; profiles=[]
    for candidate in report['candidates']:
        config=candidate['config']; profile=config.get('candidate_id',f'{config["model"]}-{config["width"]}')
        profiles.append(profile); groups=defaultdict(list)
        for line in (root/profile/'predictions.jsonl').read_text().splitlines():
            row=json.loads(line)
            groups[(row.get('capture_id',Path(row['registry']).name),row['case'])].append(row)
        for (capture,case),rows in groups.items():
            for index in sorted({0,len(rows)//2,len(rows)-1}):
                row=rows[index]
                key=f'{capture}/{case}/{index}'
                entry=examples.setdefault(key,dict(asset=row['asset']['api_name'],placement=row.get('placement'),capture=capture,
                    view=row['view'],source_stamp_ns=row['source_stamp_ns'],sha256=row['sha256'],images={},crops={},predictions={}))
                if entry['sha256']!=row['sha256']:raise ValueError('paired gallery source RGB mismatch')
                image=f'{profile}/{capture}-{case:03}-{index:02}.jpg'
                if not (root/image).is_file():raise ValueError('missing rendered source: '+image)
                entry['images'][profile]=image;entry['predictions'][profile]=row['predictions']
                if row['view']=='front':
                    crop=image.replace('.jpg','-target.jpg')
                    with Image.open(root/image) as original:
                        if original.size==(1280,720):
                            original.crop((480,200,820,480)).save(root/crop)
                            entry['crops'][profile]=crop
    data=json.dumps(dict(profiles=profiles,examples=list(examples.values())),ensure_ascii=False).replace('<','\\u003c')
    page='''<!doctype html><meta charset="utf-8"><title>Raw detector same RGB comparison</title>
<style>body{font:16px system-ui;margin:20px}select{font:inherit;margin:5px;max-width:95%}.pair{display:grid;grid-template-columns:1fr 1fr;gap:14px}img{width:100%}pre{white-space:pre-wrap}article{min-width:0}code{overflow-wrap:anywhere}</style>
<h1>동일 RGB · 모델·해상도 비교</h1>
<p>관찰용 갤러리입니다. actor 상태는 영상 정답이 아니며, 같은 라벨의 검출이 다른 객체일 수 있습니다. 빨강은 Ego mask입니다. 기존 공중 배치·스폰 실패는 별도 원본 기록에 보존합니다.</p>
<label>객체<select id="asset"></select></label><label>뷰<select id="view"></select></label><label>수집<select id="capture"></select></label><label>거리/배치<select id="placement"></select></label><label>장면<select id="sample"></select></label><label><input type="checkbox" id="crop">전방 중앙 확대 (480,200)–(820,480)</label>
<div class="pair"><article><select id="a"></select><img id="ia"><pre id="pa"></pre></article><article><select id="b"></select><img id="ib"><pre id="pb"></pre></article></div><p id="source"></p>
<script>const data=DATA;
const el=id=>document.getElementById(id);
function options(id,values){el(id).replaceChildren(...values.map(([value,label])=>{const o=document.createElement('option');o.value=value;o.textContent=label;return o;}));}
for(const id of ['a','b']) options(id,data.profiles.map(p=>[p,p]));
el('a').value=data.profiles.includes('yolo26m-seg-960')?'yolo26m-seg-960':data.profiles.find(p=>p.startsWith('yolo26m-seg-'))||data.profiles[0];
el('b').value=data.profiles.includes('yolo26l-seg-960')?'yolo26l-seg-960':data.profiles.find(p=>p.startsWith('yolo26l-seg-'))||data.profiles[data.profiles.length-1];
options('asset',[['','전체'],...[...new Set(data.examples.map(r=>r.asset))].sort().map(x=>[x,x])]);
options('view',[['','전체'],...['front','left','right'].map(x=>[x,x])]);
for(const id of ['capture','placement'])options(id,[['','전체'],...[...new Set(data.examples.map(r=>r[id]))].sort().map(x=>[x,x])]);
function draw(){const r=data.examples[Number(el('sample').value)];if(!r)return;
 for(const id of ['a','b']){const p=el(id).value;el('i'+id).src=el('crop').checked&&r.crops[p]?r.crops[p]:r.images[p];el('p'+id).textContent=JSON.stringify(r.predictions[p],null,2);}
 el('source').textContent='RGB SHA256: '+r.sha256+' / source ns: '+r.source_stamp_ns;}
function filter(){options('sample',data.examples.map((r,i)=>[i,r]).filter(([i,r])=>['asset','view','capture','placement'].every(id=>!el(id).value||r[id]===el(id).value)).map(([i,r])=>[i,r.asset+' / '+r.capture+' / '+r.placement+' / '+r.view+' / '+r.source_stamp_ns]));draw();}
for(const id of ['asset','view','capture','placement'])el(id).onchange=filter;for(const id of ['a','b','sample','crop'])el(id).onchange=draw;filter();</script>'''.replace('DATA',data)
    (root/'comparison.html').write_text(page)
    print(json.dumps(dict(profiles=len(profiles),same_rgb_samples=len(examples),output=str(root/'comparison.html'))))


if __name__=='__main__':main()
