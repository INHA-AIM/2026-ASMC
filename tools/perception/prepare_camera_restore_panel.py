import json,collections,hashlib,argparse
from pathlib import Path
from PIL import Image,ImageDraw
parser=argparse.ArgumentParser(description='Deterministic review candidates; no implicit adoption.')
for name in ('dataset','pool','assignment','output'):parser.add_argument('--'+name,type=Path,required=True)
a=parser.parse_args();D=a.dataset;O=a.output
if O.exists():raise ValueError('immutable panel exists')
O.mkdir(parents=True);proposal=json.loads(a.assignment.read_text());parents={}

for role in ['train','selection','heldout']:
 for r in map(json.loads,(a.pool/(role+'.jsonl')).open()):parents[r['id']]=r
active={r['id']:r for r in map(json.loads,(D/'manifest.jsonl').open())};parents.update(active)
classes=json.load(open(D/'pretrain.json'))['classes'];old=[r for r in active.values() if r['split']=='selection' and proposal['groups'][r['split_group']]=='selection'];selected={r['id']:r for r in old}
def cs(r):return collections.Counter(a['training_class'] for a in r['annotations'] if a['supervision']=='positive')
counts=sum((cs(r) for r in old),collections.Counter());candidates=[r for r in parents.values() if proposal['groups'][r['split_group']]=='selection' and r['id'] not in selected and cs(r)]
while candidates and any(counts[c]<(10 if c=='dog' else 20) for c in classes):
 def score(r):
  c=cs(r);gain=sum(min(max(0,(10 if k=='dog' else 20)-counts[k]),v) for k,v in c.items());anns=[a for a in r['annotations'] if a['supervision']=='positive'];size=sum(min(1,(a['xyxy'][2]-a['xyxy'][0])*(a['xyxy'][3]-a['xyxy'][1])/2500) for a in anns)/len(anns)
  return (gain/(1+max(0,sum(c.values())-4)*.5),size,hashlib.sha256(r['id'].encode()).hexdigest())
 r=max(candidates,key=score)
 if score(r)[0]==0:break
 candidates.remove(r);selected[r['id']]=r;counts.update(cs(r))
new=[r for r in selected.values() if r['id'] not in {x['id'] for x in old}]
(O/'panel-candidates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in new));(O/'retained-selection.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in old))
for start in range(0,len(new),8):
 subset=new[start:start+8];canvas=Image.new('RGB',(1280,400*((len(subset)+1)//2)),'#142332');dr=ImageDraw.Draw(canvas)
 for k,r in enumerate(subset):
  im=Image.open(r['image']).convert('RGB');d=ImageDraw.Draw(im)
  for a in r['annotations']:
   if a['supervision']=='positive':d.rectangle(a['xyxy'],outline='cyan',width=2);d.text((a['xyxy'][0],max(0,a['xyxy'][1]-12)),a['training_class'],fill='yellow')
  im.thumbnail((640,360));x=k%2*640;y=k//2*400;canvas.paste(im,(x,y+30));dr.text((x+6,y+4),f'{start+k} '+r['source'][-45:],fill='white')
 canvas.save(O/f'panel-{start//8:02}.jpg')
print(dict(retained=len(old),new=len(new),counts=counts))
