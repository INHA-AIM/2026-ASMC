from pathlib import Path
import json,hashlib,argparse
from collections import defaultdict
from PIL import Image,ImageDraw
parser=argparse.ArgumentParser();parser.add_argument('--proposals-root',type=Path,required=True);root=parser.parse_args().proposals_root;
if (root/'review/sample.json').exists():raise ValueError('immutable review exists')
rs=list(map(json.loads,(root/'proposals.jsonl').open()));groups=defaultdict(list)
for r in rs:groups[r['source']].append(r)
selected=[]
for source,rows in sorted(groups.items()):
 for r in sorted(rows,key=lambda r:hashlib.sha256(('20261013'+r['id']).encode()).hexdigest())[:50]:
  # One reproducibly selected component per sampled frame; whole-frame overlay
  # retained separately. This does not claim each component was seen manually.
  p=min(r['proposals'],key=lambda p:hashlib.sha256(p['proposal_id'].encode()).hexdigest())
  selected.append(dict(id=r['id'],source=source,view=r['view'],proposal=p))
(root/'review').mkdir(exist_ok=True)
for page,start in enumerate(range(0,len(selected),80)):
 subset=selected[start:start+80];sheet=Image.new('RGB',(1440,150*((len(subset)+7)//8)),'#15202c')
 for j,x in enumerate(subset):
  im=Image.open(root/'crops'/(x['proposal']['proposal_id']+'.jpg'));im.thumbnail((174,120));xx=(j%8)*180;yy=(j//8)*150;sheet.paste(im,(xx+(180-im.width)//2,yy+25));ImageDraw.Draw(sheet).text((xx+3,yy+2),f'{start+j} '+x['view']+' '+x['proposal']['proposal_id'][:6],fill='white')
 sheet.save(root/'review'/f'sheet-{page:02}.jpg')
(root/'review/sample.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2));print('samples',len(selected),'sources',len(groups))
