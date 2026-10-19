import sys,json,argparse
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from collections import Counter
import numpy as np,cv2
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parent))
from camera_pretrain_policy import view_masks,load_policy,sha
from prepare_camera_seg_gt import SEMANTIC
parser=argparse.ArgumentParser(description='Read-only semantic risk audit; standing_object is not automatically a target.')
parser.add_argument('--dataset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
if args.output.exists():raise ValueError('immutable report exists')
p=args.dataset; masks=view_masks(load_policy(p/'policy.json'))
rows=[r for r in map(json.loads,(p/'manifest.jsonl').open()) if r['supervision']=='reviewed_negative'];cv2.setNumThreads(1)
def job(r):
 sem=Path(r['provenance']['semantic']);s=np.array(Image.open(sem).convert('RGB'));ego=masks[r['view']].geometry(r['height'],r['width'])[0]
 standing=(s==[113,178,37]).all(2)&~ego
 n,l,stats,_=cv2.connectedComponentsWithStats(standing.astype('uint8'),8)
 targets=sum(int(((s==list(color)).all(2)&~ego).sum()) for color in SEMANTIC)
 return dict(id=r['id'],source=r['source'],view=r['view'],standing_pixels=int(standing.sum()),standing_components=[list(map(int,st)) for st in stats[1:] if st[4]>=16],known_target_pixels=targets,semantic_sha_matches=sha(sem)==r['provenance']['semantic_sha256'])
with ThreadPoolExecutor(max_workers=2) as pool:out=list(pool.map(job,rows))
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(total=len(out),with_standing_pixels=sum(r['standing_pixels']>0 for r in out),with_large_standing_components=sum(bool(r['standing_components']) for r in out),known_target_pixels_present=sum(r['known_target_pixels']>0 for r in out),sha_mismatches=sum(not r['semantic_sha_matches'] for r in out),rows=out),indent=2));print({k:v for k,v in json.loads(args.output.read_text()).items() if k!='rows'})
