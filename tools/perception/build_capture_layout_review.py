#!/usr/bin/env python3
"""Read-only native Capture placement gallery; never approves actors or labels."""
import argparse
import json
import hashlib
import html
from pathlib import Path
from PIL import Image, ImageDraw
import numpy as np
from prepare_camera_seg_gt import parse_bbox


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--sensor-root',type=Path,default=Path('/mnt/c/MoraiLauncher_Win/SensorData'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ground-calibration',type=Path,default=Path(__file__).resolve().parents[2]/'config/morai/capture_asset_ground_v1.json')
    a=p.parse_args()
    if a.output.exists():raise ValueError('choose a new immutable review snapshot')
    rows=[json.loads(l) for l in a.ledger.read_text().splitlines() if l.strip()]
    calibration=json.loads(a.ground_calibration.read_text())['assets']
    parents={str(n):r for r in rows for n in r.get('native_ids',[]) if r.get('created') and not r.get('removed')}
    manifest=(a.run/'capture_manifest.jsonl').read_bytes()
    records=[json.loads(l) for l in manifest.splitlines() if l.strip()]
    best={}
    for record in records:
        if not record.get('success'):continue
        for camera,view in ((1,'front'),(2,'left'),(3,'right')):
            base=a.sensor_root/('CAMERA_'+str(camera))/record['custom_name']
            bbox=Path(str(base)+'_BBox2D.txt')
            if not bbox.exists():continue
            boxes=parse_bbox(bbox.read_text(encoding='utf-8-sig'))
            pixels=None; color_counts=None; edge_colors=set()
            for b in boxes:
                if b['actor_id'] not in parents:continue
                expected=calibration[parents[b['actor_id']]['api_name']]['components']['0']['gt_source_label']
                if b['gt_source_label']!=expected:continue
                x1,y1,x2,y2=b['source_xyxy'];score=max(0,x2-x1)*max(0,y2-y1)
                # Review visibility only; this does not filter training GT.
                if score < 64:continue
                instance_path=Path(str(base)+'_Instance.png')
                if not instance_path.exists():continue
                if pixels is None:
                    pixels=np.array(Image.open(instance_path).convert('RGBA'))
                    values,counts=np.unique(pixels.view(np.uint32).reshape(-1),return_counts=True)
                    color_counts=dict(zip(map(int,values),map(int,counts)))
                    packed_image=pixels.view(np.uint32).reshape(pixels.shape[:2])
                    edge_colors=set(map(int,np.unique(np.concatenate((packed_image[0],packed_image[-1],packed_image[:,0],packed_image[:,-1])))))
                # Count every RGBA once per view, rather than rescan a full image
                # for each of the 120 actors. The native byte order is shared.
                packed=int(np.frombuffer(bytes(b['rgba']),dtype=np.uint32)[0])
                visible=color_counts.get(packed,0)
                if visible<16:continue
                contained=packed not in edge_colors and x1>=0 and y1>=0 and x2<=pixels.shape[1] and y2<=pixels.shape[0]
                rank=(contained,visible)
                if rank>best.get(b['actor_id'],{}).get('rank',(False,0)):
                    best[b['actor_id']]=dict(area=score,rank=rank,visible_pixels=visible,
                        fully_inside_image=contained,view=view,record=record,boxes=boxes,
                        rgb=str(base)+'_Intensity.png',bbox=str(bbox),instance=str(instance_path),parent_rgba=b['rgba'])
    a.output.mkdir(parents=True)
    cards=[];snapshots=[];thumbs=[]
    for native,row in parents.items():
        if native not in best:
            cards.append('<p>'+html.escape(row['api_name']+' id='+native)+' — not observed</p>');continue
        item=best[native];image=Image.open(item['rgb']).convert('RGB');draw=ImageDraw.Draw(image)
        for b in item['boxes']:
            draw.rectangle(b['source_xyxy'],outline='#33ff99',width=2)
            draw.text(b['source_xyxy'][:2],b['actor_id']+' '+b['gt_source_label'],fill='#33ff99')
        instance=np.array(Image.open(item['instance']).convert('RGBA'))
        ys,xs=np.where(np.all(instance==np.array(item['parent_rgba'],dtype=np.uint8),axis=2))
        visible_xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)]
        draw.rectangle(visible_xyxy,outline='#00dfff',width=2)
        name='actor_'+native+'.jpg';image.save(a.output/name)
        caption=row['api_name']+' native='+native+' view='+item['view']+' (native boxes; NOT approved labels)'
        cards.append('<article><h2>'+html.escape(caption)+'</h2><img src="'+name+'"></article>')
        thumb=image.copy();thumb.thumbnail((640,360));tile=Image.new('RGB',(640,390),'#101827');tile.paste(thumb,(0,30));ImageDraw.Draw(tile).text((5,5),caption,fill='white');thumbs.append(tile)
        snapshots.append(dict(native_id=native,api_name=row['api_name'],record=item['record'],
            visible_pixels=item['visible_pixels'],fully_inside_image=item['fully_inside_image'],parent_visible_xyxy=visible_xyxy,
            expected_parent_source_label=calibration[row['api_name']]['components']['0']['gt_source_label'],
            rgb_sha256=hashlib.sha256(Path(item['rgb']).read_bytes()).hexdigest(),native_boxes=item['boxes']))
    if thumbs:
        sheet=Image.new('RGB',(1280,390*((len(thumbs)+1)//2)),'#101827')
        for i,tile in enumerate(thumbs):sheet.paste(tile,((i%2)*640,(i//2)*390))
        sheet.save(a.output/'contact-sheet.jpg')
    summary=dict(actors=len(parents),observed=len(best),capture_records=len(records),
        source_manifest_sha256=hashlib.sha256(manifest).hexdigest(),
        ledger_sha256=hashlib.sha256(a.ledger.read_bytes()).hexdigest(),snapshots=snapshots,
        approvals_written=False,scope='native parent-ID placement review; unlisted component IDs remain unresolved')
    (a.output/'review-snapshot.json').write_text(json.dumps(summary,indent=2))
    (a.output/'index.html').write_text('<meta charset="utf-8"><title>Capture placement review 2026-10-12</title><style>body{background:#101827;color:white;font:16px system-ui}img{max-width:100%}</style><h1>Capture placement review 2026-10-12 — unapproved snapshot</h1>'+''.join(cards))
    print(json.dumps({k:v for k,v in summary.items() if k!='snapshots'},indent=2))


if __name__=='__main__':main()
