#!/usr/bin/env python3
"""F1: inspect recorded native decisions, exact RGB/timing joins and native projection.

No publishers or Python matching/geometry implementation. SQLite indexes keep
whole messages/clouds out of memory. Run within a newly registered D run; never
merge recordings or overwrite results. Replay timing is not live latency.
"""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import sqlite3
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/asmc_camera/src'))
from audit_frame_time import sha, stats, projection_cases, invariant


def timing_relation(a, b):
    """Source clocks may be compared only with explicit same-domain evidence."""
    if a is None or b is None:
        return dict(reason='timing_missing_or_ambiguous', source_delta_ns=None)
    if a['bridge_session_id'] != b['bridge_session_id']:
        return dict(reason='different_bridge_session', source_delta_ns=None)
    for key in ('source_duplicate','source_conflict','source_regression'):
        if a.get(key) or b.get(key):
            return dict(reason=key, source_delta_ns=None)
    if not (a.get('source_valid') and b.get('source_valid')):
        return dict(reason='source_invalid', source_delta_ns=None)
    if not a.get('source_clock_domain') or a['source_clock_domain'] != b['source_clock_domain']:
        return dict(reason='different_source_clock', source_delta_ns=None)
    # Device-hour ticks need an epoch mapping; equality of the domain string is insufficient.
    if a['source_clock_domain'] != 'morai_sensor':
        return dict(reason='source_epoch_mapping_unverified', source_delta_ns=None)
    return dict(reason='same_source_domain_not_exposure_verified',
                source_delta_ns=(a['source_sec']-b['source_sec'])*10**9+a['source_nsec']-b['source_nsec'])


def exact_timing(db, topic, frame, stamp, wire=None):
    sql='SELECT payload FROM timing WHERE topic=? AND frame=? AND stamp=?'
    values=[topic,frame,stamp]
    if wire is not None:
        sql+=' AND wire=?'; values.append(wire)
    rows=db.execute(sql, values).fetchmany(2)
    return json.loads(rows[0][0]) if len(rows)==1 else None


def decision_errors(row, d):
    errors=[]
    f=row['fusion']
    if f['emitting_scan_stamp_ns'] != row['stamp_ns']:
        errors.append('emitting_scan_mismatch')
    if d['action']=='match':
        if d['image_stamp_ns']-d['scan_stamp_ns'] != d['image_minus_scan_ns']:
            errors.append('signed_offset_mismatch')
        if abs(d['image_minus_scan_ns'])>50_000_000:
            errors.append('outside_50ms')
        if not d['projection_valid']: errors.append('matched_invalid_projection')
    return errors


def parity(row):
    row=invariant(row)
    row.pop('fusion',None)
    return row


def replay_compare(args, manifest):
    rows=[]; canonical=None
    for repeat in range(3):
        for mode in (('off','on') if repeat%2==0 else ('on','off')):
            dest=args.output/f'replay-{repeat}-{mode}.jsonl'
            cmd=[str(args.replay),'--bag',str(args.bag),'--output',str(dest),
                 '--config',str(ROOT/'src/asmc_lidar/config/lidar_pipeline.yaml'),
                 '--package-share',str(ROOT/'src/asmc_lidar'),
                 '--camera-calibration',str(ROOT/'src/asmc_camera/config/cameras_v3.yaml'),
                 '--camera-observations-topic',manifest['camera_observations_topic'],
                 '--camera-status-topic',manifest['camera_status_topic'],'--fusion-diagnostics',mode]
            with (args.output/f'replay-{repeat}-{mode}.log').open('x') as log:
                subprocess.run(['/usr/bin/time','-f','%M','-o',str(args.output/f'replay-{repeat}-{mode}.rss-kib'),*cmd],stdout=log,stderr=subprocess.STDOUT,check=True)
            digest=hashlib.sha256();times=[]
            with dest.open() as f:
                for line in f:
                    obj=json.loads(line);digest.update(json.dumps(parity(obj),sort_keys=True).encode())
                    if obj.get('processed'): times.append(obj['timings_ms']['total'])
            ident=digest.hexdigest()
            if canonical is None: canonical=ident
            rows.append(dict(repeat=repeat,mode=mode,output_sha=ident,unchanged=ident==canonical,peak_rss_kib=int((args.output/f'replay-{repeat}-{mode}.rss-kib').read_text().strip()),processing_ms=stats(times)))
    off=[r['processing_ms']['p95'] for r in rows if r['mode']=='off' and r['processing_ms']['count']]
    on=[r['processing_ms']['p95'] for r in rows if r['mode']=='on' and r['processing_ms']['count']]
    delta=max((b-a for a,b in zip(off,on)),default=None)
    return dict(binary_sha=sha(args.replay),runs=rows,p95_increase_ms=delta,
                passed=all(r['unchanged'] for r in rows) and delta is not None and delta<=2,
                note='Recorded-arrival logical replay; no live scheduler or supply acceptance')


def main(args):
    import rosbag
    manifest=json.loads(args.manifest.read_text())
    if args.run_id != manifest['run_id']: raise ValueError('run ID differs from registered manifest')
    for file,digest in manifest['configs'].items():
        if sha(ROOT/file)!=digest: raise ValueError('config changed: '+file)
    if args.output.exists(): raise ValueError('output exists; use a new analysis subdirectory')
    args.output.mkdir(parents=True)
    patch=subprocess.check_output(['git','-C',str(ROOT),'diff','HEAD','--binary'])
    (args.output/'source.patch').write_bytes(patch)
    db=sqlite3.connect(str(args.output/'joins.sqlite'))
    db.executescript('''CREATE TABLE timing(topic TEXT,frame TEXT,stamp INTEGER,wire INTEGER,payload TEXT);
      CREATE INDEX t_key ON timing(topic,frame,stamp,wire);
      CREATE TABLE scans(session TEXT,generation INTEGER,seq INTEGER,stamp INTEGER,arrival INTEGER,payload TEXT);
      CREATE INDEX s_key ON scans(session,generation,stamp);
      CREATE TABLE observations(frame TEXT,stamp INTEGER,wire INTEGER,payload TEXT);
      CREATE INDEX o_key ON observations(frame,stamp,wire);
      CREATE TABLE cloud_headers(frame TEXT,stamp INTEGER,wire INTEGER);
      CREATE INDEX c_key ON cloud_headers(frame,stamp);''')
    topics=[manifest['timing_topic'],manifest['trace_topic'],manifest['camera_observations_topic'],'/velodyne_points']
    counts=Counter(); publishers={}; arrivals=[]; source_arrivals=[]; latencies=[]; processing=[]; map_states=Counter(); missing_fusion=0
    with rosbag.Bag(str(args.bag)) as bag:
        for topic,msg,arrival,connection in bag.read_messages(topics=topics,return_connection_header=True):
            counts[topic]+=1;publishers.setdefault(topic,set()).add(connection.get('callerid','unknown').decode() if isinstance(connection.get('callerid'),bytes) else connection.get('callerid','unknown'))
            if topic==manifest['trace_topic']:
                row=json.loads(msg.data)
                db.execute('INSERT INTO scans VALUES(?,?,?,?,?,?)',(row.get('producer_session_id',''),row.get('generation',0),row['seq'],row['stamp_ns'],arrival.to_nsec(),msg.data))
                if row.get('processed'):
                    arrivals.append(arrival.to_nsec())
                    latencies.append((arrival.to_nsec()-row['stamp_ns'])/1e6)
                    processing.append(row.get('timings_ms',{}).get('total'))
                    map_states[str(row.get('map',{}).get('valid'))+':'+row.get('map',{}).get('reason','')]+=1
                    missing_fusion+= 'fusion' not in row
            elif topic==manifest['timing_topic']:
                # Scalar-only message. Preserve exact integer ns rather than ROS YAML float stamp.
                fields={name:getattr(msg,name) for name in msg.__slots__ if name not in ('published_stamp','publish_ros_time')}
                db.execute('INSERT INTO timing VALUES(?,?,?,?,?)',(msg.sensor_topic,msg.sensor_frame_id,msg.published_stamp.to_nsec(),msg.sensor_wire_sequence,json.dumps(fields)))
            elif topic=='/velodyne_points':
                source_arrivals.append(arrival.to_nsec())
                db.execute('INSERT INTO cloud_headers VALUES(?,?,?)',(msg.header.frame_id,msg.header.stamp.to_nsec(),msg.header.seq))
            else:
                o=msg.detections
                payload=dict(session=msg.producer_session_id,generation=msg.generation,calibration_id=msg.calibration_id,
                    boxes=[dict(xmin=b.xmin,ymin=b.ymin,xmax=b.xmax,ymax=b.ymax) for b in o.boxes])
                db.execute('INSERT INTO observations VALUES(?,?,?,?)',(o.header.frame_id,o.header.stamp.to_nsec(),o.header.seq,json.dumps(payload)))
    db.commit()
    reasons=Counter();errors=Counter();clocks=Counter();offsets=[];waits=[];views=Counter();omitted=0
    cards=[];side_cards=[];desired={};match_index=0
    # Keep at most 24 front decision panels. Full decisions remain in JSONL/SQLite.
    total=db.execute('SELECT count(*) FROM scans').fetchone()[0]
    stride=max(1,total//24)
    with (args.output/'decisions.jsonl').open('x') as journal:
        for (payload,) in db.execute('SELECT payload FROM scans ORDER BY arrival'):
            row=json.loads(payload);fusion=row.get('fusion',{});omitted+=fusion.get('decisions_omitted',0)
            for v in fusion.get('views',[]): views[v['camera_id']+':'+v['reason']]+=1
            for d in fusion.get('decisions',[]):
                reasons[d['reason']]+=1;omitted+=d['associations_omitted']
                errors.update(decision_errors(row,d))
                if d['action']!='match':continue
                offsets.append(d['image_minus_scan_ns']/1e6);waits.append(d['wait_ns']/1e6)
                ct=exact_timing(db,manifest['views'][d['camera_id']],d['camera_id'],d['image_stamp_ns'],d['wire_sequence'])
                wires=db.execute('SELECT wire FROM cloud_headers WHERE frame=? AND stamp=?',('velodyne',d['scan_stamp_ns'])).fetchmany(2)
                lt=exact_timing(db,'/velodyne_points','velodyne',d['scan_stamp_ns'],wires[0][0]) if len(wires)==1 else None
                relation=timing_relation(ct,lt);clocks[relation['reason']]+=1
                detail=dict(emitting_scan=row['seq'],session=row.get('producer_session_id'),generation=row.get('generation'),decision=d,timing=relation)
                journal.write(json.dumps(detail)+'\n')
                if match_index%stride==0 and len(cards)<24:
                    card=dict(decision=d,emit_seq=row['seq'],timing=relation)
                    key=(manifest['views'][d['camera_id']],d['image_stamp_ns'],d['wire_sequence'])
                    desired.setdefault(key,[]).append(len(cards));cards.append(card)
                    # Side projection uses the historical scan's already published
                    # base_link corners. No current IMU, map pose or new matching.
                    old=db.execute('SELECT payload FROM scans WHERE session=? AND generation=? AND stamp=?',
                        (row['producer_session_id'],row['generation'],d['scan_stamp_ns'])).fetchmany(2)
                    if args.probes and len(old)==1 and len(side_cards)<24:
                        scan=json.loads(old[0][0])
                        for v in fusion.get('views',[]):
                            if v['camera_id']=='camera_front' or not v['stamp_ns']:continue
                            obs=db.execute('SELECT payload FROM observations WHERE frame=? AND stamp=? AND wire=?',
                                (v['camera_id'],v['stamp_ns'],v['wire_sequence'])).fetchmany(2)
                            if len(obs)!=1:continue
                            obs=json.loads(obs[0][0])
                            if (obs['session'],obs['generation'],obs['calibration_id'])!=(v['session'],v['generation'],v['calibration_id']):continue
                            points=[p for o in scan.get('typed',{}).get('local',{}).get('objects',[]) for p in o['corners']]
                            side_cards.append(dict(view=v,scan=scan['seq'],observation=obs,
                                request=dict(camera_id=v['camera_id'],scan_stamp_ns=scan['stamp_ns'],image_stamp_ns=v['stamp_ns'],points=points)))
                match_index+=1
    side_projection=Counter()
    if side_cards:
        probe=args.probes/'fusion_projection_probe'
        result=subprocess.run([str(probe),str(ROOT/'src/asmc_camera/config/cameras_v3.yaml')],
            input=''.join(json.dumps(c['request'])+'\n' for c in side_cards),text=True,capture_output=True,check=True)
        projected=[json.loads(s) for s in result.stdout.splitlines()]
        if len(projected)!=len(side_cards):raise ValueError('side probe cardinality mismatch')
        for c,q in zip(side_cards,projected):
            if q['calibration_id']!=c['view']['calibration_id']:raise ValueError('side calibration ID mismatch')
            c['projection']=q
            side_projection.update(c['view']['camera_id']+':'+p['reason'] for p in q['points'])
            v=c['view'];key=(manifest['views'][v['camera_id']],v['stamp_ns'],v['wire_sequence'])
            desired.setdefault(key,[]).append(len(cards));cards.append(c)
    # Exact RGB header/sequence only. A later/nearby image is never substituted.
    image_counts=Counter()
    with rosbag.Bag(str(args.bag)) as bag:
        for topic,msg,_ in bag.read_messages(topics=list(manifest['views'].values())):
            key=(topic,msg.header.stamp.to_nsec(),msg.header.seq)
            if key not in desired:continue
            image_counts[key]+=1
            if image_counts[key]==1:
                name=f'image-{len(image_counts):03d}.jpg'
                (args.output/name).write_bytes(bytes(msg.data))
                for idx in desired[key]:cards[idx]['image']=name
            else:
                for idx in desired[key]:cards[idx].pop('image',None)
    import cv2
    panels=[]
    for i,c in enumerate(cards):
        image=c.get('image')
        if 'projection' in c:
            v=c['view'];q=c['projection'];title=f"{v['camera_id']} / historical scan {c['scan']} / projection only, no matching"
            if image:
                im=cv2.imread(str(args.output/image))
                if im is None or (im.shape[1],im.shape[0])!=(q['width'],q['height']):raise ValueError('side RGB dimensions mismatch')
                for point in q['points']:
                    if point['valid']:cv2.circle(im,(round(point['u']),round(point['v'])),2,(0,255,0),-1)
                for b in c['observation']['boxes']:
                    cv2.rectangle(im,(int(b['xmin']),int(b['ymin'])),(int(b['xmax']),int(b['ymax'])),(0,180,255),1)
                name=f'overlay-{i:03d}.jpg';cv2.imwrite(str(args.output/name),im)
                panels.append('<h3>'+html.escape(title)+'</h3><img src="'+name+'">')
            else:panels.append('<p>'+html.escape(title)+' — exact RGB missing/ambiguous</p>')
            continue
        d=c['decision']
        title=f"emitting scan {c['emit_seq']} / matched scan {d['scan_sequence']} / Δ {d['image_minus_scan_ns']/1e6:.1f}ms"
        if image:
            im=cv2.imread(str(args.output/image))
            if im is not None:
                for a in d['associations']:
                    if a['projected']:
                        x1,y1,x2,y2=map(int,a['projected_box']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,255,0),1)
                    if a['source_box_index']>=0:
                        x1,y1,x2,y2=map(int,a['camera_box']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,180,255),1)
                        cv2.putText(im,f"#{a['track_id']} {a['reason']} {a['iou']:.2f}",(x1,max(12,y1)),0,.4,(0,180,255),1)
                name=f'overlay-{i:03d}.jpg';cv2.imwrite(str(args.output/name),im)
                panels.append('<h3>'+html.escape(title)+'</h3><img src="'+name+'">')
        else:panels.append('<p>'+html.escape(title)+' — exact RGB missing/ambiguous</p>')
    gaps=[(b-a)/1e6 for a,b in zip(arrivals,arrivals[1:])]
    summary=dict(schema='asmc.fusion_verification.result.v1',run_id=args.run_id,bag_sha=sha(args.bag),
        configs=manifest['configs'],source_revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),patch_sha=sha(args.output/'source.patch'),tool_sha=sha(__file__),topics=dict(counts),publishers={k:sorted(v) for k,v in publishers.items()},
        reasons=dict(reasons),errors=dict(errors),timing=dict(clocks),views=dict(views),diagnostics_omitted=omitted,
        processed_without_fusion=missing_fusion,header_offset_ms=stats(offsets),decision_wait_ms=stats(waits),
        trace_receive_gap_ms=stats(gaps),input_receive_gap_ms=stats((b-a)/1e6 for a,b in zip(source_arrivals,source_arrivals[1:])),
        header_to_trace_receive_ms=stats(latencies),processing_ms=stats(processing),map_states=dict(map_states),
        full_5s_trace_rates=[sum(start<=t<start+5_000_000_000 for t in arrivals)/5 for start in range(arrivals[0],arrivals[-1]-5_000_000_000,5_000_000_000)] if arrivals else [],gallery_exact_images=sum('image'in c for c in cards),
        side_projection_points=dict(side_projection),side_projection_panels=len(side_cards),
        note='No source clock mapping, ego motion, side matching or independent geometric GT is claimed')
    if args.probes:summary['projection_regression']=projection_cases(args.probes/'frame_projection_probe')
    if args.replay:summary['replay_comparison']=replay_compare(args,manifest)
    summary['diagnostic_consistency_passed']=bool(offsets) and not errors and not missing_fusion and not omitted and all(len(v)==1 for v in publishers.values())
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>F1 fusion</title><style>body{background:#151b22;color:#eee;font:16px sans-serif;margin:2em}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>F1 native front fusion</h1><p>Green: native cluster projection. Orange: detector boxes. Side matching is not implemented. Source clock comparison is separate from receive-header matching.</p><pre>'+html.escape(json.dumps(summary,indent=2))+'</pre>'+''.join(panels))
    db.close();print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','bag','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--run-id',required=True);p.add_argument('--probes',type=Path);p.add_argument('--replay',type=Path)
    main(p.parse_args())
