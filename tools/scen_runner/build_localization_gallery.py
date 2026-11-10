#!/usr/bin/env python3
"""Reevaluate immutable trials in a NEW registered analysis run and make a gallery.

The C++ auditor independently projects native poses on the fixed Frenet curve.
No replay, ROS publisher, inference, controller, or simulator connection is used.
"""
import argparse,csv,hashlib,json,math,subprocess
from pathlib import Path
import yaml
from localization_assessment import assess
from scen_runner.morai.signal_trial_coordinates import Coordinates


def rows(p):return [json.loads(s) for s in p.read_text().splitlines()] if p.exists() else []
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def clean(x):
    if isinstance(x,float) and not math.isfinite(x):return None
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,list):return [clean(v) for v in x]
    return x


def recording_check(root,events,receipts):
    import rosbag
    topic_groups={};counts={};times=[];n=0
    for p in sorted(root.glob('*.bag')):
        group=p.stem.rsplit('_',1)[0]
        with rosbag.Bag(str(p)) as bag:
            for topic,info in bag.get_type_and_topic_info().topics.items():
                topic_groups.setdefault(topic,set()).add(group);counts[topic]=counts.get(topic,0)+info.message_count
            for _,_,t in bag.read_messages(topics=['/ctrl_cmd']):
                if events and events[0]['ros_time']<=t.to_sec()<=events[-1]['ros_time']:times.append(t.to_sec())
    active=[r for r in receipts if events and events[0]['monotonic']<=r['monotonic']<=events[-1]['monotonic']]
    return dict(active_bag_controls=len(times),active_independent_receipts=len(active),edge_count_tolerance=2,
        counts_match=bool(active) and abs(len(times)-len(active))<=2,
        no_topic_recorded_by_both_groups=all(len(v)==1 for v in topic_groups.values()),
        no_active_bag_files=not list(root.glob('*.active')),topic_counts=counts,
        duplicate_topics={k:sorted(v) for k,v in topic_groups.items() if len(v)>1})


def make(source, output, auditor, parameters):
    if not (output.parent/'registration.json').is_file():raise ValueError('parent must be a new registered run')
    output.mkdir(exist_ok=False)
    checkout=Path(__file__).resolve().parents[2];coordinates=Coordinates([302595.,4124145.])
    report=[]
    for number in range(1,7):
        root=source/f'aimdrive-position-20261028-r{number:02d}';dest=output/root.name;dest.mkdir()
        events=[e for e in rows(root/'events.jsonl') if e.get('event')=='native_sample']
        decisions=rows(root/'decisions.jsonl');receipts=rows(root/'command-receipts.jsonl');controls=rows(root/'control.jsonl')
        result=json.loads((root/'trial-result.json').read_text())
        # Historical runtime bundles are read-only code/config/map evidence,
        # not executable environments. Preserve the original reference policy.
        provenance=json.loads((root/'runtime-provenance.json').read_text()) if (root/'runtime-provenance.json').exists() else {}
        oldprefix=provenance.get('options',{}).get('runtime_root') or ''
        bundle=checkout/'.runtime'/('position-b1' if oldprefix.endswith('b1') else 'position-b2')
        old=yaml.safe_load((bundle/'src/asmc_planner/config/frenet/params.yaml').read_text())
        params=yaml.safe_load(parameters.read_text());params['planner']['reference']=old['planner']['reference']
        waypoint=bundle/'R_KR_PG_KATRI/2026_molit_comp_global_path.txt';maps=bundle/'src/asmc_planner/config/hd_map'
        params['waypoint_file']=str(waypoint);params['avoid_hd_map']['directory']=str(maps)
        config=dest/'audit-parameters.yaml';config.write_text(yaml.safe_dump(params,sort_keys=False))
        native=dest/'native.csv'
        with native.open('x') as f:
            w=csv.writer(f);w.writerow(['index','stamp','x','y','yaw'])
            for i,e in enumerate(events):
                p=e['ego']['pose'];w.writerow([i,e['ros_time'],p['x'],p['y'],coordinates.grid_yaw(p['x'],p['y'],p['yaw'])])
        with (dest/'native-audit.log').open('x') as log:
            subprocess.run([str(auditor),str(config),str(native),str(dest/'reference.csv')],stdout=log,stderr=log,check=True)
        reference=list(csv.DictReader((dest/'reference.csv').open()))
        recording=recording_check(root,events,receipts);(dest/'recording-assessment.json').write_text(json.dumps(recording,indent=2)+'\n')
        assessment=assess(events,decisions,reference,receipts,result,recording)
        consumed={p.name:sha(p) for p in root.glob('*.json*') if p.name in
                  ('trial-result.json','events.jsonl','decisions.jsonl','control.jsonl','command-receipts.jsonl','runtime-provenance.json')}
        assessment.update(run_id=root.name,historical_only=True,input_sha256=consumed,
            reference_waypoint_sha256=sha(waypoint),reference_policy=old['planner']['reference'],
            map_sha256={p.name:sha(p) for p in maps.iterdir() if p.is_file()},
            evaluator_sha256=sha(auditor),config_sha256=sha(config),
            python_sha256={n:sha(Path(__file__).with_name(n)) for n in ('build_localization_gallery.py','localization_assessment.py')},
            map_envelope='conservative configured rear-axle footprint 2.160835m x 4.635m, rear .79m; native contact separately checked',
            historical_note='Replay uses preserved reference policy and map bytes, with current read-only Frenet evaluator; no claim of post-rebase driving success.')
        (dest/'assessment.json').write_text(json.dumps(clean(assessment),indent=2)+'\n')
        origin=events[0]['ros_time'] if events else 0
        plot=dict(native=[dict(t=e['ros_time']-origin,x=e['ego']['pose']['x'],y=e['ego']['pose']['y'],v=e['ego']['speed_kmh'],steer=e['ego']['steer']) for e in events],
                  estimate=[dict(t=d['stamp']-origin,x=d['map_x'],y=d['map_y'],source=d.get('source'),valid=d.get('valid'),reason=d.get('reason')) for d in decisions if d.get('source')!='unavailable'],
                  reference=[dict(x=float(r['x']),y=float(r['y']),lat=float(r['lateral_m'])) for r in reference if int(r['projected'])],
                  controls=[dict(t=c['stamp']-origin,steer=c.get('steering',0)) for c in controls if 'stamp' in c],
                  timeline=[dict(t=d['stamp']-origin,source=d.get('source'),valid=d.get('valid'),reason=d.get('reason')) for d in decisions],
                  errors=[dict(t=float(r['stamp'])-origin,cm=100*float(r['lateral_m'])) for r in reference if int(r['projected'])],
                  gaps=[dict(t=b['monotonic']-(events[0]['monotonic'] if events else 0),ms=1000*(b['monotonic']-a['monotonic'])) for a,b in zip(receipts,receipts[1:]) if events and events[0]['monotonic']<=b['monotonic']<=events[-1]['monotonic']])
        report.append(dict(assessment=assessment,plot=plot))
    (output/'assessment-catalogue.json').write_text(json.dumps(clean(report),indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(13,8))
    for ax,row in zip(axes.flat,report):
        a,p=row['assessment'],row['plot'];ax.set_title(a['run_id'].rsplit('-',1)[-1]+' (historical)')
        if not p['native']:
            ax.axis('off');ax.text(.1,.5,'setup failure / no native samples',transform=ax.transAxes);continue
        for key,color,label in [('native','#228833','native'),('estimate','#cc6677','estimate'),('reference','#4477aa','fixed reference')]:
            pts=p[key]
            if pts:ax.plot([q['x'] for q in pts],[q['y'] for q in pts],color=color,label=label,linewidth=1)
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.3);ax.set_xlabel('map x [m]');ax.set_ylabel('map y [m]')
        if p['native']:ax.legend(fontsize=7)
        else:ax.text(.1,.5,'setup failure / no native movement',transform=ax.transAxes)
    fig.tight_layout();fig.savefig(output/'trajectories.png',dpi=160);plt.close(fig)
    payload=json.dumps(clean(report),ensure_ascii=False).replace('</','<\\/')
    html='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Localization M2 — historical baseline review (final analysis)</title>
<style>body{font:16px system-ui;max-width:1200px;margin:32px auto;padding:0 18px;background:#101820;color:#e8eef4}canvas{width:100%;background:#18242d;border-radius:8px}select,input{font:inherit}table{border-collapse:collapse}td,th{border-bottom:1px solid #53616d;padding:8px;text-align:left}a{color:#8bd5ff}.bad{color:#ffba9f}.ok{color:#9bddaa}pre{white-space:pre-wrap}img{max-width:100%}</style>
<h1>Localization 위치측위 · M2 중간 평가</h1><p>r01–r06 원본을 보존한 재평가입니다. 새 main 리베이스 이후 주행 수락 결과가 아닙니다. 실제 위치를 고정 Frenet 참조에 독립 투영했으며, 무효 계획·누락·정지 구간도 포함합니다. 위치 시각 대응은 수신 시각 기반이며 센서 시계 보정 결과가 아닙니다.</p>
<p>녹색: 실제 위치 · 분홍: 추정 위치 · 파랑: 고정 참조. LiDAR GPS 유효성 및 터널 전처리는 변경하지 않았습니다.</p>
<select id="run"></select><p><input id="time" type="range" min="0" max="100" value="0" style="width:85%"><span id="clock"></span></p>
<canvas id="view" width="1100" height="430"></canvas><canvas id="speed" width="1100" height="150"></canvas><canvas id="steer" width="1100" height="150"></canvas><canvas id="lateral" width="1100" height="150"></canvas><canvas id="gap" width="1100" height="150"></canvas><canvas id="timeline" width="1100" height="65"></canvas><div id="state"></div><div id="metrics"></div>
<p><a href="assessment-catalogue.json">전체 판정·입력 SHA JSON</a> · <a href="trajectories.png">경로 비교 PNG</a></p><img src="trajectories.png" alt="Six historical runs, native estimated and fixed-reference trajectories">
<script>const data=PAYLOAD;let index=0;const run=document.getElementById('run'),slider=document.getElementById('time');data.forEach((r,i)=>{let o=document.createElement('option');o.value=i;o.textContent=r.assessment.run_id;run.append(o)});
function canvas(id){let c=document.getElementById(id),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);return [c,g]}
function draw(){let r=data[index],p=r.plot,a=r.assessment,pts=[...p.native,...p.reference,...p.estimate].filter(q=>Number.isFinite(q.x)&&Number.isFinite(q.y));let [c,g]=canvas('view');let duration=p.native.length?p.native.at(-1).t:1,t=Number(slider.value)/100*duration;document.getElementById('clock').textContent=t.toFixed(1)+' s';
if(pts.length){let xs=pts.map(q=>q.x),ys=pts.map(q=>q.y),xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys),scale=Math.min((c.width-60)/Math.max(1,xmax-xmin),(c.height-60)/Math.max(1,ymax-ymin));let xy=q=>[30+(q.x-xmin)*scale,c.height-30-(q.y-ymin)*scale];for(let [key,color] of [['reference','#4477aa'],['native','#44bb88'],['estimate','#ee8899']]){g.strokeStyle=color;g.beginPath();p[key].forEach((q,i)=>{let z=xy(q);i?g.lineTo(...z):g.moveTo(...z)});g.stroke();let q=p[key].filter(q=>q.t<=t).at(-1);if(q){g.fillStyle=color;g.beginPath();g.arc(...xy(q),5,0,7);g.fill()}}}
function chart(id,key,label,color,rows=p.native){let [c,g]=canvas(id);g.fillStyle='#eef';g.fillText(label,12,16);let vals=rows.map(q=>q[key]);let lo=Math.min(0,...vals),hi=Math.max(1,...vals);g.strokeStyle=color;g.beginPath();rows.forEach((q,i)=>{let x=40+q.t/Math.max(1,duration)*(c.width-60),y=c.height-20-(q[key]-lo)/(hi-lo)*(c.height-50);i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();g.strokeStyle='#fff';let x=40+t/Math.max(1,duration)*(c.width-60);g.beginPath();g.moveTo(x,20);g.lineTo(x,c.height);g.stroke();g.fillText(lo.toFixed(2)+' … '+hi.toFixed(2),12,35)}chart('speed','v','Native speed [km/h]','#55ccaa');chart('steer','steer','Native steering value (MORAI field; unit not inferred)','#eeaabb');
chart('lateral','cm','Native lateral error against fixed reference [cm]','#ffcc77',p.errors);chart('gap','ms','Independent command-receipt interval [ms] (gate 250 ms)','#aaccff',p.gaps);
let [tc,tg]=canvas('timeline');tg.fillStyle='#eef';tg.fillText('GPS green · DR blue · recovery amber · invalid red · missing black',12,15);for(let row of p.timeline){if(row.t<0||row.t>duration)continue;tg.fillStyle=!row.valid?'#e46666':row.source==='gps_anchored'?'#44bb88':row.source==='dead_reckoning'?'#4488ee':'#ffbb44';tg.fillRect(40+row.t/Math.max(1,duration)*(tc.width-60),25,Math.max(1,.05/Math.max(1,duration)*(tc.width-60)),25)}
let q=p.timeline.filter(q=>q.t<=t).at(-1);document.getElementById('state').textContent=q?`${q.source} · plan ${q.valid?'valid':'INVALID'} · ${q.reason}`:'추정 위치/주행 표본 없음';let gates=Object.entries(a.gates).map(([k,v])=>`<tr><td>${k}</td><td class="${v?'ok':'bad'}">${v?'PASS':'FAIL / evidence absent'}</td></tr>`).join('');document.getElementById('metrics').innerHTML=`<p>전체 수락: ${a.stage_passed?'PASS':'미통과'} (과거 기준선)</p><table>${gates}</table><pre>${JSON.stringify(a.metrics,null,2)}</pre><a href="${a.run_id}/assessment.json">이 run의 근거</a>`;}
run.onchange=()=>{index=Number(run.value);draw()};slider.oninput=draw;draw();</script></html>'''.replace('PAYLOAD',payload)
    (output/'index.html').write_text(html)
    print(json.dumps([dict(run=r['assessment']['run_id'],gates=r['assessment']['gates'],metrics=r['assessment']['metrics']) for r in report],indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--auditor',type=Path,required=True);p.add_argument('--parameters',type=Path,required=True);a=p.parse_args();make(a.source,a.output,a.auditor,a.parameters)
