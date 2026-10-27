#!/usr/bin/env python3
"""Exercise the real app's hide controls in headless Chrome with synthetic data.

No ROS, MORAI, npm or third-party Python module. A private loopback server and
temporary browser profile are closed on exit. --chrome may select Chromium.
"""
import argparse
import html
import http.server
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import threading
import urllib.parse


WEB = Path(__file__).resolve().parents[2] / 'web'
IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def corners(x):
    return [[x + dx, dy, z] for z in (0, 1) for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def fixture():
    tracks = [dict(id=i, idx=i, miss=0, age=10, points=10, dist=x,
                   raw_class='unknown', **{'class': 'unknown'}, corners=corners(x),
                   obb=dict(center=[x, 0, .5], width=2, height=2, zmin=0, zmax=1, yaw=0),
                   abs_v=[0, 0], velocity=0, kf_initialized=False)
              for i, x in ((1, 10), (2, 20), (3, 30), (4, 40))]
    typed = [dict(track_id=t['id'], corners=t['corners'], center=t['obb']['center'],
                  classification='unknown', camera_class='unknown', points=10) for t in tracks]
    decisions = [dict(track_id=1, decision='undetermined', preview=dict(
        decision='exclude_candidate', reason='cluster_support_outside', point_decision='exclude_candidate',
        obb_decision='keep', height_decision='keep')),
        dict(track_id=2, decision='keep', preview=dict(decision='keep', reason='camera_dynamic_protected',
             point_decision='keep', obb_decision='keep', height_decision='keep')),
        dict(track_id=3, decision='undetermined', reason='pose_error_bound_unverified',
             preview=dict(decision='undetermined',reason='obb_overlap_unavailable',
                          point_decision='undetermined',obb_decision='undetermined',
                          height_decision='keep',obb_interior_road_ratio=None)),
        dict(track_id=4, decision='undetermined', preview=dict(decision='exclude_candidate',
             reason='obb_overlap_below_threshold', point_decision='keep',
             obb_decision='exclude_candidate', height_decision='keep',
             interior_road_ratio=.20, obb_interior_road_ratio=.05))]
    scene = dict(stages=True, stamp_ns=1000000000, seq=1, tracks=tracks, detections=[tracks[0]],
                 typed=dict(local=dict(accepted=True, objects=typed)),
                 base_from_sensor=IDENTITY, base_from_leveled=IDENTITY,
                 leveling=[1, 0, 0, 0, 1, 0, 0, 0, 1], map=dict(valid=True, map_from_base=IDENTITY),
                 counts=dict(raw=3, roi=3, voxel=3, leveled=3, ground_kept=3, cluster_input=3,
                             clusters=4, detections=4, tracks=4, observed=4, predicted=0),
                 sampled={s: dict(sent=3, actual=3) for s in ('raw', 'roi', 'voxel', 'leveled')},
                 local=dict(roll_rad=0, pitch_rad=0, yaw_rad=0, speed_mps=0),
                 hd_map_audit=dict(enabled=True, map_sha256='fixture-v7', objects=decisions,
                                  minimum_support_points=10,minimum_road_points=3,
                                  minimum_road_ratio=.1,minimum_obb_road_ratio=.1,
                                  road_inset_m=.2,road_support_rule='count_and_ratio',
                                  display_support_rule='points_and_obb'))
    # Real binary point provenance: the candidate point belongs to track #1.
    parts = [('leveled.xyzi', 'f32', '<12f', [10, 0, 0, 1, 20, 0, 0, 1, 30, 0, 0, 1]),
             ('leveled.label', 'u8', '<3B', [0, 0, 0]),
             ('leveled.road', 'u8', '<3B', [1, 3, 0]),
             ('leveled.cluster', 'i32', '<3i', [0, 1, 2]),
             ('leveled.track', 'i32', '<3i', [1, 2, 3])]
    arrays, payload = {}, b''
    for name, dtype, fmt, values in parts:
        arrays[name] = dict(offset=len(payload), length=len(values), dtype=dtype)
        value = struct.pack(fmt, *values)
        payload += value + b'\0' * (-len(value) % 4)
    header = dict(session='browser-fixture', frame_seq=1, scene_age_ms=0,
                  status={}, scene=scene, arrays=arrays)
    encoded = json.dumps(header).encode()
    return b'ALMF' + struct.pack('<II', len(encoded), len(payload)) + encoded + b'\0' * (-len(encoded) % 4) + payload


CHECKS = r"""
window.addEventListener('error',e=>{document.body.dataset.testError=e.message;});
(async()=>{
 const checks=[],sleep=()=>new Promise(r=>setTimeout(r,80));
 const check=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);};
 try{
  for(let i=0;i<40 && !(window.__asmcMonitor&&window.__asmcMonitor.state.scene);i++)await sleep();
  const app=window.__asmcMonitor,V=window.AsmcMonitorView;
  check(app&&app.state.scene,'fixture delivered to real app');app.pause(true);
  const toggle=(id,on)=>{const e=document.getElementById(id);e.checked=on;e.dispatchEvent(new Event('change'));};
  const choose=(id,value)=>{const e=document.getElementById(id);e.value=value;e.dispatchEvent(new Event('change'));};
  choose('opt-point-color','reason');
  let boxes=[],points=[];const draw=V.drawBox,fill=CanvasRenderingContext2D.prototype.fillRect;
  V.drawBox=(...a)=>{boxes.push(a[2][0][0]);return draw(...a);};
  CanvasRenderingContext2D.prototype.fillRect=function(...a){if(a[2]===2&&a[3]===2)points.push(this.fillStyle);return fill.apply(this,a);};
  const render=()=>{boxes=[];points=[];app.renderNow();};
  render();
  check(document.getElementById('opt-hide-obb').checked&&app.state.hideReasons.obb,'OBB threshold filter defaults ON');
  check(!boxes.includes(9)&&!boxes.includes(39),'default display requires points AND OBB');
  check(document.getElementById('criteria-obb').textContent.includes('10.0%'),'actual OBB threshold is displayed');
  app.selectTrack(3);render();
  check(document.getElementById('detail').textContent.includes('근거 없음 · 미계산'),'unavailable OBB ratio is not a measured zero');
  const live=app.state.scene.typed.local.objects.find(o=>o.track_id===3);
  Object.assign(live,{absolute_velocity:[1.25,-.5,0],absolute_velocity_valid:true,
    relative_velocity_valid:false,dynamic_valid:true,is_dynamic:true});
  app.selectTrack(3);app.selectTrack(3);render();
  check(document.getElementById('detail').textContent.includes('1.25 / -0.50 / 0.00')&&
    document.getElementById('detail').textContent.includes('동적 (PENDING/DYNAMIC)'),
    'detail shows actual public applied velocity and valid motion');
  Object.assign(live,{absolute_velocity_valid:false,dynamic_valid:false});
  app.selectTrack(3);app.selectTrack(3);render();
  check(document.getElementById('detail').textContent.includes('무효 · pose/운동 근거 확인')&&
    document.getElementById('detail').textContent.includes('UNKNOWN / 미제공 · 정지 확정 아님'),
    'invalid public motion never becomes confirmed stationary');
  toggle('opt-hide-candidates',true);app.selectTrack(1);render();
  check(!boxes.includes(9)&&boxes.includes(19)&&boxes.includes(29),'selected candidate stays hidden, protected and uncertain stay visible: '+JSON.stringify(boxes));
  check(!points.includes('#f472b6'),'selected candidate support stays hidden');
  check(document.getElementById('detail').textContent.includes('숨김 유지'),'hidden selection is explained in details');
  check(document.querySelector('#track-table tbody tr').classList.contains('preview-hidden'),'hidden table row agrees with canvas');
  app.setStage('tracks',false);app.setStage('typed',true);render();
  check(new Set(boxes).size===2&&!boxes.includes(9),'typed overlay cannot bypass candidate hiding: '+JSON.stringify(boxes));
  toggle('opt-hide-candidates',false);render();
  check(new Set(boxes).size===4&&points.includes('#f472b6'),'turning filters off restores original boxes and support');
  toggle('opt-hide-points',true);render();
  check(new Set(boxes).size===3&&!boxes.includes(9),'individual point switch also filters typed overlay');
  toggle('opt-hide-obb',true);render();
  check(new Set(boxes).size===2&&!boxes.includes(39),'OBB failure is hidden even when point support passes');
  app.state.road={available:true,sha256:'different-map'};render();
  check(new Set(boxes).size===4,'map mismatch preserves objects');
  app.state.road={available:true,sha256:'fixture-v7'};
  app.setStage('detections',true);render();
  check(boxes.includes(9),'pre-tracking comparison remains original');
  await sleep();
  check(document.getElementById('filter-layer-note').textContent.includes('detections'),'unfiltered original overlay is explicitly labelled');
  Object.assign(app.state.scene.hd_map_audit,{delivery_applied:true,delivery_input_count:4,
    delivery_excluded_count:2,delivery_output_count:2,road_inset_m:.1});
  app.state.scene.hd_map_audit.objects[0].delivery_excluded=true;
  app.selectTrack(2);app.selectTrack(1);render();for(let i=0;i<4;i++)await sleep();
  check(document.getElementById('audit-summary').textContent.includes('관측 4 · 발행 2 · 실제 제외 2')&&
    document.getElementById('audit-summary').textContent.includes('안쪽 경계 10cm'),'actual ROS delivery and 10cm inset are displayed');
  check(document.getElementById('detail').textContent.includes('제외 · 원본 tracker는 유지'),'actual delivery exclusion is distinct from screen hiding');
  app.setStage('detections',false);app.setStage('typed',false);app.setStage('tracks',true);
  const scene=app.state.scene,first=scene.tracks.find(t=>t.id===1),fourth=scene.tracks.find(t=>t.id===4);
  const lastExcluded=scene.hd_map_audit.objects[0];
  first.miss=2;fourth.miss=8;
  scene.delivery_history=[{stamp_ns:900000000,report:{objects:[lastExcluded]}}];
  scene.hd_map_audit={...scene.hd_map_audit,objects:scene.hd_map_audit.objects.filter(d=>d.track_id!==1&&d.track_id!==4)};
  toggle('opt-show-pred',true);toggle('opt-hide-candidates',true);app.selectTrack(1);render();
  check(!boxes.includes(9)&&!boxes.includes(39),'predictions with excluded history or missing verdict stay hidden');
  check(!points.includes('#f472b6'),'predicted exclusion also hides selected support points');
  app.state.road={available:true,sha256:'different-map'};render();
  check(!boxes.includes(9)&&!boxes.includes(39),'background map mismatch cannot restore excluded or unverified predictions');
  app.state.road={available:true,sha256:'fixture-v7'};
  toggle('opt-hide-candidates',false);render();
  check(boxes.includes(9)&&boxes.includes(39),'predictions can be inspected when exclusion filters are all off');
  toggle('opt-hide-points',true);render();
  check(!boxes.includes(9)&&!boxes.includes(39),'individual exclusion filter still applies after prediction toggle');
  delete scene.delivery_history;render();for(let i=0;i<4;i++)await sleep();
  check(!boxes.includes(9)&&document.getElementById('display-schema-note').textContent.includes('생산자 진단 구버전'),
    'old producer without prediction history is visibly identified and cannot bypass filters');
  first.miss=0;
  scene.hd_map_audit={...scene.hd_map_audit,objects:[...scene.hd_map_audit.objects,
    {track_id:1,delivery_excluded:false,preview:{decision:'keep',reason:'camera_vehicle_protected'}}]};
  render();check(boxes.includes(9),'new observed keep verdict restores a previously excluded track');
  toggle('opt-hide-candidates',false);choose('opt-point-color','road');render();
  check(points.includes('#94a3b8')&&points.includes('#34d399')&&points.includes('#facc15'),
    'point road colors are independent of retained object or camera protection');
  app.state.arrays['leveled.label'][2]=2;render();
  check(!points.includes('#facc15'),'RANSAC view cannot silently restore later structural removals');
  app.setStage('tunnel',true);choose('opt-point-color','reason');render();
  check(points.includes('#fb923c'),'structural removals are explicitly restored in reason color');
  app.setStage('tunnel',false);choose('opt-point-color','road');
  delete app.state.arrays['leveled.road'];render();for(let i=0;i<4;i++)await sleep();
  check(!points.includes('#34d399')&&points.includes('#facc15')&&
    document.getElementById('display-schema-note').textContent.includes('점별 도로 위치'),
    'missing old producer road codes never imply green inside-road evidence');
  check(!document.body.dataset.testError,'no uncaught application errors');
  const pre=document.createElement('pre');pre.id='browser-test-result';pre.textContent=JSON.stringify({ok:true,checks});document.body.appendChild(pre);
 }catch(e){const pre=document.createElement('pre');pre.id='browser-test-result';pre.textContent=JSON.stringify({ok:false,error:String(e),checks});document.body.appendChild(pre);}
})();
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chrome', default=shutil.which('google-chrome') or shutil.which('chromium'))
    args = parser.parse_args()
    if not args.chrome:
        parser.error('Chrome/Chromium required; specify --chrome')
    frame = fixture()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            path = urllib.parse.urlsplit(self.path).path
            if path == '/api/frame':
                body, content = frame, 'application/octet-stream'
            elif path == '/api/drivable':
                body, content = b'{"available":false,"sha256":"fixture-v7"}', 'application/json'
            elif path == '/api/status':
                body, content = b'{}', 'application/json'
            elif path in ('/', '/app.js', '/monitor_view.js', '/app.css'):
                name = 'index.html' if path == '/' else path[1:]
                body = (WEB / name).read_bytes()
                content = {'html': 'text/html', 'js': 'application/javascript', 'css': 'text/css'}[name.rsplit('.', 1)[1]]
                if path == '/':
                    body = body.replace(b'</body>', ('<script>' + CHECKS + '</script></body>').encode())
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content + '; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with tempfile.TemporaryDirectory(prefix='asmc-lidar-browser-') as profile:
            result = subprocess.run([args.chrome, '--headless', '--no-sandbox', '--disable-gpu',
                                     '--disable-dev-shm-usage', '--disable-background-networking',
                                     '--window-size=1600,1000',
                                     '--no-first-run', '--user-data-dir=' + profile,
                                     '--virtual-time-budget=6000', '--dump-dom',
                                     'http://127.0.0.1:' + str(server.server_port)],
                                    capture_output=True, text=True, timeout=40)
        match = re.search(r'<pre id="browser-test-result">(.*?)</pre>', result.stdout, re.S)
        if not match:
            raise RuntimeError('browser did not complete checks: ' + result.stderr[-1200:] + result.stdout[-1000:])
        report = json.loads(html.unescape(match[1]))
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report['ok'] else 1
    finally:
        server.shutdown()
        thread.join(timeout=5)
        server.server_close()


if __name__ == '__main__':
    raise SystemExit(main())
