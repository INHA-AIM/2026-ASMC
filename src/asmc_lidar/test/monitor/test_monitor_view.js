#!/usr/bin/env node
// Regression tests of the web monitor view logic (web/monitor_view.js), plain node
// (>= 10), no dependency: node src/asmc_lidar/test/monitor/test_monitor_view.js
//  - BEV draws the rotated OBB footprint (projected corners 0-3 in order), never an
//    axis-aligned rectangle, for 0/30/45/90 deg non-square boxes in every view frame,
//    for leveled (track/detection) and base_link (typed local) corners; the heading
//    marker follows the box axis; the side view is the screen bounding rectangle
//  - frame cursor: a restarted server (new session, frame_seq from 1) is shown at
//    once and the old scene is dropped; a same-session scene that is not newer is not
//  - displayed-scene freshness comes from the scene's own age, not the status
'use strict';
const assert = require('assert');
const path = require('path');
const fs = require('fs');
const V = require(path.join(__dirname, '..', '..', 'web', 'monitor_view.js'));

let passed = 0;
const failures = [];
function test(name, fn) {
  try { fn(); passed += 1; console.log('ok   ' + name); } catch (e) { failures.push(name); console.log('FAIL ' + name + '\n     ' + (e && e.stack ? e.stack : e)); }
}
const close = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= (tol || 1e-6), (what || '') + ' ' + a + ' != ' + b);

// ------------------------------------------------------------------ independent math (test side)
const deg = Math.PI / 180;
function rotZYX(roll, pitch, yaw) {  // R = Rz(yaw) Ry(pitch) Rx(roll), row-major 3x3
  const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr];
}
function rigid(r, t) { return [r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0, 0, 0, 1]; }
function m4mul(a, b) {
  const o = [];
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) { let s = 0; for (let k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j]; o.push(s); }
  return o;
}
function m4inv(m) {  // rigid inverse
  const r = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]];
  const t = [m[3], m[7], m[11]];
  const nt = [-(r[0] * t[0] + r[1] * t[1] + r[2] * t[2]), -(r[3] * t[0] + r[4] * t[1] + r[5] * t[2]), -(r[6] * t[0] + r[7] * t[1] + r[8] * t[2])];
  return rigid(r, nt);
}
function m4apply(m, p) { return [0, 1, 2].map((i) => m[i * 4] * p[0] + m[i * 4 + 1] * p[1] + m[i * 4 + 2] * p[2] + m[i * 4 + 3]); }
const ID = rigid([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 0]);

// Scene with non-trivial mount, roll+pitch leveling and map pose.
const L3 = rotZYX(2.0 * deg, -3.0 * deg, 0);          // leveled_from_sensor (rotation only)
const L = rigid(L3, [0, 0, 0]);
const Bs = rigid(rotZYX(0.5 * deg, 1.5 * deg, 0.8 * deg), [1.35, 0.02, 1.92]);  // base_from_sensor
const Bl = m4mul(Bs, m4inv(L));                         // base_from_leveled
const M = rigid(rotZYX(0.3 * deg, -0.4 * deg, 37 * deg), [402155.2, 4140122.7, 31.5]);  // map_from_base
const scene = { stages: true, base_from_sensor: Bs, base_from_leveled: Bl, leveling: L3, map: { valid: true, map_from_base: M } };
// Expected view_from_data for data frames 'leveled' and 'base' in each view frame.
const viewFrom = {
  base: { leveled: Bl, base: ID },
  leveled: { leveled: ID, base: m4inv(Bl) },
  sensor: { leveled: m4inv(L), base: m4inv(Bs) },
  map: { leveled: m4mul(M, Bl), base: M },
};

// OBB corners in the message convention: (u, v) signs 0=(-,-) 1=(+,-) 2=(+,+) 3=(-,+), 0-3 at zMin.
function obbCorners(cx, cy, length, width, yaw, zmin, zmax) {
  const c = Math.cos(yaw), s = Math.sin(yaw), out = [];
  for (const z of [zmin, zmax]) for (const [su, sv] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
    const u = su * length / 2, v = sv * width / 2;
    out.push([cx + c * u - s * v, cy + s * u + c * v, z]);
  }
  return out;
}
const VIEW = { scale: 9, panX: 13, panY: -7, side: 'xz' };
const W = 900, H = 700;
function expectedBev(viewFrame, dataFrame, corner, origin) {
  const p = m4apply(viewFrom[viewFrame][dataFrame], corner);
  return [W / 2 + VIEW.panX - (p[1] - origin[1]) * VIEW.scale, H / 2 + VIEW.panY - (p[0] - origin[0]) * VIEW.scale];
}

function recorder() {
  const ops = [];
  return {
    ops,
    set strokeStyle(v) { ops.push(['strokeStyle', v]); }, set lineWidth(v) { ops.push(['lineWidth', v]); },
    setLineDash(d) { ops.push(['setLineDash', d]); }, beginPath() { ops.push(['beginPath']); },
    moveTo(x, y) { ops.push(['moveTo', x, y]); }, lineTo(x, y) { ops.push(['lineTo', x, y]); },
    closePath() { ops.push(['closePath']); }, stroke() { ops.push(['stroke']); },
    strokeRect(x, y, w, h) { ops.push(['strokeRect', x, y, w, h]); },
  };
}
// The drawn paths: [{ points: [[x, y]...], closed }] from beginPath/moveTo/lineTo/closePath.
function paths(ops) {
  const out = []; let cur = null;
  for (const op of ops) {
    if (op[0] === 'moveTo') { cur = { points: [[op[1], op[2]]], closed: false }; out.push(cur); }
    else if (op[0] === 'lineTo' && cur) cur.points.push([op[1], op[2]]);
    else if (op[0] === 'closePath' && cur) cur.closed = true;
  }
  return out;
}

// ------------------------------------------------------------------ geometry
for (const viewFrame of ['base', 'leveled', 'sensor', 'map']) {
  const m = V.frameMatrices(scene, viewFrame);
  const origin = m4apply(viewFrom[viewFrame].base, [0, 0, 0]);
  for (const dataFrame of ['leveled', 'base']) {
    for (const yawDeg of [0, 30, 45, 90]) {
      test('bev ' + viewFrame + '/' + dataFrame + ' yaw ' + yawDeg + ': rotated footprint = projected corners', () => {
        assert.ok(m && m[dataFrame], 'matrix');
        const yaw = yawDeg * deg;
        const corners = obbCorners(14.0, -3.5, 4.6, 1.8, yaw, -1.6, 0.1);
        const ctx = recorder();
        // As renderScene passes it: a wrapper around the BEV projection (not the function itself).
        const project = (p) => V.bevProject(VIEW, origin, W, H, p[0], p[1]);
        const outline = V.drawBox(ctx, 'bev', corners, m[dataFrame], project, { color: '#f472b6', width: 2 });
        assert.ok(outline && outline.kind === 'bev');
        assert.strictEqual(ctx.ops.filter((o) => o[0] === 'strokeRect').length, 0, 'BEV must not draw an axis-aligned rectangle');
        const drawn = paths(ctx.ops);
        assert.strictEqual(drawn.length, 2, 'footprint + heading');
        const poly = drawn[0];
        assert.ok(poly.closed, 'closed footprint');
        assert.strictEqual(poly.points.length, 4);
        for (let i = 0; i < 4; i++) {
          const e = expectedBev(viewFrame, dataFrame, corners[i], origin);
          close(poly.points[i][0], e[0], 1e-6, 'corner ' + i + ' x'); close(poly.points[i][1], e[1], 1e-6, 'corner ' + i + ' y');
        }
        // A box rotated on screen (edge 0-1 > 10 deg from both screen axes) has no axis-parallel
        // edge and no corner on its bounding rectangle's corners. 30/45 deg are rotated on screen
        // in every frame except map, whose 37 deg heading adds to the box yaw (map yaw 0 is rotated).
        const edges = [0, 1, 2, 3].map((i) => [poly.points[(i + 1) % 4][0] - poly.points[i][0], poly.points[(i + 1) % 4][1] - poly.points[i][1]]);
        const ang = Math.abs(Math.atan2(edges[0][1], edges[0][0])) % (Math.PI / 2);
        const rotatedOnScreen = Math.min(ang, Math.PI / 2 - ang) > 10 * deg;
        if ((yawDeg === 30 || yawDeg === 45) && viewFrame !== 'map') assert.ok(rotatedOnScreen, 'yaw ' + yawDeg + ' not rotated on screen');
        if (viewFrame === 'map' && yawDeg === 0) assert.ok(rotatedOnScreen, 'map heading not applied');
        if (rotatedOnScreen) {
          for (const [dx, dy] of edges) assert.ok(Math.abs(dx) > 1 && Math.abs(dy) > 1, 'edge parallel to a screen axis at yaw ' + yawDeg + ': ' + dx + ',' + dy);
          const xs = poly.points.map((p) => p[0]), ys = poly.points.map((p) => p[1]);
          const aabb = [[Math.min(...xs), Math.min(...ys)], [Math.max(...xs), Math.min(...ys)], [Math.max(...xs), Math.max(...ys)], [Math.min(...xs), Math.max(...ys)]];
          const minDist = Math.min(...aabb.map((a) => Math.min(...poly.points.map((p) => Math.hypot(p[0] - a[0], p[1] - a[1])))));
          assert.ok(minDist > 3, 'footprint corners coincide with the AABB corners');
        }
        if (viewFrame === 'base' && dataFrame === 'base' && (yawDeg === 0 || yawDeg === 90)) {
          for (const [dx, dy] of edges) assert.ok(Math.abs(dx) < 1e-6 || Math.abs(dy) < 1e-6, 'axis box edge');
        }
        // Side lengths in pixels: 4.6 m and 1.8 m (up to the small roll/pitch/mount tilt).
        const l01 = Math.hypot(edges[0][0], edges[0][1]) / VIEW.scale, l12 = Math.hypot(edges[1][0], edges[1][1]) / VIEW.scale;
        close(l01, 4.6, 0.05, 'edge 0-1 length'); close(l12, 1.8, 0.05, 'edge 1-2 length');
        // Heading: centre -> middle of edge 1-2, parallel to edge 0->1 and to the projected +yaw axis.
        const head = drawn[1];
        assert.strictEqual(head.points.length, 2);
        const [c, f] = head.points;
        const centre = [poly.points.reduce((s, p) => s + p[0], 0) / 4, poly.points.reduce((s, p) => s + p[1], 0) / 4];
        close(c[0], centre[0]); close(c[1], centre[1]);
        const hx = f[0] - c[0], hy = f[1] - c[1];
        const cross = (hx * edges[0][1] - hy * edges[0][0]) / (Math.hypot(hx, hy) * Math.hypot(edges[0][0], edges[0][1]));
        close(cross, 0, 1e-9, 'heading parallel to edge 0-1');
        assert.ok(hx * edges[0][0] + hy * edges[0][1] > 0, 'heading points along +axis (corner 0 -> 1)');
        const a0 = expectedBev(viewFrame, dataFrame, [14.0, -3.5, -1.6], origin);
        const a1 = expectedBev(viewFrame, dataFrame, [14.0 + Math.cos(yaw), -3.5 + Math.sin(yaw), -1.6], origin);
        const ax = a1[0] - a0[0], ay = a1[1] - a0[1];
        close((hx * ay - hy * ax) / (Math.hypot(hx, hy) * Math.hypot(ax, ay)), 0, 1e-9, 'heading parallel to projected yaw');
        assert.ok(hx * ax + hy * ay > 0, 'heading along +yaw');
      });
    }
  }
}

test('side view: screen bounding rectangle of all 8 projected corners', () => {
  const m = V.frameMatrices(scene, 'base');
  const origin = [0, 0, 0];
  const corners = obbCorners(10, 2, 4.0, 2.0, 30 * deg, -1.5, 0.2);
  const ctx = recorder();
  const project = (p) => V.sideProject(VIEW, origin, W, H, p);
  const outline = V.drawBox(ctx, 'side', corners, m.leveled, project, { color: '#fff' });
  const rects = ctx.ops.filter((o) => o[0] === 'strokeRect');
  assert.strictEqual(rects.length, 1);
  const pts = corners.map((c) => project(m4apply(Bl, c)));
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  close(rects[0][1], Math.min(...xs)); close(rects[0][2], Math.min(...ys));
  close(rects[0][3], Math.max(...xs) - Math.min(...xs)); close(rects[0][4], Math.max(...ys) - Math.min(...ys));
  assert.strictEqual(outline.kind, 'side');
});

test('unknown view kind and invalid input', () => {
  const corners = obbCorners(0, 0, 2, 1, 0, 0, 1);
  assert.throws(() => V.boxOutline('top', corners, V.I4, (p) => [p[0], p[1]]), /unknown view kind/);
  assert.strictEqual(V.drawBox(recorder(), 'bev', corners.slice(0, 7), V.I4, (p) => p, { color: '#fff' }), null);
  assert.strictEqual(V.drawBox(recorder(), 'bev', corners, null, (p) => p, { color: '#fff' }), null);
});

test('app.js selects the BEV/side drawing explicitly, never by function identity', () => {
  const app = fs.readFileSync(path.join(__dirname, '..', '..', 'web', 'app.js'), 'utf8');
  assert.ok(!/===\s*bevProject|!==\s*bevProject|===\s*sideProject|!==\s*sideProject/.test(app), 'projection identity check');
  assert.ok(/kind:\s*state.view.mode/.test(app) && /kind:\s*'side'/.test(app), 'explicit selected view kind');
  assert.ok(/V\.drawBox\(ctx, view\.kind,/.test(app), 'drawBox gets the view kind');
});

test('3D orbit at top view equals BEV; height projects vertically at oblique view', () => {
  const v = {scale:10,panX:2,panY:3,yaw:0,pitch:Math.PI/2};
  const p=[7,3,2],o=[1,2,0];
  const a=V.orbitProject(v,o,800,600,p),b=V.bevProject(v,o,800,600,p[0],p[1]);
  close(a[0],b[0]);close(a[1],b[1]);
  v.pitch=0.5;
  const low=V.orbitProject(v,o,800,600,[7,3,0]),high=V.orbitProject(v,o,800,600,[7,3,2]);
  close(low[0],high[0]);close(low[1]-high[1],20*Math.cos(0.5));
});

test('3D OBB draws twelve edges, not an axis-aligned screen rectangle', () => {
  const ctx=recorder(),corners=obbCorners(0,0,4,2,0.4,0,2);
  const v={scale:10,panX:0,panY:0,yaw:0.3,pitch:0.5};
  const outline=V.drawBox(ctx,'orbit',corners,V.I4,p=>V.orbitProject(v,[0,0,0],800,600,p),{color:'#fff'});
  assert.strictEqual(outline.points.length,8);
  assert.strictEqual(ctx.ops.filter(x=>x[0]==='lineTo').length,12);
  assert.strictEqual(ctx.ops.filter(x=>x[0]==='strokeRect').length,0);
});

test('rotated 2D view roundtrips screen/world coordinates',()=>{
  for(const yaw of [0,0.5,Math.PI/2,-2.3]) {
    const v={scale:11,panX:30,panY:-50,bevYaw:yaw},o=[20,-30,0],p=[45,17];
    const q=V.bevProject(v,o,900,700,...p),back=V.bevUnproject(v,o,900,700,...q);
    close(back[0],p[0]);close(back[1],p[1]);
  }
});
test('polygon includes boundary but excludes a concave notch',()=>{
  const poly=[[0,0],[4,0],[4,1],[1,1],[1,4],[0,4]];
  assert(V.pointInPolygon([0.5,3],poly));assert(V.pointInPolygon([0,2],poly));
  assert(!V.pointInPolygon([3,3],poly));assert(!V.pointInPolygon([8,0],poly));
});
test('road classification uses transformed OBB center, independent of displayed view',()=>{
  const scene={stages:true,base_from_sensor:ID,base_from_leveled:ID,leveling:[1,0,0,0,1,0,0,0,1],
    map:{valid:true,map_from_base:rigid(rotZYX(0,0,Math.PI/2),[100,200,0])}};
  const road={available:true,polygons:[[[98,204],[102,204],[102,210],[98,210]]]};
  assert.strictEqual(V.roadMembership(scene,road,[5,0,0]),'inside');
  assert.strictEqual(V.roadMembership(scene,road,[20,0,0]),'outside');
  scene.map.valid=false;
  assert.strictEqual(V.roadMembership(scene,road,[5,0,0]),'unknown');
});
test('missing map and invalid center never look like a valid outside decision',()=>{
  assert.strictEqual(V.roadMembership(null,null,[1,2,0]),'unknown');
  assert.strictEqual(V.roadMembership({}, {available:true,polygons:[]},[NaN,0,0]),'unknown');
});
test('motion colors distinguish unknown, low threat, pending and dynamic',()=>{
  assert.strictEqual(V.motionAppearance(null).color,'#facc15');
  assert.strictEqual(V.motionAppearance({valid:false,state:1}).color,'#facc15');
  assert.strictEqual(V.motionAppearance({valid:true,state:0}).color,'#facc15');
  assert.strictEqual(V.motionAppearance({valid:true,state:1}).color,'#22c55e');
  assert.strictEqual(V.motionAppearance({valid:true,state:2}).color,'#ff4545');
  assert.strictEqual(V.motionAppearance({valid:true,state:3}).color,'#ff4545');
});
const idRoad={available:true,polygons:[[[-20,-20],[20,-20],[20,20],[-20,20]]]};
function idTrack(id,x,y=0){return {id,miss:0,class:'vehicle',obb:{center:[x,y,1],width:4,height:2}};}
function idScene(t,tracks,offset=0,valid=true){return {stamp_ns:t*1e9,stages:true,tracks,
  base_from_sensor:ID,base_from_leveled:ID,leveling:[1,0,0,0,1,0,0,0,1],
  map:{valid,map_from_base:rigid(rotZYX(0,0,0),[offset,0,0])}};}
test('outside toggle hides only known outside objects',()=>{
  const sc=idScene(1,[]);
  assert(V.trackVisible(sc,idRoad,idTrack(1,30),false));
  assert(!V.trackVisible(sc,idRoad,idTrack(1,30),true));
  assert(V.trackVisible(sc,idRoad,idTrack(1,5),true));
  sc.map.valid=false;assert(V.trackVisible(sc,idRoad,idTrack(1,30),true));
});
test('prediction toggle hides missed tracks without hiding observations',()=>{
  const sc=idScene(1,[]),t=idTrack(1,5);
  assert(V.trackVisible(sc,idRoad,t,false,false));
  t.miss=1;
  assert(!V.trackVisible(sc,idRoad,t,false,false));
  assert(V.trackVisible(sc,idRoad,t,false,true));
  assert(V.trackVisible(sc,idRoad,t,false));
});
test('ID change suspicion follows map position despite ego translation',()=>{
  const monitor=new V.IdChangeMonitor();
  assert.strictEqual(monitor.update(idScene(1,[idTrack(7,5)]),idRoad).size,0);
  const w=monitor.update(idScene(1.2,[idTrack(8,4.2)],1),idRoad);
  assert.strictEqual(w.get(8).from,7);
});
test('ordinary new objects and splits with the old ID still observed do not alert',()=>{
  const monitor=new V.IdChangeMonitor();
  monitor.update(idScene(1,[idTrack(7,5)]),idRoad);
  assert.strictEqual(monitor.update(idScene(1.2,[idTrack(7,5),idTrack(8,5.2)]),idRoad).size,0);
});
test('ambiguous crowded ID matches do not alert',()=>{
  const monitor=new V.IdChangeMonitor();
  monitor.update(idScene(1,[idTrack(7,5),idTrack(9,5.5)]),idRoad);
  assert.strictEqual(monitor.update(idScene(1.2,[idTrack(8,5.2)]),idRoad).size,0);
});
test('outside, long gaps and invalid GPS cannot establish an ID switch',()=>{
  for(const mode of ['outside','gap','gps']){
    const monitor=new V.IdChangeMonitor(),x=mode==='outside'?25:5;
    monitor.update(idScene(1,[idTrack(7,x)]),idRoad);
    assert.strictEqual(monitor.update(idScene(mode==='gap'?3:1.2,[idTrack(8,x)],0,mode!=='gps'),idRoad).size,0);
  }
});
test('ID warnings expire after three scan seconds and reset across sessions',()=>{
  const monitor=new V.IdChangeMonitor();
  monitor.update(idScene(1,[idTrack(7,5)]),idRoad);
  assert(monitor.update(idScene(1.2,[idTrack(8,5.2)]),idRoad).has(8));
  for(let t=1.4;t<4.6;t+=0.2)monitor.update(idScene(t,[idTrack(8,5.2)]),idRoad);
  assert.strictEqual(monitor.warnings.size,0);
  monitor.reset();assert.strictEqual(monitor.history.size,0);
});

// ------------------------------------------------------------------ frame cursor
test('restarted server: new session scene shown at once, old cursor not awaited', () => {
  const cursor = { session: 'a-1', frameSeq: 163 };
  const act = V.frameAction(cursor, { session: 'b-2', frame_seq: 1, scene: { stamp_ns: 5 } });
  assert.deepStrictEqual([act.reset, act.accept, act.reason], [true, true, 'new_session']);
});
test('restarted server without a scene yet: old scene dropped, nothing accepted', () => {
  const act = V.frameAction({ session: 'a-1', frameSeq: 163 }, { session: 'b-2', frame_seq: 0, scene: null });
  assert.deepStrictEqual([act.reset, act.accept], [true, false]);
});
test('same session: newer accepted, equal/older ignored, status-only kept', () => {
  const cursor = { session: 'a-1', frameSeq: 10 };
  assert.ok(V.frameAction(cursor, { session: 'a-1', frame_seq: 11, scene: {} }).accept);
  const same = V.frameAction(cursor, { session: 'a-1', frame_seq: 10, scene: {} });
  assert.deepStrictEqual([same.reset, same.accept, same.reason], [false, false, 'not_newer']);
  assert.ok(!V.frameAction(cursor, { session: 'a-1', frame_seq: 9, scene: {} }).accept);
  const none = V.frameAction(cursor, { session: 'a-1', frame_seq: 12, scene: null });
  assert.deepStrictEqual([none.reset, none.accept], [false, false]);
});
test('first scene of a page accepted whatever its number', () => {
  assert.ok(V.frameAction({ session: null, frameSeq: 0 }, { session: 'a-1', frame_seq: 900, scene: {} }).accept);
});
test('simulated restart: 150 frames, restart, first new frames displayed in order', () => {
  let cursor = { session: null, frameSeq: 0 }; const shown = [];
  const step = (session, seq) => {
    const act = V.frameAction(cursor, { session, frame_seq: seq, scene: { seq } });
    if (act.reset) cursor = { session, frameSeq: 0 };
    if (act.accept) { cursor = { session, frameSeq: seq }; shown.push(session + ':' + seq); }
  };
  for (let i = 1; i <= 150; i++) step('a', i);
  step('b', 1); step('b', 1); step('b', 2); step('b', 3);
  assert.deepStrictEqual(shown.slice(-3), ['b:1', 'b:2', 'b:3']);
  assert.strictEqual(shown.length, 153);
});

// ------------------------------------------------------------------ freshness
const base = { hasScene: true, staleMs: 1000, paused: false, serverOk: true, displayedSeq: 5, serverLatestSeq: 5, sameSession: true };
test('fresh scene', () => {
  const f = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 120, receivedAtMs: 1000, nowMs: 1300 }));
  assert.deepStrictEqual([f.state, f.stale, f.ageMs], ['fresh', false, 420]);
});
test('status keeps arriving but the displayed scene is 49 s old: stale', () => {
  const f = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 0, receivedAtMs: 1000, nowMs: 50000 }));
  assert.deepStrictEqual([f.state, f.stale], ['stale', true]);
  assert.strictEqual(f.ageMs, 49000);
});
test('a newly received old snapshot is not fresh', () => {
  const f = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 30000, receivedAtMs: 1000, nowMs: 1010 }));
  assert.deepStrictEqual([f.state, f.stale], ['stale', true]);
});
test('server has a newer scene that is not delivered', () => {
  const f = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 0, receivedAtMs: 0, nowMs: 5000, serverLatestSeq: 9 }));
  assert.strictEqual(f.state, 'not_delivered');
});
test('paused and server down are separate states', () => {
  const p = V.sceneFreshness(Object.assign({}, base, { paused: true, sceneAgeAtReceiptMs: 0, receivedAtMs: 0, nowMs: 5000 }));
  assert.deepStrictEqual([p.state, p.stale], ['paused', true]);
  const pf = V.sceneFreshness(Object.assign({}, base, { paused: true, sceneAgeAtReceiptMs: 0, receivedAtMs: 0, nowMs: 100 }));
  assert.deepStrictEqual([pf.state, pf.stale], ['paused', false]);
  const d = V.sceneFreshness(Object.assign({}, base, { serverOk: false, sceneAgeAtReceiptMs: 0, receivedAtMs: 0, nowMs: 100 }));
  assert.deepStrictEqual([d.state, d.stale], ['server_down', true]);
  const n = V.sceneFreshness(Object.assign({}, base, { hasScene: false }));
  assert.deepStrictEqual([n.state, n.stale, n.ageMs], ['none', true, null]);
});
test('age keeps growing on the monitor clocks (no ROS /clock involved)', () => {
  const a = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 200, receivedAtMs: 0, nowMs: 700 }));
  const b = V.sceneFreshness(Object.assign({}, base, { sceneAgeAtReceiptMs: 200, receivedAtMs: 0, nowMs: 1700 }));
  assert.deepStrictEqual([a.stale, b.stale, b.ageMs - a.ageMs], [false, true, 1000]);
});
test('fetch error kinds', () => {
  assert.strictEqual(V.fetchErrorKind({ name: 'AbortError' }, 0), 'timeout');
  assert.strictEqual(V.fetchErrorKind(new TypeError('Failed to fetch'), 0), 'network');
  assert.strictEqual(V.fetchErrorKind(new Error('HTTP 503'), 503), 'busy');
  assert.strictEqual(V.fetchErrorKind(new Error('HTTP 500'), 500), 'http');
  assert.strictEqual(V.fetchErrorKind(new Error('bad frame magic'), 0), 'protocol');
});

test('audit preview never hides protected or uncertain objects',()=>{
  const scene={hd_map_audit:{enabled:true,objects:[
    {track_id:1,decision:'undetermined',preview:{decision:'exclude_candidate',reason:'cluster_support_outside'}},
    {track_id:2,decision:'keep',preview:{decision:'keep',reason:'camera_dynamic_protected'}},
    {track_id:3,decision:'undetermined'}]}};
  assert.equal(V.previewVisible(scene,{id:1,miss:0},false,true),false);
  assert.equal(V.previewVisible(scene,{id:2,miss:0},false,true),true);
  assert.equal(V.previewVisible(scene,{id:3,miss:0},false,true),true);
  assert.equal(V.previewVisible({}, {id:1,miss:0},false,true),true);
  assert.equal(V.auditAppearance(scene,{id:2}).detail,'YOLO 동적 그룹의 유효한 양성 매칭');
  assert.equal(scene.hd_map_audit.objects.length,3);
});
// Distinct support/footprint/height candidate switches must not hide protected objects.
test('independent support/footprint/height switches preserve inspection',()=>{
  const t={id:42,miss:0};
  for(const [reason,kind] of [['cluster_outside_boundary_uncertain','points'],['ordinary_space_clearly_outside','obb'],['overhead_clearance_candidate','height']]) {
    const scene={hd_map_audit:{enabled:true,objects:[{track_id:42,preview:{decision:'exclude_candidate',reason}}]}};
    if(V.auditAppearance(scene,t).category!==kind)throw new Error('candidate reason category');
    if(V.previewVisible(scene,t,true,false,{[kind]:true}))throw new Error('reason filter failed');
    if(!V.previewVisible(scene,t,true,false,{}))throw new Error('candidate cannot be inspected');
  }
});

test('legacy OBB comparison remains inspectable when its hide switch is disabled',()=>{
 const t={id:3,miss:0},scene={hd_map_audit:{enabled:true,objects:[{track_id:3,preview:{decision:'keep',reason:'cluster_drivable_support',point_decision:'keep',obb_decision:'exclude_candidate',height_decision:'keep'}}]}};
 assert.deepStrictEqual(V.exclusionKinds(scene.hd_map_audit.objects[0]),['obb']);
 assert(V.previewVisible(scene,t,false,false,{points:true,height:true,obb:false}));
 assert(!V.previewVisible(scene,t,false,false,{obb:true}));
});

test('joint preview cannot use OBB success to rescue failed point support',()=>{
 for(const points of [false,true])for(const obb of [false,true]) {
  const d={track_id:2,preview:{decision:points&&obb?'keep':'exclude_candidate',
    reason:points&&obb?'joint_drivable_support':points?'obb_overlap_below_threshold':'cluster_support_outside',
    point_decision:points?'keep':'exclude_candidate',obb_decision:obb?'keep':'exclude_candidate',height_decision:'keep'}};
  const scene={hd_map_audit:{enabled:true,map_sha256:'v7',objects:[d]}},track={id:2,miss:0};
  assert.equal(V.displayVisibility(scene,{sha256:'v7'},track,false,false,{points:true,obb:true,height:true}).visible,points&&obb);
  assert.equal(V.previewVisible(scene,track,false,true),points&&obb);
 }
});

test('final tracks and typed boxes use the same hide decision',()=>{
  const scene={hd_map_audit:{enabled:true,map_sha256:'v7',objects:[
    {track_id:7,preview:{decision:'exclude_candidate',reason:'cluster_support_outside',point_decision:'exclude_candidate',obb_decision:'keep',height_decision:'keep'}},
    {track_id:8,preview:{decision:'keep',reason:'camera_dynamic_protected',point_decision:'keep',obb_decision:'keep',height_decision:'keep'}}]}};
  const track={id:7,miss:0},typed={track_id:7};
  const check=t=>V.displayVisibility(scene,{sha256:'v7'},t,false,true,{});
  assert.deepStrictEqual(check(track),{visible:false,reason:'candidate_hidden'});
  assert.deepStrictEqual(check({id:typed.track_id,miss:0}),check(track));
  assert(check({id:8,miss:0}).visible);
  assert(check({id:9,miss:0}).visible); // No evidence does not mean outside.
  assert.equal(V.displayVisibility(scene,{sha256:'other'},track,false,true,{}).reason,'map_mismatch');
  assert.equal(V.displayVisibility(scene,{sha256:'other'},{id:7,miss:1},false,true,{}).reason,'prediction_hidden');
  assert.equal(V.displayVisibility(scene,{sha256:'v7'},track,false,false,{points:true}).visible,false);
});

test('YOLO protection and old region protection are distinguished from motion',()=>{
  const scene={hd_map_audit:{enabled:true,objects:[
    {track_id:1,preview:{decision:'keep',reason:'camera_dynamic_protected'}},
    {track_id:2,preview:{decision:'keep',reason:'exception_space_protected'}},
    {track_id:4,preview:{decision:'keep',reason:'camera_vehicle_protected',point_decision:'keep',obb_decision:'keep',height_decision:'keep'}},
    {track_id:3,preview:{decision:'exclude_candidate',reason:'cluster_support_outside',point_decision:'exclude_candidate',obb_decision:'keep',height_decision:'keep'}}]}};
  assert.equal(V.auditAppearance(scene,{id:1}).label,'YOLO 동적 보호');
  assert.equal(V.auditAppearance(scene,{id:2}).label,'구형 미션 보호');
  assert.equal(V.auditAppearance(scene,{id:4}).label,'YOLO 차량 보호');
  assert.equal(V.previewVisible(scene,{id:4,miss:0},false,true),true);
  const movingUnknown={id:3,miss:0,class:'unknown',motion:{state:3,valid:true}};
  assert.equal(V.auditAppearance(scene,movingUnknown).label,'제외 후보');
  assert.equal(V.previewVisible(scene,movingUnknown,false,true),false);
});

console.log('\n' + passed + ' passed, ' + failures.length + ' failed');
process.exit(failures.length ? 1 : 0);
