// asmc_lidar shadow monitor — pure view logic shared by app.js and the node tests
// (test/monitor/test_monitor_view.js): frame matrices, BEV/side projection, OBB
// outlines, frame cursor across server sessions and displayed-scene freshness.
// No DOM access; loaded before app.js (window.AsmcMonitorView) or via require().
'use strict';
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.AsmcMonitorView = api;
})(typeof self !== 'undefined' ? self : this, function () {
  const I4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

  // ------------------------------------------------------------ matrices (row-major 4x4)
  function rot3To4(r) { return [r[0], r[1], r[2], 0, r[3], r[4], r[5], 0, r[6], r[7], r[8], 0, 0, 0, 0, 1]; }
  function mul(a, b) {
    const o = new Array(16);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
      o[i * 4 + j] = a[i * 4] * b[j] + a[i * 4 + 1] * b[4 + j] + a[i * 4 + 2] * b[8 + j] + a[i * 4 + 3] * b[12 + j];
    }
    return o;
  }
  function invRigid(m) {  // [R t; 0 1]^-1 = [R^T -R^T t]
    const r = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]];
    const t = [m[3], m[7], m[11]];
    return [r[0], r[1], r[2], -(r[0] * t[0] + r[1] * t[1] + r[2] * t[2]),
      r[3], r[4], r[5], -(r[3] * t[0] + r[4] * t[1] + r[5] * t[2]),
      r[6], r[7], r[8], -(r[6] * t[0] + r[7] * t[1] + r[8] * t[2]), 0, 0, 0, 1];
  }
  function apply(m, x, y, z) {
    return [m[0] * x + m[1] * y + m[2] * z + m[3], m[4] * x + m[5] * y + m[6] * z + m[7], m[8] * x + m[9] * y + m[10] * z + m[11]];
  }
  function isMatrix(m) { return Array.isArray(m) && m.length === 16 && m.every((v) => typeof v === 'number'); }

  // Matrices from each data frame to the selected view frame (null = unavailable).
  function frameMatrices(scene, viewFrame) {
    if (!scene || !scene.stages) return null;
    const Bs = isMatrix(scene.base_from_sensor) ? scene.base_from_sensor : null;
    const Bl = isMatrix(scene.base_from_leveled) ? scene.base_from_leveled : null;
    const L = Array.isArray(scene.leveling) && scene.leveling.length === 9 ? rot3To4(scene.leveling) : null;
    const M = scene.map && scene.map.valid && isMatrix(scene.map.map_from_base) ? scene.map.map_from_base : null;
    if (!Bs || !L) return null;
    const toBase = { sensor: Bs, leveled: Bl, base: I4, map: M ? invRigid(M) : null };
    let viewFromBase = null;
    if (viewFrame === 'base') viewFromBase = I4;
    else if (viewFrame === 'leveled') viewFromBase = Bl ? invRigid(Bl) : null;
    else if (viewFrame === 'sensor') viewFromBase = invRigid(Bs);
    else if (viewFrame === 'map') viewFromBase = M;
    if (!viewFromBase) return null;
    const out = {};
    for (const key of Object.keys(toBase)) out[key] = toBase[key] ? mul(viewFromBase, toBase[key]) : null;
    if (viewFrame === 'leveled') { out.leveled = I4; out.sensor = L; }
    if (viewFrame === 'sensor') { out.sensor = I4; out.leveled = invRigid(L); }
    out.vehicle = viewFromBase;  // base_link outline -> view
    return out;
  }

  // ------------------------------------------------------------ projections
  // view: { scale, panX, panY, side }; origin: base_link origin in the view frame.
  function bevProject(view, origin, w, h, x, y) {
    const dx=x-origin[0],dy=y-origin[1],a=view.bevYaw || 0,c=Math.cos(a),s=Math.sin(a);
    return [w/2+view.panX+(s*dx-c*dy)*view.scale,h/2+view.panY-(c*dx+s*dy)*view.scale];
  }
  function bevUnproject(view, origin, w, h, px, py) {
    const a=view.bevYaw || 0,c=Math.cos(a),s=Math.sin(a);
    const u=(px-w/2-view.panX)/view.scale,v=-(py-h/2-view.panY)/view.scale;
    return [origin[0]+s*u+c*v,origin[1]-c*u+s*v];
  }
  function sideProject(view, origin, w, h, p) {
    const horiz = view.side === 'xz' ? p[0] - origin[0] : -(p[1] - origin[1]);
    return [w / 2 + view.panX + horiz * view.scale, h * 0.65 - (p[2] - origin[2]) * view.scale];
  }

  // Orthographic orbit camera: yaw=0 looks forward, pitch=pi/2 is the BEV.
  function orbitProject(view, origin, w, h, p) {
    const x = p[0] - origin[0], y = p[1] - origin[1], z = p[2] - origin[2];
    const c = Math.cos(view.yaw), s = Math.sin(view.yaw);
    return [w / 2 + view.panX + (s * x - c * y) * view.scale,
      h / 2 + view.panY - (Math.sin(view.pitch) * (c * x + s * y) + Math.cos(view.pitch) * z) * view.scale];
  }

  function pointInPolygon(point, polygon) {
    let inside=false;
    const [x,y]=point;
    for(let i=0,j=polygon.length-1;i<polygon.length;j=i++) {
      const [ax,ay]=polygon[j],[bx,by]=polygon[i],dx=bx-ax,dy=by-ay;
      const len2=dx*dx+dy*dy;
      if(len2>0) {
        const t=Math.max(0,Math.min(1,((x-ax)*dx+(y-ay)*dy)/len2));
        if(Math.hypot(x-ax-t*dx,y-ay-t*dy)<=1e-7)return true;
      }
      if((ay>y)!==(by>y) && x<(bx-ax)*(y-ay)/(by-ay)+ax)inside=!inside;
    }
    return inside;
  }
  const roadIndices=new WeakMap();
  function motionAppearance(motion) {
    if(!motion || !motion.valid || ![1,2,3].includes(motion.state))return {color:'#facc15',label:'판정 불가'};
    if(motion.state===1)return {color:'#22c55e',label:'정적 준함'};
    return {color:'#ff4545',label:motion.state===2?'동적 의심':'동적'};
  }
  function roadMembership(scene,road,center) {
    if(!road || !road.available || !Array.isArray(road.polygons) || !center || !center.every(Number.isFinite))return 'unknown';
    const m=frameMatrices(scene,'map');
    if(!m || !m.leveled)return 'unknown';
    const p=apply(m.leveled,...center);
    if(!p.every(Number.isFinite))return 'unknown';
    let index=roadIndices.get(road);
    if(!index){index=road.polygons.map(poly=>({poly,bounds:poly.reduce((b,p)=>[Math.min(b[0],p[0]),Math.min(b[1],p[1]),Math.max(b[2],p[0]),Math.max(b[3],p[1])],[Infinity,Infinity,-Infinity,-Infinity])}));roadIndices.set(road,index);}
    return index.some(({poly,bounds:b})=>p[0]>=b[0]-1e-7 && p[0]<=b[2]+1e-7 && p[1]>=b[1]-1e-7 && p[1]<=b[3]+1e-7 && pointInPolygon(p,poly))?'inside':'outside';
  }

  const auditIndices=new WeakMap();
  const AUDIT_REASONS={camera_dynamic_protected:'카메라 동적 그룹 보호',cluster_drivable_support:'군집 점이 도로 안',
    cluster_boundary_protected:'경계 여유 보호',exception_space_protected:'미션 예외 공간 보호',
    cluster_support_outside:'군집 점이 도로 밖',overhead_clearance_candidate:'자차 상부 높이 후보',
    overhead_lower_surface_unverified:'하부 관측 검증 대기',insufficient_cluster_support:'근거 점 부족',
    unknown_support_map_cells:'점 위치의 지도 근거 부족',map_or_pose_unavailable:'지도/위치 무효',
    pose_error_bound_unverified:'위치 오차 검증 대기',point_policy_pose_and_scope_unverified:'위치·구간 검증 대기'};
  function auditForTrack(scene,track){
    const report=scene&&scene.hd_map_audit;
    if(!report||!report.enabled)return null;
    let index=auditIndices.get(report);
    if(!index){index=new Map((report.objects||[]).map(d=>[d.track_id,d]));auditIndices.set(report,index);}
    return index.get(track.id)||null;
  }
  function auditAppearance(scene,track){
    const d=auditForTrack(scene,track),p=d&&d.preview;
    const kind=p?p.decision:d?d.decision:'unavailable';
    return {decision:kind,reason:p?p.reason:d?d.reason:'audit_off',
      label:{keep:'유지',exclude_candidate:'제외 후보',undetermined:'보류',unavailable:'미실행'}[kind]||'보류',
      color:{keep:'#34d399',exclude_candidate:'#fb7185',undetermined:'#fbbf24',unavailable:'#94a3b8'}[kind]||'#fbbf24',
      detail:AUDIT_REASONS[p?p.reason:d?d.reason:'']||(p?p.reason:d?d.reason:'audit OFF')};
  }
  function previewVisible(scene,track,showPred,hideCandidates){
    if(!showPred&&track.miss>0)return false;
    return !hideCandidates||auditAppearance(scene,track).decision!=='exclude_candidate';
  }
  function trackVisible(scene,road,track,hideOutside,showPred=true) {
    if(!showPred && track.miss>0)return false;
    // Unknown is not outside: never silently hide objects after GPS loss.
    return !hideOutside || roadMembership(scene,road,track.obb.center)!=='outside';
  }
  class IdChangeMonitor {
    constructor(){this.reset();}
    reset(){this.history=new Map();this.warnings=new Map();this.lastTime=null;}
    update(scene,road){
      const m=frameMatrices(scene,'map'),time=scene?Number(scene.stamp_ns)/1e9:NaN;
      if(!road || !road.available || !m || !m.leveled || !Number.isFinite(time) ||
          (this.lastTime!==null && (time<=this.lastTime || time-this.lastTime>1))){this.reset();return this.warnings;}
      this.lastTime=time;
      for(const [id,v] of this.history)if(time-v.time>0.8)this.history.delete(id);
      for(const [id,v] of this.warnings)if(time>v.expires)this.warnings.delete(id);
      const observed=(scene.tracks||[]).filter(t=>t.id>=0 && t.miss===0);
      const active=new Set(observed.map(t=>t.id));
      const current=observed.filter(t=>roadMembership(scene,road,t.obb.center)==='inside')
        .map(t=>({id:t.id,p:apply(m.leveled,...t.obb.center),w:t.obb.width,h:t.obb.height,cls:t.class,time}));
      const newcomers=current.filter(t=>!this.history.has(t.id));
      const gone=[...this.history.values()].filter(t=>!active.has(t.id));
      const pairs=[];
      for(const n of newcomers)for(const old of gone){
        if(!n.p.every(Number.isFinite) || !old.p.every(Number.isFinite))continue;
        const dist=Math.hypot(n.p[0]-old.p[0],n.p[1]-old.p[1]);
        const comparable=[n.w,n.h,old.w,old.h].every(x=>Number.isFinite(x)&&x>0);
        if(!comparable || dist>2 || Math.max(n.w/old.w,old.w/n.w,n.h/old.h,old.h/n.h)>1.8)continue;
        if(n.cls && old.cls && n.cls!=='unknown' && old.cls!=='unknown' && n.cls!==old.cls)continue;
        pairs.push({n,old,dist});
      }
      // Require an unambiguous, reciprocal nearest pair; do not guess in a crowd.
      for(const pair of pairs){
        const others=pairs.filter(p=>p!==pair && (p.n.id===pair.n.id || p.old.id===pair.old.id));
        if(others.some(p=>p.dist<pair.dist+0.5))continue;
        this.warnings.set(pair.n.id,{from:pair.old.id,expires:time+3,dist:pair.dist});
        this.history.delete(pair.old.id);
      }
      for(const t of current)this.history.set(t.id,t);
      return this.warnings;
    }
  }

  // ------------------------------------------------------------ OBB outlines
  // corners: the 8 OBB corners of the message (0-3 at zMin counter-clockwise from
  // +z with edge 0->1 along the box axis, 4-7 the same XY at zMax), in the data
  // frame of `matrix`; project(p) maps a view-frame point to screen pixels.
  // 'bev' keeps the rotated footprint: the 4 projected bottom corners in order and
  // a heading segment from the footprint centre to the middle of edge 1-2 (the +axis
  // end), so the marker is always parallel to edge 0->1. 'side' is the screen
  // bounding rectangle of all 8 projected corners (a vertical cut has no yaw).
  function boxOutline(kind, corners, matrix, project) {
    if (!matrix || !Array.isArray(corners) || corners.length < 8) return null;
    const pts = corners.slice(0, 8).map((c) => project(apply(matrix, c[0], c[1], c[2])));
    if (kind === 'orbit') return { kind, points: pts };
    if (kind === 'bev') {
      const polygon = pts.slice(0, 4);
      const centre = [(polygon[0][0] + polygon[1][0] + polygon[2][0] + polygon[3][0]) / 4,
        (polygon[0][1] + polygon[1][1] + polygon[2][1] + polygon[3][1]) / 4];
      const front = [(polygon[1][0] + polygon[2][0]) / 2, (polygon[1][1] + polygon[2][1]) / 2];
      return { kind, polygon, heading: [centre, front], points: pts };
    }
    if (kind === 'side') {
      let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
      for (const p of pts) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
      return { kind, rect: [x0, y0, x1 - x0, y1 - y0], points: pts };
    }
    throw new Error('unknown view kind ' + kind);
  }
  function drawBox(ctx, kind, corners, matrix, project, style) {
    const outline = boxOutline(kind, corners, matrix, project);
    if (!outline) return null;
    ctx.strokeStyle = style.color; ctx.lineWidth = style.width || 1.5; ctx.setLineDash(style.dash || []);
    if (kind === 'orbit') {
      const pts = outline.points;
      ctx.beginPath();
      for (const [a, b] of [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]) {
        ctx.moveTo(pts[a][0], pts[a][1]); ctx.lineTo(pts[b][0], pts[b][1]);
      }
      ctx.stroke();
    } else if (kind === 'bev') {
      const p = outline.polygon;
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (let i = 1; i < 4; i++) ctx.lineTo(p[i][0], p[i][1]);
      ctx.closePath(); ctx.stroke();
      const [c, f] = outline.heading;
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(f[0], f[1]); ctx.stroke();
    } else {
      ctx.strokeRect(outline.rect[0], outline.rect[1], outline.rect[2], outline.rect[3]);
    }
    ctx.setLineDash([]);
    return outline;
  }

  // ------------------------------------------------------------ frame cursor / freshness
  // cursor: { session, frameSeq } of the displayed scene (session null = none yet).
  // header: /api/frame header ({ session, frame_seq, scene, ... }).
  // reset: the server is another process (new session) -> drop the displayed scene,
  // its arrays and the cursor, whether or not this response carries a new scene.
  // accept: show header.scene. A scene of the same session that is not newer than
  // the cursor is never shown again.
  function frameAction(cursor, header) {
    const session = header && header.session ? String(header.session) : null;
    const reset = cursor.session !== null && session !== null && session !== cursor.session;
    if (!header || !header.scene) return { reset, accept: false, reason: reset ? 'new_session' : 'no_scene' };
    if (reset || cursor.session === null) return { reset, accept: true, reason: reset ? 'new_session' : 'first' };
    if (Number(header.frame_seq) > cursor.frameSeq) return { reset: false, accept: true, reason: 'newer' };
    return { reset: false, accept: false, reason: 'not_newer' };
  }

  // Age and state of the displayed scene, from the monitor's own monotonic clocks
  // (server time.monotonic age at send + browser performance.now since receipt), so
  // a stopped /clock or a status-only response can never make it fresh.
  // o: { hasScene, sceneAgeAtReceiptMs, receivedAtMs, nowMs, staleMs, paused,
  //      serverOk, displayedSeq, serverLatestSeq, sameSession }
  // state: none | paused | server_down | not_delivered | stale | fresh
  function sceneFreshness(o) {
    if (!o.hasScene) return { ageMs: null, stale: true, state: o.paused ? 'paused' : o.serverOk ? 'none' : 'server_down' };
    const atReceipt = Number.isFinite(o.sceneAgeAtReceiptMs) ? Math.max(0, o.sceneAgeAtReceiptMs) : 0;
    const ageMs = atReceipt + Math.max(0, o.nowMs - o.receivedAtMs);
    const old = ageMs > o.staleMs;
    let state = 'fresh';
    if (o.paused) state = 'paused';
    else if (!o.serverOk) state = 'server_down';
    else if (old) state = (o.sameSession && o.serverLatestSeq > o.displayedSeq) ? 'not_delivered' : 'stale';
    return { ageMs, stale: old || !o.serverOk, state };
  }

  // Classify a failed poll for the status chip.
  function fetchErrorKind(error, httpStatus) {
    if (httpStatus === 503) return 'busy';
    if (httpStatus) return 'http';
    if (error && error.name === 'AbortError') return 'timeout';
    if (error && error.name === 'TypeError') return 'network';
    return 'protocol';
  }

  return { I4, rot3To4, mul, invRigid, apply, isMatrix, frameMatrices, bevProject, bevUnproject, sideProject, orbitProject,
    boxOutline, drawBox, frameAction, sceneFreshness, fetchErrorKind, pointInPolygon, roadMembership, motionAppearance,
    trackVisible, IdChangeMonitor, auditForTrack, auditAppearance, previewVisible };
});
