// asmc_lidar read-only canvas viewer of pipeline stage clouds and tracks.
// Polls /api/frame (binary) at up to 10 Hz; every control on
// this page changes only how data is drawn. No external library; the pure view
// logic (matrices, projection, OBB outlines, cursor, freshness) is monitor_view.js.
'use strict';
(function () {
  const V = window.AsmcMonitorView;
  const POLL_MS = 100;
  const STALE_MS = 1000;         // displayed scene older than this: STALE
  const FETCH_TIMEOUT_MS = 3000;  // a poll still unanswered after this is aborted and retried
  const STATUS_NAMES = { 1: 'OBSERVED', 2: 'OBSERVED_EMPTY', 3: 'INPUT_INVALID', 4: 'POSE_UNAVAILABLE' };
  const COLORS = {
    raw: '#6b7280', roi: '#a78bfa', voxel: '#60a5fa', leveled: '#38bdf8', ground: '#4b5563',
    tunnel: '#fb923c', nonground: '#34d399', compare: '#1d4ed8', detections: '#fbbf24', typed: '#facc15',
    predicted: '#9ca3af', grid: '#1f2937', axis: '#374151', vehicle: '#e5e7eb', text: '#e5e7eb',
  };
  const CLASS_COLORS = { vehicle: '#f472b6', car: '#f472b6', truck: '#f472b6', bus: '#f472b6',
    pedestrian: '#fb923c', person: '#fb923c', unknown: '#ffffff' };
  const CLUSTER_PALETTE = ['#ef4444', '#f59e0b', '#22c55e', '#3b82f6', '#a855f7', '#14b8a6', '#eab308',
    '#ec4899', '#84cc16', '#06b6d4', '#f97316', '#8b5cf6'];
  const { I4, apply, frameMatrices } = V;
  const idChanges=new V.IdChangeMonitor();

  const el = (id) => document.getElementById(id);
  const fmt = (v, d) => (v === null || v === undefined || Number.isNaN(Number(v))) ? '-' : Number(v).toFixed(d === undefined ? 2 : d);
  const fmtInt = (v) => (v === null || v === undefined || v === '') ? '-' : String(v);

  const state = {
    // Displayed scene: its server session, frame cursor, server-side age when sent, receipt time.
    session: null, frameSeq: 0, scene: null, arrays: {}, sceneAgeAtReceipt: null, sceneReceivedAt: 0,
    // Last response: status, the server's session and newest frame_seq, poll outcome.
    status: null, serverSession: null, serverLatestSeq: 0, statusAt: 0, serverOk: false, serverError: '', errorKind: '',
    sessionChanges: 0, sessionChangedAt: 0, ignoredFrames: 0, pollsAborted: 0,
    paused: false, inflight: false, inflightSince: 0, controller: null,
    view: { frame: 'base', mode: 'bev', bevYaw: 0, yaw: -0.4, pitch: 0.65, road: true, scale: 9, panX: 0, panY: 0, side: 'xz', psize: 2, compare: false, labels: true },
    road: null,
    hideCandidates:false,hideReasons:{points:true,obb:true,height:true,policy:true},colorMode:"audit",pointColorMode:"road",showPred:false,idWarnings:new Map(),
    stages: {}, selected: null, webFrames: [], matrices: null,
  };

  // ------------------------------------------------------------ frame decoding / polling
  function decodeFrame(buf) {
    const dv = new DataView(buf);
    if (buf.byteLength < 12 || dv.getUint32(0, false) !== 0x414c4d46) throw new Error('bad frame magic');
    const hl = dv.getUint32(4, true);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 12, hl)));
    const po = 12 + hl + ((4 - (hl % 4)) % 4);
    const arrays = {};
    for (const [name, a] of Object.entries(header.arrays || {})) {
      const off = po + a.offset;
      if (a.dtype === 'f32') arrays[name] = new Float32Array(buf, off, a.length);
      else if (a.dtype === 'u8') arrays[name] = new Uint8Array(buf, off, a.length);
      else if (a.dtype === 'i32') arrays[name] = new Int32Array(buf, off, a.length);
    }
    return { header, arrays };
  }

  // Drop the displayed scene (server restarted: its objects must not look current).
  function clearScene() {
    idChanges.reset();state.idWarnings=new Map();
    state.scene = null; state.arrays = {}; state.frameSeq = 0; state.sceneAgeAtReceipt = null; state.sceneReceivedAt = 0;
    state.matrices = null; state.webFrames = [];
  }
  function noteServerSession(session) {
    if (!session) return;
    if (state.serverSession !== null && session !== state.serverSession) { state.sessionChanges += 1; state.sessionChangedAt = performance.now(); }
    state.serverSession = session;
  }

  async function poll() {
    const now = performance.now();
    if (state.inflight) {
      // fetch() itself is aborted at FETCH_TIMEOUT_MS; this only guards against a stuck promise.
      if (now - state.inflightSince < FETCH_TIMEOUT_MS + 2000) return;
      if (state.controller) state.controller.abort();
    }
    const controller = new AbortController();
    state.controller = controller; state.inflight = true; state.inflightSince = now;
    const timer = setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
    let httpStatus = 0;
    try {
      if (state.paused) {
        const r = await fetch('/api/status', { cache: 'no-store', signal: controller.signal });
        if (!r.ok) { httpStatus = r.status; throw new Error('HTTP ' + r.status); }
        const st = await r.json();
        if (state.controller !== controller) return;
        state.status = st;
        noteServerSession(st.session);
        if (st.latest_scene && st.session === state.session) state.serverLatestSeq = st.latest_scene.frame_seq;
      } else {
        const url = '/api/frame?since=' + state.frameSeq + (state.session ? '&session=' + encodeURIComponent(state.session) : '');
        const r = await fetch(url, { cache: 'no-store', signal: controller.signal });
        if (!r.ok) { httpStatus = r.status; throw new Error('HTTP ' + r.status); }
        const f = decodeFrame(await r.arrayBuffer());
        if (state.controller !== controller) return;
        const h = f.header;
        state.status = h.status;
        noteServerSession(h.session);
        const action = V.frameAction({ session: state.session, frameSeq: state.frameSeq }, h);
        if (action.reset) clearScene();
        if (action.accept) {
          state.session = h.session || null;
          state.scene = h.scene;
          if(!state.lastWarningStamp || state.lastWarningStamp!==state.scene.stamp_ns) {state.idWarnings=idChanges.update(state.scene,state.road);state.lastWarningStamp=state.scene.stamp_ns;}
          state.arrays = f.arrays;
          state.frameSeq = h.frame_seq;
          state.sceneAgeAtReceipt = typeof h.scene_age_ms === 'number' ? h.scene_age_ms : null;
          state.sceneReceivedAt = performance.now();
          if(state.lastWebStamp!==state.scene.stamp_ns){state.webFrames.push(state.sceneReceivedAt);state.lastWebStamp=state.scene.stamp_ns;}
          state.matrices = frameMatrices(state.scene, state.view.frame);
        } else if (action.reason === 'not_newer') {
          state.ignoredFrames += 1;
        }
        if (action.reset && !action.accept) state.session = h.session || null;
        state.serverLatestSeq = h.frame_seq;  // newest ready scene of the server, sent or not
        if (action.accept || action.reset) { renderScene(); renderTracks(); renderDetail(); }
      }
      state.statusAt = performance.now();
      state.serverOk = true; state.serverError = ''; state.errorKind = '';
    } catch (e) {
      if (state.controller !== controller) return;
      state.serverOk = false;
      state.errorKind = V.fetchErrorKind(e, httpStatus);
      if (state.errorKind === 'timeout') state.pollsAborted += 1;
      state.serverError = state.errorKind === 'timeout' ? FETCH_TIMEOUT_MS / 1000 + ' s 내 응답 없음' : String(e && e.message ? e.message : e);
    } finally {
      clearTimeout(timer);
      if (state.controller === controller) { state.inflight = false; state.controller = null; }
    }
    renderStatus();
  }

  // ------------------------------------------------------------ status rendering
  function chip(id, text, cls) {
    const c = el(id);
    c.textContent = text;
    c.className = 'chip ' + (cls || '');
  }
  function stampText(ns) {
    if (!ns) return '-';
    const s = Math.floor(Number(ns) / 1e9);
    const frac = String(Number(ns) % 1e9).padStart(9, '0');
    return s + '.' + frac;
  }
  function msText(ms) {
    if (ms === null || ms === undefined) return '-';
    return ms < 10000 ? (ms / 1000).toFixed(2) + ' s' : Math.round(ms / 1000) + ' s';
  }
  // Server-side input age: time since the monitor server last received a debug scan JSON.
  function serverScanAgeMs() {
    const st = state.status;
    if (!st || !st.ages_ms || st.ages_ms.scan === null || st.ages_ms.scan === undefined) return null;
    return st.ages_ms.scan + (performance.now() - state.statusAt);
  }
  // Displayed scene: its own age and state, independent of the status values.
  function freshness() {
    return V.sceneFreshness({
      hasScene: !!state.scene, sceneAgeAtReceiptMs: state.sceneAgeAtReceipt, receivedAtMs: state.sceneReceivedAt,
      nowMs: performance.now(), staleMs: STALE_MS, paused: state.paused, serverOk: state.serverOk,
      displayedSeq: state.frameSeq, serverLatestSeq: state.serverLatestSeq,
      sameSession: state.session !== null && state.session === state.serverSession,
    });
  }
  function isStale() { return freshness().stale; }

  function renderStatus() {
    const st = state.status;
    const counts={keep:0,exclude_candidate:0,undetermined:0};
    const report=state.scene&&state.scene.hd_map_audit;
    for(const d of report&&report.objects||[]) {const v=d.preview?d.preview.decision:d.decision;counts[v]=(counts[v]||0)+1;}
    const mapMismatch=report&&report.map_sha256&&state.road&&state.road.sha256&&report.map_sha256!==state.road.sha256;
    el('audit-summary').textContent=(report&&report.delivery_applied?'관측 '+report.delivery_input_count+' · 발행 '+report.delivery_output_count+' · 실제 제외 '+report.delivery_excluded_count+' · 예측 '+((state.scene.tracks||[]).filter(t=>t.miss>0).length):report&&report.enabled?'원본 '+(report.objects||[]).length+' · 유지 '+counts.keep+' · 비교상 제외 후보 '+counts.exclude_candidate+' · 보류 '+counts.undetermined:'audit OFF · 원본 표시')+
      (report&&report.delivery_applied?' · ROS 발행 '+report.delivery_output_count+' / '+report.delivery_input_count+' · 실제 제외 '+report.delivery_excluded_count:' · audit 비교 · ROS 발행 유지')+
      (report?' · 안쪽 경계 '+fmt(report.road_inset_m*100,0)+'cm':'')+(mapMismatch?' · 주의: 배경/판정 지도 SHA 다름':'')+
      (report&&report.pose_source_age_ms>=0?' · GPS age '+fmt(report.pose_source_age_ms,0)+'ms':'')+' · 그리기 '+fmt(state.lastRenderMs,1)+'ms';
    const tracks=state.scene&&state.scene.tracks||[];
    const observed=tracks.filter(t=>t.miss===0),shown=observed.filter(visibleTrack).length;
    el('filter-summary').textContent='현재 관측 '+observed.length+' · 표시 대상 '+shown+' · 숨김 '+(observed.length-shown)+
      ' · 예측 '+(tracks.length-observed.length)+' (별도 스위치)';
    const originals=['raw','roi','voxel','leveled','detections'].filter(k=>state.stages[k]);
    if(state.view.compare)originals.push('voxel 비교');
    el('filter-layer-note').textContent=originals.length?'원본 대조 레이어 ON: '+originals.join(', ')+' · 발행 제외 숨김 미적용':'';
    const now = performance.now();
    state.webFrames = state.webFrames.filter((t) => now - t < 2000);
    const sinceServer = state.statusAt ? now - state.statusAt : null;
    const errText = { timeout: '응답 정지 (timeout, 요청 취소 후 재시도)', network: '연결 실패', busy: '서버 혼잡 (HTTP 503, 재시도)',
      http: 'HTTP 오류', protocol: '응답 형식 오류' }[state.errorKind] || '연결 끊김';
    if (state.serverOk) chip('chip-server', '서버: 연결됨 · 세션 ' + (state.serverSession ? state.serverSession.slice(-8) : '-'), 'ok');
    else chip('chip-server', '서버: ' + errText + ' ' + (sinceServer !== null ? '(' + msText(sinceServer) + ' 전 마지막 응답)' : '') + (state.serverError ? ' · ' + state.serverError : ''), 'bad');
    if (!st) { showBanner(); return; }
    const m = st.master || {};
    if (m.ok === false) chip('chip-ros', 'ROS master 불가: ' + (m.error || ''), 'bad');
    else if (m.ok === true) chip('chip-ros', 'ROS: master OK · shadow 노드 ' + (m.shadow_nodes && m.shadow_nodes.length ? m.shadow_nodes.join(',') : '없음') + ' · ' + (st.config && st.config.topics ? st.config.topics.points : '/velodyne_points') + ' publisher ' + fmtInt(m.points_publishers), m.shadow_nodes && m.shadow_nodes.length ? 'ok' : 'warn');
    else chip('chip-ros', 'ROS: 확인 중', '');
    const d = st.pipeline_detail || {};
    const pipelineText = {
      ok: ['pipeline: 정상 처리', 'ok'],
      no_lidar_input: ['pipeline: ROS 연결됨, LiDAR 입력 없음 (input_silence)', 'warn'],
      lidar_stale: ['pipeline: LiDAR 입력 stale', 'warn'],
      input_invalid: ['pipeline: IMU/CAN 무효 → INPUT_INVALID (' + (d.input_reason || d.pose_reason || '') + ')', 'bad'],
      no_shadow_node: ['pipeline: shadow 노드 diagnostics 없음', 'bad'],
      ros_master_unreachable: ['pipeline: ROS master 불가', 'bad'],
    }[st.pipeline_state] || ['pipeline: ' + st.pipeline_state, ''];
    chip('chip-pipeline', pipelineText[0], pipelineText[1]);
    const dbg = { live: ['모니터 데이터: 수신 중', 'ok'], stale: ['모니터 데이터: stale', 'warn'], no_data: ['모니터 데이터: 대기 (debug 토픽 미수신)', 'warn'],
      disabled: ['모니터 데이터: shadow 노드 debug_enabled=false', 'bad'], unknown: ['모니터 데이터: -', ''] }[st.debug_state] || ['모니터 데이터: ' + st.debug_state, ''];
    chip('chip-debug', dbg[0], dbg[1]);
    const o = st.objects_local;
    if (o) {
      const name = STATUS_NAMES[o.status] || String(o.status);
      chip('chip-typed', 'typed local: ' + name + (o.status === 1 ? ' (' + o.count + ')' : '') + (o.status >= 3 ? ' ' + o.reason : ''), o.status === 1 || o.status === 2 ? 'ok' : o.status === 4 ? 'warn' : 'bad');
    } else chip('chip-typed', 'typed local: 미수신', '');
    const mapText = st.map_state === 'valid' ? 'map: 사용 가능 (' + (st.map_reason || '') + ')'
      : st.map_state === 'unavailable' ? 'map: 사용 불가 — POSE_UNAVAILABLE (' + (st.map_reason || '') + ')'
      : st.map_state === 'input_invalid' ? 'map: INPUT_INVALID (' + (st.map_reason || '') + ')' : 'map: 미확인';
    chip('chip-map', mapText, st.map_state === 'valid' ? 'ok' : st.map_state === 'unavailable' ? 'warn' : st.map_state === 'input_invalid' ? 'bad' : '');
    const cam = st.camera_state;
    const camText = cam === 'enabled' ? ('camera: fusion ' + (st.camera_active === 'true' ? '활성' : '비활성') + ' (' + (st.camera_reason || '') + ')') : cam === 'lidar_only' ? 'camera: LiDAR 단독 (' + (st.camera_reason || '') + ')' : cam === 'disabled' ? 'camera: fusion 비활성화' : 'camera: -';
    chip('chip-camera', camText, cam === 'enabled' ? (st.camera_active === 'true' ? 'ok' : 'warn') : 'info');
    chip('chip-leveling', 'roll/pitch 보정: ' + (st.roll_leveling === 'true' ? 'roll+pitch' : st.roll_leveling === 'false' ? 'pitch only' : '-'), 'info');
    chip('chip-tunnel', 'TunnelFilter: ' + (st.tunnel_filter === 'true' ? 'ON (실험)' : st.tunnel_filter === 'false' ? 'OFF' : '-'), st.tunnel_filter === 'true' ? 'warn' : 'info');
    const v = st.diagnostics || {};
    const fr = freshness();
    el('n-stamp').textContent = state.scene ? stampText(state.scene.stamp_ns) : '- (장면 없음)';
    el('n-age').textContent = msText(fr.ageMs);
    el('n-server-age').textContent = msText(serverScanAgeMs());
    el('n-session').textContent = state.session ? state.session.slice(-8) + (state.serverSession && state.serverSession !== state.session ? ' (서버 ' + state.serverSession.slice(-8) + ')' : '') : '-';
    el('n-frame').textContent = (state.scene ? state.frameSeq : '-') + ' / ' + fmtInt(state.serverLatestSeq);
    const r = st.rates || {};
    el('n-input-hz').textContent = fmt(r.input_hz, 1);
    el('n-proc-hz').textContent = fmt(r.processed_hz, 1);
    el('n-debug-hz').textContent = fmt(r.debug_hz, 1);
    el('n-web-hz').textContent = (state.webFrames.length / 2).toFixed(1);
    el('n-received').textContent = fmtInt(v.scans_received);
    el('n-processed').textContent = fmtInt(v.scans_processed);
    el('n-overwritten').textContent = fmtInt(v.overwritten_scans);
    el('n-rejected').textContent = fmtInt(v.rejected_scans);
    el('n-p50').textContent = fmt(v.processing_ms_p50, 1);
    el('n-p95').textContent = fmt(v.processing_ms_p95, 1);
    renderStatusTable(st);
    showBanner();
    if (fr.stale !== state.lastStale) { state.lastStale = fr.stale; renderScene(); }
  }

  function showBanner() {
    const b = el('banner');
    const fr = freshness();
    const st = state.status;
    let text = '', cls = 'hidden';
    const sceneText = state.scene ? ' (표시 장면 stamp ' + stampText(state.scene.stamp_ns) + ', ' + msText(fr.ageMs) + ' 전)' : '';
    const restarted = state.sessionChangedAt && performance.now() - state.sessionChangedAt < 10000;
    if (fr.state === 'paused') {
      text = '화면 정지 (Pause) — 표시 장면은 갱신되지 않습니다' + sceneText + '. 서버와 pipeline은 계속 동작하며 상태값만 갱신됩니다'
        + (state.serverSession && state.session && state.serverSession !== state.session ? ' · 서버 재시작됨: 재개하면 이전 장면을 버리고 새 장면을 받습니다' : '');
      cls = 'pause';
    } else if (fr.state === 'server_down') {
      text = '서버 ' + ({ timeout: '응답 정지 (timeout)', busy: '혼잡 (503)' }[state.errorKind] || '연결 끊김') + ' — 재시도 중' + (state.scene ? '. 마지막 화면은 과거 데이터입니다' + sceneText : '');
      cls = 'down';
    } else if (!state.scene) {
      text = restarted ? '서버 재시작 감지 (세션 변경) — 이전 장면과 객체는 폐기했습니다. 새 장면을 기다립니다'
        : '데이터 대기 중 — shadow 노드의 debug 출력(' + ((st && st.config && st.config.topics) ? st.config.topics.scan : '') + ')을 기다립니다';
      cls = 'wait';
    } else if (fr.stale) {
      const input = st && st.debug_state === 'live' ? '서버는 debug scan을 받고 있지만 새 장면이 화면에 오지 않습니다'
        : '서버는 응답하지만 새 장면이 없습니다 (debug 입력: ' + (st ? st.debug_state : '-') + ', 서버 scan 경과 ' + msText(serverScanAgeMs()) + ')';
      text = 'STALE — ' + (fr.state === 'not_delivered' ? '새 장면 미수신 (서버 frame ' + state.serverLatestSeq + ' > 표시 ' + state.frameSeq + '). ' : '') + input + sceneText + '. 표시된 객체는 현재 장애물이 아닙니다';
      cls = 'stale';
    }
    b.textContent = text;
    b.className = 'banner ' + cls;
    b.dataset.state = state.scene ? fr.state : (fr.state === 'paused' || fr.state === 'server_down' ? fr.state : 'none');
    el('btn-pause').textContent = state.paused ? '재개' : '일시정지';
    el('btn-pause').className = state.paused ? 'active' : '';
  }

  function renderStatusTable(st) {
    const report=state.scene&&state.scene.hd_map_audit;
    const v = st.diagnostics || {};
    const sc = state.scene;
    const rows = [];
    const add = (k, val) => rows.push([k, val === undefined || val === null || val === '' ? '-' : val]);
    add('diagnostics', (st.pipeline_detail && st.pipeline_detail.message) + ' · ' + msText(st.ages_ms && st.ages_ms.diagnostics) + ' 전');
    add('input_reason / pose', (v.input_reason || '-') + ' / ' + (v.pose_reason || '-'));
    add('last_reject_reason', v.last_reject_reason);
    add('scan 입력/처리/덮어씀/거부', [v.scans_received, v.scans_processed, v.overwritten_scans, v.rejected_scans].map(fmtInt).join(' / '));
    add('typed processed/invalid/stamp_err', [v.processed, v.invalid, v.stamp_errors].map(fmtInt).join(' / '));
    add('처리시간 p50/p95/max ms', [v.processing_ms_p50, v.processing_ms_p95, v.processing_ms_max].map((x) => fmt(x, 1)).join(' / '));
    add('debug 전송 (admitted/published)', fmtInt(v.debug_admitted) + ' / ' + fmtInt(v.debug_published) + ' · throttle ' + fmtInt(v.debug_skipped_throttle) + ' · 덮어씀 ' + fmtInt(v.debug_overwritten));
    add('debug capture/encode ms (last/max)', fmt(v.debug_capture_ms_last, 2) + '/' + fmt(v.debug_capture_ms_max, 2) + ' · ' + fmt(v.debug_encode_ms_last, 2) + '/' + fmt(v.debug_encode_ms_max, 2));
    add('GPS', (v.map_reason || '-') + ' · last_gps ' + (v.last_gps_reason || '-') + (v.gps_age_ms ? ' · age ' + fmt(v.gps_age_ms, 0) + ' ms' : ''));
    add('IMU/CAN/GPS 거부', [v.imu_rejected, v.can_rejected, v.gps_rejected].map(fmtInt).join(' / '));
    add('camera', (v.camera_fusion || '-') + ' · active ' + (v.camera_active || '-') + ' · ' + (v.camera_reason || '-'));
    add('fusion calls/matched/dropped', [v.fusion_calls, v.fusion_matched, v.fusion_dropped].map(fmtInt).join(' / '));
    for (const [frame, label] of [['camera_front','전방'],['camera_left','좌측'],['camera_right','우측']]) {
      const key = 'fusion_view_'+frame;
      add(label+(v[key+'_matching']==='true' ? ' · matching' : ' · 수신만'), (v[key+'_reason'] || '미수신') + ' · 후보 ' + fmtInt(v[key+'_candidates']) + ' · header Δ ' + fmt(v[key+'_offset_ms'],1) + 'ms');
    }
    add('fusion 진단 최근 scan / 생략', fmtInt(v.fusion_window_scans)+' / '+fmtInt(v.fusion_diagnostics_omitted));
    for (const key of Object.keys(v).filter(k=>k.startsWith('fusion_reason_')).sort()) add(key.slice(14),fmtInt(v[key]));
    add('시각 기준', 'receive header · source/device clock 직접 비교 안 함');
    add('config', (v.config_sha256 ? v.config_sha256.slice(0, 12) : '-') + ' · mount ' + (v.mount_config_sha256 ? v.mount_config_sha256.slice(0, 12) : '-'));
    if (sc && sc.stages) {
      const c = sc.counts || {}, s = sc.sampled || {};
      const sm = (k) => s[k] ? s[k].actual + ' (화면 ' + s[k].sent + ', stride ' + s[k].stride + ')' : '-';
      add('scan seq / stamp', sc.seq + ' / ' + stampText(sc.stamp_ns));
      add('raw 점', sm('raw'));
      add('ROI 점', sm('roi'));
      add('voxel 점', sm('voxel'));
      add('leveled 점', sm('leveled'));
      add('RANSAC 비지면 / 제거 지면', c.ground_kept + ' / ' + (c.leveled - c.ground_kept));
      add('tunnel 제거', (sc.tunnel && sc.tunnel.ran) ? c.tunnel_removed : 'OFF');
      add('군집 입력 / cluster', c.cluster_input + ' / ' + c.clusters);
      add('detections / tracks (obs/pred)', c.detections + ' / ' + c.tracks + ' (' + c.observed + '/' + c.predicted + ')');
      add('roll / pitch (deg)', fmt(sc.local.roll_rad * 57.29578, 2) + ' / ' + fmt(sc.local.pitch_rad * 57.29578, 2) + (sc.config && sc.config.roll_leveling ? ' · roll+pitch 보정' : ' · pitch only'));
      add('yaw (deg) / 속도 (m/s)', fmt(sc.local.yaw_rad * 57.29578, 1) + ' / ' + fmt(sc.local.speed_mps, 2));
      add('지면 평면', sc.plane && sc.plane.valid ? sc.plane.coefficients.map((x) => fmt(x, 3)).join(', ') : '무효');
      add('map pose (이 scan)', sc.map && sc.map.valid ? '유효' : '무효: ' + (sc.map ? sc.map.reason : '-'));
      add('scan 처리 ms', fmt(sc.timings && sc.timings.total_ms, 2) + ' · capture ' + fmt(sc.capture_ms, 2));
      add('누락 단계', sc.missing && sc.missing.length ? sc.missing.join(', ') + ' (waiting/missing)' : '없음');
    } else if (sc) {
      add('scan ' + sc.seq, 'pipeline 미처리: ' + (sc.reason || '-') + ' (pose_ready ' + sc.pose_ready + ')');
    }
    const t = st.store || {};
    add('모니터 수신 scan/raw/roi/voxel/leveled', [t.scan, t.raw, t.roi, t.voxel, t.leveled].map(fmtInt).join(' / ') + ' · partial ' + fmtInt(t.partial));
    add('웹 세션 / 재시작 감지 / 무시 frame / timeout', (state.session || '-') + ' / ' + state.sessionChanges + ' / ' + state.ignoredFrames + ' / ' + state.pollsAborted);
    const hs = st.http || {};
    add('HTTP 연결 (활성/상한 · 거부 · timeout)', fmtInt(hs.active) + '/' + fmtInt(hs.max_clients) + ' · ' + fmtInt(hs.rejected_busy) + ' · ' + fmtInt(hs.timed_out));
    const table = el('status-table');
    if(table.closest('details').open){
    table.innerHTML = '';
    for (const [k, val] of rows) {
      const tr = document.createElement('tr');
      const a = document.createElement('td'); a.textContent = k;
      const b = document.createElement('td'); b.textContent = val;
      tr.appendChild(a); tr.appendChild(b); table.appendChild(tr);
    }
    }
    // Stage counters next to the checkboxes.
    const c = sc && sc.stages ? sc.counts : null, s = sc && sc.stages ? sc.sampled : null;
    const cnt = (id, text) => { el(id).textContent = text; };
    cnt('c-raw', c ? c.raw + ' / ' + s.raw.sent : '-');
    cnt('c-roi', c ? c.roi + ' / ' + s.roi.sent : '-');
    cnt('c-voxel', c ? c.voxel + ' / ' + s.voxel.sent : '-');
    cnt('c-leveled', c ? c.leveled + ' / ' + s.leveled.sent : '-');
    cnt('c-ground', c ? String((c.height_kept===undefined?c.leveled:c.height_kept) - c.ground_kept) : '-');
    cnt('c-height',c?String(c.height_removed||0):'-');
    const roi=sc&&sc.config&&sc.config.roi;
    el('roi-range').textContent=roi?'3D '+roi.range_m.join('–')+'m':'';
    el('roi-xy').textContent=roi?'센서 X '+roi.x_m.join('–')+'m · Y '+roi.y_m.join('–')+'m':'';
    el('roi-sensor-z').textContent=roi?'센서 Z '+roi.sensor_z_m.join('–')+'m':'';
    el('roi-height').textContent=roi?(roi.height_enabled?'평지 지면 위 ≤'+roi.height_above_ground_m+'m · IMU 수평화':'OFF'):'';
    el('criteria-points').textContent=report&&report.minimum_support_points?
      (report.road_support_rule==='count_and_ratio'?
        '도로 안쪽 '+fmt(report.road_inset_m,2)+'m · 유지 ≥'+report.minimum_road_points+'개 AND ≥'+fmt(report.minimum_road_ratio*100,1)+'%':
        '구형 기준: 도로 점 ≥'+report.minimum_road_points+'개 OR ≥'+fmt(report.minimum_road_ratio*100,1)+'%')+
        ' · 전체 ≥'+report.minimum_support_points+'점':'판정 설정 수신 대기';
    el('criteria-height').textContent=report&&report.roof_limit_base_z?
      '하부 점·OBB 하단 >'+fmt(report.roof_limit_base_z,3)+'m (base_link)':'판정 설정 수신 대기';
    el('criteria-obb').textContent=report&&report.minimum_obb_road_ratio!==undefined?
      '도로 안쪽 '+fmt(report.road_inset_m,2)+'m · OBB 면적 ≥'+fmt(report.minimum_obb_road_ratio*100,1)+'%':'구형 비교 · 겹침0% 초과';
    cnt('c-tunnel', c ? (sc.tunnel && sc.tunnel.ran ? String(c.tunnel_removed) : 'OFF') : '-');
    cnt('c-nonground', c ? String(c.ground_kept) : '-');
    cnt('c-clusters', c ? String(c.clusters) : '-');
    cnt('c-detections', c ? String(c.detections) : '-');
    cnt('c-tracks', c ? c.observed + ' obs + ' + c.predicted + ' pred' : '-');
    cnt('c-typed', sc && sc.typed && sc.typed.local && sc.typed.local.accepted ? (STATUS_NAMES[sc.typed.local.status] || sc.typed.local.status) + ' ' + (sc.typed.local.objects || []).length : '-');
    const mapRadio = el('frame-map');
    const mapOk = !!(sc && sc.stages && sc.map && sc.map.valid);
    mapRadio.disabled = !mapOk;
    el('frame-map-note').textContent = mapOk ? '' : '(사용 불가' + (sc && sc.map ? ': ' + sc.map.reason : '') + ')';
    if (!mapOk && state.view.frame === 'map') setFrame('base');
  }

  // ------------------------------------------------------------ canvases
  const bev = el('bev'), side = el('side');
  function fitCanvas(canvas) {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(50, Math.floor(rect.width)), h = Math.max(50, Math.floor(rect.height));
    if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
      canvas.width = Math.floor(w * dpr); canvas.height = Math.floor(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
  }
  function vehicleOrigin() {  // base_link origin in the view frame
    const m = state.matrices;
    return m && m.vehicle ? apply(m.vehicle, 0, 0, 0) : [0, 0, 0];
  }
  function bevProject(w, h, x, y) { return V.bevProject(state.view, vehicleOrigin(), w, h, x, y); }
  function projectMain(w, h, p) {
    return state.view.mode === 'orbit' ? V.orbitProject(state.view, vehicleOrigin(), w, h, p) : bevProject(w, h, p[0], p[1]);
  }
  function bevUnproject(w, h, px, py) { return V.bevUnproject(state.view, vehicleOrigin(), w, h, px, py); }
  function sideProject(w, h, p) { return V.sideProject(state.view, vehicleOrigin(), w, h, p); }

  function drawGrid(ctx, w, h) {
    if (state.view.mode === 'orbit' || state.view.bevYaw !== 0) {
      const o = vehicleOrigin(); ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1; ctx.setLineDash([]);
      for (let k = -100; k <= 100; k += 10) for (const axis of [0, 1]) {
        const a = [o[0], o[1], o[2]], b = a.slice();
        a[axis] -= 100; b[axis] += 100; a[1-axis] += k; b[1-axis] += k;
        const p = projectMain(w,h,a), q = projectMain(w,h,b);
        ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();
      }
      return;
    }
    const s = state.view.scale;
    const stepM = s > 30 ? 1 : s > 12 ? 5 : s > 4 ? 10 : 20;
    ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1; ctx.fillStyle = COLORS.axis; ctx.font = '11px sans-serif';
    const [cx, cy] = bevProject(w, h, vehicleOrigin()[0], vehicleOrigin()[1]);
    for (let r = stepM; r * s < Math.max(w, h) * 1.5; r += stepM) {
      ctx.beginPath(); ctx.arc(cx, cy, r * s, 0, Math.PI * 2); ctx.stroke();
      ctx.fillText(r + ' m', cx + 3, cy - r * s - 2);
    }
    ctx.strokeStyle = COLORS.axis;
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();
    ctx.fillStyle = COLORS.text;
    ctx.fillText('x +', cx + 4, 12); ctx.fillText('y +', 4, cy - 4);
  }
  function drawVehicle(ctx, w, h, alpha) {
    const sc = state.scene; const veh = (sc && sc.vehicle) || state.lastVehicle;
    // Without stage matrices (waiting, INPUT_INVALID) the base_link view still shows the vehicle at its origin.
    const m = state.matrices || (state.view.frame === 'base' ? { vehicle: I4 } : null);
    if (!veh || !m) return;
    state.lastVehicle = veh;
    const x0 = -veh.rear_overhang, x1 = veh.wheelbase + veh.front_overhang, y0 = -veh.width / 2, y1 = veh.width / 2;
    const pts = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y]) => projectMain(w, h, apply(m.vehicle, x, y, 0)));
    ctx.globalAlpha = alpha; ctx.strokeStyle = COLORS.vehicle; ctx.lineWidth = 1.5; ctx.setLineDash([]);
    ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.stroke();
    if(state.view.mode==='orbit') {
      // Stylized display-only body; height is not a calibrated vehicle dimension.
      const body=[[x0,y0,-.36],[x1,y0,-.36],[x1,y1,-.36],[x0,y1,-.36],
        [x0,y0,1.284],[x1,y0,1.284],[x1,y1,1.284],[x0,y1,1.284]];
      V.drawBox(ctx,'orbit',body,m.vehicle,p=>projectMain(w,h,p),{color:'#93c5fd',width:2});
      const roof=[[x0+0.6,y0*0.8,1.284],[x1-1,y0*0.8,1.284],[x1-1,y1*0.8,1.284],[x0+0.6,y1*0.8,1.284]]
        .map(p=>projectMain(w,h,apply(m.vehicle,...p)));
      ctx.fillStyle='rgba(147,197,253,0.22)';ctx.beginPath();roof.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.fill();
    }
    const o = apply(m.vehicle, 0, 0, 0); const po = projectMain(w, h, o);
    ctx.fillStyle = COLORS.vehicle; ctx.beginPath(); ctx.arc(po[0], po[1], 3, 0, Math.PI * 2); ctx.fill();
    const l = apply(m.vehicle, veh.lidar[0], veh.lidar[1], veh.lidar[2]); const pl = projectMain(w, h, l);
    ctx.strokeStyle = COLORS.accent || '#38bdf8'; ctx.beginPath(); ctx.arc(pl[0], pl[1], 4, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText('base_link', po[0] + 6, po[1] + 12); ctx.fillText('LiDAR', pl[0] + 6, pl[1] - 4);
    ctx.globalAlpha = 1;
  }

  function stagePoints(name) {
    const a = state.arrays[name + '.xyzi'];
    return a && a.length ? a : null;
  }
  function drawPoints(ctx, w, h, xyzi, matrix, colorFn, project, alpha) {
    if (!xyzi || !matrix) return;
    const ps = state.view.psize;
    ctx.globalAlpha = alpha;
    let last = null;
    for (let i = 0, n = xyzi.length / 4; i < n; i++) {
      const c = colorFn(i);
      if (!c) continue;
      const p = apply(matrix, xyzi[4 * i], xyzi[4 * i + 1], xyzi[4 * i + 2]);
      const q = project(w, h, p);
      if (q[0] < -2 || q[1] < -2 || q[0] > w + 2 || q[1] > h + 2) continue;
      if (c !== last) { ctx.fillStyle = c; last = c; }
      ctx.fillRect(q[0] - ps / 2, q[1] - ps / 2, ps, ps);
    }
    ctx.globalAlpha = 1;
  }
  function leveledColor(label, cluster, track) {
    const st = state.stages;
    const styles=new Map((state.scene.tracks||[]).map(t=>[t.id,{color:V.auditAppearance(state.scene,t).color,visible:visibleTrack(t)}]));
    return (i) => {
      const l = label ? label[i] : 0;
      if(!V.pointStageVisible(l,st))return null;
      const style=track&&styles.get(track[i]);
      if(style&&!style.visible)return null;
      if(state.pointColorMode==='road')return V.pointRoadColor((state.arrays['leveled.road']||[])[i]);
      if(l===3)return '#fb923c';
      if(l===2)return COLORS.tunnel;
      if(l===1)return COLORS.ground;
      if(st.clusters&&cluster){const c=cluster[i];return c>=0?CLUSTER_PALETTE[c%CLUSTER_PALETTE.length]:'#64748b';}
      return style?style.color:'#64748b';
    };
  }
  function rawColor(labels) {
    const reasons={1:['range_removed','#9ca3af'],2:['xy_removed','#a78bfa'],3:['sensor_z_removed','#60a5fa'],4:['ego_removed','#f87171'],5:['range_removed','#9ca3af']};
    return i=>{const r=labels?reasons[labels[i]]:null;return r?(state.stages.raw||state.stages[r[0]]?r[1]:null):state.stages.raw?COLORS.raw:null;};
  }
  function classColor(cls) { return CLASS_COLORS[cls] || CLASS_COLORS[String(cls).toLowerCase()] || '#7dd3fc'; }
  const ROAD_COLORS={inside:'#ff4545',outside:'#ffffff',unknown:'#facc15'};
  const ROAD_NAMES={inside:'영역 안',outside:'영역 밖',unknown:'판정 불가'};
  function membership(t){return V.roadMembership(state.scene,state.road,t.obb.center);}
  function trackVisibility(t){
    return V.displayVisibility(state.scene,state.road,t,state.showPred,state.hideCandidates,state.hideReasons);
  }
  function visibleTrack(t){return trackVisibility(t).visible;}

  // The view kind is explicit ('bev' rotated footprint, 'side' screen rectangle); see monitor_view.js.
  function drawBox(ctx, view, corners, matrix, style) {
    return V.drawBox(ctx, view.kind, corners, matrix, (p) => view.project(view.w, view.h, p), style);
  }

  function drawRoad(ctx, w, h) {
    const road = state.road, sc = state.scene;
    const map = sc && sc.stages && sc.map && sc.map.valid && sc.map.map_from_base;
    const note = el('road-note');
    if (!road || !road.available) { note.textContent = '지도 미표시: ' + (road ? road.reason : '불러오는 중'); return; }
    if (!map || !state.matrices || !state.matrices.map) { note.textContent = '지도 숨김 · 유효한 GPS pose 필요'; return; }
    const version=(road.build_id||'').match(/_v(\d+)$/);
    note.textContent = 'HD맵 '+(version?'v'+version[1]:'')+' · '+(road.lanes_available?'실제 차선':'차선 미표시: '+(road.lane_reason||'없음'))+' · 지면 높이는 참고';
    note.title = road.build_id || '';
    if (!state.view.road) return;
    const o = apply(map,0,0,0), transform = state.matrices.map;
    ctx.fillStyle = 'rgba(190,150,255,0.16)';ctx.setLineDash([]);ctx.beginPath();
    for (const {polygon,bounds:b} of (road.index||[])) {
      if(b[2]<o[0]-160||b[0]>o[0]+160||b[3]<o[1]-160||b[1]>o[1]+160)continue;
      polygon.forEach((p,i)=>{const q=projectMain(w,h,apply(transform,p[0],p[1],o[2]));if(i)ctx.lineTo(...q);else ctx.moveTo(...q);});
      ctx.closePath();
    }
    ctx.fill('nonzero'); // No per-link borders, centers or lane divider guides.
    for(const lane of road.lane_markings||[]) {
      ctx.strokeStyle=lane.color==='yellow'?'#ffdc55':lane.color==='blue'?'#62a8ff':'#f5f5f5';
      ctx.lineWidth=Math.max(1,Math.min(4,lane.width_m*state.view.scale));ctx.beginPath();
      for(const segment of lane.segments){
        if(segment.every(p=>Math.abs(p[0]-o[0])>160||Math.abs(p[1]-o[1])>160))continue;
        const a=projectMain(w,h,apply(transform,...segment[0],o[2]));
        const b=projectMain(w,h,apply(transform,...segment[1],o[2]));
        ctx.moveTo(...a);ctx.lineTo(...b);
      }
      ctx.stroke();
    }
  }

  let drawPending=false;
  function renderScene(){
    if(drawPending)return;
    drawPending=true;
    requestAnimationFrame(()=>{drawPending=false;const t=performance.now();renderSceneNow();state.lastRenderMs=performance.now()-t;});
  }
  function renderSceneNow() {
    const sc = state.scene, m = state.matrices;
    const fr = freshness(), stale = fr.stale;
    const alphaPts = stale ? 0.45 : 1, alphaBox = stale ? 0.3 : 1;
    const B = fitCanvas(bev), S = fitCanvas(side);
    for (const { ctx, w, h } of [B, S]) { ctx.fillStyle = '#0b0f14'; ctx.fillRect(0, 0, w, h); }
    drawGrid(B.ctx, B.w, B.h);
    drawRoad(B.ctx, B.w, B.h);
    // Side view ground line and axes.
    S.ctx.strokeStyle = COLORS.axis; S.ctx.beginPath(); const gz = sideProject(S.w, S.h, vehicleOrigin());
    S.ctx.moveTo(0, gz[1]); S.ctx.lineTo(S.w, gz[1]); S.ctx.moveTo(gz[0], 0); S.ctx.lineTo(gz[0], S.h); S.ctx.stroke();
    S.ctx.fillStyle = COLORS.text; S.ctx.font = '11px sans-serif';
    S.ctx.fillText((state.view.side === 'xz' ? 'x +' : '−y (우) →'), S.w - 60, gz[1] - 4); S.ctx.fillText('z +', gz[0] + 4, 12);
    const stepM = state.view.scale > 12 ? 1 : 5;
    for (let k = -4; k <= 4; k++) { if (!k) continue; const q = sideProject(S.w, S.h, [vehicleOrigin()[0], vehicleOrigin()[1], vehicleOrigin()[2] + k * stepM]); S.ctx.fillStyle = COLORS.axis; S.ctx.fillText((k * stepM) + ' m', gz[0] + 4, q[1] - 2); S.ctx.strokeStyle = COLORS.grid; S.ctx.beginPath(); S.ctx.moveTo(0, q[1]); S.ctx.lineTo(S.w, q[1]); S.ctx.stroke(); }
    const frameName = { base: 'base_link (local, 후륜축)', leveled: 'lidar_leveled (수평화)', sensor: 'sensor (velodyne, 보정 전)', map: 'map' }[state.view.frame];
    el('bev-label').textContent = (state.view.mode === 'orbit' ? '3D · 드래그 회전 · ' : 'BEV · ') + (state.view.frame === 'map' ? 'Map · 지도 축' : 'Local · 차량 기준') + (fr.state === 'paused' ? ' · PAUSED' : stale && sc ? ' · STALE' : '');
    el('side-label').textContent = '측면 ' + (state.view.side === 'xz' ? 'X–Z (전후·높이)' : 'Y–Z (좌우·높이)') + ' · frame: ' + frameName + ' · 같은 scan';
    if (!sc || !sc.stages) { drawVehicle(B.ctx, B.w, B.h, 0.6); if (sc && !sc.stages) { B.ctx.fillStyle = COLORS.text; B.ctx.font = '14px sans-serif'; B.ctx.fillText('scan ' + sc.seq + ' 미처리: ' + (sc.reason || '') + ' — 점군/객체 없음', 12, B.h - 14); } return; }
    if (!m) { B.ctx.fillStyle = COLORS.text; B.ctx.font = '14px sans-serif'; B.ctx.fillText('선택한 frame 변환이 이 scan에서 무효합니다 (' + state.view.frame + ')', 12, B.h - 14); return; }
    const st = state.stages;
    const label = state.arrays['leveled.label'], cluster = state.arrays['leveled.cluster'];
    const views = [{ kind: state.view.mode, ctx: B.ctx, w: B.w, h: B.h, project: projectMain },
      ...(!el('view-side').hidden ? [{ kind: 'side', ctx: S.ctx, w: S.w, h: S.h, project: sideProject }] : [])];
    for (const view of views) {
      const { ctx, w, h, project } = view;
      if (st.raw||st.range_removed||st.xy_removed||st.sensor_z_removed||st.ego_removed)
        drawPoints(ctx,w,h,stagePoints('raw'),m.sensor,rawColor(state.arrays['raw.label']),project,alphaPts);
      if (st.roi) drawPoints(ctx, w, h, stagePoints('roi'), m.sensor, () => COLORS.roi, project, alphaPts);
      if (st.voxel) drawPoints(ctx, w, h, stagePoints('voxel'), m.sensor, () => COLORS.voxel, project, alphaPts);
      if (state.view.compare) drawPoints(ctx, w, h, stagePoints('voxel'), m.leveled, () => COLORS.compare, project, alphaPts * 0.9);
      if (st.leveled) drawPoints(ctx, w, h, stagePoints('leveled'), m.leveled, () => COLORS.leveled, project, alphaPts);
      if (st.ground || st.tunnel || st.nonground || st.height_removed) drawPoints(ctx, w, h, stagePoints('leveled'), m.leveled, leveledColor(label, cluster,state.arrays['leveled.track']), project, alphaPts);
      ctx.globalAlpha = alphaBox;
      if (st.detections) for (const d of sc.detections || []) drawBox(ctx, view, d.corners, m.leveled, { color: COLORS.detections, dash: [4, 3], width: 1 });
      if (st.typed && sc.typed && sc.typed.local && sc.typed.local.accepted) for (const o of sc.typed.local.objects || []) {
        if(!visibleTrack({id:o.track_id,miss:0}))continue;
        drawBox(ctx, view, o.corners, m.base, { color: COLORS.typed, width: 1, dash: [2, 2] });
      }
      if (st.tracks) for (const t of sc.tracks || []) {
        if(!visibleTrack(t))continue;
        const observed = t.miss === 0;
        const sel = state.selected === t.id;
        const motion=V.motionAppearance(t.motion),audit=V.auditAppearance(sc,t),color=state.colorMode==='audit'?audit.color:state.colorMode==='class'?classColor(t.class):motion.color;
        const outline = drawBox(ctx, view, t.corners, m.leveled, { color, width: sel ? 3 : observed ? 2 : 1, dash: observed ? [] : [5, 4] });
        if (outline && view.kind !== 'side' && state.view.labels) {
          const c = apply(m.leveled, t.obb.center[0], t.obb.center[1], t.obb.center[2]); const q = projectMain(w, h, c);
          ctx.fillStyle = color; ctx.font = (sel ? 'bold ' : '') + '12px sans-serif';
          ctx.fillText('#' + t.id + ' ' + t.raw_class + ' · ' + (state.colorMode==='audit'?audit.label:motion.label) + (observed ? '' : ' (pred, miss ' + t.miss + ')'), q[0] + 6, q[1] - 6);
        }
        const warning=state.idWarnings.get(t.id);
        if(view.kind!=='side' && warning && observed && membership(t)==='inside'){
          const q=projectMain(w,h,apply(m.leveled,...t.obb.center));
          ctx.fillStyle='#facc15';ctx.font='bold 15px sans-serif';
          ctx.fillText('! ID 변경 의심 '+warning.from+' → '+t.id,q[0]+6,q[1]-24);
        }
      }
      ctx.globalAlpha = 1;
    }
    drawVehicle(B.ctx, B.w, B.h, 1);
    const sv = sc.vehicle; if (sv) { // vehicle extent in side view
      const o = vehicleOrigin(); const a = sideProject(S.w, S.h, [o[0] - (state.view.side === 'xz' ? sv.rear_overhang : sv.width / 2), o[1] + (state.view.side === 'xz' ? 0 : sv.width / 2), o[2]]);
      const len = (state.view.side === 'xz' ? sv.wheelbase + sv.front_overhang + sv.rear_overhang : sv.width) * state.view.scale;
      S.ctx.strokeStyle = COLORS.vehicle; S.ctx.strokeRect(a[0], a[1] - 1.5 * state.view.scale, len, 1.5 * state.view.scale);
    }
  }

  // ------------------------------------------------------------ track list / detail
  function renderTracks() {
    const sc = state.scene; const tbody = el('track-table').querySelector('tbody');
    tbody.innerHTML = '';
    if (!sc || !sc.stages) return;
    for (const t of sc.tracks || []) {
      if(!state.showPred && t.miss>0)continue;
      const tr = document.createElement('tr');
      tr.className = (t.miss ? 'predicted' : '') + (state.selected === t.id ? ' selected' : '');
      const area=membership(t);
      const motion=V.motionAppearance(t.motion);
      const warning=state.idWarnings.get(t.id);
      const d=V.auditForTrack(sc,t),audit=V.auditAppearance(sc,t),p=d&&d.preview;
      const ratios=p?fmt((p.interior_road_ratio===undefined?p.road_point_ratio:p.interior_road_ratio)*100,0)+' / '+
        (p.obb_interior_road_ratio!==undefined?(Number.isFinite(p.obb_interior_road_ratio)?fmt(p.obb_interior_road_ratio*100,0):'—'):d.spatial&&d.spatial.available?fmt(d.spatial.road_ratio_grid*100,0):'—'):'—';
      const visibility=trackVisibility(t);
      const values=[String(t.id)+(warning && area==='inside'?' !':''),t.raw_class,audit.label,ratios,p?fmt(p.min_support_base_z,2):'—',motion.label];
      for (const v of values) {const td=document.createElement('td');td.textContent=v;tr.appendChild(td);}
      tr.title=(visibility.visible?'화면 표시':'화면 숨김')+' · '+audit.detail+' · group '+t.class+' · '+audit.reason;
      if(!visibility.visible)tr.classList.add('preview-hidden');
      if(warning && area==='inside')tr.title+=' · ID 변경 의심 '+warning.from+' → '+t.id+' (위치·크기 기반, 확정 아님)';
      tr.style.color=state.colorMode==="audit"?audit.color:state.colorMode==="class"?classColor(t.class):motion.color;
      tr.addEventListener('click', () => selectTrack(t.id));
      tbody.appendChild(tr);
    }
  }
  function selectTrack(id) { state.selected = (state.selected === id) ? null : id; renderTracks(); renderDetail(); renderScene(); }
  function renderDetail() {
    const box = el('detail'); const sc = state.scene; const m = state.matrices;
    if (state.selected === null || !sc || !sc.stages) { box.textContent = '객체를 클릭하면 상세값을 표시합니다.'; return; }
    const t = (sc.tracks || []).find((x) => x.id === state.selected);
    if (!t) { box.textContent = 'track #' + state.selected + ' 은(는) 현재 scan에 없습니다.'; return; }
    const typed = sc.typed && sc.typed.local && sc.typed.local.accepted ? (sc.typed.local.objects || []).find((o) => o.track_id === t.id) : null;
    const viewC = m && m.leveled ? apply(m.leveled, t.obb.center[0], t.obb.center[1], t.obb.center[2]) : null;
    let yawView = t.obb.yaw;
    if (m && m.leveled) { const d = [Math.cos(t.obb.yaw), Math.sin(t.obb.yaw), 0]; const r = m.leveled; yawView = Math.atan2(r[4] * d[0] + r[5] * d[1], r[0] * d[0] + r[1] * d[1]); }
    const audit=V.auditForTrack(sc,t),a=V.auditAppearance(sc,t),p=audit&&audit.preview;
    const fd = sc.fusion && sc.fusion.decisions || [];
    let match = null, assoc = null;
    for (const d of fd) for (const a of d.associations || []) if (a.track_id === t.id) { match=d; assoc=a; }
    const rows = [
      ['이번 snapshot matching',match ? 'scan '+match.scan_sequence+' · '+match.action+' / '+match.reason : '이번 snapshot에 연결 기록 없음 · 이전 분류/보호와 구분'],
      ['영상 관측 / bbox',match ? match.observation_id+' / '+assoc.source_box_index : '-'],
      ['matching Δ / 대기',match ? fmt(match.image_minus_scan_ns/1e6,1)+' / '+fmt(match.wait_ns/1e6,1)+'ms' : '-'],
      ['실제 IoU / 분류 반영',assoc ? fmt(assoc.iou,3)+' · '+assoc.reason+' · '+assoc.class_before+' → '+assoc.class_after : '-'],
      ['화면 표시',visibleTrack(t)?'표시 대상 · 레이어 선택에 따름':'숨김 유지 · 상세 선택은 숨김을 해제하지 않음'],
      ['필터 미리보기',a.label+' · '+a.detail],
      ['실제 ROS 전달',t.miss>0?(audit&&audit.historical?'마지막 관측 판정 · '+audit.observed_stamp_ns:'예측 · 현재 전달 대상 아님'):audit&&audit.delivery_excluded?'제외 · 원본 tracker는 유지':sc.hd_map_audit&&sc.hd_map_audit.delivery_applied?'보존':'전달 필터 OFF'],
      ['독립 근거 · 점군 / OBB / 높이',p?[p.point_decision,p.obb_decision,p.height_decision].map(x=>({keep:'유지',exclude_candidate:sc.hd_map_audit.delivery_applied?'제외':'비교 후보',undetermined:'보류'}[x]||'이전 버전')).join(' / '):'미계산'],
      ['전달 판단 / 보류 근거',audit?audit.decision+' / '+audit.reason:'OFF'],
      ['군집 도로 점 / 전체',p?p.road_points+' / '+p.support_points+' ('+fmt(p.road_point_ratio*100,1)+'%)':'미계산'],
      ['안쪽 도로 점 / 전체',p&&p.interior_road_points!==undefined?p.interior_road_points+' / '+p.support_points+' ('+fmt(p.interior_road_ratio*100,1)+'%)':'이전 생산자 · 미계산'],
      ['OBB 원본 도로 겹침률',audit&&audit.spatial&&audit.spatial.available?fmt(audit.spatial.road_ratio_grid*100,1)+'%':'미계산'],
      ['OBB 안쪽 도로 겹침률',p&&Number.isFinite(p.obb_interior_road_ratio)?fmt(p.obb_interior_road_ratio*100,1)+'%':'근거 없음 · 미계산'],
      ['종합 표시 규칙',sc&&sc.hd_map_audit&&sc.hd_map_audit.display_support_rule==='points_and_obb'?'점군 AND OBB · 유효 YOLO 보호 예외':'구형 비교 생산자'],
      ['경계 보호 점 / 지도 불명 점',p?p.guard_points+' / '+p.unknown_points:'미계산'],
      ['하부 점 / OBB 하단 / 지붕+여유',p?[p.min_support_base_z,p.obb_bottom_base_z,p.roof_limit_base_z].map(x=>fmt(x,2)).join(' / ')+'m (base_link)':'미계산'],
      ['카메라 동적 그룹 보호',p?(p.camera_dynamic_protected?'YOLO 양성 매칭 · 세대/시간 유효 · 전 구간 보호':'해당 없음 · LiDAR 운동 판정은 사용하지 않음'):'미계산'],
      ['카메라 차량 그룹 보호',p?(p.camera_vehicle_protected?'YOLO car/bus/truck 양성 매칭 · 세대/시간 유효 · 전 구간 보호':'해당 없음'):'미계산'],
      ['보호 정책',p&&p.protection_policy||'구형 생산자 · 최신 바이너리로 재시작 필요'],
      ['최신 YOLO 매칭 원본 / 그룹',p?[p.camera_raw_class||'미기록',p.camera_group||'미기록'].join(' / '):'미계산'],
      ['미션 예외 공간 (비교용)',audit?(audit.exception_types||[]).join(', ')+' · YOLO 분류 근거 아님':'미계산'],
      ['track ID / age / miss' , t.id + ' / ' + t.age + ' / ' + t.miss + (t.miss ? ' (예측, 관측 없음)' : ' (관측)')],
      ['classification / camera_class', t.class + ' / ' + t.raw_class],
      ['OBB 중심 위치 (참고만)', ROAD_NAMES[membership(t)] + ' · 토픽 발행 필터 아님'],
      ['적용 운동 상태', V.motionAppearance(t.motion).label],
      ['ID 변경 의심', state.idWarnings.has(t.id) ? state.idWarnings.get(t.id).from+' → '+t.id+' · 동일 물체 확정 아님' : '없음'],
      ['원시 map 속도 km/h',t.motion?[t.motion.raw_map_vx_kph,t.motion.raw_map_vy_kph].map(x=>fmt(x,2)).join(' / '):'미수신'],
      ['판제 적용 속도 km/h',t.motion?[t.motion.planner_vx_kph,t.motion.planner_vy_kph].map(x=>fmt(x,2)).join(' / '):'미수신'],
      ['정적 semantic 보조',t.motion?(t.motion.semantic_applied?'적용 · ':'미적용 · ')+(t.motion.semantic_reason||'이전 생산자'):'미수신'],
      ['판정 vx / vy / 실제 이동 km/h', t.motion ? [t.motion.vx_kph,t.motion.vy_kph,t.motion.displacement_kph].map(x=>fmt(x,2)).join(' / ') : '미수신'],
      ['frame · stamp', state.view.frame + ' · ' + stampText(sc.stamp_ns)],
      ['OBB center (표시 frame) m', viewC ? viewC.map((x) => fmt(x, 2)).join(', ') : '-'],
      ['OBB center (lidar_leveled) m', t.obb.center.map((x) => fmt(x, 2)).join(', ')],
      ['length / width / height m', fmt(t.obb.width) + ' / ' + fmt(t.obb.height) + ' / ' + fmt(t.obb.zmax - t.obb.zmin)],
      ['yaw (표시 frame / leveled) deg', fmt(yawView * 57.29578, 1) + ' / ' + fmt(t.obb.yaw * 57.29578, 1)],
      ['source point count', t.points],
      ['distance (leveled) m', fmt(t.dist, 2)],
      ['공용 적용 속도 (base_link) m/s', typed && typed.absolute_velocity_valid && Array.isArray(typed.absolute_velocity) ? typed.absolute_velocity.map(x=>fmt(x,2)).join(' / ')+' · 기존 판제 추정값 (UNKNOWN은 정지 확인 아님)' : typed && Object.prototype.hasOwnProperty.call(typed,'absolute_velocity_valid') ? '무효 · pose/운동 근거 확인' : '이 scan에 미제공 · 구 생산자는 validity 미기록'],
      ['공용 상대속도', typed && typed.relative_velocity_valid ? '유효' : '미제공 (validity=false)'],
      ['공용 실제 운동 판정', typed && typed.dynamic_valid ? (typed.is_dynamic?'동적 (PENDING/DYNAMIC)':'LOW_THREAT') : 'UNKNOWN / 미제공 · 정지 확정 아님'],
      ['typed local 배열', typed ? 'base_link center ' + typed.center.map((x) => fmt(x, 2)).join(', ') + ' · ' + typed.classification + '/' + typed.camera_class + ' · ' + typed.points + ' pts' : (t.miss ? '미발행 (예측 track)' : '이 scan 배열에 없음')],
    ];
    box.innerHTML = '';
    const h3 = document.createElement('h3'); h3.textContent = 'track #' + t.id + ' ' + t.class; box.appendChild(h3);
    const table = document.createElement('table');
    for (const [k, v] of rows) { const tr = document.createElement('tr'); const a = document.createElement('td'); a.textContent = k; const b = document.createElement('td'); b.textContent = v; tr.appendChild(a); tr.appendChild(b); table.appendChild(tr); }
    box.appendChild(table);
    const diag = document.createElement('div'); diag.className = 'diag';
    diag.textContent = '진단용 미검증 값 (KF 내부, 좌표계 미검증): abs_v = [' + fmt(t.abs_v[0]) + ', ' + fmt(t.abs_v[1]) + '] · velocity = ' + fmt(t.velocity) + ' · kf_initialized ' + t.kf_initialized;
    box.appendChild(diag);
  }

  // ------------------------------------------------------------ controls
  function setFrame(frame) {
    state.view.frame = frame;
    document.querySelectorAll('input[name=frame]').forEach((r) => { r.checked = r.value === frame; });
    state.matrices = frameMatrices(state.scene, frame);
    state.view.panX = 0; state.view.panY = 0;
    renderScene(); renderDetail();
  }
  function readStages() {
    document.querySelectorAll('input[data-stage]').forEach((c) => { state.stages[c.dataset.stage] = c.checked; });
  }
  document.querySelectorAll('input[name=frame]').forEach((r) => r.addEventListener('change', () => setFrame(r.value)));
  document.querySelectorAll('input[data-stage]').forEach((c) => c.addEventListener('change', () => { readStages(); renderScene(); }));
  el('opt-compare').addEventListener('change', (e) => { state.view.compare = e.target.checked; renderScene(); });
  el('opt-labels').addEventListener('change', (e) => { state.view.labels = e.target.checked; renderScene(); });
  el('opt-side').addEventListener('change', (e) => { state.view.side = e.target.value; renderScene(); });
  el('opt-psize').addEventListener('change', (e) => { state.view.psize = Number(e.target.value); renderScene(); });
  el('opt-view').addEventListener('change', e => {state.view.mode=e.target.value;renderScene();});
  el('opt-road').addEventListener('change', e => {state.view.road=e.target.checked;renderScene();});
  el('opt-hide-candidates').addEventListener('change',e=>{
    e.target.indeterminate=false;
    state.hideCandidates=e.target.checked;
    for(const k of ['points','obb','height','policy']){state.hideReasons[k]=e.target.checked;el('opt-hide-'+k).checked=e.target.checked;}
    renderScene();renderTracks();renderDetail();});
  for(const k of ['points','obb','height','policy'])el('opt-hide-'+k).addEventListener('change',e=>{
    state.hideCandidates=false;el('opt-hide-candidates').checked=false;state.hideReasons[k]=e.target.checked;
    el('opt-hide-candidates').indeterminate=Object.values(state.hideReasons).some(Boolean);
    renderScene();renderTracks();renderDetail();});
  el('opt-point-color').addEventListener('change',e=>{state.pointColorMode=e.target.value;renderScene();});
  el('opt-color').addEventListener('change',e=>{state.colorMode=e.target.value;renderScene();renderTracks();});
  el('opt-side-show').addEventListener('change',e=>{el('view-side').hidden=!e.target.checked;el('views').style.gridTemplateRows=e.target.checked?'2fr 1fr':'1fr';renderScene();});
  el('opt-show-pred').addEventListener('change',e=>{state.showPred=e.target.checked;renderScene();renderTracks();renderDetail();});
  el('btn-center').addEventListener('click', () => { state.view.panX = 0; state.view.panY = 0; state.view.bevYaw=0;state.view.yaw=-0.4;state.view.pitch=0.65;renderScene(); });
  el('btn-zoom-in').addEventListener('click', () => zoomBy(1.25));
  el('btn-zoom-out').addEventListener('click', () => zoomBy(0.8));
  function setPaused(on) { state.paused = !!on; showBanner(); renderScene(); }
  el('btn-pause').addEventListener('click', () => setPaused(!state.paused));
  function zoomBy(f, px, py) {
    const v = state.view; const old = v.scale; v.scale = Math.min(200, Math.max(1, v.scale * f));
    if (px !== undefined) { const k = v.scale / old; v.panX = px - (px - v.panX) * k; v.panY = py - (py - v.panY) * k; }
    renderScene();
  }
  let drag = null;
  for (const canvas of [bev, side]) {
    canvas.addEventListener('mousedown', (e) => {
      if(e.button!==0 && e.button!==1)return;
      e.preventDefault();
      drag = { x:e.clientX,y:e.clientY,px:state.view.panX,py:state.view.panY,
        yaw:state.view.yaw,bevYaw:state.view.bevYaw,pitch:state.view.pitch,
        rotate:canvas===bev && e.button===0 && !e.shiftKey,button:e.button,moved:false };
    });
    canvas.addEventListener('mousemove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      if(drag.rotate && state.view.mode==='orbit'){state.view.yaw=drag.yaw+dx*0.008;state.view.pitch=Math.max(0.08,Math.min(Math.PI/2,drag.pitch+dy*0.008));}
      else if(drag.rotate){state.view.bevYaw=drag.bevYaw+dx*0.008;}
      else {state.view.panX = drag.px + dx; state.view.panY = drag.py + dy;} renderScene();
    });
    canvas.addEventListener('mouseup', (e) => {
      if (drag && drag.button===0 && !drag.moved && canvas === bev) pick(e);
      drag = null;
    });
    canvas.addEventListener('mouseleave', () => { drag = null; });
    canvas.addEventListener('auxclick',e=>{if(e.button===1)e.preventDefault();});
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const cx = e.clientX - rect.left - rect.width / 2, cy = e.clientY - rect.top - rect.height / 2;
      zoomBy(e.deltaY < 0 ? 1.15 : 0.87, cx, cy);
    }, { passive: false });
  }
  function pick(e) {
    const sc = state.scene, m = state.matrices;
    if (!sc || !sc.stages || !m) return;
    const rect = bev.getBoundingClientRect();
    const px = e.clientX - rect.left, py = e.clientY - rect.top;
    let best = null, bestD = 25;
    for (const t of sc.tracks || []) {
      if(!visibleTrack(t))continue;
      const c = apply(m.leveled, t.obb.center[0], t.obb.center[1], t.obb.center[2]);
      const q = projectMain(rect.width, rect.height, c);
      const d = Math.hypot(q[0] - px, q[1] - py);
      if (d < bestD) { bestD = d; best = t.id; }
    }
    if (best !== null) selectTrack(best);
  }
  window.addEventListener('resize', renderScene);

  // Read-only automation hooks (display state only; used by the browser checks).
  window.__asmcMonitor = {
    state, setFrame, selectTrack, zoomBy,
    setStage: (name, on) => { const c = document.querySelector('input[data-stage="' + name + '"]'); if (c) { c.checked = on; readStages(); renderScene(); } },
    pause: setPaused, freshness,
    render: () => renderScene(),
    renderNow: () => renderSceneNow(),
    pan: (dx, dy) => { state.view.panX += dx; state.view.panY += dy; renderScene(); },
    bevUnproject: (px, py) => bevUnproject(bev.getBoundingClientRect().width, bev.getBoundingClientRect().height, px, py),
  };

  readStages();
  el('opt-hide-candidates').indeterminate=true;
  fetch('/api/drivable',{cache:'no-store',signal:AbortSignal.timeout(5000)})
    .then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.json();})
    .then(data=>{data.index=(data.polygons||[]).map(polygon=>({polygon,bounds:polygon.reduce((b,p)=>[Math.min(b[0],p[0]),Math.min(b[1],p[1]),Math.max(b[2],p[0]),Math.max(b[3],p[1])],[Infinity,Infinity,-Infinity,-Infinity])}));state.road=data;renderScene();renderTracks();renderDetail();})
    .catch(e=>{state.road={available:false,reason:String(e)};renderScene();});
  renderScene();
  renderStatus();
  setInterval(poll, POLL_MS);
  setInterval(renderStatus, 250);
  poll();
})();
