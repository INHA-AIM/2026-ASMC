(() => {
  const canvas = document.getElementById('map');
  const ctx = canvas.getContext('2d');
  const $ = id => document.getElementById(id);
  const maneuverNames = ['KEEP', 'FOLLOW', 'MERGE', 'LANE CHANGE', 'AVOID', 'STOP', 'EMERGENCY STOP'];
  const decisionNames = ['KEEP', 'FOLLOW', 'LEFT CHANGE', 'RIGHT CHANGE', 'STOP', 'YIELD', 'EMERGENCY STOP', 'AVOID'];
  const executionNames = ['IDLE', 'PREPARING', 'WAITING', 'READY', 'COMMITTED', 'EXECUTING', 'COMPLETED', 'FAILED'];
  const resultNames = ['NONE', 'NOT INITIALIZED', 'INVALID COMMAND', 'STALE COMMAND', 'TARGET PROJECTION FAILED', 'TARGET OBJECT LOST', 'GENERATION FAILURE', 'CURVATURE BLOCKED', 'COLLISION BLOCKED', 'TOO LATE TO CANCEL'];
  const requestNames = ['PREPARE', 'EXECUTE', 'CANCEL'];
  const trajectorySourceNames = ['NONE', 'NEW', 'REUSED', 'LOCKED', 'SAFE STOP'];
  const lightNames = ['UNKNOWN', 'RED', 'YELLOW', 'GREEN'];
  const lightColors = ['#87939b', '#ff5263', '#ffd447', '#39e58c'];
  const movementNames = ['UNKNOWN', 'STRAIGHT', 'LEFT', 'LEFT UNPROTECTED', 'RIGHT UNPROTECTED', 'UTURN'];
  const vehicleStateSources = ['UNKNOWN', 'GPS/IMU', 'EGO STATUS'];
  const objectTypes = {
    0: {name: 'UNKNOWN', css: 'unknown', color: '#a8b6c0'},
    1: {name: 'VEHICLE', css: 'vehicle', color: '#ff9b4a'},
    2: {name: 'PEDESTRIAN', css: 'pedestrian', color: '#ff75cc'},
    3: {name: 'STATIC', css: 'static', color: '#ff5263'}
  };
  const motionStates = {
    UNKNOWN: {name: 'UNKNOWN', css: 'motion-unknown', color: '#87939b'},
    LOW_THREAT: {name: 'LOW_THREAT', css: 'motion-low', color: '#39e58c'},
    PENDING: {name: 'PENDING', css: 'motion-pending', color: '#ffd447'},
    DYNAMIC: {name: 'DYNAMIC', css: 'motion-dynamic', color: '#ff5263'}
  };

  const data = {
    links: [], nodes: [], route: [], lights: {}, command: null, feedback: null,
    feature: null, trajectory: null, ego: null, lidarObjects: null,
    lidarMotionDebug: null, debugConfig: null, planningMarkers: null,
    recentPlanningIssue: null, receivedAt: {}
  };
  const view = {scale: 1, x: 0, y: 0, dragging: false, lastX: 0, lastY: 0, follow: false};
  const tracks = new Map();
  let bounds = null;
  let lidarFrameKey = null;
  let lidarFrameIndex = 0;
  let previousObjects = new Map();
  let lidarChanges = {appeared: [], disappeared: [], typeChanged: []};
  let motionDebugById = new Map();

  const finite = value => Number.isFinite(Number(value));
  const fmt = (value, digits = 1) => finite(value) && Math.abs(Number(value)) < 1e8 ? Number(value).toFixed(digits) : '∞';
  const esc = value => String(value ?? '').replace(/[&<>]/g, character => ({'&': '&amp;', '<': '&lt;', '>': '&gt;'}[character]));
  const worldToScreen = point => ({x: point[0] * view.scale + view.x, y: -point[1] * view.scale + view.y});
  const screenToWorld = (x, y) => [(x - view.x) / view.scale, -(y - view.y) / view.scale];
  const typeInfo = value => objectTypes[Number(value)] || {name: `TYPE ${value}`, css: 'unknown', color: '#a8b6c0'};
  const motionInfo = value => motionStates[String(value || 'UNKNOWN')] || motionStates.UNKNOWN;
  const filterKeep = debug => {
    if (!debug) return true;
    if (typeof debug.final_keep === 'boolean') return debug.final_keep;
    return debug.boundary_keep !== false && debug.known_static_keep !== false &&
      debug.configured_static_keep !== false &&
      debug.planning_relevance_keep !== false;
  };
  const filterStage = debug => {
    if (!debug || filterKeep(debug)) return 'keep';
    if (debug.filter_stage) return String(debug.filter_stage);
    if (debug.boundary_keep === false) return 'drivable';
    if (debug.known_static_keep === false) return 'known_static';
    if (debug.configured_static_keep === false) return 'static_zone';
    if (debug.planning_relevance_keep === false) return 'planning_relevance';
    return 'unknown';
  };
  const filterStageName = stage => ({
    keep: 'KEEP', drivable: 'ROAD', known_static: 'CATALOG',
    static_zone: 'ZONE', planning_relevance: 'RELEVANCE', unknown: 'UNKNOWN'
  })[String(stage)] || String(stage || 'UNKNOWN').toUpperCase();
  const debugTypeInfo = debug => {
    const label = `${debug?.classification || ''} ${debug?.yolo_class || ''}`.toLowerCase();
    if (/person|pedestrian/.test(label)) return objectTypes[2];
    if (/car|vehicle|truck|bus/.test(label)) return objectTypes[1];
    if (/static|obstacle/.test(label)) return objectTypes[3];
    return objectTypes[0];
  };
  const debugAsObject = debug => ({
    id: debug?.id,
    type: Number(debug?.type ?? 0),
    center: {x: Number(debug?.center_x), y: Number(debug?.center_y)},
    obb_corners: (debug?.obb_corners || []).map(point => ({
      x: Number(Array.isArray(point) ? point[0] : point?.x),
      y: Number(Array.isArray(point) ? point[1] : point?.y)
    })),
    length: Number(debug?.length),
    width: Number(debug?.width),
    vx: Number(debug?.applied_map_vx_mps || 0),
    vy: Number(debug?.applied_map_vy_mps || 0)
  });

  function resize() {
    const density = devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * density;
    canvas.height = rect.height * density;
    ctx.setTransform(density, 0, 0, density, 0, 0);
    draw();
  }

  function fitMap() {
    if (!bounds) return;
    const rect = canvas.getBoundingClientRect();
    const padding = 45;
    view.scale = Math.min((rect.width - 2 * padding) / (bounds.maxX - bounds.minX), (rect.height - 2 * padding) / (bounds.maxY - bounds.minY));
    view.x = rect.width / 2 - (bounds.minX + bounds.maxX) * view.scale / 2;
    view.y = rect.height / 2 + (bounds.minY + bounds.maxY) * view.scale / 2;
    view.follow = false;
    draw();
  }

  function centerEgo() {
    if (!data.ego || !finite(data.ego.x) || !finite(data.ego.y)) return;
    const rect = canvas.getBoundingClientRect();
    view.x = rect.width / 2 - Number(data.ego.x) * view.scale;
    view.y = rect.height / 2 + Number(data.ego.y) * view.scale;
  }

  function path(points, color, width = 1, alpha = 1) {
    if (!points || points.length < 2) return;
    ctx.beginPath();
    let point = worldToScreen(points[0]);
    ctx.moveTo(point.x, point.y);
    for (let i = 1; i < points.length; ++i) {
      point = worldToScreen(points[i]);
      ctx.lineTo(point.x, point.y);
    }
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  function circle(x, y, radius, color, stroke = null) {
    const point = worldToScreen([x, y]);
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    if (stroke) {
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  function convexHull(points) {
    const unique = new Map();
    for (const point of points) {
      if (!finite(point[0]) || !finite(point[1])) continue;
      unique.set(`${Number(point[0]).toFixed(8)}:${Number(point[1]).toFixed(8)}`, [Number(point[0]), Number(point[1])]);
    }
    const sorted = [...unique.values()].sort((left, right) => left[0] - right[0] || left[1] - right[1]);
    if (sorted.length < 3) return [];
    const cross = (origin, left, right) => (left[0] - origin[0]) * (right[1] - origin[1]) - (left[1] - origin[1]) * (right[0] - origin[0]);
    const lower = [];
    for (const point of sorted) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0) lower.pop();
      lower.push(point);
    }
    const upper = [];
    for (let i = sorted.length - 1; i >= 0; --i) {
      const point = sorted[i];
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0) upper.pop();
      upper.push(point);
    }
    lower.pop();
    upper.pop();
    return lower.concat(upper);
  }

  function filledPolygon(points, fillColor, strokeColor = fillColor, width = 2) {
    if (points.length < 3) return;
    ctx.beginPath();
    let point = worldToScreen(points[0]);
    ctx.moveTo(point.x, point.y);
    for (let i = 1; i < points.length; ++i) {
      point = worldToScreen(points[i]);
      ctx.lineTo(point.x, point.y);
    }
    ctx.closePath();
    ctx.globalAlpha = 0.18;
    ctx.fillStyle = fillColor;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = width;
    ctx.stroke();
  }

  function drawVelocityArrow(object, vx, vy, color, dashed = false) {
    if (!finite(object.center?.x) || !finite(object.center?.y) || !finite(vx) || !finite(vy)) return;
    const start = worldToScreen([Number(object.center.x), Number(object.center.y)]);
    const end = worldToScreen([Number(object.center.x) + Number(vx), Number(object.center.y) + Number(vy)]);
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    if (Math.hypot(dx, dy) < 2) return;
    const angle = Math.atan2(dy, dx);
    ctx.save();
    if (dashed) ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.lineTo(end.x - 7 * Math.cos(angle - 0.45), end.y - 7 * Math.sin(angle - 0.45));
    ctx.moveTo(end.x, end.y);
    ctx.lineTo(end.x - 7 * Math.cos(angle + 0.45), end.y - 7 * Math.sin(angle + 0.45));
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.restore();
  }

  function velocityArrows(object, debug) {
    if (debug) {
      drawVelocityArrow(object, debug.raw_map_vx_mps, debug.raw_map_vy_mps, '#ff75cc', true);
    }
    drawVelocityArrow(object, object.vx, object.vy, '#39e58c');
  }

  function strokePolygon(points, color, width = 3) {
    if (points.length < 3) return;
    ctx.save();
    ctx.beginPath();
    let point = worldToScreen(points[0]);
    ctx.moveTo(point.x, point.y);
    for (let i = 1; i < points.length; ++i) {
      point = worldToScreen(points[i]);
      ctx.lineTo(point.x, point.y);
    }
    ctx.closePath();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.shadowColor = color;
    ctx.shadowBlur = 8;
    ctx.stroke();
    ctx.restore();
  }

  function drawLidarObject(object) {
    const info = typeInfo(object.type);
    const debug = motionDebugById.get(Number(object.id));
    const motion = motionInfo(debug?.motion_state);
    const selectedAvoidTarget = Boolean(data.feature?.static_obstacle_ahead) &&
      Number(data.feature?.avoidance_object_id) === Number(object.id);
    const track = tracks.get(Number(object.id));
    if (track && track.points.length > 1) path(track.points, info.color, 1, 0.28);
    const hull = convexHull((object.obb_corners || []).map(point => [point.x, point.y]));
    if (hull.length >= 3) filledPolygon(hull, info.color, motion.color, selectedAvoidTarget ? 3 : 2);
    else if (finite(object.center?.x) && finite(object.center?.y)) circle(object.center.x, object.center.y, 4, info.color);
    if (selectedAvoidTarget && hull.length >= 3) strokePolygon(hull, '#ffffff', 3.5);
    if (finite(object.center?.x) && finite(object.center?.y)) circle(object.center.x, object.center.y, 2.2, info.color, '#061018');
    if ($('showVelocity').checked) velocityArrows(object, debug);
    if ($('showLabels').checked && finite(object.center?.x) && finite(object.center?.y)) {
      const point = worldToScreen([object.center.x, object.center.y]);
      ctx.font = 'bold 11px ui-monospace, monospace';
      ctx.fillStyle = selectedAvoidTarget ? '#ffffff' : motion.color;
      const target = selectedAvoidTarget ? ' AVOID TARGET' : '';
      const roadRatio = Number(debug?.road_ratio);
      const road = Number.isFinite(roadRatio) && roadRatio >= 0
        ? ` road ${Math.round(roadRatio * 100)}%` : '';
      ctx.fillText(`ID ${object.id} ${info.name}/${motion.name}${road}${target}`, point.x + 7, point.y - 7);
    }
  }

  function drawDroppedLidarObject(debug) {
    const object = debugAsObject(debug);
    const info = debugTypeInfo(debug);
    const motion = motionInfo(debug?.motion_state);
    const hull = convexHull((object.obb_corners || []).map(point => [point.x, point.y]));
    ctx.save();
    ctx.setLineDash([7, 5]);
    if (hull.length >= 3) {
      ctx.beginPath();
      let point = worldToScreen(hull[0]);
      ctx.moveTo(point.x, point.y);
      for (let i = 1; i < hull.length; ++i) {
        point = worldToScreen(hull[i]);
        ctx.lineTo(point.x, point.y);
      }
      ctx.closePath();
      ctx.globalAlpha = 0.08;
      ctx.fillStyle = motion.color;
      ctx.fill();
      ctx.globalAlpha = 0.95;
      ctx.strokeStyle = '#e7f1f6';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
    ctx.restore();
    if (finite(object.center?.x) && finite(object.center?.y)) {
      const center = worldToScreen([object.center.x, object.center.y]);
      ctx.save();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(center.x - 5, center.y - 5);
      ctx.lineTo(center.x + 5, center.y + 5);
      ctx.moveTo(center.x + 5, center.y - 5);
      ctx.lineTo(center.x - 5, center.y + 5);
      ctx.stroke();
      ctx.restore();
      if ($('showVelocity').checked) velocityArrows(object, debug);
      if ($('showLabels').checked) {
        ctx.font = 'bold 11px ui-monospace, monospace';
        ctx.fillStyle = '#ffffff';
        const roadRatio = Number(debug?.road_ratio);
        const road = Number.isFinite(roadRatio) && roadRatio >= 0
          ? ` road ${Math.round(roadRatio * 100)}%` : '';
        ctx.fillText(
          `ID ${debug.id} DROP/${filterStageName(filterStage(debug))} ${info.name}/${motion.name}${road}`,
          center.x + 8, center.y - 8);
      }
    }
  }

  function offsetPolyline(points, offset) {
    const result = [];
    for (let i = 0; i < points.length; ++i) {
      const before = points[Math.max(0, i - 1)];
      const after = points[Math.min(points.length - 1, i + 1)];
      const dx = Number(after[0]) - Number(before[0]);
      const dy = Number(after[1]) - Number(before[1]);
      const norm = Math.hypot(dx, dy);
      if (norm < 1e-6) {
        result.push([Number(points[i][0]), Number(points[i][1])]);
      } else {
        result.push([
          Number(points[i][0]) - dy / norm * offset,
          Number(points[i][1]) + dx / norm * offset
        ]);
      }
    }
    return result;
  }

  function drawAvoidCorridor() {
    if (!$('showCorridor').checked) return;
    const halfWidth = Number(data.debugConfig?.path_corridor_half_width_m);
    if (!Number.isFinite(halfWidth) || halfWidth <= 0) return;
    ctx.save();
    ctx.setLineDash([7, 6]);
    for (const link of data.route) {
      path(offsetPolyline(link.points, halfWidth), '#b784ff', 1.4, 0.8);
      path(offsetPolyline(link.points, -halfWidth), '#b784ff', 1.4, 0.8);
    }
    ctx.restore();
  }

  function stopTarget(x, y) {
    const point = worldToScreen([x, y]);
    ctx.save();
    ctx.translate(point.x, point.y);
    ctx.rotate(Math.PI / 4);
    ctx.fillStyle = '#ff9b4a';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.fillRect(-4.5, -4.5, 9, 9);
    ctx.strokeRect(-4.5, -4.5, 9, 9);
    ctx.restore();
  }

  function drawSignalNode(node, active = false) {
    const state = data.lights[node.traffic_light_id] ?? 0;
    circle(node.point[0], node.point[1], active ? 7.2 : 4.8, lightColors[state], active ? '#fff' : '#061018');
  }

  function draw() {
    if (view.follow) centerEgo();
    const rect = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, rect.width, rect.height);
    ctx.fillStyle = '#071018';
    ctx.fillRect(0, 0, rect.width, rect.height);
    for (const link of data.links) path(link.points, '#20313d', 0.65, 0.55);
    for (const link of data.route) path(link.points, '#20c8f5', 2.4, 0.95);
    drawAvoidCorridor();

    if ($('showTrajectory').checked && data.planningMarkers) {
      for (const points of data.planningMarkers.valid || []) path(points, '#75bfff', 1.2, 0.6);
      for (const points of data.planningMarkers.rejected || []) path(points, '#ff5263', 1, 0.5);
      const selected = data.planningMarkers.selected || [];
      if (selected.length) path(selected, '#ffd447', 3.2, 1);
      else if (data.trajectory?.valid) path((data.trajectory.points || []).map(point => [point.x, point.y]), '#ffd447', 3.2, 1);
    }

    if ($('showSignals').checked) {
      const activeId = data.feature?.traffic_light_id || '';
      for (const node of data.nodes) {
        if (node.on_stop_line && node.traffic_light_id && node.traffic_light_id !== activeId) drawSignalNode(node);
      }
      for (const node of data.nodes) {
        if (node.on_stop_line && node.traffic_light_id === activeId) drawSignalNode(node, true);
      }
    }
    if ($('showDroppedObjects').checked) {
      for (const debug of data.lidarMotionDebug?.objects || []) {
        if (!filterKeep(debug)) drawDroppedLidarObject(debug);
      }
    }
    if ($('showObjects').checked) {
      for (const object of data.lidarObjects?.objects || []) drawLidarObject(object);
    }

    if (data.ego) {
      const point = worldToScreen([data.ego.x, data.ego.y]);
      ctx.save();
      ctx.translate(point.x, point.y);
      ctx.rotate(-Number(data.ego.yaw || 0));
      ctx.beginPath();
      ctx.moveTo(11, 0);
      ctx.lineTo(-7, -7);
      ctx.lineTo(-4, 0);
      ctx.lineTo(-7, 7);
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
      ctx.strokeStyle = data.ego.valid ? '#39e58c' : '#ff5263';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
    }
    const command = data.command || {};
    if (command.has_target_pose && Number(command.maneuver_type) === 5) stopTarget(command.target_pose_map.position.x, command.target_pose_map.position.y);
  }

  function updateLidarTracks(message) {
    const key = `${message?.header?.frame_id ?? ''}:${message?.header?.stamp}`;
    if (key === lidarFrameKey) return;
    lidarFrameKey = key;
    ++lidarFrameIndex;
    const current = new Map();
    for (const object of message.objects || []) current.set(Number(object.id), object);
    const appeared = [];
    const disappeared = [];
    const typeChanged = [];
    for (const [id, object] of current) {
      if (!previousObjects.has(id)) appeared.push(id);
      else if (Number(previousObjects.get(id).type) !== Number(object.type)) typeChanged.push(`${id}:${typeInfo(previousObjects.get(id).type).name}→${typeInfo(object.type).name}`);
      if (!finite(object.center?.x) || !finite(object.center?.y)) continue;
      const track = tracks.get(id) || {points: [], lastSeen: lidarFrameIndex};
      track.points.push([Number(object.center.x), Number(object.center.y)]);
      if (track.points.length > 20) track.points.shift();
      track.lastSeen = lidarFrameIndex;
      tracks.set(id, track);
    }
    for (const id of previousObjects.keys()) if (!current.has(id)) disappeared.push(id);
    for (const [id, track] of tracks) if (lidarFrameIndex - track.lastSeen > 10) tracks.delete(id);
    previousObjects = current;
    lidarChanges = {appeared, disappeared, typeChanged};
  }

  function kv(items) {
    return items.map(([key, value]) => `<div><span>${esc(key)}</span><strong title="${esc(value)}">${esc(value)}</strong></div>`).join('');
  }

  function changeText(values) {
    return values.length ? values.slice(0, 6).join(', ') + (values.length > 6 ? ' …' : '') : '–';
  }

  function updateLidarPanel() {
    const message = data.lidarObjects || {};
    const objects = message.objects || [];
    const debugMessage = data.lidarMotionDebug || {};
    const debugObjects = debugMessage.objects || [];
    const outputById = new Map(objects.map(object => [Number(object.id), object]));
    const counts = {vehicle: 0, pedestrian: 0, unknown: 0, static: 0};
    const stateCounts = {UNKNOWN: 0, LOW_THREAT: 0, PENDING: 0, DYNAMIC: 0};
    const stageCounts = {drivable: 0, known_static: 0, static_zone: 0, planning_relevance: 0, unknown: 0};
    for (const object of objects) {
      const name = typeInfo(object.type).css;
      if (Object.prototype.hasOwnProperty.call(counts, name)) ++counts[name];
      else ++counts.unknown;
    }
    for (const debug of debugObjects) {
      const state = String(debug.motion_state || 'UNKNOWN');
      stateCounts[state] = (stateCounts[state] || 0) + 1;
      if (!filterKeep(debug)) {
        const stage = filterStage(debug);
        stageCounts[stage] = (stageCounts[stage] || 0) + 1;
      }
    }
    const rawCount = debugObjects.length || Number(debugMessage.filter_counts?.raw) || objects.length;
    const keptCount = debugObjects.length
      ? debugObjects.filter(filterKeep).length
      : Number(debugMessage.filter_counts?.kept) || objects.length;
    const droppedCount = Math.max(0, rawCount - keptCount);
    const sourceStamp = Number(message.header?.stamp);
    const debugStamp = Number(debugMessage.stamp);
    const stampDeltaMs = Number.isFinite(sourceStamp) && Number.isFinite(debugStamp)
      ? Math.abs(sourceStamp - debugStamp) * 1000 : NaN;
    const avoidTargetId = data.feature?.static_obstacle_ahead
      ? Number(data.feature?.avoidance_object_id) : null;
    const corridorHalfWidth = Number(data.debugConfig?.path_corridor_half_width_m);
    const boundary = debugMessage.boundary_filter || {};
    const knownStatic = debugMessage.known_static_filter || {};
    const staticZone = debugMessage.static_structure_filter || {};
    $('lidarGrid').innerHTML = kv([
      ['Frame', message.header?.frame_id || 'WAIT'],
      ['Source stamp', finite(message.header?.stamp) ? fmt(message.header.stamp, 3) : '–'],
      ['RAW / KEEP / DROP', `${rawCount} / ${keptCount} / ${droppedCount}`],
      ['DROP road/catalog/zone/relevance', `${stageCounts.drivable}/${stageCounts.known_static}/${stageCounts.static_zone}/${stageCounts.planning_relevance}`],
      ['Road mask', boundary.available === false ? 'FAIL-OPEN' : `margin ${fmt(boundary.margin_m)}m · ≥${fmt(Number(boundary.road_ratio_threshold) * 100, 0)}%`],
      ['Fixed filters', `catalog ${knownStatic.available ? 'ON' : 'OFF'} · zones ${staticZone.available ? staticZone.zone_count ?? 0 : 'OFF'}`],
      ['Output objects', objects.length],
      ['Vehicle / Ped', `${counts.vehicle} / ${counts.pedestrian}`],
      ['Unknown', counts.unknown],
      ['Motion U/L/P/D', `${stateCounts.UNKNOWN}/${stateCounts.LOW_THREAT}/${stateCounts.PENDING}/${stateCounts.DYNAMIC}`],
      ['Debug ↔ object stamp', finite(stampDeltaMs) ? `${fmt(stampDeltaMs, 1)} ms` : '–'],
      ['AVOID corridor', finite(corridorHalfWidth) ? `±${fmt(corridorHalfWidth, 2)} m` : '–'],
      ['AVOID target', avoidTargetId === null ? 'NONE' : `ID ${avoidTargetId}`],
      ['Appeared IDs', changeText(lidarChanges.appeared)],
      ['Disappeared IDs', changeText(lidarChanges.disappeared)],
      ['Type changes', changeText(lidarChanges.typeChanged)],
      ['Yaw rate', '0 assumed / unmeasured']
    ]);
    const filterSummary = `<span class="summary-filter">LiDAR raw ${rawCount} → planner 입력 ${objects.length}; 제거 ${droppedCount}</span>`;
    const behaviorSummary = avoidTargetId === null
      ? '<span class="summary-clear">Behavior 경로 회랑을 막는 UNKNOWN/STATIC 대상 없음</span>'
      : `<span class="summary-blocked">ID ${esc(avoidTargetId)}가 경로 회랑을 막아 Behavior AVOID 후보로 선택됨</span>`;
    $('motionSummary').innerHTML = `${filterSummary}${behaviorSummary}`;

    const rows = debugObjects.map(debug => ({
      id: Number(debug.id), debug, object: outputById.get(Number(debug.id)) || null,
      keep: filterKeep(debug)
    }));
    const debugIds = new Set(rows.map(row => row.id));
    for (const object of objects) {
      if (!debugIds.has(Number(object.id))) {
        rows.push({id: Number(object.id), debug: null, object, keep: true});
      }
    }
    rows.sort((left, right) => {
      const leftTarget = left.id === avoidTargetId ? 0 : 1;
      const rightTarget = right.id === avoidTargetId ? 0 : 1;
      if (leftTarget !== rightTarget) return leftTarget - rightTarget;
      if (left.keep !== right.keep) return left.keep ? 1 : -1;
      return left.id - right.id;
    });
    $('lidarObjects').innerHTML = rows.length ? rows.slice(0, 50).map(row => {
      const object = row.object || debugAsObject(row.debug);
      const debug = row.debug;
      const info = row.object ? typeInfo(object.type) : debugTypeInfo(debug);
      const motion = motionInfo(debug?.motion_state);
      const isAvoidTarget = row.id === avoidTargetId;
      const speed = Math.hypot(Number(object.vx || 0), Number(object.vy || 0));
      const geometry = `map (${fmt(object.center?.x)}, ${fmt(object.center?.y)})  OBB ${fmt(object.length)}×${fmt(object.width)}m`;
      const velocity = debug
        ? `raw ${fmt(debug.raw_speed_kph)} · judge ${fmt(debug.decision_speed_kph)} · applied ${fmt(debug.applied_speed_kph)} km/h`
        : `applied ${fmt(speed * 3.6)} km/h · motion debug WAIT`;
      const roadRatio = Number(debug?.road_ratio);
      const road = Number.isFinite(roadRatio) && roadRatio >= 0
        ? `${fmt(roadRatio * 100, 0)}%` : 'N/A';
      const predictedRoadRatio = Number(debug?.predicted_road_ratio);
      const predictedRoad = Number.isFinite(predictedRoadRatio) && predictedRoadRatio >= 0
        ? `→${fmt(predictedRoadRatio * 100, 0)}%` : '';
      const stage = filterStage(debug);
      const filterDetail = debug
        ? `filter ${row.keep ? 'KEEP' : `DROP/${filterStageName(stage)}`} · road ${road}${predictedRoad} · h ${fmt(debug.height, 2)}m · confirm ${debug.height_confirmations ?? 0} · ${debug.filter_reason || debug.boundary_reason || '–'}${debug.geometry_clipped ? ' · OBB CLIPPED' : ''}${debug.configured_static_zone ? ` · zone ${debug.configured_static_zone}` : ''}`
        : '–';
      const displacement = debug
        ? `center ${fmt(debug.map_displacement_speed_kph)}→${fmt(debug.corrected_displacement_speed_kph)} · jump ${fmt(debug.velocity_jump_kph)} km/h · sample ${debug.accepted_samples ?? 0}/${debug.candidate_samples ?? 0}`
        : '–';
      return `<div class="object-row ${info.css} ${motion.css} ${row.keep ? 'filter-kept' : 'filter-dropped'} ${isAvoidTarget ? 'avoid-selected' : ''}">
        <span class="object-id">ID ${esc(row.id)}${isAvoidTarget ? '<b>AVOID</b>' : ''}</span>
        <span class="object-type">${esc(info.name)}</span>
        <span class="motion-state">${esc(motion.name)}</span>
        <span class="filter-state">${row.keep ? 'KEEP' : 'DROP'}</span>
        <span class="object-detail" title="${esc(geometry)}">${esc(geometry)}</span>
        <span class="object-detail velocity-detail" title="${esc(velocity)}">${esc(velocity)}</span>
        <span class="object-detail displacement-detail" title="${esc(displacement)}">${esc(displacement)}</span>
        <span class="object-detail filter-detail" title="${esc(filterDetail)}">${esc(filterDetail)}</span>
      </div>`;
    }).join('') : '<div class="object-empty">LiDAR raw 진단 또는 planner 입력을 기다리는 중입니다.</div>';
  }

  function updatePanel() {
    const command = data.command || {};
    const feedback = data.feedback || {};
    const feature = data.feature || {};
    const trajectory = data.trajectory || {};
    const behavior = Number(feature.selected_behavior);
    const commandManeuver = Number(command.maneuver_type);
    const frenetManeuver = Number(feedback.active_maneuver);
    if (feature.traffic_light_id && finite(feature.traffic_light_state)) data.lights[feature.traffic_light_id] = Number(feature.traffic_light_state);
    $('behaviorName').textContent = decisionNames[behavior] || 'WAITING';
    $('behaviorCode').textContent = finite(behavior) ? behavior : '–';
    $('commandManeuverName').textContent = maneuverNames[commandManeuver] || 'WAITING';
    $('commandManeuverCode').textContent = finite(commandManeuver) ? `CMD ${commandManeuver}` : '–';
    $('frenetManeuverName').textContent = maneuverNames[frenetManeuver] || 'IDLE';
    $('frenetManeuverCode').textContent = finite(frenetManeuver) ? `ACTIVE ${frenetManeuver}` : '–';
    $('stateName').textContent = executionNames[feedback.execution_state] || 'WAITING';
    $('stateCode').textContent = finite(feedback.execution_state) ? `EXEC ${feedback.execution_state}` : '–';

    const trajectoryPoints = (trajectory.points || []).length;
    const horizon = trajectory.valid && trajectoryPoints > 1 && finite(trajectory.dt) ? (trajectoryPoints - 1) * Number(trajectory.dt) : NaN;
    const reportsCandidateCounts = commandManeuver === 3 || commandManeuver === 4 ||
      frenetManeuver === 3 || frenetManeuver === 4;
    const candidateText = reportsCandidateCounts
      ? `${feedback.valid_candidate_count ?? 0}/${feedback.generated_candidate_count ?? 0}`
      : 'N/A';
    const issue = data.recentPlanningIssue || {};
    const issueAge = Date.now() / 1000 - Number(issue.observed_at);
    const recentIssue = Number.isFinite(issueAge) && issueAge >= 0 && issueAge <= 5
      ? `${issue.source || 'PLANNING'}: ${issue.reason || issue.detail || 'UNKNOWN'} (${issueAge.toFixed(1)}s ago)`
      : 'NONE';
    $('planningGrid').innerHTML = kv([
      ['Vehicle state', `${vehicleStateSources[feature.vehicle_state_source] || 'UNKNOWN'} / ${feature.vehicle_state_valid ? 'VALID' : 'INVALID'}`],
      ['Ego speed', `${fmt(feature.ego_speed)} m/s`],
      ['Desired', `${fmt(command.desired_speed_mps)} m/s`],
      ['Route s / d', `${fmt(feature.ego_s)} / ${fmt(feature.ego_d)}`],
      ['Command ID', command.command_id ?? '–'],
      ['Request', requestNames[command.execution_request] || '–'],
      ['Accepted', feedback.command_accepted ? 'YES' : 'NO'],
      ['Result', resultNames[feedback.result_code] || feedback.result_code || '–'],
      ['Plan', feedback.plan_valid ? 'VALID' : 'INVALID'],
      ['Candidates', candidateText],
      ['Applied speed', `${fmt(feedback.applied_speed_mps)} m/s`],
      ['Trajectory', trajectory.valid ? `${trajectoryPoints} pts` : `INVALID (${trajectory.failure_reason ?? '–'})`],
      ['Trajectory source', trajectorySourceNames[feedback.trajectory_source] || feedback.trajectory_source || '–'],
      ['Selected horizon', finite(horizon) ? `${fmt(horizon, 2)} s` : '–'],
      ['Recent stop / issue', recentIssue]
    ]);
    $('riskGrid').innerHTML = kv([
      ['Raw signal', feature.traffic_light_raw_label || '–'],
      ['Required movement', movementNames[feature.required_movement] || 'UNKNOWN'],
      ['Signal', `${feature.traffic_light_id || '–'} / ${lightNames[feature.traffic_light_state] || 'UNKNOWN'}`],
      ['Movement distance', `${fmt(feature.distance_to_signal_movement)} m`],
      ['Stop distance', `${fmt(feature.distance_to_stop_line)} m`],
      ['Stop target', feature.has_stop_target ? `${fmt(feature.stop_x)}, ${fmt(feature.stop_y)}` : '–'],
      ['Front vehicle', feature.has_front_vehicle ? 'YES' : 'NO'],
      ['Front gap', `${fmt(feature.front_gap)} m`],
      ['Front TTC', `${fmt(feature.front_ttc)} s`],
      ['Predicted TTC', `${fmt(feature.predicted_front_ttc)} s`],
      ['Required decel', `${fmt(feature.required_deceleration)} m/s²`],
      ['Min TTC', `${fmt(feature.min_ttc)} s`],
      ['Pedestrian', feature.pedestrian_risk ? 'RISK' : 'CLEAR'],
      ['Route obstruction', feature.static_obstacle_ahead ? `YES / ID ${feature.avoidance_object_id}` : 'NO'],
      ['Corridor half-width', `${fmt(data.debugConfig?.path_corridor_half_width_m, 2)} m`],
      ['Avoid max distance', `${fmt(data.debugConfig?.avoid_max_distance_m)} m`]
    ]);

    const scores = [['KEEP', feature.keep_score], ['FOLLOW', feature.follow_score], ['LEFT', feature.left_change_score], ['RIGHT', feature.right_change_score], ['AVOID', feature.avoid_score], ['STOP', feature.stop_score], ['E-STOP', feature.emergency_stop_score]];
    const values = scores.map(item => Number(item[1])).filter(Number.isFinite);
    const minimum = Math.min(0, ...values);
    const maximum = Math.max(1, ...values);
    $('scores').innerHTML = scores.map(([name, value]) => {
      const number = Number(value);
      const percent = Number.isFinite(number) ? Math.max(0, Math.min(100, (number - minimum) / (maximum - minimum) * 100)) : 0;
      return `<div class="score"><span>${name}</span><div class="bar"><i style="width:${percent}%"></i></div><b>${fmt(value, 0)}</b></div>`;
    }).join('');
    $('reasons').innerHTML = `<div>${esc(command.reason || feature.decision_reason || '–')}</div><div class="hard">${esc(feedback.result_detail || feature.hard_rule_reason || '–')}</div>`;
    updateLidarPanel();
    updateHealth();
  }

  function health(key) {
    const received = Number(data.receivedAt[key]);
    if (!Number.isFinite(received)) return 'WAIT';
    const age = Date.now() / 1000 - received;
    return age < 1.0 ? 'LIVE' : age < 2.5 ? 'DELAYED' : 'STALE';
  }

  function updateHealth() {
    const lidarHealth = health('lidar_objects');
    const motionHealth = health('lidar_motion_debug');
    $('lidarStatus').textContent = `LIDAR ${lidarHealth}`;
    $('lidarStatus').className = `chip ${lidarHealth === 'LIVE' ? 'ok' : 'bad'}`;
    $('motionStatus').textContent = `MOTION ${motionHealth}`;
    $('motionStatus').className = `chip ${motionHealth === 'LIVE' ? 'ok' : 'bad'}`;
    $('health').innerHTML = kv([
      ['LiDAR canonical', lidarHealth],
      ['LiDAR motion debug', motionHealth],
      ['BehaviorCommand', health('command')],
      ['FrenetFeedback', health('feedback')],
      ['FeatureDebug', health('feature')],
      ['Frenet trajectory', health('trajectory')],
      ['Frenet markers', health('planning_markers')],
      ['Ego', health('ego')]
    ]);
  }

  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mouseX = event.clientX - rect.left;
    const mouseY = event.clientY - rect.top;
    const [worldX, worldY] = screenToWorld(mouseX, mouseY);
    const factor = Math.exp(-event.deltaY * 0.001);
    view.scale = Math.max(0.05, Math.min(80, view.scale * factor));
    view.x = mouseX - worldX * view.scale;
    view.y = mouseY + worldY * view.scale;
    view.follow = false;
    draw();
  }, {passive: false});
  canvas.addEventListener('mousedown', event => {
    view.dragging = true;
    view.lastX = event.clientX;
    view.lastY = event.clientY;
  });
  window.addEventListener('mouseup', () => { view.dragging = false; });
  window.addEventListener('mousemove', event => {
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
    $('cursor').textContent = `x ${worldX.toFixed(1)}  y ${worldY.toFixed(1)}`;
    if (!view.dragging) return;
    view.x += event.clientX - view.lastX;
    view.y += event.clientY - view.lastY;
    view.lastX = event.clientX;
    view.lastY = event.clientY;
    view.follow = false;
    draw();
  });
  $('fit').onclick = fitMap;
  $('follow').onclick = () => {
    view.follow = !view.follow;
    if (view.follow) centerEgo();
    $('follow').textContent = view.follow ? '추적 중' : 'Ego 추적';
    draw();
  };
  for (const id of ['showObjects', 'showDroppedObjects', 'showVelocity', 'showLabels', 'showCorridor', 'showSignals', 'showTrajectory']) $(id).onchange = draw;
  window.addEventListener('resize', resize);

  async function loadMap() {
    try {
      const noCache = {cache: 'no-store'};
      const [linkData, nodeData, routeText] = await Promise.all([
        fetch('../src/config/link_set.json', noCache).then(response => response.json()),
        fetch('../src/config/node_set.json', noCache).then(response => response.json()),
        fetch('../src/config/route_links_1.yaml', noCache).then(response => response.text())
      ]);
      data.links = Array.isArray(linkData) ? linkData : linkData.links;
      data.nodes = Array.isArray(nodeData) ? nodeData : nodeData.nodes;
      const linksById = new Map(data.links.map(link => [link.idx, link]));
      const routeIds = [...routeText.matchAll(/-\s*["']?([A-Za-z0-9_]+)["']?/g)].map(match => match[1]);
      data.route = routeIds.map(id => linksById.get(id)).filter(Boolean);
      bounds = {minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity};
      for (const link of data.links) for (const point of link.points) {
        bounds.minX = Math.min(bounds.minX, point[0]);
        bounds.maxX = Math.max(bounds.maxX, point[0]);
        bounds.minY = Math.min(bounds.minY, point[1]);
        bounds.maxY = Math.max(bounds.maxY, point[1]);
      }
      $('mapStatus').textContent = 'MAP READY';
      $('mapStatus').classList.add('ok');
      resize();
      fitMap();
    } catch (error) {
      $('mapStatus').textContent = 'MAP ERROR';
      $('mapStatus').classList.add('bad');
      console.error(error);
    }
  }

  async function pollSnapshot() {
    try {
      const snapshot = await fetch('../api/snapshot', {cache: 'no-store'}).then(response => response.json());
      data.command = snapshot.command || data.command;
      data.feedback = snapshot.feedback || data.feedback;
      data.feature = snapshot.feature || data.feature;
      data.trajectory = snapshot.trajectory || data.trajectory;
      data.ego = snapshot.ego || data.ego;
      data.planningMarkers = snapshot.planning_markers || data.planningMarkers;
      data.debugConfig = snapshot.debug_config || data.debugConfig;
      data.recentPlanningIssue = snapshot.recent_planning_issue || data.recentPlanningIssue;
      data.receivedAt = snapshot.received_at || {};
      if (snapshot.lidar_objects) {
        data.lidarObjects = snapshot.lidar_objects;
        updateLidarTracks(snapshot.lidar_objects);
      }
      if (snapshot.lidar_motion_debug) {
        data.lidarMotionDebug = snapshot.lidar_motion_debug;
        motionDebugById = new Map(
          (snapshot.lidar_motion_debug.objects || []).map(object => [Number(object.id), object]));
      }
      $('rosStatus').textContent = 'ROS CONNECTED';
      $('rosStatus').className = 'chip ok';
      updatePanel();
      draw();
    } catch (error) {
      $('rosStatus').textContent = 'ROS OFFLINE';
      $('rosStatus').className = 'chip bad';
      console.warn('dashboard snapshot unavailable', error);
    }
  }

  setInterval(updateHealth, 500);
  setInterval(pollSnapshot, 100);
  loadMap();
  pollSnapshot();
  updatePanel();
})();
