const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const path = require('path');
const sourcePath = path.join(__dirname, '../../web/dashboard_v2.js');
const nodes = new Map();
const drawnLabels = [];
const context = {beginPath() {}, arc() {}, fill() {}, stroke() {},
  fillText(text) { drawnLabels.push(text); }};
const element = id => {
  if (!nodes.has(id)) nodes.set(id, {getContext: () => context, checked: false,
    classList: {add() {}}, innerHTML: '', textContent: ''});
  return nodes.get(id);
};
let source = fs.readFileSync(sourcePath, 'utf8');
const hook = "  canvas.addEventListener('wheel',";
assert(source.includes(hook), 'dashboard event boundary missing');
source = source.replace(hook,
  "  globalThis.probe = {data, objectMotion, drawLidarObject, motionDebugById, updateLidarPanel, updateHealth}; return;\n" + hook);
const sandbox = {window:{},document: {getElementById: element}, console, Date, performance: {now: () => 1000}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(__dirname,"../../web/local_scene_renderer.js"),"utf8"),sandbox);
vm.runInContext(source, sandbox);
const p = sandbox.probe;
assert.strictEqual(p.objectMotion({motion_state: 0}, {motion_state: 'DYNAMIC'}).name, 'UNKNOWN');
assert.strictEqual(p.objectMotion({motion_state: 1}, null).name, 'LOW_THREAT');
assert.strictEqual(p.objectMotion({motion_state: 2}, null).name, 'PENDING');
assert.strictEqual(p.objectMotion({motion_state: 3}, null).name, 'DYNAMIC');
assert.strictEqual(p.objectMotion({motion_state: 99}, null).name, 'UNKNOWN');
assert.strictEqual(p.objectMotion({}, {motion_state: 'PENDING'}).name, 'PENDING');
// Live regression: box is delivered correctly but legacy policy type is UNKNOWN.
// Exercise the map canvas, not just the sidebar, without mutating planner input.
element('showLabels').checked = true;
const box = {id: 0, type: 0, camera_class: 'box', classification: 'static_obstacle',
  motion_state: 1, center: {x: 1, y: 2}, vx: 0, vy: 0};
const originalBox = JSON.stringify(box);
p.data.feature = {static_obstacle_ahead: true, avoidance_object_id: 0};
p.drawLidarObject(box);
assert(drawnLabels.some(text => text.includes('box · static_obstacle') && text.includes('AVOID TARGET')));
assert(drawnLabels.includes('policy 미확인 (입력 UNKNOWN) · motion LOW_THREAT'));
assert.strictEqual(JSON.stringify(box), originalBox, 'rendering must not change policy or motion');
for (const [camera_class, classification, type] of [
  ['car', 'vehicle', 1], ['person', 'dynamic_obstacle', 2]]) {
  drawnLabels.length = 0;
  p.drawLidarObject({...box, id: 1, camera_class, classification, type});
  assert(drawnLabels.some(text => text.includes(`${camera_class} · ${classification}`)));
  assert(drawnLabels.some(text => text.includes('motion LOW_THREAT')),
    'semantic vehicle/dynamic group does not imply moving');
}
drawnLabels.length = 0;
p.motionDebugById.set(0, {classification: 'vehicle', yolo_class: 'truck', motion_state: 'DYNAMIC'});
p.drawLidarObject({...box, camera_class: '', classification: ' ', motion_state: 0});
assert(drawnLabels.some(text => text.includes('unknown · unknown')));
assert(drawnLabels.includes('policy 미확인 (입력 UNKNOWN) · motion UNKNOWN'));
assert(!drawnLabels.some(text => text.includes('truck')), 'no stale debug semantic fallback');
p.motionDebugById.clear();
p.data.lidarObjects = {header: {frame_id: 'map', stamp: 100},
  objects: [0,1,2,3].map((state, id) => ({id, type: 1, motion_state: state, classification: 'static_obstacle', camera_class: 'plastic_barrier',
    center: {x: 0, y: 0}, length: 4, width: 2, vx: 2, vy: 0}))};
p.data.receivedAt.lidar_objects = Date.now() / 1000;
p.updateLidarPanel();
p.updateHealth();
assert(element('lidarGrid').innerHTML.includes('1/1/1/1'), 'motion counts must use output objects');
assert(element('lidarObjects').innerHTML.includes('LOW_THREAT'), 'output motion must render without JSON');
assert(!element('lidarObjects').innerHTML.includes('motion debug WAIT'));
assert(element('motionSummary').innerHTML.includes('원시 필터 진단 없음'));
assert.strictEqual(element('motionStatus').textContent, 'MOTION LIVE');
p.data.receivedAt.lidar_objects -= 5;
p.updateHealth();
assert.strictEqual(element('motionStatus').textContent, 'MOTION STALE');
p.data.lidarObjects.objects = [];
p.data.receivedAt.lidar_objects = Date.now() / 1000;
p.updateHealth();
assert.strictEqual(element('motionStatus').textContent, 'MOTION LIVE', 'fresh empty output is valid');
assert(element('lidarObjects').innerHTML.includes('plastic_barrier'));
assert(element('lidarObjects').innerHTML.includes('static_obstacle'));
console.log('PASS dashboard motion rendering: zero/authority, four states, diagnostics absent, stale and empty frames');
