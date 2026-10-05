const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const path = require('path');
const sourcePath = path.join(__dirname, '../web/dashboard_v2.js');
const nodes = new Map();
const element = id => {
  if (!nodes.has(id)) nodes.set(id, {getContext: () => ({}), checked: false,
    classList: {add() {}}, innerHTML: '', textContent: ''});
  return nodes.get(id);
};
let source = fs.readFileSync(sourcePath, 'utf8');
const hook = "  canvas.addEventListener('wheel',";
assert(source.includes(hook), 'dashboard event boundary missing');
source = source.replace(hook,
  "  globalThis.probe = {data, objectMotion, updateLidarPanel, updateHealth}; return;\n" + hook);
const sandbox = {document: {getElementById: element}, console, Date};
vm.createContext(sandbox);
vm.runInContext(source, sandbox);
const p = sandbox.probe;
assert.strictEqual(p.objectMotion({motion_state: 0}, {motion_state: 'DYNAMIC'}).name, 'UNKNOWN');
assert.strictEqual(p.objectMotion({motion_state: 1}, null).name, 'LOW_THREAT');
assert.strictEqual(p.objectMotion({motion_state: 2}, null).name, 'PENDING');
assert.strictEqual(p.objectMotion({motion_state: 3}, null).name, 'DYNAMIC');
assert.strictEqual(p.objectMotion({motion_state: 99}, null).name, 'UNKNOWN');
assert.strictEqual(p.objectMotion({}, {motion_state: 'PENDING'}).name, 'PENDING');
p.data.lidarObjects = {header: {frame_id: 'map', stamp: 100},
  objects: [0,1,2,3].map((state, id) => ({id, type: 1, motion_state: state,
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
console.log('PASS dashboard motion rendering: zero/authority, four states, diagnostics absent, stale and empty frames');
