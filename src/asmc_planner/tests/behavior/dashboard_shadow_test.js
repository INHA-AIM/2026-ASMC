const fs = require('fs'), vm = require('vm'), assert = require('assert'), path = require('path');
let now = 1000;
const nodes = new Map();
const element = id => {
  if (!nodes.has(id)) nodes.set(id, {getContext: () => ({}), checked: false, value: 'auto',
    classList: {add() {}}, innerHTML: '', textContent: ''});
  return nodes.get(id);
};
let source = fs.readFileSync(path.join(__dirname,'../../web/dashboard_v2.js'),'utf8');
const hook = "  canvas.addEventListener('wheel',";
source = source.replace(hook, '  globalThis.probe={data,sceneEntry,localView,updateShadowPanel,localExperimentFresh,localExperimentView,updateLocalExperimentPanel}; return;\n'+hook);
const sandbox = {window:{},document:{getElementById:element},console,Date,performance:{now:()=>now}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(__dirname,"../../web/local_scene_renderer.js"),"utf8"),sandbox); vm.runInContext(source,sandbox);
const p = sandbox.probe;
p.data.shadowReceived=now;
const envelope = payload => ({payload, valid:true, available:true, receive_age_s:.1, source_age_s:.1, stamp_ns:100});
p.data.shadow = {selector:envelope({mode:'JAMMING',selected_source:'/ctrl_cmd/jamming',command_fresh:'true'}),
  controller:envelope({reason:'lane_change_timeout_restart_required',change_state:'changing',mode_active:'true'}),
  bbox:envelope({boxes:[{label:'plastic_barrier',assignment:0,reason:'contact_overlap_too_small'}]})};
assert(p.localView()); p.updateShadowPanel();
assert(element('shadowGrid').innerHTML.includes('/ctrl_cmd/jamming'));
assert(element('shadowGrid').innerHTML.includes('lane_change_timeout_restart_required'));
assert(element('shadowGrid').innerHTML.includes('비선택 제어기'));
assert(element('shadowBoxes').innerHTML.includes('plastic_barrier'));
now += 501;
assert(!p.sceneEntry('controller').available); p.updateShadowPanel();
assert.equal(element('selectedMode').textContent, 'JAMMING'); // 1 Hz diagnostics
assert(!element('shadowBoxes').innerHTML.includes('plastic_barrier'));
now += 2500; p.updateShadowPanel();
assert(element('selectedMode').textContent.includes('STALE'));
element('sceneMode').value='map'; assert(!p.localView());
element('sceneMode').value='local'; assert(p.localView());
assert(!source.includes('setInterval(pollSnapshot'), 'HTTP must not accumulate overlapping requests');
console.log('PASS shadow dashboard: selected controller, stop reason, labels, local/map, monotonic expiry');

element('sceneMode').value='auto';
p.data.localExperiment={valid:true,reason:'near_field_assisted_sweep',run_id:'test',plan_sequence:1};
p.data.localReceiveAge=.1;p.data.localReceived=now;p.data.command=null;
assert(p.localExperimentView(), 'fresh local preview is visible inside the main console');
assert(!p.localView(), 'experiment geometry must not mix with legacy unaligned overlay');
p.updateLocalExperimentPanel();
assert(element('localStatus').textContent.includes('PREVIEW'));
p.data.localControl={control_enabled:true,reason:'replan_grace_crawl'};
p.updateLocalExperimentPanel();assert(element('localStatus').textContent.includes('CONTROL'));
now+=700;assert(!p.localExperimentFresh());
element('sceneMode').value='experiment';assert(p.localExperimentView(), 'explicit local view remains selectable when stale');
p.updateLocalExperimentPanel();assert(element('localStatus').textContent.includes('STALE'));
element('sceneMode').value='map';assert(!p.localExperimentView());
console.log('PASS integrated LOCAL-1 selection, preview/control distinction, monotonic expiry and map isolation');
