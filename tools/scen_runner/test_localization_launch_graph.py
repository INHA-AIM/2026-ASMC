#!/usr/bin/env python3
"""Load three complete launch graphs without starting any ROS node."""
from pathlib import Path
import roslaunch.config,roslaunch.xmlloader

path=Path(__file__).resolve().parents[2]/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch'
def graph(choice, extra=None):
    config=roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path),config,argv=([] if choice is None else ['jamming_planner:='+choice])+['dashboard:=false']+(extra or []),verbose=False)
    return config
for choice in ('camera','local','localization'):
    c=graph(choice);names=[n.name for n in c.nodes]
    assert len(names)==len(set(names)),names
    assert names.count('asmc_control_selector')==1 and names.count('asmc_planner')==1
    assert all(n.package!='asmc_bridge' for n in c.nodes)
    assert ('camera_bbox_lane_change' in names)==(choice=='camera'),names
    assert ('camera_bbox_frame_bridge' in names)==(choice=='camera'),names
    for name in ('localization_motion','localization','localization_controller'):
        assert (name in names)==(choice=='localization'),names
    assert c.params['/mpc_controller/suspend_in_jamming'].value is True
    assert c.params['/planning_mode/approach_speed_kph'].value==25.
    assert abs(c.params['/planning_mode/shadow_zone/station_start_m'].value-1859.6415782400395)<1e-6
    assert ('local_planner' in names)==(choice=='local')
    assert ('local_controller' in names)==(choice=='local')
    assert ('semantic_space_trial' in names)==(choice=='local')
    owner={'camera':'camera_bbox_lane_change','local':'local_planner','localization':'localization'}[choice]
    cap_key={'camera':'target_speed_kph','local':'speed_cap_kph','localization':'speed_cap_kph'}[choice]
    assert c.params['/'+owner+'/'+cap_key].value=={'camera':10.,'local':10.,'localization':15.}[choice]
    assert c.params['/behavior_planner/maximum_speed_kph'].value==57.5
    assert bool(c.params['/planning_mode/recovery_ready_topic'].value)==(choice=='localization')
    if choice=='localization':
        assert c.params['/localization/dynamic_obstacles'].value is True
        assert c.params['/localization/shadow_lane_rules'].value is True
        assert c.params['/localization/mode_gated'].value is True
        assert c.params['/localization_motion/yaw_source'].value=='orientation'
        assert c.params['/localization_motion/velocity_turn_gain_m2ps'].value==1.54
        assert c.params['/localization_controller/speed_cost'].value==24.
        assert c.params['/localization_controller/experimental_speed_cap_ceiling_kph'].value==20.
        assert c.params['/localization_controller/comfort_acceleration_limit_mps2'].value==1.
        assert c.params['/localization_controller/comfort_acceleration_rise_rate_mps3'].value==2.
        assert c.params['/localization_controller/bounded_standstill_rollback_mps'].value==.05
        assert c.params['/localization/longitudinal/jerk_limit_mps3'].value==2.
        assert c.params['/localization_controller/normal_controller_node'].value=='/mpc_controller'
    else:
        assert '/localization_motion/velocity_turn_gain_m2ps' not in c.params
        assert not any(k.endswith('/experimental_speed_cap_ceiling_kph') for k in c.params)
        assert not any(k.endswith('/comfort_acceleration_limit_mps2') for k in c.params)
    print('PASS',choice,names)
c=graph('localization',['shadow_start_motion:=false','shadow_motion_topic:=/shared/relative_motion'])
assert 'localization_motion' not in [n.name for n in c.nodes]
assert c.params['/localization/state_topic'].value=='/shared/relative_motion'
controller=next(n for n in c.nodes if n.name=='localization_controller')
assert dict(controller.remap_args)['/planning/local/odometry']=='/shared/relative_motion'
print('PASS external relative-motion source reuses one existing integrator')
c=graph('localization',['shadow_motion_turn_gain_m2ps:=0','shadow_motion_reference_offset_m:=1.4'])
assert c.params['/localization_motion/velocity_turn_gain_m2ps'].value==0
assert c.params['/localization_motion/velocity_reference_offset_m'].value==1.4
print('PASS constant-offset replay compatibility remains explicit')
c=graph('localization',['shadow_static_avoidance:=true','shadow_dynamic_obstacles:=true','shadow_lane_rules:=true'])
assert all(c.params['/localization/'+key].value is True for key in ('static_avoidance','dynamic_obstacles','shadow_lane_rules'))
print('PASS Localization comparison profile is selected explicitly; main remains unchanged')
try:graph('typo')
except Exception:print('PASS unknown jamming_planner rejected before launch')
else:raise AssertionError('unknown mode silently accepted')

c=graph('localization',['shadow_motion_yaw_source:=rate'])
assert c.params['/localization_motion/yaw_source'].value=='rate'
try:graph('localization',['shadow_motion_yaw_source:=bad'])
except Exception:print('PASS unsupported motion yaw source rejected before launch')
else:raise AssertionError('unknown yaw source accepted')

# Default selection and the single public selector must stay unambiguous.
import xml.etree.ElementTree as ET
arguments={a.get('name'):a.get('default') for a in ET.parse(path).findall('arg')}
assert arguments['jamming_planner']=='camera'
assert 'shadow_planner' not in arguments
assert [(n.package,n.type,n.name) for n in graph(None).nodes]==[(n.package,n.type,n.name) for n in graph('camera').nodes]
assert 'local_planner' in [n.name for n in graph('local').nodes]
print('PASS one selector and A448 entry for camera/local/localization; camera default')
