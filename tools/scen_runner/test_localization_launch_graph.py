#!/usr/bin/env python3
"""Load both complete launch graphs without starting any ROS node."""
from pathlib import Path
import roslaunch.config,roslaunch.xmlloader

path=Path(__file__).resolve().parents[2]/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch'
def graph(choice, extra=None):
    config=roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path),config,argv=['shadow_planner:='+choice,'dashboard:=false']+(extra or []),verbose=False)
    return config
for choice in ('main','localization'):
    c=graph(choice);names=[n.name for n in c.nodes]
    assert len(names)==len(set(names)),names
    assert names.count('asmc_control_selector')==1 and names.count('asmc_planner')==1
    assert all(n.package!='asmc_bridge' for n in c.nodes)
    assert ('camera_bbox_lane_change' in names)==(choice=='main'),names
    assert ('camera_bbox_frame_bridge' in names)==(choice=='main'),names
    for name in ('localization_motion','localization','localization_controller'):
        assert (name in names)==(choice=='localization'),names
    assert c.params['/mpc_controller/suspend_in_jamming'].value==(choice=='localization')
    for key in ('/mpc_controller/localization_approach_speed_kph','/planning_mode/localization_approach_speed_kph'):
        assert c.params[key].value==(15. if choice=='localization' else 0.)
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
        assert c.params['/localization_controller/integrated_normal_controller'].value=='/mpc_controller'
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
except Exception:print('PASS unknown shadow_planner rejected before launch')
else:raise AssertionError('unknown mode silently accepted')

c=graph('localization',['shadow_motion_yaw_source:=rate'])
assert c.params['/localization_motion/yaw_source'].value=='rate'
c=graph('localization',['shadow_speed_cap_kph:=15'])
assert c.params['/mpc_controller/localization_approach_speed_kph'].value==15.
assert c.params['/planning_mode/localization_approach_speed_kph'].value==15.
try:graph('localization',['shadow_motion_yaw_source:=bad'])
except Exception:print('PASS unsupported motion yaw source rejected before launch')
else:raise AssertionError('unknown yaw source accepted')

# PR138 uses jamming_planner:=camera/local; share that selector vocabulary
# without importing the still-unmerged LOCAL implementation. Legacy commands work.
for canonical, legacy in [('camera','main'),('localization','localization')]:
    c=graph('main',['jamming_planner:='+canonical])
    expected=graph(legacy)
    assert [(n.package,n.type,n.name) for n in c.nodes]==[(n.package,n.type,n.name) for n in expected.nodes]
try:graph('main',['jamming_planner:=local'])
except Exception:print('PASS unmerged LOCAL cannot silently select a different controller')
else:raise AssertionError('unavailable LOCAL was accepted')
print('PASS canonical jamming_planner and existing shadow_planner commands share one graph')
