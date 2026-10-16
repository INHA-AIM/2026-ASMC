#!/usr/bin/env python3
"""Exercise the optional example through real ROS services, without GPU/driving."""
from asmc_camera.runtime.parameters import canonical_control_path
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml

import rospy
from asmc_msgs.srv import SetInferenceMode, SetInferenceModeResponse
from asmc_camera.runtime.state import InferenceState
from asmc_hd_map.msg import HdMapContext
from asmc_msgs.msg import MissionZone, MissionZoneArray


def main():
    rospy.set_param('/use_sim_time',False)
    rospy.init_node('camera_region_example_ros_fixture')
    nodes=dict(twinlite='/camera_semantic_inference',yolo='/camera_object_detector',
               resnet18='/traffic_light_infer')
    states={name:InferenceState(['fixture'],managed=True) for name in nodes}
    services=[]
    def request(name, req):
        return SetInferenceModeResponse(*states[name].request(req.mode,req.target_hz))
    for name,node in nodes.items():
        services.append(rospy.Service(canonical_control_path(node, 'set_mode'),SetInferenceMode,
            lambda req,name=name:request(name,req)))
    context_pub=rospy.Publisher('/hd_map/context',HdMapContext,queue_size=1)
    zone_pub=rospy.Publisher('/hd_map/global/mission_zones',MissionZoneArray,queue_size=1,latch=True)
    zones=MissionZoneArray()
    for name,start,end in [('gps_shadow_candidate',100.,200.),
                           ('traffic_light_activation_A236_to_A308',300.,400.)]:
        zones.zones.append(MissionZone(id=name,status=MissionZone.STATUS_DERIVED_OFFICIAL,
            route_station_start_m=start,route_station_end_m=end))
    zone_pub.publish(zones)
    def start_example(config=None):
        command=['roslaunch','asmc_camera','camera_region_example.launch',
                 'allow_example_control:=true']
        if config is not None:
            command.append('config:='+str(config))
        return subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,
                                start_new_session=True)
    def stop_example():
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=10)
    process=None
    checks={}
    def context(station,valid):
        msg=HdMapContext()
        msg.header.stamp=msg.pose_stamp=rospy.Time.now()
        msg.pose_valid=msg.lane_valid=msg.route_valid=valid
        msg.pose_status=msg.POSE_VALID if valid else msg.POSE_GPS_UNAVAILABLE
        msg.route_station_m=station
        context_pub.publish(msg)
    def until(station, valid, expected, duration=.8):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            context(station,valid);time.sleep(.04)
        return {k:v.mode for k,v in states.items()}==expected
    def wait_for_example():
        deadline=time.monotonic()+10
        while context_pub.get_num_connections()==0 and time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError('region example exited before subscribing')
            time.sleep(.05)
        if context_pub.get_num_connections()==0:
            raise RuntimeError('region example did not subscribe to HD-map context')
    try:
        # The shipped policy must preserve continuous inference regardless of
        # mission station or GPS availability, at the current 15/15/10Hz rates.
        process=start_example()
        wait_for_example()
        active=dict(twinlite=2,yolo=2,resnet18=2)
        for name,station,valid in [('outside',50,True),('lane_approach',85,True),
                ('shadow',150,True),('gps_loss',150,False),('recovery',206,True),
                ('signal_approach',285,True),('signal',350,True)]:
            checks['continuous_'+name]=until(station,valid,active)
        checks['continuous_rates']={k:v.target_hz for k,v in states.items()}==dict(
            twinlite=15.,yolo=15.,resnet18=10.)
        stop_example()

        # Region activation remains configurable, but it is not the shipped
        # default. Exercise that API with an explicit test-only policy so the
        # production policy cannot silently revert to region publication gates.
        policy_path=Path(__file__).resolve().parents[2]/'src/asmc_camera/config/runtime/camera_region_example.yaml'
        policy=yaml.safe_load(policy_path.read_text())
        policy['gps_loss_modes']=dict(twinlite=2,yolo=2,resnet18=0)
        for name in ('twinlite','resnet18'):
            policy['models'][name].update(initial_mode=0,always_active=False)
        with tempfile.TemporaryDirectory(prefix='camera-region-fixture-') as directory:
            config=Path(directory)/'regional-policy.yaml'
            config.write_text(yaml.safe_dump(policy))
            process=start_example(config)
            wait_for_example()
            checks['outside']=until(50,True,dict(twinlite=0,yolo=2,resnet18=0))
            checks['lane_prepare']=until(85,True,dict(twinlite=1,yolo=2,resnet18=0))
            checks['lane_active']=until(150,True,dict(twinlite=2,yolo=2,resnet18=0))
            checks['gps_loss_lane_stays_active']=until(150,False,dict(twinlite=2,yolo=2,resnet18=0))
            checks['recovery_hold']=until(206,True,dict(twinlite=2,yolo=2,resnet18=0),.2)
            checks['fresh_recovery_after_exit']=until(206,True,dict(twinlite=0,yolo=2,resnet18=0))
            checks['resnet_prepare']=until(285,True,dict(twinlite=0,yolo=2,resnet18=1))
            checks['resnet_active']=until(350,True,dict(twinlite=0,yolo=2,resnet18=2))
            stop_example()
        # Replacing the example needs no changes in any model provider.
        for name in nodes:
            rospy.ServiceProxy(canonical_control_path(nodes[name], 'set_mode'),SetInferenceMode)(0,0)
        checks['example_removed_external_controller_works']=all(v.mode==0 for v in states.values())
    finally:
        stop_example()
        for service in services:service.shutdown()
        context_pub.unregister();zone_pub.unregister()
    passed=all(checks.values())
    print(json.dumps(dict(passed=passed,checks=checks)))
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
