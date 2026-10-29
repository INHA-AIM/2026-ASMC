#!/usr/bin/env python3
"""Agent-owned integrated trial on a private master; no native truth enters planning.

The parent is an active registered experiment. Each child trial is append-only.
Camera and LiDAR can be started under the physical lease with --own-perception.
This process owns
the physical lease, temporary raw bridge, full autonomy launch and bag recorder.
"""
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from xml.sax.saxutils import escape


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--trial', required=True)
    parser.add_argument('--jamming-planner',choices=['camera','local','localization'],default='localization')
    parser.add_argument('--gps-outage', action='store_true')
    parser.add_argument('--own-perception', action='store_true')
    parser.add_argument('--lidar-debug', action='store_true',
                        help='record sampled LiDAR debug stages for the read-only viewer; adds monitoring work')
    parser.add_argument('--lease-wait-seconds', type=float,default=0)
    parser.add_argument('--dashboard-port',type=int,default=8089,
                        help='persistent observer on this private master; campaign uses 8089')
    parser.add_argument('--observer-contact-baseline',action='store_true',
                        help='5 km/h empty-corridor observer diagnostic only; inherited contacts revoke driving acceptance')
    parser.add_argument('--static-avoidance', action='store_true')
    parser.add_argument('--dynamic-obstacles',action='store_true')
    parser.add_argument('--shadow-lane-rules',action='store_true')
    from localization_fixtures import NAMES
    parser.add_argument('--fixture', choices=NAMES, default='none')
    parser.add_argument('--park-prop', action='append', default=[])
    parser.add_argument('--park-corridor', action='store_true')
    parser.add_argument('--preserve-scene-label',default='',
                        help='observe an already loaded static scene without spawning, parking or deleting its actors')
    parser.add_argument('--cap', type=float, default=5.)
    parser.add_argument('--normal-cap',type=float,default=None,
                        help='NORMAL trial cap, default same as shadow; 0 preserves production speed policy')
    parser.add_argument('--yaw-source', choices=['orientation','rate'],default='orientation')
    parser.add_argument('--motion-reference-offset',type=float,default=1.4)
    parser.add_argument('--motion-turn-gain',type=float,default=1.54)
    parser.add_argument('--start', type=float, default=1830.)
    parser.add_argument('--lateral-offset',type=float,default=0)
    parser.add_argument('--heading-offset',type=float,default=0)
    parser.add_argument('--end', type=float, default=2008.)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--main-view-size',nargs=2,type=int,metavar=('WIDTH','HEIGHT'))
    parser.add_argument('--restore-view-size',nargs=2,type=int,metavar=('WIDTH','HEIGHT'))
    args = parser.parse_args()
    if args.preserve_scene_label and (len(args.preserve_scene_label)>80 or args.fixture!='none' or
                                     args.park_corridor or args.park_prop):
        raise ValueError('preserved scenes cannot be combined with actor placement/parking')
    from localization_run import active_registration,storage_minimum_free_bytes
    storage_floor=storage_minimum_free_bytes()
    registration=active_registration(args.root)
    if (args.dynamic_obstacles or args.shadow_lane_rules) and (
            not args.static_avoidance or args.jamming_planner!='localization'):
        raise ValueError('motion/lane experiments require the explicit Localization M3 preview')
    if args.jamming_planner=='localization' and args.fixture in ('crossing_left','crossing_right','standing_person') and not args.dynamic_obstacles:
        raise ValueError('vehicle/pedestrian motion fixtures require explicit dynamic preview')
    if Path(args.trial).name != args.trial or not 0 < args.cap <= 20 or not 0 < args.seconds <= 240 or not 0<=args.lease_wait_seconds<=600:
        raise ValueError('invalid bounded trial')
    if args.observer_contact_baseline and (args.cap>5 or args.fixture!='none' or
            not 0<args.end-args.start<=50 or args.seconds>60 or not args.dashboard_port):
        raise ValueError('observer contact diagnostic requires empty corridor, <=5 km/h, <=50m, <=60s and monitoring')
    normal_cap=args.cap if args.normal_cap is None else args.normal_cap
    if not math.isfinite(normal_cap) or not 0<=normal_cap<=30:
        raise ValueError('invalid NORMAL trial cap')
    if normal_cap==0:
        import yaml
        behavior=Path(__file__).resolve().parents[2]/'src/asmc_planner/config/behavior/behavior_params.yaml'
        normal_ceiling=float(yaml.safe_load(behavior.read_text())['maximum_speed_kph'])
    else:normal_ceiling=normal_cap
    if not math.isfinite(args.motion_reference_offset) or abs(args.motion_reference_offset)>2.5:
        raise ValueError('invalid velocity reference point calibration')
    if not math.isfinite(args.motion_turn_gain) or not 0<=args.motion_turn_gain<=2.5:
        raise ValueError('invalid velocity turn model')
    if bool(args.main_view_size)!=bool(args.restore_view_size):
        raise ValueError('viewport experiment requires an explicit original size for restoration')
    if any(not (320<=size[0]<=4096 and 240<=size[1]<=2160)
           for size in (args.main_view_size,args.restore_view_size) if size):
        raise ValueError('invalid bounded viewport dimensions')
    output = args.root/'trials'/args.trial
    output.parent.mkdir(exist_ok=True)
    output.mkdir(exist_ok=False)
    if args.fixture!='none' and args.jamming_planner=='localization' and not args.static_avoidance:raise ValueError('Localization obstacle trials require explicit avoidance')
    if args.static_avoidance and args.jamming_planner!='localization':raise ValueError('M3 fixture requires Localization')
    import rospy
    import rosgraph
    from std_msgs.msg import Bool, String, UInt8
    from diagnostic_msgs.msg import DiagnosticArray
    from morai_msgs.msg import CtrlCmd,CollisionData
    from asmc_msgs.msg import LocalTrajectory, VehicleState, InferenceStatus
    from google.protobuf.json_format import MessageToDict
    from scen_runner.config import load_config
    from scen_runner.runtime.validation_episode import EpisodeLink, EpisodeRoute
    from scen_runner.records.collision_archive import actor_state
    from localization_sim_session import SimulatorSession, foreign_ros_owners

    import uuid
    from localization_observer import ensure_dashboard,ExperimentReporter,private_master_uri
    private_master_uri(os.environ.get('ROS_MASTER_URI',''))
    preflight_master=rosgraph.Master('/localization_trial_preflight')
    if preflight_master.hasParam('/use_sim_time') and preflight_master.getParam('/use_sim_time'):
        raise RuntimeError('physical trials require wall ROS time; replay clock must be stopped first')
    rospy.init_node('localization_trial_'+uuid.uuid4().hex[:12], disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    monitoring=ensure_dashboard(args.root,args.dashboard_port,os.environ['ROS_MASTER_URI'])
    reporter=ExperimentReporter(output,registration['run_id'],args.trial,
        'physical_observer_diagnostic_not_driving_acceptance' if args.observer_contact_baseline else 'physical_trial',monitoring,
        conditions=dict(weather='Foggy',hour=13,planner=args.jamming_planner,
                        scene_label=args.preserve_scene_label,policy='user_pinned_foggy_20261029'))
    # Check ownership by ROS API PID and the current container's /proc. The
    # private master must not silently supply perception from another worktree.
    import xmlrpc.client
    class OwnershipTransport(xmlrpc.client.Transport):
        def make_connection(self,host):
            connection=super().make_connection(host);connection.timeout=1.;return connection
    required = ('/camera_objects', '/camera_lane', '/perception/lidar/lidar_pipeline')
    def perception_owners():
        owners={}
        for name in required:
            api=master.lookupNode(name)
            code,message,pid=xmlrpc.client.ServerProxy(api,transport=OwnershipTransport()).getPid(rospy.get_name())
            if code!=1 or not Path('/proc',str(pid)).exists():
                raise RuntimeError('perception producer is outside this container: '+name)
            owners[name]=dict(pid=pid,api=api)
        return owners
    if args.own_perception:
        existing={node for rows in master.getSystemState()[:2] for _,nodes in rows for node in nodes}
        if existing.intersection(required):raise RuntimeError('private perception already running; will not replace it')
    else:
        (output/'owners.json').write_text(json.dumps(perception_owners(),indent=2))
    checkout=Path(__file__).resolve().parents[2]
    source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=checkout,text=True).strip()
    patch=subprocess.check_output(['git','diff','HEAD','--binary'],cwd=checkout)
    (output/'source.patch').write_bytes(patch)
    files=[Path('/devel/lib/asmc_planner/asmc_shadow_localization_node'),
           Path('/devel/lib/asmc_planner/asmc_local_planner_node'),
           Path('/devel/lib/asmc_planner/asmc_control_selector_node'),
           Path('/devel/lib/asmc_planner/asmc_planner_node'),
           Path('/devel/lib/asmc_controller/asmc_local_controller_node'),
           Path('/devel/lib/asmc_controller/mpc_controller_node'),
           Path('/devel/lib/libasmc_shadow_localization_core.so'),
           checkout/'tools/scen_runner/.runtime/camera_runtime.json']
    files+=sorted(Path('/devel/lib').glob('lib*.so'))
    files+=[Path('/devel/lib/asmc_planner/localization_evaluate')]
    files+=[checkout/path for path in (
        'src/asmc_planner/config/shadow_localization.yaml',
        'src/asmc_planner/config/local_planner.yaml','src/asmc_planner/config/jamming_transition.yaml',
        'src/asmc_geometry/include/asmc_geometry/motion_evidence.hpp',
        'src/asmc_planner/config/localization_motion_calibration.json',
        'src/asmc_planner/config/frenet/params.yaml',
        'src/asmc_controller/config/controller.yaml',
        'src/asmc_controller/config/local_controller.yaml',
        'src/asmc_lidar/config/lidar_pipeline.yaml')]
    (output/'camera_runtime.json').write_bytes((checkout/'tools/scen_runner/.runtime/camera_runtime.json').read_bytes())
    (output/'provenance.json').write_text(json.dumps(dict(revision=source_revision,
        patch_sha256=hashlib.sha256(patch).hexdigest(),
        sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}),indent=2))
    children, logs, subscribers = [], [], []
    latest = {}
    udp_contacts=[]
    driving=[False]
    rows = (output/'events.jsonl').open('x')
    def emit(event, **data):
        rows.write(json.dumps(dict(event=event, monotonic=time.monotonic(),
                                   ros_time=rospy.Time.now().to_sec(), **data))+'\n')
        rows.flush()
    def observe(topic, kind):
        def callback(message):
            if hasattr(message, 'data'):
                value = message.data
                if isinstance(value, str):
                    try: value = json.loads(value)
                    except ValueError: pass
            elif kind is InferenceStatus:
                value=dict(model=message.model,loaded=message.loaded,warmed=message.warmed_up,
                           pipeline_ready=message.pipeline_ready,last_output_valid=message.last_output_valid,
                           mode=message.mode,successful_batches=message.successful_batches,errors=message.errors,
                           last_source_stamp=message.last_source_stamp.to_sec())
            elif kind is CollisionData:
                groups={field:[dict(unique_id=o.unique_id,type=o.type,name=o.name,
                            position=[o.position.x,o.position.y,o.position.z])
                            for o in getattr(message,field,[])]
                        for field in ('collision_object','collision_objecta')}
                value=dict(stamp=message.header.stamp.to_sec(),frame=message.header.frame_id,
                           global_offset=[message.global_offset_x,message.global_offset_y,message.global_offset_z],
                           phase='running' if driving[0] else 'startup_or_cleanup',**groups)
                if any(groups.values()):udp_contacts.append(value)
            elif kind is CtrlCmd:
                value = dict(steering=message.steering, accel=message.accel, brake=message.brake)
            elif kind is DiagnosticArray:
                status=next((s for s in message.status if s.name=='camera_bbox_lane_change'),None)
                if status is None:return
                value=dict(valid=status.level==0,reason=status.message,
                           values={entry.key:entry.value for entry in status.values},
                           source_stamp=message.header.stamp.to_sec())
            elif kind is LocalTrajectory:
                value = dict(valid=message.valid, reason=message.reason, sequence=message.plan_sequence)
            else:
                value = dict(valid=message.valid, x=message.x, y=message.y, speed=message.speed)
            latest[topic] = (time.monotonic(), value)
            emit('topic', topic=topic, value=value)
        subscribers.append(rospy.Subscriber(topic, kind, callback, queue_size=100, tcp_nodelay=True))
    for topic, kind in (
        ('/CollisionData',CollisionData),
        ('/ctrl_cmd', CtrlCmd), ('/planning/mode', UInt8),
        ('/planning/mode/status', String),
        ('/planning/jamming/approach',Bool),
        ('/planning/local/trace',String),('/planning/local/control_trace',String),
        ('/planning/local/trajectory',LocalTrajectory),
        ('/planning/camera_bbox_lane_change/diagnostics',DiagnosticArray),
        ('/planning/localization/anchor_ready', Bool),
        ('/planning/localization/trace', String),
        ('/planning/localization/debug', String),
        ('/planning/localization/experiment', String),
        ('/planning/jamming/handoff_trace', String),
        ('/planning/localization/control_trace', String),
        ('/planning/jamming/prepare', Bool),
        ('/planning/localization/approach_trace', String),
        ('/planning/localization/trajectory', LocalTrajectory),
        ('/planning/vehicle/state', VehicleState),
        ('/perception/camera/objects/status',InferenceStatus),
        ('/perception/camera/lane/status',InferenceStatus),
        ('/perception/camera/traffic_light/status',InferenceStatus)):
        observe(topic, kind)
    def start(command, name):
        log = (output/(name+'.log')).open('x'); logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process
    errors, contacts, startup_contacts = [], [], []
    section=None
    parked=None
    world_mutated=False
    rendering_changed=False
    reason, started, original = 'preparation_failed', None, None
    route = EpisodeRoute()
    from scen_runner.morai.signal_trial_coordinates import Coordinates
    from localization_trial_policy import offset_start
    coordinates=Coordinates([302595.,4124145.])
    x, y, z, yaw = route.curve.pose(args.start)
    pose = dict(x=x, y=y, z=z+route.cfg['section_height_offset_m'],
                roll=0., pitch=0., yaw=coordinates.enu_yaw_degrees(x,y,yaw))
    pose=offset_start(pose,args.lateral_offset,args.heading_offset)
    link = EpisodeLink(load_config())
    reporter.phase('waiting_for_lease')
    with SimulatorSession(output/'session', gps_fault=args.gps_outage,lease_wait=args.lease_wait_seconds):
        reporter.phase('preparing_scene')
        link.connect()
        from proto.morai.common.type_pb2 import Empty
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VehicleControlMode
        original = actor_state(link.ego())
        initial_states=link.states()
        control = link._require()._actor_stub.GetVehicleControlMode(link.ego_info(), timeout=3)
        weather = link._require()._environment_stub.GetWeather(Empty(), timeout=3)
        daytime = link._require()._environment_stub.GetTime(Empty(), timeout=3)
        (output/'sim-before.json').write_text(json.dumps(dict(
            ego=original, actors=[actor_state(s) for s in initial_states],
            control=MessageToDict(control), weather=MessageToDict(weather),
            time=MessageToDict(daytime)), indent=2))
        try:
            if args.gps_outage:
                start(['python3', str(Path(__file__).with_name('localization_gps_fault.py'))], 'gps_fault')
            from scen_runner.runtime.signal_trial_fixture import StaticFixture
            parked=StaticFixture(link,lambda row:emit(row.pop('event'),**row))
            existing={s.actor_info.id.value:s for s in initial_states}
            park_ids=list(args.park_prop)
            scene_ids=set()
            for aid in park_ids:
                if aid not in existing:raise RuntimeError('park fixture actor missing: '+aid)
                link.actor_scopes[aid]=existing[aid].actor_info.client_key
            for s in initial_states:
                if s.actor_info.id.value==link.ego_id:continue
                position=s.transform.location;station,error=route.curve.project(position.x,position.y)
                if args.start-5<=station<=args.end+10 and error<10 and s.actor_info.id.value not in park_ids:
                    if args.preserve_scene_label:
                        from localization_trial_policy import validate_preserved_static
                        validate_preserved_static(actor_state(s))
                        scene_ids.add(s.actor_info.id.value)
                    elif args.park_corridor and s.actor_info.object_type==3:
                        park_ids.append(s.actor_info.id.value)
                        link.actor_scopes[s.actor_info.id.value]=s.actor_info.client_key
                    else:raise RuntimeError('unowned actor in trial corridor: '+s.actor_info.id.value)
            if args.preserve_scene_label:
                scene=[actor_state(s) for s in initial_states if s.actor_info.id.value in scene_ids]
                emit('preserved_scene',label=args.preserve_scene_label,actors=scene,
                     scene_sha256=hashlib.sha256(json.dumps(scene,sort_keys=True).encode()).hexdigest(),
                     scope='user-loaded scene; observed only; native truth never enters planning')
            world_mutated=True
            link.reset(pose)
            if args.main_view_size:
                from proto.morai.simulator.rendering_pb2 import RenderingMode,WindowSize
                rendering_changed=True
                link.check(link._require()._simulator_stub.SetRenderingMode(
                    RenderingMode(enable=True,size=WindowSize(width=args.main_view_size[0],height=args.main_view_size[1])),
                    timeout=3),'temporary main viewport')
                emit('main_viewport',requested=args.main_view_size,restore=args.restore_view_size,
                     scope='temporary rendering-size request; sensor/network JSON files unchanged')
            for aid in park_ids:parked.park(aid,300.,0.)
            if args.fixture!='none':
                from scen_runner.records.collision_replay import ReplaySection
                import uuid
                from localization_fixtures import fixture_definition
                definition=fixture_definition(args.fixture,route,coordinates)
                (output/'fixture.json').write_text(json.dumps(definition,indent=2))
                section=ReplaySection('tunnel',42,'localization_'+uuid.uuid4().hex,load_config(),
                                      lambda:False,print,definition=definition)
                deadline=time.monotonic()+15
                while not section.poll_prepare():
                    if time.monotonic()>deadline:raise RuntimeError('owned fixture preparation timeout')
                    time.sleep(.1)
                emit('fixture_ready',owned=section.owned())
                owned_ids=set(section.owned())
                fixture_states=[actor_state(s) for s in link.states() if s.actor_info.id.value in owned_ids]
                if len(fixture_states)!=len(owned_ids):raise RuntimeError('owned fixture native snapshot incomplete')
                emit('fixture_native',actors=fixture_states,
                     scope='evaluation only; these native states are never planner inputs')
            link.set_environment('Foggy', 13)
            emit('environment_applied',requested_weather='Foggy',requested_hour=13,
                 weather=MessageToDict(link._require()._environment_stub.GetWeather(Empty(),timeout=3)),
                 time=MessageToDict(link._require()._environment_stub.GetTime(Empty(),timeout=3)))
            link.gear_ego(False); link.pause_ego(False)
            # This simulator suppresses status UDP in keyboard mode after a
            # scenario reload. Enable AUTO while still in P, before waiting
            # for CAN/anchor readiness. D is selected only at departure.
            request = VehicleControlModeParam(); request.actor_info.CopyFrom(link.ego_info())
            request.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE
            link.check(link._require()._actor_stub.SetVehicleControlMode(request, timeout=3), 'auto while parked')
            # Never confuse an inherited native collision marker with a new
            # episode, or silently ignore it. Wait in P before inference/control
            # startup, preserving the original marker and world snapshot.
            from localization_trial_policy import ClearContactWindow
            contact_window=ClearContactWindow();contact_deadline=time.monotonic()+10
            while True:
                contact_state=actor_state(link.ego())
                startup_contacts=[c for c in contact_state['native_contacts'] if c and c!=link.ego_id]
                emit('startup_contact_query',ego=contact_state,contacts=startup_contacts)
                if contact_window.observe(time.monotonic(),contact_state['native_contacts'],link.ego_id):break
                if time.monotonic()>contact_deadline:
                    if args.observer_contact_baseline:
                        emit('inherited_contact_diagnostic',contacts=startup_contacts,
                             scope='observer only; collision-free driving acceptance revoked')
                        break
                    raise RuntimeError('startup_contacts_not_clear')
                time.sleep(.1)
            if args.own_perception:
                reporter.phase('warming_inputs')
                start(['roslaunch','asmc_camera','camera.launch'],'camera')
                start(['roslaunch','asmc_lidar','lidar.launch','debug:='+str(args.lidar_debug).lower(),
                       'camera_calibration_config:=/root/ws/src/asmc_camera/config/cameras_v3.yaml'],'lidar')
                deadline=time.monotonic()+45
                while True:
                    try:
                        owners=perception_owners();break
                    except (rosgraph.MasterError,OSError,xmlrpc.client.Error):
                        if time.monotonic()>deadline:raise RuntimeError('owned perception startup timeout')
                        if any(p.poll() is not None for p in children):raise RuntimeError('perception child exited')
                        time.sleep(.1)
                (output/'owners.json').write_text(json.dumps(owners,indent=2))
            # All parameter overrides exist before any node starts. The normal
            # production speed profile stays untouched on disk.
            wrapper = output/'trial.launch'
            wrapper.write_text('''<launch>
  <include file="$(find asmc_planner)/launch/autonomy_no_bridges_v2.launch">
    <arg name="jamming_planner" value="%s"/>
    <arg name="dashboard" value="false"/>
    <arg name="shadow_run_id" value="%s"/>
    <arg name="jamming_speed_kph" value="%s"/>
    <arg name="shadow_static_avoidance" value="%s"/>
    <arg name="shadow_dynamic_obstacles" value="%s"/>
    <arg name="shadow_lane_rules" value="%s"/>
    <arg name="shadow_motion_yaw_source" value="%s"/>
    <arg name="shadow_motion_reference_offset_m" value="%s"/>
    <arg name="shadow_motion_turn_gain_m2ps" value="%s"/>
    <arg name="shadow_journal_path" value="%s"/>
  </include>
  <param name="/behavior_planner/maximum_speed_kph" value="%s"/>
  <param name="/mpc_controller/validation_speed_cap_kph" value="%s"/>
  <param name="/localization/trial_id" value="%s"/>
  <param name="/localization_motion/speed_callerid" value="/morai_udp_bridge"/>
</launch>\n''' % (args.jamming_planner,escape(registration['run_id']), args.cap, str(args.static_avoidance).lower(), str(args.dynamic_obstacles).lower(), str(args.shadow_lane_rules).lower(), args.yaw_source, args.motion_reference_offset, args.motion_turn_gain, escape(str(output/'decisions.jsonl')), normal_ceiling,normal_cap,escape(args.trial)))
            bag_topics = [t for t, k in (
                ('/gps', None), ('/simulation/raw/gps', None), ('/simulation/gps_fault', None), ('/imu', None), ('/Ego_topic', None),
                ('/planning/localization/motion', None),
                ('/planning/localization/trajectory', None),
                ('/planning/localization/trace', None),
                ('/planning/localization/debug', None),
                ('/planning/localization/experiment', None),
                ('/planning/localization/control_trace', None),
                ('/planning/jamming/prepare', None),
                ('/planning/jamming/approach',None),('/planning/jamming/handoff_request',None),('/planning/jamming/handoff_candidate',None),
                ('/planning/local/trace',None),('/planning/local/control_trace',None),('/planning/local/trajectory',None),('/planning/local/odometry',None),
                ('/planning/camera_bbox_lane_change/diagnostics',None),('/planning/camera_bbox_lane_change/markers',None),
                ('/perception/camera/lane_boundary_family',None),('/perception/lidar/observations_local',None),('/planning/objects',None),
                ('/experimental/shadow/semantic_space_trial/context',None),('/experimental/shadow/semantic_space_trial/coverage',None),('/experimental/shadow/semantic_space_trial/drivable_probability',None),
                ('/planning/jamming/handoff_request', None),
                ('/planning/jamming/handoff_candidate', None),
                ('/planning/jamming/handoff_trace', None),
                ('/planning/mode', None), ('/planning/mode/status', None),
                ('/planning/vehicle/state', None), ('/planning/trajectory', None),
                ('/behavior/command_v2', None), ('/planning/frenet_feedback_v2', None),
                ('/perception/lidar/tracks_local', None),
                ('/perception/lidar/diagnostics', None),
                ('/perception/camera/objects/status', None),
                ('/perception/camera/lane/status', None), ('/perception/camera/traffic_light/status', None),
                ('/morai_sensor_bridge/frame_timing', None),
                ('/ctrl_cmd', None), ('/ctrl_cmd/normal', None), ('/ctrl_cmd/jamming', None),('/CollisionData',None))]
            if args.static_avoidance or args.own_perception:
                # Raw sensor evidence is retained for later reproduction.
                bag_topics+=['/velodyne_points','/perception/camera/objects','/perception/lidar/trace',
                    '/perception/camera/objects/observations',
                    '/image_jpeg/compressed','/cam_left/image_jpeg/compressed',
                    '/cam_right/image_jpeg/compressed','/cam_extra/image_jpeg/compressed']
            import roslib.packages
            bag_topics=list(dict.fromkeys(bag_topics))
            record_binary=roslib.packages.find_node('rosbag','record')[0]
            # Full raw LiDAR + four JPEG streams can exceed 64MiB during D
            # write stalls. Keep every original topic; lossless LZ4 and a
            # bounded 1GiB queue preserve evidence without a second recorder.
            start([record_binary, '--buffsize=1024', '--chunksize=4096', '--lz4', '--tcpnodelay', '-O',
                   str(output/'critical'), *bag_topics], 'record')
            start(['roslaunch', str(wrapper)], 'autonomy')
            deadline = time.monotonic()+10
            while True:
                now = time.monotonic()
                anchor = latest.get('/planning/localization/anchor_ready', (0, False))
                command = latest.get('/ctrl_cmd', (0, {}))
                state = latest.get('/planning/vehicle/state', (0, {}))
                model_states=[latest.get('/perception/camera/'+name+'/status',(0,{}))[1]
                              for name in ('objects','lane','traffic_light')]
                models_ready=all(s.get('loaded') and s.get('warmed') and s.get('successful_batches',0)>0 and
                                 -.05<=rospy.Time.now().to_sec()-s.get('last_source_stamp',0)<=.75
                                 for s in model_states)
                anchor_ready=args.jamming_planner!='localization' or (anchor[1] and now-anchor[0]<.2)
                if (models_ready and anchor_ready and now-command[0] < .2 and
                    state[1].get('valid') and command[1].get('brake', 1) < .9): break
                if now > deadline: raise RuntimeError('startup_readiness_timeout')
                if any(p.poll() is not None for p in children): raise RuntimeError('child_exit')
                time.sleep(.05)
            emit("perception_ready",models=model_states)
            if args.main_view_size:
                # A viewport experiment must preserve all sensor image sizes.
                # This one-frame preflight audit is not another inference path.
                import cv2,numpy as np
                from sensor_msgs.msg import CompressedImage
                preset=checkout/'config/morai/sensors/261027_ASMC_Local_Sensor_25.S4.MolitComp03.json'
                cameras=json.loads(preset.read_text())['cameraList']
                topics=('/image_jpeg/compressed','/cam_left/image_jpeg/compressed',
                        '/cam_right/image_jpeg/compressed','/cam_extra/image_jpeg/compressed')
                for camera,topic in zip(cameras,topics):
                    image=rospy.wait_for_message(topic,CompressedImage,timeout=3.)
                    decoded=cv2.imdecode(np.frombuffer(image.data,dtype=np.uint8),cv2.IMREAD_COLOR)
                    expected=(camera['cc']['cameraResHeight'],camera['cc']['cameraResWidth'])
                    if decoded is None or decoded.shape[:2]!=expected:
                        raise RuntimeError('viewport changed sensor dimensions: '+topic)
                    emit('viewport_sensor_size',topic=topic,height=expected[0],width=expected[1],
                         preset_sha256=hashlib.sha256(preset.read_bytes()).hexdigest())
            contact_state=actor_state(link.ego())
            startup_contacts=[c for c in contact_state['native_contacts'] if c and c!=link.ego_id]
            emit('departure_contact_query',ego=contact_state,contacts=startup_contacts)
            if startup_contacts and not args.observer_contact_baseline:raise RuntimeError('startup_contacts_not_clear')
            (output/'runtime-parameters.json').open('x').write(json.dumps(master.getParam('/'),indent=2))
            request = VehicleControlModeParam(); request.actor_info.CopyFrom(link.ego_info())
            request.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE
            link.check(link._require()._actor_stub.SetVehicleControlMode(request, timeout=3), 'auto mode')
            link.release(); started = stationary = time.monotonic()
            if section:section.resume_schedule(started)
            last_fixture_tick=0.
            driving[0]=True
            reporter.phase('running')
            reason = 'time_limit'
            last_owner_check=0.
            emit('departure', start_station=args.start, end_station=args.end, cap_kph=args.cap,
                 lateral_offset_m=args.lateral_offset,heading_offset_deg=args.heading_offset,
                 start_pose=pose,heading_contract='native ENU from fixed-reference UTM grid')
            transition_mode=None;transition_started=0.;transition_speed=0.
            while time.monotonic()-started < args.seconds:
                now = time.monotonic()
                if section and now-last_fixture_tick>=.1:
                    last_fixture_tick=now
                    fixture_live=section.link.states();section.tick(fixture_live)
                    owned=set(section.owned())
                    emit('fixture_motion',actors=[actor_state(s) for s in fixture_live if s.actor_info.id.value in owned],
                         scope='scene orchestration/evaluation only')
                elif args.preserve_scene_label and now-last_fixture_tick>=.1:
                    last_fixture_tick=now
                    scene_live=[actor_state(s) for s in link.states() if s.actor_info.id.value in scene_ids]
                    emit('fixture_motion',actors=scene_live,scope='preserved scene observation only')
                    if {s['id'] for s in scene_live}!=scene_ids:
                        reason='preserved_scene_actor_missing';break
                if now-last_owner_check>=.5:
                    last_owner_check=now
                    if foreign_ros_owners(): reason='foreign_physical_owner'; break
                if any(p.poll() is not None for p in children): reason='child_exit'; break
                if (args.root/'stop-request.json').exists(): reason='external_stop'; break
                if os.statvfs(output).f_bavail*os.statvfs(output).f_frsize < storage_floor:
                    reason='disk_floor'; break
                state = actor_state(link.ego()); point=state['pose']
                station, error = route.curve.project(point['x'], point['y'])
                _, _, _, heading = route.curve.pose(station)
                angle=abs(math.degrees(math.remainder(math.radians(point['yaw'])-heading, 2*math.pi)))
                mode=latest.get('/planning/mode',(None,None))[1]
                from localization_trial_policy import native_speed_limit
                limit=native_speed_limit(mode,args.cap,normal_ceiling)
                if mode!=transition_mode:
                    transition_mode=mode;transition_started=now;transition_speed=state['speed_kmh']
                    emit('control_mode_speed_transition',mode=mode,station=station,speed_kph=transition_speed)
                if mode==1 and limit is not None:
                    limit=max(limit,transition_speed-3.6*max(0.,now-transition_started-.5)+1.)

                emit('native', ego=state, station=station, lateral_m=error, heading_deg=angle,
                     selected_mode=mode,native_speed_limit_kph=limit)
                contacts.extend(x for x in state['native_contacts'] if x and x != link.ego_id)
                collision=latest.get('/CollisionData')
                if collision and now-collision[0]<=.2:
                    from localization_trial_policy import udp_contact_classification
                    verdict=udp_contact_classification(collision[1],state,rospy.Time.now().to_sec())
                    emit('udp_contact_evidence',verdict=verdict,source_stamp=collision[1]['stamp'],native_ego=state)
                    if verdict not in ('empty','position_matched_self_status'):
                        reason='udp_collision_event_requires_review';break
                if contacts and (not args.observer_contact_baseline or set(contacts)-set(startup_contacts)):
                    reason='contact'; break
                if error > (5. if args.fixture!='none' or args.preserve_scene_label else 1.5): reason='route_departure'; break
                if limit is None:reason='unknown_control_mode';break
                if state['speed_kmh'] > limit: reason='overspeed'; break
                if state['speed_kmh'] > .36: stationary=now
                if now-stationary > 12: reason='stationary_12s'; break
                if station >= args.end: reason='completed'; break
                time.sleep(.05)
        except BaseException as error:
            reason='interrupted' if isinstance(error,KeyboardInterrupt) else str(error)
            emit('exception', error=repr(error))
        finally:
            driving[0]=False
            reporter.phase('restoring',reason)
            try:
                if world_mutated and not foreign_ros_owners(): link.hold()
            except Exception as error: errors.append('hold: '+str(error))
            for process in reversed(children):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM); process.wait(5)
            # The rosbag wrapper can exit before its record child flushes and
            # renames .bag.active. Keep the lease until raw recording closes.
            recording_deadline=time.monotonic()+15
            while list(output.glob('*.bag.active')) and time.monotonic()<recording_deadline:
                time.sleep(.1)
            if list(output.glob('*.bag.active')):errors.append('raw recording finalization timeout')
            try:
                if section:section.cleanup()
            except Exception as error:errors.append('fixture cleanup: '+str(error))
            try:
                if world_mutated:
                    if foreign_ros_owners():
                        raise RuntimeError('foreign physical owner appeared; own nodes stopped, original snapshot retained for coordinated restore')
                    if parked:parked.restore()
                    link.reset(original['pose']); link.pause_ego(False)
                    request = VehicleControlModeParam(); request.actor_info.CopyFrom(link.ego_info())
                    request.mode=control.mode
                    link.check(link._require()._actor_stub.SetVehicleControlMode(request, timeout=3), 'restore control')
                    # User pinned Foggy for this campaign. The original
                    # Cloudy snapshot is evidence, not the post-trial policy.
                    from proto.morai.environment.weather_pb2 import Weather
                    from proto.morai.environment.environment_enum_pb2 import WEATHER_TYPE_FOGGY
                    daytime.instantly=True; daytime.data_only=False
                    link.check(link._require()._environment_stub.SetWeather(
                        Weather(type=WEATHER_TYPE_FOGGY,instantly=True),timeout=3),'retain requested Foggy weather')
                    link.check(link._require()._environment_stub.SetTime(daytime, timeout=3), 'restore time')
                    actual_weather=link._require()._environment_stub.GetWeather(Empty(),timeout=3)
                    if actual_weather.type!=WEATHER_TYPE_FOGGY:raise RuntimeError('Foggy weather readback mismatch after cleanup')
                    emit('environment_after_cleanup',weather=MessageToDict(actual_weather),policy='user_pinned_foggy_20261029')
                    (output/'sim-after.json').write_text(json.dumps(actor_state(link.ego()), indent=2))
            except Exception as error: errors.append('restore: '+str(error))
            try:
                if rendering_changed:
                    if foreign_ros_owners():raise RuntimeError('foreign owner appeared; viewport snapshot retained')
                    from proto.morai.simulator.rendering_pb2 import RenderingMode,WindowSize
                    link.check(link._require()._simulator_stub.SetRenderingMode(
                        RenderingMode(enable=True,size=WindowSize(width=args.restore_view_size[0],height=args.restore_view_size[1])),
                        timeout=3),'restore main viewport')
                    emit('main_viewport_restored',requested=args.restore_view_size)
            except Exception as error:errors.append('viewport restore: '+str(error))
            link.disconnect()
    for subscriber in subscribers: subscriber.unregister()
    emit('finish', reason=reason, contacts=contacts, cleanup_errors=errors)
    rows.close()
    for log in logs: log.close()
    result=dict(reason=reason, contacts=contacts, cleanup_errors=errors, cap_kph=args.cap,
                elapsed_s=time.monotonic()-started if started else 0,
                bag_active=list(map(str,output.glob('*.active'))),
                static_avoidance=args.static_avoidance,fixture=args.fixture,yaw_source=args.yaw_source,perception_lifecycle="lease_owned" if args.own_perception else "reused")
    result['dynamic_obstacles']=args.dynamic_obstacles
    result['shadow_lane_rules']=args.shadow_lane_rules
    result['recording_policy']=dict(buffer_mib=1024,chunk_kib=4096,compression='lz4_lossless',raw_topics_retained=True)
    result['conditions']=dict(weather='Foggy',hour=13,after_cleanup_weather='Foggy',policy='user_pinned_foggy_20261029')
    result['motion_reference_offset_m']=args.motion_reference_offset
    result['normal_validation_speed_cap_kph']=normal_cap
    result['jamming_contract']='A448 common approach25; explicit option cap; transition overspeed envelope recorded'
    result['minimum_free_bytes']=storage_floor
    result['normal_speed_ceiling_kph']=normal_ceiling
    result['startup_contacts']=startup_contacts
    result['udp_collision_observations']=dict(nonempty_messages=len(udp_contacts),active_messages=sum(c['phase']=='running' for c in udp_contacts),raw_evidence='events.jsonl and critical.bag')
    result['collision_evidence_policy']='raw UDP retained; only time/position-matched type=-1/id=0 ego status is neutral; unresolved active evidence stops for joint review'
    result['motion_turn_gain_m2ps']=args.motion_turn_gain
    result['planner']=args.jamming_planner
    result['preserved_scene_label']=args.preserve_scene_label
    result['main_view_size']=args.main_view_size
    result['restore_view_size']=args.restore_view_size
    result['monitoring']=monitoring
    result['lidar_debug_enabled']=args.lidar_debug
    result['diagnostic_geometry_recorded']=True
    result['observer_contact_baseline']=args.observer_contact_baseline
    result['driving_acceptance_eligible']=not args.observer_contact_baseline
    (output/'result.json').write_text(json.dumps(result, indent=2))
    reporter.finish('completed' if reason=='completed' and not errors else 'failed',reason)
    print(json.dumps(result), flush=True)


if __name__ == '__main__': main()
