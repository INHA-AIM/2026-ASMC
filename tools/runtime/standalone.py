#!/usr/bin/env python3
"""One container-owned driving session. No existing master or bridge is reused."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[2]


def ports_from_params(params, master_port):
    """Use the expanded canonical launch values, not a second sensor port list."""
    udp=[]
    def enabled(prefix,flag,port):
        if params.get(prefix+'/'+flag,False):udp.append(int(params[prefix+'/'+port]))
    net='/morai_udp_bridge';sensor='/morai_sensor_bridge'
    for flag,port in [('enable_ego_status','ego_port'),('enable_competition_status','competition_port'),
                      ('enable_object','object_port'),('enable_collision','collision_port')]:
        enabled(net,flag,port)
    for flag,port in [('enable_lidar','lidar_port'),('enable_instance_lidar','instance_lidar_port'),
                      ('enable_gps','gps_port'),('enable_imu','imu_port')]:
        enabled(sensor,flag,port)
    for flag,key in [('enable_camera','cameras'),('enable_semantic_camera','semantic_cameras')]:
        if params.get(sensor+'/'+flag,False):
            udp.extend(int(row['port']) for row in params[sensor+'/'+key])
    http=[int(v) for k,v in params.items() if k in
          ('/behavior_dashboard_server/port','/perception/lidar/lidar_monitor/port')]
    if len(udp)!=len(set(udp)):raise ValueError('Duplicate UDP receive ports in expanded launch')
    if len(http)!=len(set(http)) or master_port in http:raise ValueError('Duplicate master/web ports')
    if any(not 1<=p<=65535 for p in udp+http+[master_port]):raise ValueError('Invalid runtime port')
    return dict(master=int(master_port),udp=sorted(udp),http=sorted(http),
                command_destination=dict(ip=params.get(net+'/cmd_ip'),port=params.get(net+'/cmd_port')))


def require_tcp_free(ports, workspace=None, reclaim_idle=False):
    for port in ports:
        for attempt in range(21):
            with socket.socket() as probe:
                try:probe.bind(('0.0.0.0',port));break
                except OSError as exc:
                    if attempt==0 and reclaim_idle and workspace is not None:
                        sys.path.insert(0,str(Path(workspace)/'src/asmc_planner/scripts'))
                        from dashboard_startup import release_idle_trial_monitor
                        if release_idle_trial_monitor(port,workspace):
                            time.sleep(.05);continue
                    elif attempt and reclaim_idle:
                        time.sleep(.05);continue
                    raise RuntimeError('TCP port already owned: %d. Existing processes were preserved.'%port) from exc
        else:raise RuntimeError('TCP port was not released: %d'%port)


def identity(workspace,session,port,ports):
    def git(*args):
        try:return subprocess.check_output(['git','-c','safe.directory='+str(workspace),*args],cwd=workspace,
                    stderr=subprocess.DEVNULL,text=True,timeout=2).strip()
        except (OSError,subprocess.SubprocessError):return 'unavailable'
    return dict(protocol='asmc-standalone-v1',session_id=session,run_id=session,
                workspace=str(workspace),branch=git('branch','--show-current'),revision=git('rev-parse','HEAD'),
                dirty=bool(git('status','--porcelain')),master_uri='http://127.0.0.1:%d'%port,
                ports=ports,container=os.environ.get('HOSTNAME','unknown'),phase='running')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-port',type=int,default=int(os.environ.get('ASMC_MASTER_PORT','11311')))
    parser.add_argument('--check',action='store_true',help='Expand launch and check ports without starting nodes')
    args,launch_args=parser.parse_known_args()
    if any(':=' not in a for a in launch_args):parser.error('Pass launch arguments as name:=value')
    if not 1<=args.master_port<=65535:parser.error('Invalid master port')
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.master_port
    os.environ['ROS_HOSTNAME']='127.0.0.1';os.environ.pop('ROS_IP',None)
    import roslaunch.config
    launch=WORKSPACE/'src/asmc_planner/launch/autonomy.launch'
    config=roslaunch.config.load_config_default([(str(launch),launch_args)],args.master_port)
    names=[n.namespace+n.name for n in config.nodes]
    if len(names)!=len(set(names)):raise ValueError('Duplicate node names in complete runtime')
    params={name:p.value for name,p in config.params.items()}
    ports=ports_from_params(params,args.master_port)
    sys.path.insert(0,str(WORKSPACE/'tools/scen_runner'))
    from scen_runner.runtime.private_master import PrivateMaster,require_udp_ports_free
    require_tcp_free([args.master_port])
    require_tcp_free(ports['http'],WORKSPACE,reclaim_idle=not args.check)
    require_udp_ports_free(ports['udp'])
    if args.check:print(json.dumps(dict(workspace=str(WORKSPACE),ports=ports,nodes=names),ensure_ascii=False));return 0
    from scen_runner.runtime.validation_lease import ValidationLease
    lease=ValidationLease(os.environ.get('ASMC_SIMULATOR_HOST','host.docker.internal'),
                          int(os.environ.get('ASMC_SIMULATOR_PORT','7789')))
    session='standalone-'+uuid.uuid4().hex[:12]
    # These are standard ROS process logs, not sensor recording/experiment output.
    output=Path(os.environ.get('ROS_LOG_DIR',str(Path.home()/'.ros/log')))/session
    output.mkdir(parents=True,exist_ok=False)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        lease.acquire()
        # Check again after acquiring the shared physical-resource lease.
        require_udp_ports_free(ports['udp']);require_tcp_free(ports['http'])
        with PrivateMaster(output,args.master_port,strict_port=True):
            meta=identity(WORKSPACE,session,args.master_port,ports)
            os.environ['ASMC_RUNTIME_SESSION_ID']=session
            os.environ['ASMC_RUNTIME_SESSION']=json.dumps(meta)
            import rosgraph
            rosgraph.Master('/standalone_runtime_owner').setParam('/runtime/session',meta)
            (output/'session.json').write_text(json.dumps(meta,indent=2))
            print('STANDALONE '+json.dumps(meta,ensure_ascii=False),flush=True)
            print('Planning http://localhost:%s/web/index_v2.html | LiDAR http://localhost:%s/' %
                  (params['/behavior_dashboard_server/port'],params.get('/perception/lidar/lidar_monitor/port','off')),flush=True)
            child=subprocess.Popen(['roslaunch',str(launch),*launch_args],start_new_session=True)
            try:
                return child.wait()
            finally:
                # Keep our master alive until its ROS nodes have shut down.
                # Never find/kill processes by port or broad process name.
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    except KeyboardInterrupt:return 0
    finally:
        lease.release()


if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:print('Standalone start refused: '+str(exc),file=sys.stderr);sys.exit(2)
