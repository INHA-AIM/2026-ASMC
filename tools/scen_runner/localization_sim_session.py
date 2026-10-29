"""Exclusive physical simulator session for an already-running private ROS master."""
import os,signal,socket,subprocess,time
from pathlib import Path
from scen_runner.runtime.validation_lease import ValidationLease,ValidationBusy


def foreign_ros_owners():
    """A free advisory lease does not imply an idle manually started bridge."""
    import xmlrpc.client
    class Transport(xmlrpc.client.Transport):
        def make_connection(self,host):
            connection=super().make_connection(host);connection.timeout=.3;return connection
    owners={}
    for port in (11311,11348):
        try:
            rpc=xmlrpc.client.ServerProxy('http://127.0.0.1:'+str(port),transport=Transport())
            code,_,state=rpc.getSystemState('/localization_foreign_owner_guard')
        except (OSError,xmlrpc.client.Error):continue
        if code!=1:raise RuntimeError('cannot inspect foreign master '+str(port))
        if port!=11311:
            # Independent bag masters may publish recorded commands. They do
            # not own physical transport unless an actual bridge is present.
            nodes={n for topic,ns in state[0]+state[1] for n in ns}
            if not nodes.intersection({'/morai_sensor_bridge','/morai_udp_bridge'}):continue
        for topic,nodes in state[0]:
            if topic not in ('/ctrl_cmd','/ctrl_cmd/normal','/ctrl_cmd/jamming','/planning/mode','/gps','/velodyne_points'):continue
            for node in nodes:
                code,_,uri=rpc.lookupNode('/localization_foreign_owner_guard',node)
                if code!=1:continue
                try:
                    response=xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid('/localization_foreign_owner_guard')
                except (OSError,xmlrpc.client.Error):continue
                if response[0]==1:owners.setdefault(str(port),{}).setdefault(topic,[]).append(node)
    return owners


class SimulatorSession:
    def __init__(self,output,gps_fault=False,lease_wait=0):
        self.gps_fault=gps_fault;self.output=Path(output);self.leases=[];self.process=None;self.log=None
        self.lease_wait=lease_wait
    def __enter__(self):
        import rosgraph
        uri=os.environ.get('ROS_MASTER_URI','')
        if uri in ('','http://127.0.0.1:11311','http://localhost:11311'):
            raise RuntimeError('automated simulation requires a private ROS master')
        self.master=rosgraph.Master('/localization_sim_session');self.master.getPid()
        directory=os.environ.get('ASMC_VALIDATION_LEASE_DIR')
        if not directory:raise RuntimeError('shared physical lease directory required')
        try:
            deadline=time.monotonic()+self.lease_wait
            notified=False
            while True:
                try:
                    for host in ('host.docker.internal','127.0.0.1'):
                        lease=ValidationLease(host,7789,directory);lease.acquire();self.leases.append(lease)
                    break
                except ValidationBusy:
                    for lease in reversed(self.leases):lease.release()
                    self.leases.clear()
                    if time.monotonic()>=deadline:raise
                    if not notified:print('Waiting for the shared physical lease; no nodes or scene changes started.',flush=True);notified=True
                    time.sleep(.2)
            foreign=foreign_ros_owners()
            if foreign:raise RuntimeError('foreign physical bridge/control active: '+str(foreign))
            occupied={t:n for t,n in self.master.getSystemState()[0] if n and t in
                      ('/ctrl_cmd','/planning/mode','/gps','/velodyne_points')}
            if occupied:raise RuntimeError('private graph already owns simulation input/control: '+str(occupied))
            # Test all receiver sockets together; never kill another UDP owner.
            sockets=[]
            try:
                for port in (9099,9798,9092,9291,9293,9295,9297,9299,9301,9303):
                    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',port));sockets.append(s)
            finally:
                for s in sockets:s.close()
            self.output.mkdir(exist_ok=False)
            self.log=(self.output/'bridge.log').open('x')
            command=['roslaunch','--wait','asmc_bridge','bridge.launch',
                'cmd_ip:=127.0.0.1','runtime_profile:=morai_udp','transport_profile:=local',
                'stream_profile:=competition','competition_status_required:=false']
            if self.gps_fault:
                wrapper=self.output/'raw-bridge.launch'
                wrapper.write_text('''<launch><group><remap from="/gps" to="/simulation/raw/gps"/>
<include file="$(find asmc_bridge)/launch/bridge.launch">
<arg name="cmd_ip" value="127.0.0.1"/><arg name="runtime_profile" value="morai_udp"/>
<arg name="transport_profile" value="local"/><arg name="stream_profile" value="competition"/>
<arg name="competition_status_required" value="false"/>
</include></group></launch>''')
                command=['roslaunch','--wait',str(wrapper)]
            self.process=subprocess.Popen(command,
                stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+12
            while time.monotonic()<end:
                if self.process.poll() is not None:raise RuntimeError('private bridge exited')
                topics=dict(self.master.getSystemState()[0])
                if all(topics.get(t) for t in (('/simulation/raw/gps' if self.gps_fault else '/gps'),'/imu','/Ego_topic','/velodyne_points')):return self
                time.sleep(.1)
            raise RuntimeError('private bridge startup timeout')
        except BaseException as error:
            import json
            (self.output.parent/'session-preflight-error.json').write_text(json.dumps(dict(error=str(error),physical_mutations=False),indent=2))
            self.__exit__(None,None,None);raise
    def __exit__(self,*args):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(8)
            except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(5)
        if self.log:self.log.close()
        for lease in reversed(self.leases):lease.release()
