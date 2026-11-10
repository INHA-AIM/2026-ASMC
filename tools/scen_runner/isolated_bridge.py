#!/usr/bin/env python3
"""Relay raw bridge input into a private master. Control requires a physical lease.

Agent tool only. Interactive camera/lidar/autonomy launches use the shared master.
Raw bytes, source stamps, types and MD5s are preserved. No perception output is shared.
"""
import argparse,json,os,signal,struct,subprocess,sys,threading,time,uuid
from pathlib import Path

RAW = ('/gps','/gps/fix','/imu','/Ego_topic','/velodyne_points',
       '/image_jpeg/compressed','/cam_left/image_jpeg/compressed',
       '/cam_right/image_jpeg/compressed','/cam_extra/image_jpeg/compressed',
       '/tf_static','/CollisionData')


def read_exact(stream,n):
    data=bytearray()
    while len(data)<n:
        part=stream.read(n-len(data))
        if not part:return None
        data.extend(part)
    return bytes(data)


def worker(args):
    import rospy,rosgraph,roslib.message
    output=sys.stdout.buffer;sys.stdout=sys.stderr
    rospy.init_node('localization_'+args.worker+'_'+args.token,disable_signals=True,disable_rosout=True)
    master=rosgraph.Master(rospy.get_name());master.getPid()
    if args.worker=='export':
        pending={};condition=threading.Condition();counts={};subs=[]
        topics=('/ctrl_cmd',) if args.controls else RAW
        def callback(msg,topic):
            h=msg._connection_header
            if args.controls and h.get('callerid')!='/asmc_control_selector':return
            item=(dict(topic=topic,type=h['type'],md5=h['md5sum'],latch=h.get('latching')=='1',
                       producer=h.get('callerid'),received=time.monotonic()),msg._buff)
            with condition:pending[topic]=item;condition.notify()
        for topic in topics:
            subs.append(rospy.Subscriber(topic,rospy.AnyMsg,callback,callback_args=topic,queue_size=1,buff_size=32*1024*1024))
        while not rospy.is_shutdown():
            with condition:
                if not pending:condition.wait(.1)
                items=list(pending.values());pending.clear()
            for meta,payload in items:
                header=json.dumps(meta,separators=(',',':')).encode()
                output.write(struct.pack('!II',len(header),len(payload))+header+payload);output.flush()
        return
    publishers={};last_control=0.;done=threading.Event();failure=[]
    def guard():
        from morai_msgs.msg import CtrlCmd
        while not done.wait(.05):
            if not args.controls:continue
            try:
                pubs=dict(master.getSystemState()[0]);foreign=set(pubs.get('/ctrl_cmd',[]))-{rospy.get_name()}
                if foreign:raise RuntimeError('foreign control publisher: '+repr(sorted(foreign)))
                if last_control and time.monotonic()-last_control>.25:
                    publishers['/ctrl_cmd'].publish(CtrlCmd(longlCmdType=1,brake=1.))
            except BaseException as e:
                failure.append(repr(e));os._exit(4)
    thread=threading.Thread(target=guard,daemon=True);thread.start()
    try:
        while not rospy.is_shutdown():
            frame=read_exact(sys.stdin.buffer,8)
            if frame is None:break
            nh,nb=struct.unpack('!II',frame)
            if nh>65536 or nb>32*1024*1024:raise ValueError('oversized relay frame')
            meta=json.loads(read_exact(sys.stdin.buffer,nh));data=read_exact(sys.stdin.buffer,nb)
            allowed=('/ctrl_cmd',) if args.controls else RAW
            if meta['topic'] not in allowed:raise ValueError('non-raw relay input rejected')
            if args.controls and time.monotonic()-meta['received']>.1:continue
            cls=roslib.message.get_message_class(meta['type'])
            if cls is None or cls._md5sum!=meta['md5']:raise RuntimeError('relay type/MD5 mismatch: '+meta['topic'])
            topic=meta['topic']
            if topic not in publishers:publishers[topic]=rospy.Publisher(topic,cls,queue_size=1,latch=meta['latch'])
            message=cls();message.deserialize(data);publishers[topic].publish(message)
            if args.controls:last_control=time.monotonic()
    finally:
        done.set()
        if args.controls and '/ctrl_cmd' in publishers:
            from morai_msgs.msg import CtrlCmd
            foreign=set(dict(master.getSystemState()[0]).get('/ctrl_cmd',[]))-{rospy.get_name()}
            if not foreign:
                for _ in range(3):publishers['/ctrl_cmd'].publish(CtrlCmd(longlCmdType=1,brake=1.));time.sleep(.05)
        rospy.signal_shutdown('relay ended')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shared-master',default='http://127.0.0.1:11311')
    p.add_argument('--isolated-master',default='http://127.0.0.1:11371')
    p.add_argument('--control',action='store_true')
    p.add_argument('--lease-directory',type=Path,default=Path(os.environ.get('ASMC_VALIDATION_LEASE_DIR','/validation-lease')))
    p.add_argument('--output',type=Path)
    p.add_argument('--worker',choices=['export','import'],help=argparse.SUPPRESS)
    p.add_argument('--controls',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--token',default='',help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.worker:return worker(args)
    if args.shared_master==args.isolated_master:raise RuntimeError('two distinct masters required')
    import rosgraph
    from scen_runner.runtime.validation_lease import ValidationLease
    masters=[rosgraph.Master('/localization_relay_check',master_uri=uri) for uri in (args.shared_master,args.isolated_master)]
    for m in masters:m.getPid()
    # Input relay refuses an existing producer in the private graph as well.
    private_pubs=dict(masters[1].getSystemState()[0])
    if any(private_pubs.get(t) for t in RAW):raise RuntimeError('private raw input already owned')
    if args.control and dict(masters[0].getSystemState()[0]).get('/ctrl_cmd'):raise RuntimeError('shared control already owned')
    if args.output is None or not (args.output.parent/'registration.json').is_file():raise RuntimeError('fresh registered-run output required')
    args.output.mkdir(exist_ok=False);processes=[];leases=[];logs=[];token=uuid.uuid4().hex
    def pair(source,destination,control=False):
        flags=['--controls'] if control else []
        err=(args.output/('control' if control else 'raw')).with_suffix('.log').open('x');logs.append(err)
        env=dict(os.environ,ROS_MASTER_URI=source,ROS_HOSTNAME='127.0.0.1');env.pop('ROS_IP',None)
        send=subprocess.Popen([sys.executable,__file__,'--worker','export','--token',token,*flags],env=env,stdout=subprocess.PIPE,stderr=err,start_new_session=True)
        env=dict(env,ROS_MASTER_URI=destination)
        receive=subprocess.Popen([sys.executable,__file__,'--worker','import','--token',token,*flags],env=env,stdin=send.stdout,stdout=err,stderr=err,start_new_session=True)
        send.stdout.close();processes.extend([send,receive])
    try:
        if args.control:
            for host in ('host.docker.internal','127.0.0.1'):
                lease=ValidationLease(host,7789,args.lease_directory);lease.acquire();leases.append(lease)
            if dict(masters[0].getSystemState()[0]).get('/ctrl_cmd'):raise RuntimeError('shared control became occupied')
        pair(args.shared_master,args.isolated_master)
        if args.control:pair(args.isolated_master,args.shared_master,True)
        (args.output/'profile.json').write_text(json.dumps(dict(shared=args.shared_master,isolated=args.isolated_master,raw_topics=RAW,control=args.control,token=token),indent=2)+'\n')
        while all(q.poll() is None for q in processes):time.sleep(.2)
        raise RuntimeError('relay worker exited')
    except KeyboardInterrupt:pass
    finally:
        for q in reversed(processes):
            if q.poll() is None:
                os.killpg(q.pid,signal.SIGINT)
                try:q.wait(3)
                except subprocess.TimeoutExpired:q.terminate();q.wait(3)
        for lease in reversed(leases):lease.release()
        for f in logs:f.close()

if __name__=='__main__':main()
