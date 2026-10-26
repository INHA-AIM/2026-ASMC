#!/usr/bin/env python3
"""Tiny generated GPS-zero/missing/recovery recording; compares production cores.

The original mismatch is preserved in C0 evidence. Current bridge and planning
cores must both reject the sentinel and reacquire a fresh baseline on recovery.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import rosbag
import rospy
from sensor_msgs.msg import Imu,PointCloud2
from morai_msgs.msg import GPSMessage
ROOT=Path(__file__).resolve().parents[3]

def main(probes):
    with tempfile.TemporaryDirectory(prefix='c0-state-fixture-') as temp:
        temp=Path(temp);phases={}
        with rosbag.Bag(str(temp/'input.bag'),'w') as bag:
            for i in range(300):
                elapsed=i*.02;stamp=rospy.Time.from_sec(100+elapsed);receipt=rospy.Time.from_sec(100+elapsed+.002)
                imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;imu.linear_acceleration.z=9.81
                bag.write('/imu',imu,receipt)
                if i%10==0 and not 4<=elapsed<4.8:
                    gps=GPSMessage();gps.header.stamp=stamp;gps.status=1;gps.latitude=37.239;gps.longitude=126.773
                    gps.eastOffset=302595.;gps.northOffset=4124145.
                    if 3<=elapsed<4:gps.latitude=gps.longitude=0.
                    bag.write('/gps',gps,receipt)
                if i%4==0:
                    cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='velodyne'
                    bag.write('/velodyne_points',cloud,receipt);phases[stamp.to_nsec()]=elapsed
        result={}
        for kind,pkg in [('planning','planning_inputs'),('bridge','asmc_bridge')]:
            output=temp/(kind+'.jsonl')
            subprocess.run([str(probes/(kind+'_state_probe')),str(temp/'input.bag'),str(ROOT/'src'/pkg/'config/vehicle_state.yaml'),str(output)],check=True)
            rows=[json.loads(x) for x in output.read_text().splitlines()]
            def count(start,end,valid):return sum(r['valid']==valid for r in rows if start<=phases[r['scan_ns']]<end)
            assert count(1,2.9,True)>10,kind+' did not initialize'
            assert count(5.5,6,True)>2,kind+' did not recover'
            assert count(4.6,4.8,False)>0,kind+' stale gate missing'
            assert count(3.2,3.9,True)==0,kind+' accepted zero GPS'
            assert count(4.8,4.96,True)==0,kind+' reused pre-dropout speed on the first recovered fix'
            result[kind]=dict(scans=len(rows),zero_gps_valid=count(3.2,3.9,True),recovered=count(5.5,6,True))
        print(json.dumps(result))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--probes',type=Path,required=True);main(p.parse_args().probes)
