#!/usr/bin/env python3
"""Synthetic independent geometry/speed truth for the real LiDAR pipeline.

Truth is generated before the detector runs, never inferred from a returned box.
Point intensity identifies synthetic objects for review, not production classes.
"""
import argparse
import json
import math
import hashlib
from pathlib import Path
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu,PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from morai_msgs.msg import GPSMessage,EgoVehicleStatus
from tf.transformations import quaternion_from_euler,euler_matrix


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);truth=[]
    with rosbag.Bag(str(args.output/'synthetic.bag'),'w') as bag:
        # Objects have separate world positions; one moves at 1 m/s, Ego at 0.5.
        # Every object's bottom touches z=-1.45 ground. In particular the 30cm
        # object challenges the unchanged ground rejection band; do not move
        # it above that band merely to make a preservation test pass.
        specs=[('vehicle',(16.,0.,-.85),(1.6,.8,1.2),0.),
            ('person',(20.,3.,-.65),(.4,.4,1.6),1.),
            ('low',(12.,-3.,-1.30),(.6,.6,.3),0.),
            ('wall',(24.,4.,-.45),(3.,.4,2.),0.)]
        for i in range(80):
            t=i*.1;stamp=rospy.Time.from_sec(100.+t)
            roll=math.radians([0,5,-5,10,-10][(i//16)%5]);pitch=0. if i<16 else math.radians(3*math.sin(t))
            yaw=0 if t<4 else .03*(t-4);ego=np.array([.5*t,0,0.])
            # Ry(pitch)Rx(roll): same stated contract, independent truth generator.
            rx=euler_matrix(roll,0,0)[:3,:3];ry=euler_matrix(0,pitch,0)[:3,:3];level=ry@rx
            rz=euler_matrix(0,0,yaw)[:3,:3]
            points=[];objects=[]
            for x in np.arange(5,11,.35):
                for y in np.arange(-4,4,.35):
                    q=level.T@np.array([x,y,-1.45]);points.append([*q,0.])
            for k,(name,position,size,speed) in enumerate(specs,1):
                # Disappearance and re-entry is explicit truth, not detector miss.
                if name=='person' and 30<=i<36:continue
                world=np.array(position);world[0]+=speed*t
                sensor_center=rz.T@(world-ego)
                center_raw=level.T@sensor_center
                axes=[np.linspace(-d/2,d/2,max(3,math.ceil(d/.15)+1)) for d in size]
                for dx in axes[0]:
                    for dy in axes[1]:
                        for dz in axes[2]:
                            q=level.T@(sensor_center+np.array([dx,dy,dz]));points.append([*q,float(k)])
                objects.append(dict(name=name,center_sensor_raw=center_raw.tolist(),
                    center_base=(center_raw+np.array([.8,0,1.45])).tolist(),size=size,
                    absolute_speed_mps=speed))
            imu=Imu(header=Header(stamp=stamp,frame_id='imu'))
            q=quaternion_from_euler(roll,pitch,yaw)
            imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=q
            can=EgoVehicleStatus(header=Header(stamp=stamp));can.velocity.x=.5
            gps=GPSMessage(header=Header(stamp=stamp));gps.status=1;gps.latitude=37.241;gps.longitude=126.774
            gps.altitude=1.25;gps.eastOffset=302595.;gps.northOffset=4124145.
            fields=[PointField(n,j*4,PointField.FLOAT32,1) for j,n in enumerate(('x','y','z','intensity'))]
            cloud=point_cloud2.create_cloud(Header(stamp=stamp,frame_id='velodyne'),fields,points)
            for topic,message in (('/imu',imu),('/Competition_topic',can),('/gps',gps),('/velodyne_points',cloud)):
                bag.write(topic,message,stamp)
            truth.append(dict(stamp_ns=stamp.to_nsec(),roll=roll,pitch=pitch,yaw=yaw,objects=objects,
                quantitative_speed_truth=(i<16),
                motion_scope='straight speed GT first 16 frames; later attitude stress with scalar CAN, not complete ego kinematics'))
    (args.output/'truth.json').write_text(json.dumps(truth,indent=2)+'\n')
    (args.output/'manifest.json').write_text(json.dumps(dict(generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        numpy_version=np.__version__,frames=80,ground_z_sensor=-1.45,object_bottoms_on_ground=True,
        ego_sensor_translation_mps=.5,gps='constant map-ready fixture; map accuracy is not evaluated',
        quantitative_speed_scope='straight/no attitude first 16 frames only',
        files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}),indent=2)+'\n')

if __name__=='__main__':main()
