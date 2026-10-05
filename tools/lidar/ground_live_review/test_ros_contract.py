#!/usr/bin/env python3
"""Run against isolated master + ground_compare_live on /review/input,/review/imu."""
import argparse
import json
import threading
import time
from pathlib import Path
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


def wait_until(predicate, timeout=6):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("ROS contract condition timed out")


def main():
    p=argparse.ArgumentParser()
    p.add_argument("input",type=Path)
    p.add_argument("result",type=Path)
    a=p.parse_args()
    rospy.init_node("ground_live_contract_test",anonymous=True)
    clouds={}; statuses=[]; lock=threading.Lock()
    def callback(msg,key):
        with lock:
            clouds[(msg.header.stamp.to_nsec(),key)]=msg
    subscriptions=[]
    for method in ("jaeho","patchwork","linefit"):
        for kind in ("ground","nonground","uncertain","structural_removed","native_retained"):
            key=method+"/"+kind
            subscriptions.append(rospy.Subscriber("/lidar_candidate/ground_live/"+key,
                PointCloud2,callback,callback_args=key,queue_size=50))
    subscriptions.append(rospy.Subscriber("/lidar_candidate/ground_live/status",
        DiagnosticArray,lambda msg:statuses.extend(msg.status),queue_size=100))
    pub=rospy.Publisher("/review/input",PointCloud2,queue_size=1)
    imu_pub=rospy.Publisher("/review/imu",Imu,queue_size=5)
    wait_until(lambda:pub.get_num_connections() and imu_pub.get_num_connections())
    fields=[PointField(n,i*4,PointField.FLOAT32,1) for i,n in enumerate(("x","y","z","intensity"))]
    def send(cloud,stamp,roll=0.,pitch=0.,imu=True):
        if imu:
            m=Imu();m.header.stamp=rospy.Time.from_sec(0)
            m.header.stamp=rospy.Time(stamp//10**9,stamp%10**9)
            q=quaternion_from_euler(roll,pitch,0)
            m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=q
            imu_pub.publish(m);time.sleep(.04)
        header=Header(stamp=rospy.Time(stamp//10**9,stamp%10**9),frame_id="velodyne")
        pub.publish(point_cloud2.create_cloud(header,fields,cloud.tolist()))
    frames=0
    # Establish a fresh sequence even when this test is repeated against the
    # same node. Rewind intentionally precedes the first valid IMU callback.
    send(np.zeros((1,4),np.float32),1,imu=False)
    wait_until(lambda:any(any(v.key=="source_stamp_ns" and v.value=="1" for v in s.values) for s in statuses))
    with (a.input/"snapshots.jsonl").open() as src,a.result.open() as results:
        for text,row in zip(src,results):
            source=json.loads(text);expected=json.loads(row)
            points=np.fromfile(a.input/source["cloud_path"],dtype="<f4").reshape(-1,4)
            stamp=source["scan_effective_stamp_ns"]
            send(points,stamp,source["roll"],source["pitch"])
            wait_until(lambda:len([k for k in clouds if k[0]==stamp])==15)
            for method in expected["methods"]:
                mask=np.asarray(expected["methods"][method]["mask"])
                structure=np.asarray(expected["methods"][method]["structural_removed"])
                for kind,value in (("ground",1),("nonground",2),("uncertain",3),
                                   ("structural_removed",None),("native_retained",None)):
                    msg=clouds[(stamp,method+"/"+kind)]
                    assert msg.header.frame_id=="base_link"
                    data=list(point_cloud2.read_points(msg,("x","y","z","intensity","source_index"),skip_nans=False))
                    indices=np.array([x[4] for x in data],dtype=int)
                    expected_ids=np.flatnonzero(mask==value) if value is not None else np.flatnonzero(
                        structure if kind=="structural_removed" else (mask==2)&(structure==0))
                    assert np.array_equal(indices,expected_ids),(method,kind,"index")
                    if len(data):
                        actual=np.asarray(data)[:,:4]
                        truth=points[indices].astype(float);truth[:,:3]+=[.8,0,1.45]
                        assert np.allclose(actual[:,:3],truth[:,:3],atol=4e-6,rtol=0)
                        assert np.array_equal(actual[:,3].astype(np.float32),points[indices,3])
            frames+=1
            if frames==12:
                break
    def expect_state(name,action):
        begin=len(statuses);action()
        wait_until(lambda:any(s.message==name for s in statuses[begin:]))
    expect_state("duplicate_stamp",lambda:send(points,stamp,imu=False))
    expect_state("zero_stamp",lambda:send(points,0,imu=False))
    stamp+=2*10**9
    expect_state("imu_stale",lambda:send(points,stamp,imu=False))
    stamp+=10**8
    expect_state("observed_empty",lambda:send(np.empty((0,4),np.float32),stamp))
    stamp+=10**8
    invalid=np.full((4,4),np.nan,dtype=np.float32)
    expect_state("roi_empty",lambda:send(invalid,stamp))
    # Rewind clears old IMU history and explicitly resets all adaptive state.
    expect_state("imu_missing",lambda:send(points,stamp-10**9,imu=False))
    stamp-=9*10**8
    expect_state("valid",lambda:send(points,stamp))
    expect_state("sensor_missing",lambda:time.sleep(.6))
    wait_until(lambda:all(msg.width==0 for (ns,_),msg in clouds.items() if ns==stamp))
    # One deliberately overloaded burst checks node-local shared dropping.
    # ROS transport may additionally drop before callback; that is not counted.
    stamp+=10**8
    send(points,stamp)
    wait_until(lambda:any(s.message=="valid" and any(v.key=="source_stamp_ns" and v.value==str(stamp)
                     for v in s.values) for s in statuses))
    packet=point_cloud2.create_cloud(Header(frame_id="velodyne"),fields,points.tolist())
    for i in range(100):
        stamp+=1000
        packet.header.stamp=rospy.Time(stamp//10**9,stamp%10**9)
        pub.publish(packet)
        time.sleep(.001)
    wait_until(lambda:any(any(v.key=="queue_dropped" and int(v.value)>0 for v in s.values) for s in statuses))
    print(json.dumps({"live_offline_frames_exact":frames,
                      "field_preservation":True,"edge_cases":7,"shared_queue_overflow":True}))


if __name__=="__main__":
    main()
