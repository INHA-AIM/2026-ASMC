#!/usr/bin/env python3
"""Read-only GPS antenna height versus Ego/IMU audit; no ROS master or GPU."""
import argparse
import json
import math
from pathlib import Path
import rosbag


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=[];ego=imu=None;unavailable=0
    with rosbag.Bag(args.bag) as bag:
        for topic,message,_ in bag.read_messages(topics=['/gps','/Ego_topic','/imu']):
            if topic=='/Ego_topic':ego=message;continue
            if topic=='/imu':imu=message;continue
            if message.status==0:continue
            if ego is None or imu is None:
                unavailable+=1;continue
            stamp=message.header.stamp.to_nsec()
            ages=[(stamp-m.header.stamp.to_nsec())/1e9 for m in (ego,imu)]
            if any(age<0 or age>.12 for age in ages):
                unavailable+=1;continue
            q=imu.orientation
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
            pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
            lever=-math.sin(pitch)*.35+math.cos(pitch)*math.cos(roll)*1.25
            rows.append(dict(gps_ns=stamp,ego_ns=ego.header.stamp.to_nsec(),imu_ns=imu.header.stamp.to_nsec(),
                gps_altitude=message.altitude,ego_z=ego.position.z,antenna_lever_z=lever,
                delta_m=message.altitude-lever-ego.position.z))
    out=dict(source=args.bag,rows=rows,samples=len(rows),unavailable=unavailable,
        maximum_abs_height_difference_m=max((abs(r['delta_m']) for r in rows),default=None),
        calibration='ASMC_sensor_set_v3 GPS lever [.35,0,1.25], pinned map altitude origin 0m',
        scope='read-only height-convention audit; no performance/recall measurement or other-datum guarantee')
    args.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='rows'}))
    return 0 if rows else 1


if __name__=='__main__':raise SystemExit(main())
