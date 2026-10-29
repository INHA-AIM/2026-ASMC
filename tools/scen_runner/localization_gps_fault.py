#!/usr/bin/env python3
"""Private-master GPS outage fixture; original packets decide the fault region.

This never publishes native actor truth or a substituted estimated position.
The simulator's saved sensor configuration is unchanged.
"""
import argparse
import copy
import json
import os


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--start', type=float, default=1880.)
    p.add_argument('--end', type=float, default=1985.)
    a=p.parse_args()
    if os.environ.get('ROS_MASTER_URI','').rstrip('/').endswith(':11311'):
        raise RuntimeError('fault injection is private-master only')
    import rospy
    from morai_msgs.msg import GPSMessage
    from std_msgs.msg import String
    from pyproj import Transformer
    from scen_runner.runtime.validation_episode import EpisodeRoute
    route=EpisodeRoute().curve
    transform=Transformer.from_crs('EPSG:4326','EPSG:32652',always_xy=True)
    rospy.init_node('localization_gps_fault')
    output=rospy.Publisher('/gps',GPSMessage,queue_size=8)
    trace=rospy.Publisher('/simulation/gps_fault',String,queue_size=8)
    def callback(packet):
        fault=False;station=None
        if packet.latitude and packet.longitude:
            x,y=transform.transform(packet.longitude,packet.latitude)
            station,_=route.project(x-packet.eastOffset,y-packet.northOffset)
            fault=a.start<=station<=a.end
        result=copy.deepcopy(packet)
        if fault: result.latitude=result.longitude=0.;result.status=0
        output.publish(result)
        trace.publish(String(json.dumps(dict(stamp=packet.header.stamp.to_sec(),
                                            station=station, injected=fault))))
    rospy.Subscriber('/simulation/raw/gps',GPSMessage,callback,queue_size=8,tcp_nodelay=True)
    rospy.spin()


if __name__=='__main__':main()
