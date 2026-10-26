#!/usr/bin/env python3
"""Read-only development probe: valid common velocity vs nearby MORAI GT.

GT is evaluation-only. Prints candidate response events, not an accuracy verdict.
Does not start actors, control, inference or a recorder.
"""
import collections
import math
import threading
import time
from tracking_samples import observation_samples


def main():
    import rospy
    from asmc_msgs.msg import LidarTrackingObservation
    from morai_msgs.msg import ObjectStatusList
    rospy.init_node('velocity_transition_probe',anonymous=True)
    lock=threading.Lock();truth=collections.deque(maxlen=32);history={};epoch=[None]
    def gt_callback(message):
        stamp=message.header.stamp.to_sec()
        with lock:
            truth.append((stamp,[dict(id=o.unique_id,name=o.name,x=o.position.x,y=o.position.y,
                speed=math.hypot(o.velocity.x,o.velocity.y)/3.6)
                for o in list(message.npc_list)+list(message.pedestrian_list)]))
    def tracking_callback(message):
        rows,reason=observation_samples(message,minimum_age=3)
        obs=message.observation;identity=(obs.producer_session_id,obs.generation)
        with lock:
            if identity!=epoch[0]:history.clear();epoch[0]=identity
            if not rows or not truth:
                rospy.loginfo_throttle(1.,'velocity probe: '+reason);return
            stamp=obs.objects.header.stamp.to_sec()
            gt_stamp,objects=min(truth,key=lambda r:abs(r[0]-stamp))
            if abs(gt_stamp-stamp)>.1:
                rospy.logwarn_throttle(1.,'velocity probe: no GT within 100ms');return
            used=set()
            for row in rows:
                candidates=[(math.hypot(row['x']-o['x'],row['y']-o['y']),o) for o in objects if o['id'] not in used]
                if not candidates:continue
                distance,obj=min(candidates,key=lambda r:r[0])
                if distance>5.:continue
                used.add(obj['id']);key=(row['id'],obj['id'])
                previous=history.get(key)
                if previous and stamp<=previous[0]:continue
                if previous and previous[1]<2/3.6<=obj['speed']:
                    print('GT_ONSET gt_id={} track_id={} source={:.6f} estimate_kph={:.2f} motion={}'.format(
                        obj['id'],row['id'],stamp,row['speed']*3.6,row['motion_state']),flush=True)
                if previous and previous[2]<2/3.6<=row['speed']:
                    print('TRACK_ONSET gt_id={} track_id={} source={:.6f} gt_kph={:.2f} motion={}'.format(
                        obj['id'],row['id'],stamp,obj['speed']*3.6,row['motion_state']),flush=True)
                history[key]=(stamp,obj['speed'],row['speed'])
            for key,value in list(history.items()):
                if stamp-value[0]>3.:del history[key]
    rospy.Subscriber('/Object_topic',ObjectStatusList,gt_callback,queue_size=2)
    rospy.Subscriber('/perception/lidar/tracks_map',LidarTrackingObservation,tracking_callback,queue_size=1,tcp_nodelay=True)
    end=time.monotonic()+120
    while not rospy.is_shutdown() and time.monotonic()<end:time.sleep(.1)
if __name__=='__main__':main()
