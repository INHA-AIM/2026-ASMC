#!/usr/bin/env python3
"""Records every final V2 output as received by an external consumer (tcp_nodelay).
usage: rt_observer.py OUT_CSV   (runs until SIGINT/SIGTERM)
OUT_CSV: topic, recv_mono_ns, stamp_ns (= LiDAR source stamp of the scan), status, n_objects
"""
import csv, signal, sys, time
import rospy
from asmc_msgs.msg import LidarObjectArrayV2

out = open(sys.argv[1], "w", newline="")
w = csv.writer(out)
w.writerow(["topic", "recv_mono_ns", "stamp_ns", "status", "n_objects"])


def cb(topic):
    def f(msg):
        w.writerow([topic, time.monotonic_ns(), msg.header.stamp.to_nsec(), msg.status, len(msg.objects)])
    return f


rospy.init_node("rt_observer", anonymous=True, disable_signals=True)
rospy.Subscriber("/lidar/local_object", LidarObjectArrayV2, cb("local"), queue_size=50, tcp_nodelay=True)
rospy.Subscriber("/lidar/map_object", LidarObjectArrayV2, cb("map"), queue_size=50, tcp_nodelay=True)
stop = [False]
signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__(0, True))
signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True))
while not stop[0]:
    time.sleep(0.05)
out.flush(); out.close()
print("OBSERVER_DONE")
