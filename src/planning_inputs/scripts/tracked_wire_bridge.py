#!/usr/bin/env python3
"""Frozen legacy wire gate and timestamped validated planning publication."""
import json
import threading
from collections import OrderedDict
import rospy
from asmc_msgs.msg import TrackedObjectArray
from planning_inputs.msg import ValidatedTrackedObjectArray
from std_msgs.msg import String

def compatible(header, expected_md5):
    return header.get('type') in ('Msg/TrackedObjectArray','asmc_msgs/TrackedObjectArray') and header.get('md5sum') == expected_md5

def debug_key(debug):
    sec, nsec = debug.get('stamp_secs'), debug.get('stamp_nsecs')
    if type(sec) is not int or type(nsec) is not int or sec < 0 or not 0 <= nsec < 1000000000:
        return None
    return (sec,nsec)

def validated_states(frame, debug):
    if debug_key(debug) != (frame.header.stamp.secs, frame.header.stamp.nsecs):
        return None
    rows=debug.get('objects')
    if not isinstance(rows,list):
        return None
    states={}
    for row in rows:
        if not isinstance(row,dict):
            return None
        identity,state=row.get('id'),row.get('motion_state_code')
        if type(identity) is not int or type(state) is not int or state not in range(4) or identity in states:
            return None
        states[identity]=(state,row.get('final_keep') is True)
    ids=[o.id for o in frame.objects]
    kept={identity for identity,(_,keep) in states.items() if keep}
    if len(ids)!=len(set(ids)) or set(ids)!=kept:
        return None
    return [states[identity][0] for identity in ids]

class TrackedWireBridge:
    def __init__(self):
        input_topic=rospy.get_param('~input_topic','/last_lidar')
        output_topic=rospy.get_param('~output_topic','/planning/input/tracked_objects')
        if input_topic==output_topic:
            raise ValueError('planning wire bridge input/output must differ')
        self.policy=rospy.get_param('~source_policy','validated_lidar')
        if self.policy not in ('validated_lidar','main_raw'):
            raise ValueError('unsupported LiDAR source_policy')
        self.lock=threading.Lock()
        self.frames,self.debug_frames=OrderedDict(),OrderedDict()
        output_type=ValidatedTrackedObjectArray if self.policy=='validated_lidar' else TrackedObjectArray
        self.pub=rospy.Publisher(output_topic,output_type,queue_size=2)
        if self.policy=='validated_lidar':
            self.debug_sub=rospy.Subscriber(rospy.get_param('~motion_debug_topic','/lidar/motion_debug'),String,self.debug_callback,queue_size=10)
        self.sub=rospy.Subscriber(input_topic,rospy.AnyMsg,self.callback,queue_size=2)

    def callback(self,wire):
        if not compatible(wire._connection_header,TrackedObjectArray._md5sum):
            rospy.logerr_throttle(1.0,'Rejected incompatible LiDAR type/MD5; rebuild the nested sensor workspace')
            return
        message=TrackedObjectArray()
        try:
            message.deserialize(wire._buff)
        except Exception as error:
            rospy.logerr_throttle(1.0,'LiDAR decode failed: %s',error)
            return
        if self.policy=='main_raw':
            self.pub.publish(message)
            return
        key=(message.header.stamp.secs,message.header.stamp.nsecs)
        if message.header.frame_id!='map' or message.header.stamp.is_zero():
            rospy.logerr_throttle(1.0,'Invalid validated LiDAR header')
            return
        with self.lock:
            self.frames[key]=message
            self.flush(key)

    def debug_callback(self,message):
        try:
            debug=json.loads(message.data)
            key=debug_key(debug)
            if key is None:
                raise ValueError('missing exact source stamp; rebuild Main')
        except (AttributeError,TypeError,ValueError) as error:
            rospy.logerr_throttle(1.0,'Invalid validated LiDAR diagnostics: %s',error)
            return
        with self.lock:
            self.debug_frames[key]=debug
            self.flush(key)

    def flush(self,key):
        frame,debug=self.frames.get(key),self.debug_frames.get(key)
        if frame is not None and debug is not None:
            states=validated_states(frame,debug)
            if states is not None:
                output=ValidatedTrackedObjectArray()
                output.tracked,output.motion_states=frame,states
                self.pub.publish(output)
            else:
                rospy.logerr_throttle(1.0,'Validated LiDAR frame/debug ID or keep-state mismatch')
            self.frames.pop(key,None)
            self.debug_frames.pop(key,None)
        for pending in (self.frames,self.debug_frames):
            while len(pending)>20:
                pending.popitem(last=False)

if __name__=='__main__':
    rospy.init_node('planning_tracked_wire_bridge')
    TrackedWireBridge()
    rospy.spin()
