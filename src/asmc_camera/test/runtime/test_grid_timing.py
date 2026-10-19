import io
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

import rospy
from rospy.msg import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from asmc_msgs.msg import CameraModelGridTiming, SensorFrameTiming
from asmc_camera.camera.frame_buffer import EncodedFrame
from asmc_camera.runtime.grid_timing import GridTimingPublisher
from asmc_camera.runtime.sensor_timing import SensorTimingIndex


class GridTimingTest(unittest.TestCase):
    def test_actual_grid_wire_and_original_jpeg_sequences_survive_serialization(self):
        class Impl:
            seq=24
            lock=threading.RLock()
            def acquire(self): self.lock.acquire()
            def release(self): self.lock.release()
        pub=Mock(); pub.impl=Impl()
        def publish(message):
            pub.impl.seq+=1
            serialize_message(io.BytesIO(),pub.impl.seq,message)
        pub.publish.side_effect=publish
        image=Image(header=Header(seq=123,stamp=rospy.Time(2),frame_id='camera_front'))
        header, valid=GridTimingPublisher.publish_image(pub,image)
        self.assertTrue(valid); self.assertEqual(header.seq,25)
        source=SensorFrameTiming(bridge_session_id='bridge',stream_id='cam1',
            sensor_topic='/input',sensor_frame_id='camera_front',published_stamp=rospy.Time(2),
            sensor_wire_sequence=123,sensor_wire_sequence_valid=True,source_valid=True,
            source_clock_domain='morai_sensor',source_sec=1)
        timing=GridTimingPublisher.__new__(GridTimingPublisher)
        timing.index=SensorTimingIndex(); timing.index.add(source)
        timing.topics={'front':'/input'}; timing.publishers={'front':Mock()}
        timing.control=NS(session_id='lane',state=NS(generation=7))
        frame=EncodedFrame('front',rospy.Time(2),'camera_front',b'',0,source_sequence=123)
        timing.publish('front',frame,[(header,True),(Header(seq=26,stamp=frame.stamp,frame_id=frame.frame_id),True)])
        message=timing.publishers['front'].publish.call_args.args[0]
        raw=io.BytesIO(); serialize_message(raw,99,message)
        restored=CameraModelGridTiming(); restored.deserialize(raw.getvalue()[4:])
        self.assertEqual(restored.header.seq,99)
        self.assertEqual(restored.image_header.seq,123)
        self.assertEqual(restored.class_header.seq,25)
        self.assertEqual(restored.confidence_header.seq,26)
        self.assertEqual(restored.sensor_timing.sensor_wire_sequence,123)
        self.assertTrue(restored.source_timing_valid)
        self.assertEqual(restored.producer_session_id,'lane')

    def test_missing_source_is_explicit_and_does_not_invent_source_time(self):
        timing=GridTimingPublisher.__new__(GridTimingPublisher)
        timing.index=SensorTimingIndex();timing.topics={'front':'/input'}
        timing.publishers={'front':Mock()};timing.control=NS(session_id='lane',state=NS(generation=1))
        frame=EncodedFrame('front',rospy.Time(2),'camera_front',b'',0,source_sequence=10)
        timing.publish('front',frame,[(Header(),False),(Header(),False)])
        result=timing.publishers['front'].publish.call_args.args[0]
        self.assertFalse(result.source_timing_valid)
        self.assertFalse(result.grid_wire_sequences_valid)
        self.assertEqual(result.sensor_timing.source_sec,0)


if __name__=='__main__':unittest.main()
