"""ROS adapter preserving exact JPEG -> compact semantic grid lineage."""
from copy import copy
import rospy
from std_msgs.msg import Header
from asmc_msgs.msg import CameraModelGridTiming, SensorFrameTiming
from .sensor_timing import SensorTimingIndex, source_usable


class GridTimingPublisher:
    def __init__(self, views, camera_topics, control):
        self.index = SensorTimingIndex()
        self.topics = {view:rospy.resolve_name(topic) for view,topic in camera_topics.items()}
        self.control = control
        self.publishers = {view: rospy.Publisher(
            '/perception/camera/{}/road_marking/model_grid/timing'.format(view),
            CameraModelGridTiming, queue_size=4) for view in views}
        self.subscriber = rospy.Subscriber('/morai_sensor_bridge/frame_timing', SensorFrameTiming,
                                           self.index.add, queue_size=256)

    @staticmethod
    def publish_image(publisher, message):
        # rospy rewrites header.seq while serializing. Observe it under the
        # same reentrant publisher lock; no-subscriber outputs have no wire key.
        publisher.impl.acquire()
        try:
            before = publisher.impl.seq
            publisher.publish(message)
            return copy(message.header), publisher.impl.seq == before + 1
        finally:
            publisher.impl.release()

    def publish(self, view, frame, headers):
        msg = CameraModelGridTiming()
        msg.header = Header(stamp=frame.stamp, frame_id=frame.frame_id)
        msg.producer_session_id = self.control.session_id
        msg.generation = self.control.state.generation
        msg.view = view
        msg.image_header = Header(seq=frame.source_sequence, stamp=frame.stamp, frame_id=frame.frame_id)
        msg.class_header, class_valid = headers[0]
        msg.confidence_header, confidence_valid = headers[1]
        msg.grid_wire_sequences_valid = class_valid and confidence_valid
        key = self.topics[view], frame.frame_id, frame.stamp.to_nsec(), frame.source_sequence
        timing = self.index.lookup(key, max_age_sec=.35)
        if timing is not None:
            msg.sensor_timing = timing
        msg.source_timing_valid = timing is not None and source_usable(timing)
        msg.reason = ('matched' if msg.source_timing_valid else
                      'invalid_raw_source' if timing is not None else 'missing_or_ambiguous_source_timing')
        self.publishers[view].publish(msg)

    def close(self):
        self.subscriber.unregister()
