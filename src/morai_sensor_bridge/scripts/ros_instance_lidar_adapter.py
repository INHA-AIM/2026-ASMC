#!/usr/bin/env python3
"""Normalize MORAI ROS Instance LiDAR to the canonical instance_id schema."""

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField


POINT_TYPES = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


class RosInstanceLidarAdapter(object):
    def __init__(self):
        self.input_topic = str(
            rospy.get_param("~input_topic", "/morai/gt/lidar_instance_raw")
        )
        self.output_topic = str(
            rospy.get_param("~output_topic", "/velodyne_points_instance")
        )
        if self.input_topic == self.output_topic:
            raise ValueError("input_topic and output_topic must differ")
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self.callback, queue_size=1,
            buff_size=32 << 20
        )
        self.converted = 0
        self.rejected = 0

    @staticmethod
    def _field_view(message, field):
        dtype = POINT_TYPES.get(field.datatype)
        if dtype is None or field.count != 1:
            raise ValueError("unsupported PointField {} type/count".format(field.name))
        dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
        return np.ndarray(
            shape=(message.height, message.width),
            dtype=dtype,
            buffer=message.data,
            offset=field.offset,
            strides=(message.row_step, message.point_step),
        ).reshape(-1)

    def callback(self, message):
        fields = dict((field.name, field) for field in message.fields)
        value_name = "instance_id" if "instance_id" in fields else "intensity"
        required = ("x", "y", "z", value_name)
        if any(name not in fields for name in required):
            self.rejected += 1
            rospy.logerr_throttle(
                2.0, "Instance LiDAR fields missing; got %s", sorted(fields)
            )
            return
        try:
            values = dict(
                (name, self._field_view(message, fields[name])) for name in required
            )
        except (TypeError, ValueError) as exc:
            self.rejected += 1
            rospy.logerr_throttle(2.0, "Instance LiDAR conversion failed: %s", exc)
            return
        count = message.height * message.width
        output = np.zeros(
            count,
            dtype=[
                ("x", np.float32), ("y", np.float32), ("z", np.float32),
                ("instance_id", np.uint32),
            ],
        )
        for axis in ("x", "y", "z"):
            output[axis] = values[axis].astype(np.float32, copy=False)
        instance_values = np.rint(values[value_name])
        if (
            not np.all(np.isfinite(instance_values))
            or np.any(instance_values < 0)
            or np.any(instance_values > 255)
        ):
            self.rejected += 1
            rospy.logerr_throttle(
                2.0, "Instance LiDAR values must be finite IDs in 0..255"
            )
            return
        output["instance_id"] = instance_values.astype(np.uint32, copy=False)

        result = PointCloud2()
        result.header = message.header
        result.height = 1
        result.width = count
        result.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("instance_id", 12, PointField.UINT32, 1),
        ]
        result.is_bigendian = False
        result.point_step = 16
        result.row_step = result.point_step * count
        result.is_dense = message.is_dense
        result.data = output.tobytes()
        self.publisher.publish(result)
        self.converted += 1


def main():
    rospy.init_node("morai_ros_instance_lidar_adapter")
    RosInstanceLidarAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
