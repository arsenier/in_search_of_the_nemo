#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.declare_parameter("image_topic", "/marker_debug_img")
        self.declare_parameter("pointcloud_source", "/livox/lidar")
        self.declare_parameter("marker_topic", "/marker")
        self.get_logger().info("Initializing node...")
        self.image_topic = self.get_parameter("image_topic").value
        self.pointcloud_source = self.get_parameter("pointcloud_source").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.point_publisher = self.create_publisher(PointCloud2, self.marker_topic, 10)
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        self.lidar_subscriber = self.create_subscription(
            PointCloud2, self.pointcloud_source, self.process_points, 10
        )

        self.image_height = 256
        self.image_width = 256

    def process_points(self, msg: PointCloud2):

        # https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_subscriber/pcd_subscriber_node.py
        pcd_as_numpy_array = np.array(list(read_points(msg)))

        # self.get_logger().info(f"pcd_as_numpy_array: {pcd_as_numpy_array}")
        self.get_logger().info(f"one point: {pcd_as_numpy_array[0]}")

        image_array = np.zeros((self.image_height, self.image_width, 1), dtype=np.uint8)

        max_dist = 10

        for point in pcd_as_numpy_array:
            x, y, z, intensity, _, _, _ = point

            dist = math.sqrt(x * x + y * y)

            dist_norm = dist / max_dist * 255
            intensity_log = math.log(intensity + 1) / math.log(255) * 255

            if dist_norm > 255:
                dist_norm = 255

            image_array[int(intensity), int(dist_norm), 0] = intensity

        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(image_array, "mono8")
        self.image_publisher.publish(ros_image)


## The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def main(args=None):
    rclpy.init(args=args)

    node = Detector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
