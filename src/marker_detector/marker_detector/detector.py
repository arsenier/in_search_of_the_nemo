#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
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
        self.marker_publisher = self.create_publisher(
            PointCloud2, self.marker_topic, 10
        )
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        self.lidar_subscriber = self.create_subscription(
            PointCloud2, self.pointcloud_source, self.process_points, 2
        )

        self.image_height = 256
        self.image_width = 256

    def is_anomaly(self, dist, intensity):
        min_dist = 0.3
        max_dist = 2

        if min_dist < dist < max_dist and dist * 255 / 2.0 - intensity < 0:
            return True

    def process_points(self, msg: PointCloud2):

        # https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_subscriber/pcd_subscriber_node.py
        pcd_as_numpy_array = np.array(list(read_points(msg)))

        # self.get_logger().info(f"pcd_as_numpy_array: {pcd_as_numpy_array}")
        # self.get_logger().info(f"one point: {pcd_as_numpy_array[0]}")

        image_array = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        max_dist = 2
        max_intensity = 50

        anomaly_points = []

        for point in pcd_as_numpy_array:
            x, y, z, intensity, _, _, _ = point

            dist = math.sqrt(x * x + y * y)

            dist_norm = dist / max_dist * 255
            # intensity_log = math.log(intensity + 1) / math.log(255) * 255

            if dist_norm > 255:
                dist_norm = 255

            intensity_norm = intensity / max_intensity * 255
            if intensity_norm > 255:
                intensity_norm = 255

            image_array[int(intensity), int(dist_norm), 0] += 10
            if self.is_anomaly(dist, intensity):
                image_array[int(intensity), int(dist_norm), 1] += 255
                image_array[int(intensity), int(dist_norm), 2] += 255

                anomaly_points.append([x, y, dist, intensity])

        for x in range(self.image_width):
            for y in range(self.image_height):
                if image_array[y, x, 0] != 0:
                    image_array[y, x, 0] += 50
                    image_array[y, x, 0] = min(255, image_array[y, x, 0])

        if len(anomaly_points) > 10:
            # self.marker_publisher.publish(self.create_marker(anomaly_points))
            cmx = np.mean([x for x, _, _, _ in anomaly_points])
            cmy = np.mean([y for _, y, _, _ in anomaly_points])

            print(f"Marker found at {cmx}, {cmy}")

            point = point_cloud(np.array([[-cmy, -cmx, 0]]), "base_link")

            self.marker_publisher.publish(point)

        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(image_array, "bgr8")
        self.image_publisher.publish(ros_image)


def point_cloud(points, parent_frame):
    """Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [
        sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyz")
    ]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data,
    )


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
