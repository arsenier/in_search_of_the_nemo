#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

# https://wiki.ros.org/pointcloud_to_laserscan
# https://www.theconstruct.ai/ros-qa-120-how-to-convert-a-pointcloud-into-a-laser-scan/
# https://stepik.org/lesson/1505350/step/1?unit=1525496


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/livox/lidar"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "transform_tolerance": 0.01,
                        "min_height": 0.2,
                        "max_height": 1.0,
                        "angle_min": -3.14,
                        "angle_max": 3.14,
                        "angle_increment": 0.0087,
                        "scan_time": 0.3333,
                        "range_min": 0.45,
                        "range_max": 10.0,
                        "use_inf": True,
                        "concurrency_level": 1,
                    },
                ],
            ),
        ]
    )
