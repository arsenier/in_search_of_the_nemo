from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
from launch_ros.actions import Node


def generate_launch_description():

    pc2l = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("pc2l"), "launch", "start.launch.py"]
                )
            ]
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "slam_params_file": "./config/mapper_params_online_async.yaml"
        }.items(),
    )

    marker_detector = Node(
        package="marker_detector",
        executable="detector",
        name="detector",
        parameters=[
            {
                "image_topic": "/marker_debug_img",
                "pointcloud_source": "/livox/lidar",
                "marker_topic": "/marker",
            }
        ],
    )

    return LaunchDescription([pc2l, slam, marker_detector])
