# Ignore folders with same names
.PHONY: rviz slam pc2l

rviz:
	QT_QPA_PLATFORM=xcb rviz2 -d ./config/rviz_config.rviz

slam:
	ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./config/mapper_params_online_async.yaml

pc2l:
	ros2 launch pc2l start.launch.py
