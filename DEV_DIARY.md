# Заметки

## Установка ROS2 Jazzy

```bash
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Просмотр bag файла

```bash
ros2 bag play bag_files/tb_office_v02_0.mcap
```

## Запуск rViz на wayland для визуализации

```bash
QT_QPA_PLATFORM=xcb rviz2
```

## 2D SLAM with slam_toolbox

```bash
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-pointcloud-to-laserscan
```

Для запуска в разных терминалах:

```bash
make rviz
make pc2l
make slam
ros2 bag play bag_files/tb_office_v02_0.mcap
```

## Запуск детектора маркеров

```bash
ros2 run marker_detector detector
```

## Запуск всего и сразу

```bash
make all
```