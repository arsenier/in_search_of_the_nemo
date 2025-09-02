# Репозиторий команды "В поисках Немо"

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
