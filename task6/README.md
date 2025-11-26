### ex01-ex03

```
colcon build
source install/setup.bash
ros2 launch robot_bringup diff_drive.launch.py
```

### ex04

```
cd ~/workbench/ex04/ros2_ws/src
ros2 pkg create --build-type ament_python obstacle_stop --dependencies rclpy sensor_msgs geometry_msgs
```
Терминал для ex01
```
cd ~/workbench/ex01/ros2_ws
source install/setup.bash
ros2 launch robot_bringup diff_drive.launch.py
```

Терминал для ex04
```
cd ~/workbench/ex04/ros2_ws
colcon build --packages-select obstacle_stop
source install/setup.bash
ros2 launch obstacle_stop move_control.launch.py
```

### ex05

```
cd ~/workbench/ex05/ros2_ws/src
ros2 pkg create --build-type ament_python obstacle_stop_depth --dependencies rclpy sensor_msgs geometry_msgs
```

Терминал для ex02
```
cd ~/workbench/ex02/ros2_ws
source install/setup.bash
ros2 launch robot_bringup diff_drive.launch.py
```

Терминал для ex05
```
cd ~/workbench/ex05/ros2_ws
colcon build --packages-select obstacle_stop_depth
source install/setup.bash
ros2 launch obstacle_stop_depth move_control_depth.launch.py
```
