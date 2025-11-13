## task5

### ex01

Создать папку `ros2_ws/src`.  
Создать каталог проекта, имя робота sam_bot  
```
ros2 pkg create --build-type ament_cmake sam_bot_description
```

Установка необходимых пакетов:  
```
cd ~/workbench/ex01/ros2_ws/src
git clone https://github.com/ros/joint_state_publisher.git
cd ~/workbench/ex01/ros2_ws/
colcon build --symlink-install --packages-select joint_state_publisher joint_state_publisher_gui sam_bot_description
source install/setup.bash
```

В директории `ex01/ros2_ws/src/sam_bot_description/src` создать `/description/sam_bot_description.urdf`. Задать описание. Создать `sam_bot_description/launch/robot_display.launch.py`.

В `package.xml` дописать:
```
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz</exec_depend>
<exec_depend>xacro</exec_depend>
```

Дописать в `CMakeLists.txt`
```
install(
  DIRECTORY src launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```


Внутри папки `sam_bot_description` создать папку `rviz` и файл конфигурации RViz с именем `config.rviz`.

```
colcon build --packages-select sam_bot_description
source install/setup.bash
ros2 launch sam_bot_description robot_display.launch.py
```

### ex02

Начало аналогично 1 заданию

Сздать sam_bot_description.urdf.xacro

Дописать в `CMakeLists.txt`
```
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)
```

```
colcon build --packages-select sam_bot_description
source install/setup.bash
ros2 launch sam_bot_description robot_display.launch.py
```

### ex03

```
mkdir -p ros2_ws/src
cd ros2_ws
```

```
ros2 pkg create --build-type ament_cmake robot_description
ros2 pkg create --build-type ament_cmake robot_bringup
ros2 pkg create --build-type ament_cmake robot_app
```

```
colcon build --symlink-install
source install/setup.bash
```

```
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
```

```
sudo apt update
sudo apt install ros-jazzy-rqt-robot-steering
```

```
ros2 launch robot_bringup diff_drive.launch.py
ros2 run rqt_robot_steering rqt_robot_steering
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Активно нажимайте клавиши для управления: i, j, k, l или w, a, s, d в терминале с teleop_twist_keyboard.

Управление через команду:  
```
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### ex04

Создать пакет
```
ros2 pkg create --build-type ament_python circle_movement
```

Создать файл `circle_movement/circle_movement/circle_movement.py`  
Создать `circle_movement/launch/circle_movement.launch.py`  

В файле setup.py:  
```
entry_points={
    'console_scripts': [
        'circle_movement = circle_movement.circle_movement:main',
    ],
},
```

```
('share/' + package_name + '/launch', 
            ['launch/circle_movement.launch.py']),
```

Собрать пакет
```
colcon build --packages-select circle_movement
```

Запуск:  
В отдельном терминале запустить лаунч 3 задачи
```
cd ex03/ros2_ws
source install/setup.bash
ros2 launch robot_bringup diff_drive.launch.py
```

В терминале для 4 задачи
```
source install/setup.bash
ros2 launch circle_movement circle_movement.launch.py
```