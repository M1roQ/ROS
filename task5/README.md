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