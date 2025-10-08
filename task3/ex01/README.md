### ex01

```
colcon build --packages-select service_full_name
source install/setup.bash
ros2 run service_full_name service_name
```

В другом терминале

```
source install/setup.bash
ros2 run service_full_name client_name Bakumova Valerie Evgenievna
```

### ex02

**Изменение размера окна**

В файле `ros2_turtle_ws/turtlesim/src/turtle_frame.cpp` в строке 61 
```
setFixedSize(1920, 1080);
```

**Клонирование и сборка turtlesim**

```
git clone https://github.com/ros/ros_tutorials.git -b jazzy ros2_turtle_ws
cd ros2_turtle_ws
colcon build --packages-select turtlesim
source install/setup.bash
```

- Клонировать официальный репозиторий с исходниками.
- Собрать только пакет `turtlesim`.
- Активировать рабочее пространство.

**Запуск симулятора и teleop**

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- Запустить окно с черепахой.
- Запустить Teleop для управления черепахой с клавиатуры.

**Список топиков для проверки**

```
ros2 topic list
```

- Проверить доступные топики, среди них должен быть `/turtle1/cmd_vel` и `/turtle1/pose`.

```
valerie@valbook:~/workbench$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**Запись rosbag в формате MCAP с командой скорости**

```
ros2 bag record -s mcap -o turtle_cmd_vel /turtle1/cmd_vel
```

- Записать команды скорости в файл `turtle_cmd_vel.mcap`.

**Воспроизведение и сохранение координат**

```
ros2 bag play turtle_cmd_vel
ros2 topic echo /turtle1/pose > pose_speed_x1.yaml
```

- Воспроизвести запись.
- Сохранить координаты из топика `/turtle1/pose` в файл для обычной скорости.

**Воспроизведение с удвоенной скоростью**

```
ros2 bag play turtle_cmd_vel -r 2.0
ros2 topic echo /turtle1/pose > pose_speed_x2.yaml
```

- Воспроизведение с ускорением в 2 раза.
- Сохранить координаты в отдельный YAML.

**Вывод и структура**

- По завершении, файлы `turtle_cmd_vel.mcap`, `pose_speed_x1.yaml`, `pose_speed_x2.yaml` нужно положить в папку `ex02`.
