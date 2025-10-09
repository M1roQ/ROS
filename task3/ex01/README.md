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

### ex03

Запустить черепашку и управление

```
ros2 doctor
ros2 doctor --report | grep -A 50 "PLATFORM INFORMATION\|RMW MIDDLEWARE\|ROS 2 INFORMATION\|TOPIC LIST" > doctor.txt
```

- Опция --report выводит подробный отчёт по установленным пакетам, версиям, конфигурации платформы, и другим важным аспектам вашей ROS 2 установки.

- Команда grep ищет в выводе текстовые шаблоны по строкам.

- Опция -A 50 означает «вывести найденную строку и 50 строк после неё» — захватывая блок с интересующей информацией.

- Паттерн "PLATFORM INFORMATION\|RMW MIDDLEWARE\|ROS 2 INFORMATION\|TOPIC LIST" ищет строки, содержащие любой из этих ключевых разделов отчёта (информация о платформе, о middleware, о ROS 2 и списке топиков).

### ex04

Создание пакета

```
mkdir -p ros2_ws/src
cd ros2_ws/src
ros2 pkg create --build-type ament_python move_to_goal --dependencies rclpy geometry_msgs turtlesim

cd ..
colcon build --packages-select move_to_goal
source install/setup.bash
```

Запуск черепашки

```
ros2 run turtlesim turtlesim_node
```

Из ros2_ws!

```
ros2 run move_to_goal move_to_goal 5.0 5.0 0.0
```

- Первый move_to_goal — название ROS 2 пакета, в котором содержится узел.

- Второй move_to_goal — имя исполняемого файла (узла), который запускается из этого пакета.

- 5.0 5.0 0.0 — параметры, передаваемые ноде, целевые координаты x=5.0, y=5.0 и угол поворота theta=0.0.

### ex05

Создание пакета

```
mkdir -p ros2_ws/src
cd ros2_ws/src
ros2 pkg create --build-type ament_python action_cleaning_robot
colcon build --packages-select action_cleaning_robot
source install/setup.bash
```

Проверка

```
ros2 interface show action_cleaning_robot/action/CleaningTask
```

Вывод:
```
# goal
string task_type
float64 area_size
float64 target_x
float64 target_y
---
# result
bool success
int32 cleaned_points
float64 total_distance
---
# feedback
int32 progress_percent
int32 current_cleaned_points
float64 current_x
float64 current_y
```

Запуск:
```
ros2 run turtlesim turtlesim_node
ros2 run action_cleaning_robot cleaning_action_server
ros2 run action_cleaning_robot cleaning_action_client
```
Запускать в отдельных терминалах из ros2_ws. Перед запуском выполнить
```
source install/setup.bash
```