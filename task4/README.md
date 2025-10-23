## Task 4

#### ex01

Установка tf2 и тест
```
sudo apt update
sudo apt-get install ros-jazzy-rviz2 ros-jazzy-turtle-tf2-py ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-turtlesim
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```

Сохранение в pdf через tf2_tools
```
ros2 run tf2_tools view_frames
```

Вывод результатов трансформации координат между фреймами turtle1 и turtle2 в файл transform.txt 
```
ros2 run turtlesim turtle_teleop_key
ros2 run tf2_ros tf2_echo turtle1 turtle2 > transform.txt
```
Во аремя записи подвигать черепашку.

Откройте текстовым редактором файл transform.txt, посмотрите последнее преобразование координат между фреймами turtle1 и turtle2 по времени и впишите руками в файл last_transform.txt три числа через запятую: смещение по оси X, смещение по оси Y, угол поворота в градусах(!) по оси Z. 

Данные из строки `Translation: [-0.000, 0.000, 0.000]` и `Rotation: in RPY (degree) [0.000, 0.000, -16.133]`.

#### ex02
Для отображения окон (в терминале системы)
```
xhost +local:docker
```

Создание пакета
```
ros2 pkg create --build-type ament_python two_turtles_one_carrot
```

Сборка и обновление пакета
```
colcon build --packages-select two_turtles_one_carrot
source install/setup.bash
```

Запуск 
ros2 launch two_turtles_one_carrot carrot.launch.py radius:=5.0

- `radius` - радиус для морковки
- `target_frame` - цель, опционально

Запуск окна rviz
```
rviz2
```
Левый нижний угол -> нажать "Add" и в списке выбрать tf. Для сохранения файла перейти в File (левый верхний угол) -> "Save as" и сохранить в нужную папку с именем `carrot.rviz`.


