## Task2
### ex02
1. получить путь к ROS пакету «ros2topic».
    ```
    ros2 pkg prefix ros2topic
    ```
    Резульат: /opt/ros/jazzy
    
2. Путь к пакету action_tutorials_py:
    ```
    ros2 pkg executables action_tutorials_py
    ```
    Результат: 
    ```
    action_tutorials_py fibonacci_action_client
    action_tutorials_py fibonacci_action_server
    ```

3. Записать в файл list_exec_action_tutorials.txt
    ```
    ros2 pkg executables action_tutorials_py | awk '{print $2}' > ~/workbench/ex02/list_exec_action_tutorials.txt
    ```
    - awk — это утилита для обработки текстовых данных построчно, разбивая каждую строку на поля
    - print $2 - выводит только значение этого 2-го поля.

    #### Про ros2 pkg
    Справка по команде:
    ```
    ros2 pkg --help
    ```
    | Commands |  |
    |-|-|
    | create       | Create a new ROS 2 package |
    | executables  | Output a list of package specific executables |
    | list         | Output a list of available packages |
    | prefix       | Output the prefix path of a package |
    | xml          | Output the XML of the package manifest or a specific tag |

    Для более детальной информации использовать `ros2 pkg <command> -h`.

### ex03

Перейти в ex03

```
# Создаём структуру рабочего пространства ROS 2 (ros2-ws/src) и переходим в папку ros2-ws.
mkdir -p ros2-ws/src && cd ros2-ws

# Клонируем официальный репозиторий с примерами ROS 2 в папку src/examples, используя ветку jazzy.
git clone https://github.com/ros2/examples ./src/examples -b jazzy

# Собираем всё workspace colcon-ом с символическими ссылками, чтобы быстро обновлять изменения без полной установки.
colcon build --symlink-install

# Запускаем тесты для всех собранных пакетов в workspace, чтобы убедиться, что всё работает корректно.
colcon test

# Активируем собранное workspace — добавляем пути и переменные окружения, чтобы ROS 2 мог находить собранные пакеты.
source install/setup.bash
```
--- 
*Отступление*

#### Что такое underlay и overlay?

**Underlay** — это основное рабочее пространство или системная установка ROS 2, где находятся базовые пакеты и библиотеки (например, /opt/ros/jazzy). Оно служит фундаментом.

**Overlay** — дополнительное рабочее пространство, поверх underlay, в котором вы можете разрабатывать и изменять пакеты, не нарушая базовую установку. Overlay содержит ваши собственные версии пакетов, которые могут переопределять (перекрывать) пакеты из underlay.

- Вы можете вносить изменения и собирать пакеты в overlay, эти изменения будут использоваться при запуске.

- При одновременном наличии одного и того же пакета в overlay и underlay, ROS 2 использует версию из overlay — она имеет приоритет.

**Пример с turtlesim**
Вы устанавливаете turtlesim в underlay (основная установка ROS 2).

Вы создаёте overlay workspace, копируете туда пакет turtlesim и изменяете, например, название окна.

Собираете overlay (colcon build), вызываете:

```
ros2 run turtlesim turtlesim_node
```
ROS 2 запустит turtlesim из overlay, поскольку overlay имеет приоритет.

Если вы откроете новый терминал и не будете использовать overlay, а только underlay, то запустится оригинальный turtlesim из базовой установки.

Как работает на практике?
Для использования overlay в терминале выполняется:
```
source /opt/ros/jazzy/setup.bash  # источник underlay
source ~/ros2_ws/install/local_setup.bash  # источник overlay
```
Обе среды объединяются, и overlay кладётся "пораньше" в путь поиска пакетов.

[Подробнее тут](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#new-directory)

---

**Создание собственного пакета ROS 2 на Python и запуск**
```
# Создаём новый пакет valerie (ваше название) с типом сборки ament_python, указываем лицензию 
# и имя узла my_node (ваше название).
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node valerie

cd .. # вернуться в /ros2_ws

colcon build
# colcon build --packages-select valerie - пересобрать только valerie пакет
```

В новом терминале активируем workspace и запускаем узел my_node из пакета valerie.

```
source install/local_setup.bash
ros2 run valerie my_node
```

 > Hi from valerie.


*Замечания по редактированию*

Далее при необходимости отредактируйте файлы package.xml и setup.py в пакете valerie, особенно поля:
- maintainer
- maintainer_email
- description

Эти данные должны соответствовать друг другу для корректной сборки и документации.

### ex04

Из папки ex04 выполнить
```
colcon build --packages-select valerie > ../ex04/colcon_build.txt 2>&1
```
- 2>&1 — это перенаправление потока ошибок (stderr, дескриптор 2) в стандартный вывод (stdout, дескриптор 1). Это означает, что все сообщения об ошибках также попадут в тот же файл colcon_build.txt, а не будут показаны отдельно или в терминале.

### ex05
В отдельных терминалах:
```
ros2 run turtlesim turtle_teleop_key
``` 
и
```
ros2 node list
```

Резульат:
> /teleop_turtle

Тоже в разных терминалах
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=valerie_turtle

ros2 node list
```
Результат:
> /valerie_turtle

Получение информации о ноде
```
ros2 node info valerie_turtle > ex05/rosnode_info.txt
```

### ex06

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 run rqt_graph rqt_graph
```
- В графе rqt_graph отображаются узлы /turtlesim и /teleop_turtle, а также топик /turtle1/cmd_vel, через который они общаются.

- Узел /teleop_turtle публикует данные (нажатия клавиш для управления черепашкой) в топик /turtle1/cmd_vel.

- Узел /turtlesim подписан на этот топик и получает данные для перемещения черепашки.

```
ros2 topic list
```
Результат:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

```
ros2 topic list -t
```
Результат:
```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

Если запустить (в отдельном терминале)
```
ros2 topic echo /turtle1/cmd_vel
```
То при управлении черепашкой увидим, какие данные отправляются в топик /turtle1/cmd_vel.

```
ros2 topic info /turtle1/cmd_vel
```
Показывает подробную информацию о топике /turtle1/cmd_vel.
- **Тип сообщений (Type)**:
Например, `geometry_msgs/msg/Twist` — это тип сообщения, передаваемого по топику. Он описывает структуру данных, которую должны публиковать и подписываться ноды.

- **Количество издателей (Publisher count)**:
Сколько узлов (нод) в данный момент публикуют сообщения в этот топик.

- **Количество подписчиков (Subscription count)**:
Сколько нод в данный момент подписаны на этот топик и получают оттуда данные.

Результат:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

Теперь мы можем запустить `ros2 interface show <msg_type>` для этого типа, чтобы узнать его подробности. В частности, какую структуру данных ожидает сообщение.
```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z
```

`ros2 topic pub <topic_name> <msg_type> '<args>'` - публикует данные в топик. Аргумент '<args>' - это фактические данные, которые вы передадите в раздел в структуре, которую вы только что обнаружили в предыдущем разделе.

```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Параметр `--once` - выполнить один раз.

```
ros2 topic pub --once -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
- -w 2 - количество подписчиков (необязательный параметр)

```
# publishing turtle pose
ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'

# to see topic publications frequensy
ros2 topic hz /turtle1/pose

# to see topic bandwidth
ros2 topic bw /turtle1/pose

# to find what topics using this type of messages
ros2 topic find geometry_msgs/msg/Twist
```

**Рисование восьмерки:**
Время движения установлено 1 секунда (параметр --once — одна публикация).
Линейная скорость соответствует длине окружности круга, который нужно обойти за 1 секунду:

$V=S/t = S \text{, \\\ где } S = 2 \pi r$

Угловая скорость соответствует полному повороту за 1 секунду:

$\omega = \varphi/t = \varphi = 2 \pi$

Где:  
- $r$ — радиус круга  
- $\varphi$ — угол поворота (радианы) для полного круга (360°)  
- $V$ — линейная скорость (м/с)  
- $\omega$ — угловая скорость (радиан/с)

Для первого круга с радиусом примерно 1:

$ S=2 \pi \times 1 = 6.2831853$

$V = 6.2831853$

$ \omega = 6.2831853 $

Для круга с радиусом 2

$S = 2 \pi \times 2 = 12.5663706144$

$V = 12.5663706144$

$\omega = 6.28318530718$

$\omega$ берем с минусом, чтобы черепаха делала круг ниже первого.

```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 6.0}, angular: { z: 6.0}}"

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 12.0}, angular: { z: -6.0}}"
```

### ex07

После запуска turtlesim_node и turtle_teleop_key

**Services:**
```
ros2 service list
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/get_type_description
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/get_type_description
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

```
# Узнать тип сервиса
ros2 service type <service_name>


# Узнать тип всех активных сервисов:
ros2 service list -t

# Найти сервис по типу
ros2 service find <type_name>

# Узнуть структуру. Символ --- отделяет структуру запроса (вверху) от структуры ответа (внизу). 
ros2 interface show <type_name>

# Вызов сервиса
ros2 service call <service_name> <service_type> <arguments>

# Спавн черепашки
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

# Удалить черепаху
ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"
```

**Parametrs:**
```
# Посмотреть параметры нод
ros2 param list

# Отобразить тип и текущее значение параметра:
ros2 param get <node_name> <parameter_name>

# Задать значение параметра
ros2 param set <node_name> <parameter_name> <value>

# Просмотреть все текущие значения параметров ноды:
ros2 param dump <node_name>

# Загрузить параметры из файла
ros2 param load <node_name> <parameter_file>


```

Заспавнить 4 черепахи
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'Leonardo'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 7.0, y: 2.0, theta: 0.0, name: 'Raphael'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 7.0, theta: 0.0, name: 'Donatello'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 7.0, y: 7.0, theta: 0.0, name: 'Michelangelo'}"
```

Изменить параметр фона
```
ros2 param set /turtlesim background_g 124
```

Список серверов
```
ros2 service list > ex07/rosservice_list.txt
```

Параметры сервисов:
```
./all_parameters
```

### ex08

В разных терминалах:
```
# Запуск графического окна rqt_console для просмотра логов ROS 2
ros2 run rqt_console rqt_console

# Запуск симулятора черепахи с уровнем логирования WARN
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN

# Запуск launch файла turtlesim_mimic_launch.xml
ros2 launch turtlesim_mimic_launch.xml

# Публикация сообщений в топик управления черепахой

ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- Публикует команду скорости (Twist) в топик управления черепахой /turtlesim1/turtle1/cmd_vel.

- -r 1 — публикует сообщение с частотой 1 раз в секунду.

- linear.x = 2.0 — движение вперёд с скоростью 2.0, остальные значения — вращение и движение по другим осям.


### ex10
Запуск симулятора turtlesim

```
source ~/workbench/ex10/ros2-ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

Запуск узла обработки текстовых команд

```
source ~/workbench/ex10/ros2-ws/install/setup.bash
ros2 run text_to_cmd_vel_pkg text_to_cmd_vel
```

Публикация текстовых команд в топик /cmd_text

```
ros2 topic pub /cmd_text std_msgs/msg/String "data: 'move_forward'" -r 1
```

```
ros2 topic pub /cmd_text std_msgs/msg/String "data: 'turn_left'" -r 1
```

```
ros2 topic pub /cmd_text std_msgs/msg/String "data: 'turn_right'" -r 1
```

```
ros2 topic pub /cmd_text std_msgs/msg/String "data: 'move_backward'" -r 1
```

```
ros2 topic pub /cmd_text std_msgs/msg/String "data: 'stop'" -r 1
```

- Опция -r 1 задаёт частоту отправки сообщений (можно менять).

- Чтобы отправить сообщение один раз, используйте опцию -1 вместо -r.

Чтобы увидеть публикуемые команды, можно открыть монитор топика:
```
ros2 topic echo /cmd_text
```
