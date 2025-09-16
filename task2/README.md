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
```
cd ex03

# Создаём структуру рабочего пространства ROS 2 (ros2-ws/src) и переходим в папку ros2-ws.
mkdir -p ros2-ws/src && cd ros2-ws

# Клонируем официальный репозиторий с примерами ROS 2 в папку src/examples, используя ветку jazzy.
git clone https://github.com/ros2/examples ./src/examples -b jazzy

# Собираем всё workspace colcon-ом с символическими ссылками (--symlink-install), чтобы быстро обновлять изменения без полной установки.
colcon build --symlink-install

# Запускаем тесты для всех собранных пакетов в workspace, чтобы убедиться, что всё работает корректно.
colcon test

# Активируем собранное workspace — добавляем пути и переменные окружения, чтобы ROS 2 мог находить собранные пакеты.
source install/setup.bash
```

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

**Создание собственного пакета ROS 2 на Python и запуск**
```
# Создаём новый пакет valerie (ваше название) с типом сборки ament_python, указываем лицензию и имя узла my_node (ваше название).
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node valerie


cd .. # вернуться в /ros2_ws


colcon build
# colcon build --packages-select valerie - пересобрать только valerie пакет
```
```
# В новом терминале активируем workspace и запускаем ваш узел my_node из пакета valerie.
source install/local_setup.bash
ros2 run valerie my_node
```
 > Hi from valerie.
```
*Замечания по редактированию*
Далее при необходимости отредактируйте файлы package.xml и setup.py в пакете valerie, особенно поля:
- maintainer
- maintainer_email
- description

Эти данные должны соответствовать друг другу для корректной сборки и документации.