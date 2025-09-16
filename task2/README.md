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