### Task2

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