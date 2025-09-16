### Task2

1. получить путь к ROS пакету «ros2topic».
    ```
    valerie@valbook:~/workbench$ ros2 pkg prefix ros2topic
    ```
    Резульат: /opt/ros/jazzy
    
2. Путь к пакету action_tutorials_py:
    ```
    valerie@valbook:~/workbench$ ros2 pkg prefix action_tutorials_py
    ```
    Результат: /opt/ros/jazzy

3. Сохранить путь в переменную:
    ```
    ACTION_TUTORIALS_PATH=$(ros2 pkg prefix action_tutorials_py)
    ```

4. Просмотреть исполняемые файлы:
    ```
    ls -l $ACTION_TUTORIALS_PATH/lib/action_tutorials_py/ | grep -E '^-.{2}x' | awk '{print $9}' > ~/workbench/ex02/list_exec_action_tutorials.txt
    ```
    - $ACTION_TUTORIALS_PATH
    - grep -E '^-.{2}x' - фильтрует строки, где есть право на выполнение хотя бы для владельца
        - -E - расширенный режим регулярных выражений
        - ^-.{2}x - шаблон поиска строк, которые начинаются (^) с символа -, за которым следуют любые 2 символа (.{2}), и на четвёртом месте стоит символ x
        - awk — это утилита для обработки текстовых данных построчно, разбивая каждую строку на поля
        - print $9 - выводит только значение этого 9-го поля.
    
    Результат: 
    ```
    fibonacci_action_client
    fibonacci_action_server
    ```
