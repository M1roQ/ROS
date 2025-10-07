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