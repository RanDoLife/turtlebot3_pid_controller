# Порядок запуска 
Добавочно склонировать:
https://github.com/RanDoLife/turtlebot3_lidar_tracker.git
## Выполнить (репозиторий по ссылке)
Запуск мира и модели turtlebot3:
```bash
roslaunch turtlebot3_lidar_tracker wall_world.launch
```
Запуск фильтра для scan:
```bash
rosrun turtlebot3_lidar_tracker scan_filter.py
```
Запуск трекера движения:
```bash
roslaunch turtlebot3_lidar_tracker wall_motion.launch
```
##  Выполнить (текущий репозиторий)
```bash
rosrun turtlebot3_pid_controller turtlebot3_pid_controller.py
```
