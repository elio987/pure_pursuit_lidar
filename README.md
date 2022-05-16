# pure_pursuit_lidar

- Para poder correr el codigo corre:

```
roslaunch puzzlebot_gazebo puzzlebot_room.launch
```
- Despues en una nueva terminal:

```
roslaunch puzzlebot_nav2d puzzlebot_navigation.launch
```
- En una nueva terminal:
```
roslaunch puzzlebot_nav2d pure_pursuit.launch
```
- En caso de querer ingresar nuevos waypoints ve al archivo:
```
/puzzlebot_nav2d/config/waypoints.yalm
```
- En ese archivo mete los dos componentes de los waypoints separados por una coma. Ejemplo:
```
waypoints: [0, 0, 0, 4, 2, 5, 4, 2, 5, 1, 7, 1, 8.5, 3.5, 7, 7]
```
- Arbol usado:
![Alt text](PyTree.drawio.png?raw=true "Behaviour Tree")
