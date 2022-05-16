# pure_pursuit_lidar
- Recuerda copiar los paquetes en el src de tu worspace de ROS y hacer el catkin_make y el source devel/setup.bash.<br>
Estos paquetes mueven el robot de dos ruedas en un entorno de gazebo con paredes, yendo a una serie de puntos usando un arbol de desiciones<br>
y el algoritmo PurePursuit.
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
