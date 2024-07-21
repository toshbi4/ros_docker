## Docker: Ubuntu 22.04 + ROS humble + Gazebo

### Docker manipulation scripts
- Install docker: `./install_docker.bash`
- Build docker: `./build_docker.sh`
- Startup docker: `./run_docker.bash`
- Connect to started docker: `./exec_docker.sh`

---
**NOTE**

Use `-n` argument with docker scripts for Nvidia graphics

---

### Startup examples

#### ROBOTICS: Slam

- Create ros workspace dir:
```
export TURTLEBOT3_MODEL=waffle
```
- Launch Gazebo simulation (in the first time you may need some pation):
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
- Operate robots movements with:
```
ros2 run turtlebot3_teleop teleop_keyboard
```
- Launch the algorithm itself and cartographer node for result checking:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
