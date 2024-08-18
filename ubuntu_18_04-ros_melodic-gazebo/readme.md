## Docker: Ubuntu 18.04 + ROS melodic + Gazebo

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

- Create ros workspace dir: `mkdir -p catkin_ws/src`
- Move all folders from `./examples` to `catkin_ws/src`
- Run docker: `./run_docker.bash`
- Change dir inside docker: `<Your path>/catkin_ws`
- Build copied ros packages: `catkin build`
- Source Ros env variables: `source /catkin_ws/devel/setup.bash`
- Start the quadcopter scene: `roslaunch drone_scene start_scene.launch`
- Run script with drone control commands: `rosrun drone_solution line_follower.py`

