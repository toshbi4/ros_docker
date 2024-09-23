## Docker: Ubuntu 18.04 + ROS melodic + Gazebo + DQN

### Docker manipulation scripts
- Install docker: `./install_docker.bash`
- Build docker: `make build` `make build-nvidia`
- Startup docker: `make run` `make run-nvidia`
- Connect to started docker: `make connect`

### Startup examples

- Startup docker: `make run-nvidia`
- Inside docker console: `bash ../dqn_start.sh`

