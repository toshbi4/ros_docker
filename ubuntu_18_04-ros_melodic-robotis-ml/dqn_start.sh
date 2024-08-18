export TURTLEBOT3_MODEL=waffle
screen -dmS dq_gazebo roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
sleep 5
screen -dmS dq_algorithm roslaunch turtlebot3_dqn turtlebot3_dqn_stage_1.launch
sleep 2
screen -dmS dq_graph roslaunch turtlebot3_dqn result_graph.launch
sleep 1