#!/bin/zsh

#distrobox container name
CONTAINER_NAME="02f5db314aaa"
WRAPPER="source /opt/ros/humble/setup.zsh &&"

distrobox enter $CONTAINER_NAME -- pkill -9 -f nav2
distrobox enter $CONTAINER_NAME -- pkill -9 -f lifecycle_manager
distrobox enter $CONTAINER_NAME -- pkill -9 -f gzserver
distrobox enter $CONTAINER_NAME -- pkill -9 -f gzclient

konsole --new-tab -e zsh -c "distrobox enter $CONTAINER_NAME -- zsh -c '$WRAPPER && ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True; zsh'" &
sleep 3

konsole --new-tab -e zsh -c "distrobox enter $CONTAINER_NAME -- zsh -c '$WRAPPER && ros2 launch gazebo_ros gazebo.launch.py world:=./simulation/config/mdd_turtlebot_test_gazebo_world.world; zsh'" &
sleep 10

konsole --new-tab -e zsh -c "distrobox enter $CONTAINER_NAME -- zsh -c '$WRAPPER && ros2 launch nav2_bringup navigation_launch.py params_file:=/simulation/config/my_nav2_params.yaml; zsh'" &
sleep 10

konsole --new-tab -e zsh -c "distrobox enter $CONTAINER_NAME -- zsh -c '$WRAPPER && python3 ./simulation/oak_node_mock.py --ros-args -p use_sim_time:=true; zsh'" &
sleep 3

konsole --new-tab -e zsh -c "distrobox enter $CONTAINER_NAME -- zsh -c '$WRAPPER && python3 state_machine.py; zsh'" &
