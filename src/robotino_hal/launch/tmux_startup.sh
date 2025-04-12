#!/bin/bash
source /home/robotino/.bashrc
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
# Set Session Name
SESSION="ROS2"
#SESSIONEXISTS=$(tmux list-sessions | grep $SESSION 2>/dev/null)
# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
    # Start New Session with our name
    tmux new-session -d -s $SESSION
    tmux rename-window -t 0 'HAL'
    tmux send -t 'HAL' 'ros2 launch robotino_hal total.launch.py' ENTER
    sleep 5
    tmux new-window -t $SESSION:1 -n 'URDF'
    tmux send -t 'URDF' 'ros2 launch robotino_hal display.launch.py' ENTER
    sleep 5
    tmux new-window -t $SESSION:2 -n 'nav2'
    tmux send -t 'nav2' 'ros2 launch robotino_hal bringup.launch.py' ENTER
    sleep 5
    #tmux new-window -t $SESSION:3 -n 'SLAM1'
    #tmux send-keys -t 'SLAM1' 'ros2 launch slam_toolbox online_async_launch.py' C-m
    #tmux new-window -t $SESSION:4 -n 'SLAM2'
    #tmux send-keys -t 'SLAM2' 'ros2 launch robotino_hal navigation.launch' C-m
    echo "ROS2 tmux session created successfully"
else
    echo "ROS2 tmux session already exists"
fi