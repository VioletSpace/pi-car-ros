#!/usr/bin/env bash

xhost +SI:localuser:$USER

# Run the command within the distrobox
distrobox enter ros2-kilted <<EOF
source /opt/ros/kilted/setup.bash
source /home/deck/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0

# Launch the ROS2 system
ros2 launch picar_steamdeck system.launch

EOF
