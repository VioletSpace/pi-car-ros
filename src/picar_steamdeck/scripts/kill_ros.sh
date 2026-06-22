xhost +SI:localuser:$USER

# Run the command within the distrobox
distrobox enter ros2-kilted <<EOF

# Get the PIDs of ROS nodes
ros_pids=\$(pgrep -f 'ros')

# Check if any ROS nodes are running
if [ -z "\$ros_pids" ]; then
    echo "No ROS nodes are currently running."
else
    # Iterate over each PID and kill the corresponding process
    for pid in \$ros_pids
    do
        echo "Killing ROS node with PID \$pid"
        kill \$pid
        # kill -9 \$pid # Force kill
    done
    echo "All ROS nodes have been killed."
fi

sleep 10s
EOF