#!/bin/bash
set -e

# Activate virtual environment first
source /opt/venv/bin/activate

# Source ROS2 base (complete setup)
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the base workspace (local setup only)
if [ -f "/root/ros2_ws/install/local_setup.bash" ]; then
    source "/root/ros2_ws/install/local_setup.bash"
fi

# Source the development workspace (local setup only)
if [ -f "/root/workspace/ws_startup/install/local_setup.bash" ]; then
    source "/root/workspace/ws_startup/install/local_setup.bash"
fi

# Welcome information
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION
echo 'ROS 2 Workspaces:' $COLCON_PREFIX_PATH
echo 'ROS 2 Domain ID:' $ROS_DOMAIN_ID
echo 'Machine IPs:' $ROS_IP
echo "---"  
echo 'Available ZED packages:'
ros2 pkg list | grep zed
echo "---------------------"    
exec "$@"
