#!/bin/bash
set -e

# Activate Python venv and source ROS2 environment
source /opt/venv/bin/activate
source /opt/ros/$ROS_DISTRO/setup.bash || {
    echo "Error: Failed to source ROS 2 environment"
    exit 1
}

# Source workspaces if they exist
if [ -f "/root/workspace/ws_zed/install/local_setup.bash" ]; then
    source "/root/workspace/ws_zed/install/local_setup.bash"
fi
if [ -f "/root/workspace/ws_startup/install/local_setup.bash" ]; then
    source "/root/workspace/ws_startup/install/local_setup.bash"
fi

# Verify ROS2 installation and environment
if ! which ros2 &>/dev/null; then
    echo "Error: ROS 2 CLI not found"
    exit 1
fi

# Print environment info
echo "---------------------"
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo "ROS distro: ${ROS_DISTRO:-Not Set}"
echo "DDS middleware: ${RMW_IMPLEMENTATION:-Not Set}"
echo "ROS2 Workspaces: ${COLCON_PREFIX_PATH:-Not Set}"
echo "ROS2 Domain ID: ${ROS_DOMAIN_ID:-Not Set}"
echo "Machine IPs: ${ROS_IP:-Not Set}"
echo "---------------------"

# List available ZED packages
echo "Available ZED packages:"
ros2 pkg list | grep zed || echo "No ZED packages found!"
echo "---------------------"

# Change to workspace directory
cd /root/workspace

# Execute passed command
exec "$@"