#!/bin/bash
set -e

echo "Starting entrypoint script..."

# Base environment setup
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

# Source bash completion and aliases
if [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
fi

# Setup command completion for ROS2
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# Setup Hugging Face authentication
echo "Setting up Hugging Face authentication..."
if [ -z "${HF_TOKEN}" ]; then
    echo "WARNING: HF_TOKEN not found in environment!"
else
    if ! command -v huggingface-cli &> /dev/null; then
        pip install --quiet huggingface_hub
    fi
    huggingface-cli login --token $HF_TOKEN
    if ! huggingface-cli whoami &> /dev/null; then
        echo "ERROR: Failed to authenticate with Hugging Face!"
    else
        echo "Hugging Face authentication completed successfully"
    fi
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

# # List available ZED packages
# echo "Available ZED packages:"
# ros2 pkg list | grep zed || echo "No ZED packages found!"
# echo "---------------------"

# Execute passed command
exec "$@"