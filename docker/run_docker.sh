#!/bin/bash

# Default values
DEFAULT_IMAGE_NAME="zed_ros2_desktop_image"
DEFAULT_TAG="latestplus"

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "Run Docker container for ZED ROS2 Desktop"
    echo
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -i, --image NAME        Specify Docker image name (default: $DEFAULT_IMAGE_NAME)"
    echo "  -t, --tag TAG           Specify Docker image tag (default: $DEFAULT_TAG)"
    echo
    echo "Examples:"
    echo "  $0 --image custom_image --tag latest"
    echo "  $0 -i my_image -t dev"
    echo "  $0 # Uses defaults"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -i|--image)
            DEFAULT_IMAGE_NAME="$2"
            shift 2
            ;;
        -t|--tag)
            DEFAULT_TAG="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

xhost +local:docker

# Get the current user's ID and group ID
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Setup audio with error checking
if ! ./audio_control.sh setup; then
    echo "Audio setup failed!"
    exit 1
fi

# Create Hugging Face cache directory if it doesn't exist
mkdir -p $HOME/.cache/huggingface

# Docker run command
docker run --network host --gpus all --runtime nvidia -it --privileged --ipc=host --pid=host \
  --env-file $HOME/vars.env \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e HF_TOKEN=$HF_TOKEN \
  -e TRANSFORMERS_CACHE=/root/.cache/huggingface \
  -e HF_HOME=/root/.cache/huggingface \
  -v /dev:/dev \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $HOME/Desktop/workspace/ws_startup:/root/workspace/ws_startup \
  -v $HOME/.cache/huggingface:/root/.cache/huggingface \
  -e KA_CONFIG_PATH=/root/workspace/ws_startup/artifex/kuka_assistant/config/kuka_assistant_config.yaml \
  -v "$HOME/Desktop/workspace/ws_zed/zed_components":/root/workspace/ws_zed/src/zed_components \
  -v "$HOME/Desktop/workspace/ws_zed/zed-ros2-interfaces":/root/workspace/ws_zed/src/zed-ros2-interfaces \
  -v "$HOME/Desktop/workspace/ws_zed/zed_ros2":/root/workspace/ws_zed/src/zed_ros2 \
  -v "$HOME/Desktop/workspace/ws_zed/zed_wrapper":/root/workspace/ws_zed/src/zed_wrapper \
  -v /run/user/$USER_ID/pulse:/run/user/1000/pulse \
  -v $HOME/.config/pulse/cookie:/root/.config/pulse/cookie \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  -e PULSE_COOKIE=/root/.config/pulse/cookie \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=0 \
  ${DEFAULT_IMAGE_NAME}:${DEFAULT_TAG}

# Cleanup Hugging Face (will run even if docker exits with error)
echo "Cleaning up Hugging Face authentication..."
if command -v huggingface-cli &> /dev/null; then
    huggingface-cli logout || true  # Continue even if logout fails
fi

# Cleanup audio (will run even if docker exits with error)
./audio_control.sh cleanup

# Change ownership and permissions of the zed_docker_ai directory after exiting the container
if [ -d "$HOME/zed_docker_ai" ]; then
    echo "Changing ownership and permissions of the zed_docker_ai directory..."
    sudo chown -R $USER:$USER $HOME/zed_docker_ai/
    chmod -R ugo+rw $HOME/zed_docker_ai/
fi

# Change ownership of the workspace directory if it exists
if [ -d "$HOME/Desktop/workspace" ]; then
    echo "Changing ownership of the workspace directory..."
    sudo chown -R $USER:$USER $HOME/Desktop/workspace
fi

# Revoke access to the X server
xhost -local:docker