#!/bin/bash

# Default values
DEFAULT_IMAGE_NAME="zed_ros2_l4t_35.3.1_sdk_4.2.5"
DEFAULT_TAG="latest"
DETACH="false"

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
    echo "  -d, --detach           Run container in background (default: $DETACH)"
    echo
    echo "Examples:"
    echo "  $0 --image custom_image --tag latest"
    echo "  $0 -i my_image -t dev"
    echo "  $0 -d  # Run in background"
    echo "  $0     # Run in foreground"
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
        -d|--detach)
            DETACH="true"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Set detach flag
DOCKER_FLAGS="-it"  # Always interactive
if [ "$DETACH" = "true" ]; then
    DOCKER_FLAGS="$DOCKER_FLAGS -d"
fi

xhost +local:docker

# Get the current user's ID and group ID
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Setup audio with error checking
if ! ./audio_control.sh setup; then
    echo "Audio setup failed!"
    # exit 1
fi

# Create Hugging Face cache directory if it doesn't exist
mkdir -p $HOME/.cache/huggingface

# Docker run command
docker run --network host --gpus all --runtime nvidia $DOCKER_FLAGS --privileged --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e HF_TOKEN=$HF_TOKEN \
  -v $HOME/.cache/huggingface:/root/.cache/huggingface \
  -e HF_HOME=/root/.cache/huggingface \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $HOME/Desktop/workspace/:/root/workspace/ \
  -e KA_CONFIG_PATH=/root/workspace/ws_startup/artifex/kuka_assistant/config/kuka_assistant_config.yaml \
  -v /run/user/$USER_ID/pulse:/run/user/1000/pulse \
  -v $HOME/.config/pulse/cookie:/root/.config/pulse/cookie \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  -e PULSE_COOKIE=/root/.config/pulse/cookie \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=0 \
  ${DEFAULT_IMAGE_NAME}:${DEFAULT_TAG}

# Run cleanup for non-detached mode
if [ "$DETACH" = "false" ]; then
    ./cleanup_docker.sh
else
    echo "Container started in detached mode. Run ./cleanup_docker.sh when you want to clean up."
fi