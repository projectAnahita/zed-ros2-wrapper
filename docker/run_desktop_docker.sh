#!/bin/bash

# Default values
DEFAULT_IMAGE_NAME="cuda12.1_ros2_desktop_zed4.2.5"
DEFAULT_TAG="latest"
# DEFAULT_IMAGE_NAME="zed_ros2_desktop_image"
# DEFAULT_TAG="latestppp"
DETACH="fasle"

# Set detach flag
DOCKER_FLAGS="-it"  # Always interactive
if [ "$DETACH" = "true" ]; then
    DOCKER_FLAGS="$DOCKER_FLAGS -d"
fi

xhost +local:docker

# Get the current user's ID, group ID, and username
USER_ID=$(id -u)
GROUP_ID=$(id -g)
USER_NAME=$(id -un)

# Setup audio with error checking
if ! ./audio_control.sh setup; then
    echo "Audio setup failed!"
    exit 1
fi

# Create Hugging Face cache directory if it doesn't exist
mkdir -p $HOME/.cache/huggingface

# Docker run command
docker run --network host --gpus all --runtime nvidia $DOCKER_FLAGS --privileged --ipc=host --pid=host \
  --env-file $HOME/vars.env \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e HF_TOKEN=$HF_TOKEN \
  -e HF_HOME=/root/.cache/huggingface \
  -v /dev:/dev \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $HOME/Desktop/workspace:/root/workspace \
  -v $HOME/Desktop/workspace/ws_startup:/root/workspace/ws_startup \
  -v $HOME/.cache/huggingface:/root/.cache/huggingface \
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