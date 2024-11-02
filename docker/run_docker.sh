#!/bin/bash

xhost +local:docker

# Get the current user's ID and group ID
USER_ID=$(id -u)
GROUP_ID=$(id -g)


# Setup audio with error checking
if ! ./audio_control.sh setup; then
    echo "Audio setup failed!"
    exit 1
fi

docker run --network host --gpus all --runtime nvidia -it --privileged --ipc=host --pid=host \
  --env-file $HOME/vars.env \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -v /dev:/dev \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $HOME/Desktop/workspace/ws_startup:/root/workspace/ws_startup \
  -v /run/user/$USER_ID/pulse:/run/user/1000/pulse \
  -v $HOME/.config/pulse/cookie:/root/.config/pulse/cookie \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  -e PULSE_COOKIE=/root/.config/pulse/cookie \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=0 \
  zed_ros2_desktop_image:ws_zed /bin/bash

# Cleanup audio (will run even if docker exits with error)
./audio_control.sh cleanup

# Change ownership and permissions of the zed_docker_ai directory after exiting the container
echo "Changing ownership and permissions of the zed_docker_ai directory..."
sudo chown -R $USER:$USER $HOME/zed_docker_ai/
chmod -R ugo+rw $HOME/zed_docker_ai/

# Change ownership of the workspace directory
echo "Changing ownership of the workspace directory..."
sudo chown -R $USER:$USER $HOME/Desktop/workspace

# Revoke access to the X server
xhost -local:docker