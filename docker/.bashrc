# Source global definitions
if [ -f /etc/bashrc ]; then
    . /etc/bashrc
fi

# Activate Python virtual environment
source /opt/venv/bin/activate

# Source ROS2 environments
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspaces if they exist
if [ -f "/root/ws_startup/ws_zed/install/local_setup.bash" ]; then
    source "/root/ws_startup/ws_zed/install/local_setup.bash"
fi

# Setup command completion
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# Enable bash completion
if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
    . /etc/bash_completion
fi

# Source aliases
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi 