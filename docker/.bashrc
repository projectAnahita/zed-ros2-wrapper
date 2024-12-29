# Source global definitions
if [ -f /etc/bashrc ]; then
    . /etc/bashrc
fi

# Activate Python virtual environment
source /opt/venv/bin/activate

# Source ROS2 environments
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspaces if they exist
for ws in "ws_zed" "ws_startup"; do
    if [ -f "/root/workspace/${ws}/install/local_setup.bash" ]; then
        source "/root/workspace/${ws}/install/local_setup.bash"
    fi
done

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