#!/bin/bash

echo "Cleaning up Hugging Face authentication..."
if command -v huggingface-cli &> /dev/null; then
    huggingface-cli logout || true
fi

# Cleanup audio
./audio_control.sh cleanup

# Change ownership and permissions
if [ -d "$HOME/zed_docker_ai" ]; then
    echo "Changing ownership and permissions of the zed_docker_ai directory..."
    sudo chown -R $USER:$USER $HOME/zed_docker_ai/
    chmod -R ugo+rw $HOME/zed_docker_ai/
fi

if [ -d "$HOME/Desktop/workspace" ]; then
    echo "Changing ownership of the workspace directory..."
    sudo chown -R $USER:$USER $HOME/Desktop/workspace
fi

xhost -local:docker 