#!/bin/bash

echo "Cleaning up Hugging Face authentication..."
if command -v huggingface-cli &> /dev/null; then
    huggingface-cli logout || true
fi

# Cleanup audio
./audio_control.sh cleanup

# Change ownership of all mounted volumes back to host user
echo "Changing ownership of workspace directories..."
if [ -d "$HOME/Desktop/workspace" ]; then
    echo "Fixing permissions for workspace directory..."
    sudo chown -R $USER:$USER $HOME/Desktop/workspace
fi

if [ -d "$HOME/.cache/huggingface" ]; then
    echo "Fixing permissions for Hugging Face cache..."
    sudo chown -R $USER:$USER $HOME/.cache/huggingface
fi

# Revoke Docker X11 access
xhost -local:docker

echo "Cleanup completed!" 