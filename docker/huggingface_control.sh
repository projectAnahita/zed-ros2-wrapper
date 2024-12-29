#!/bin/bash

# Function to setup Hugging Face authentication
setup_huggingface() {
    echo "Setting up Hugging Face authentication..."
    
    # Check if token exists in environment
    if [ -z "${HF_TOKEN}" ]; then
        echo "WARNING: HF_TOKEN not found in environment!"
        echo "Please set the environment variable before running:"
        echo "export HF_TOKEN=your_token_here"
        echo "Or add it to your docker run command:"
        echo "docker run -e HF_TOKEN=your_token_here ..."
        return 1
    fi
    
    # Install huggingface_hub CLI if not present
    if ! command -v huggingface-cli &> /dev/null; then
        echo "Installing Hugging Face CLI..."
        pip install --quiet huggingface_hub
    fi
    
    # Login using the CLI tool with the --token option
    echo "Logging in to Hugging Face..."
    huggingface-cli login --token $HF_TOKEN
    
    # Verify login was successful
    if ! huggingface-cli whoami &> /dev/null; then
        echo "ERROR: Failed to authenticate with Hugging Face!"
        echo "Please check your token is valid"
        return 1
    fi
    
    echo "Hugging Face authentication completed successfully"
    return 0
}

# Function to cleanup Hugging Face authentication
cleanup_huggingface() {
    echo "Cleaning up Hugging Face authentication..."
    
    # Unset environment variables first
    unset HF_TOKEN
    unset HUGGING_FACE_HUB_TOKEN
    
    # Logout from Hugging Face
    if command -v huggingface-cli &> /dev/null; then
        huggingface-cli logout || true  # Continue even if logout fails
    fi
    
    # Remove token file if it exists and we have permissions
    if [ -f "/root/.cache/huggingface/token" ]; then
        rm -f /root/.cache/huggingface/token || true
    fi
    
    echo "Hugging Face cleanup completed"
}

# Check for command line argument
if [ "$1" = "setup" ]; then
    setup_huggingface
elif [ "$1" = "cleanup" ]; then
    cleanup_huggingface
else
    echo "Usage: $0 [setup|cleanup]"
    exit 1
fi 