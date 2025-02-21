#!/bin/bash
cd $(dirname $0)

# Default values
DEFAULT_DOCKERFILE="Dockerfile.desktop_cuda12.1_only"
DEFAULT_IMAGE_NAME="cuda12_1_ros2_desktop"
USE_CACHE=true
TAG="latest"

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "Build Docker image for CUDA 12.1 ROS2 Desktop"
    echo
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  --no-cache             Build without using Docker cache"
    echo "  -t, --tag TAG          Specify Docker image tag (default: latest)"
    echo "  -f, --file DOCKERFILE  Specify Dockerfile to use (default: $DEFAULT_DOCKERFILE)"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-cache)
            USE_CACHE=false
            shift
            ;;
        -t|--tag)
            TAG="$2"
            shift 2
            ;;
        -f|--file)
            DEFAULT_DOCKERFILE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check if Dockerfile exists
if [ ! -f "$DEFAULT_DOCKERFILE" ]; then
    echo "Error: Dockerfile '$DEFAULT_DOCKERFILE' not found"
    exit 1
fi

# Prepare build command
BUILD_CMD="docker build"
if [ "$USE_CACHE" = false ]; then
    BUILD_CMD+=" --no-cache"
fi

echo "Building using Dockerfile: $DEFAULT_DOCKERFILE"
echo "Building with tag: $TAG"
$BUILD_CMD -t ${DEFAULT_IMAGE_NAME}:${TAG} -f $DEFAULT_DOCKERFILE . 