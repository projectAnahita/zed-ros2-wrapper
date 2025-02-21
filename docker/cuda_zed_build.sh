#!/bin/bash
cd $(dirname $0)

# Default values
DEFAULT_UBUNTU="ubuntu22.04"
DEFAULT_CUDA="cuda12.1.0"
DEFAULT_ZEDSDK="zedsdk4.1.4"
DEFAULT_DOCKERFILE="Dockerfile.desktop_cuda12.1"
DEFAULT_IMAGE_NAME="zed_ros2_desktop_image"
USE_CACHE=true
TAG="latest4p"

# Extract version numbers
ubuntu_version="${DEFAULT_UBUNTU#ubuntu}"
IFS='.' read -r ubuntu_major ubuntu_minor <<< "$ubuntu_version"

cuda_version="${DEFAULT_CUDA#cuda}"
IFS='.' read -r cuda_major cuda_minor cuda_patch <<< "$cuda_version"

zed_version="${DEFAULT_ZEDSDK#zedsdk}"
IFS='.' read -r zed_major zed_minor zed_patch <<< "$zed_version"

# Prepare build context
echo "Preparing build context..."
rm -rf ./tmp_sources
mkdir -p ./tmp_sources

# Copy ROS2 packages
cp -rL ../zed* ./tmp_sources/

# Prepare build command
BUILD_CMD="docker build"
if [ "$USE_CACHE" = false ]; then
    BUILD_CMD+=" --no-cache"
fi

# Build the image
echo "Building using Dockerfile: $DEFAULT_DOCKERFILE"
echo "Building with tag: $TAG"
$BUILD_CMD -t ${DEFAULT_IMAGE_NAME}:${TAG} \
--build-arg ZED_SDK_MAJOR=$zed_major \
--build-arg ZED_SDK_MINOR=$zed_minor \
--build-arg ZED_SDK_PATCH=$zed_patch \
--build-arg UBUNTU_MAJOR=$ubuntu_major \
--build-arg UBUNTU_MINOR=$ubuntu_minor \
--build-arg CUDA_MAJOR=$cuda_major \
--build-arg CUDA_MINOR=$cuda_minor \
--build-arg CUDA_PATCH=$cuda_patch \
-f $DEFAULT_DOCKERFILE .

# Clean up temporary files
echo "Cleaning up..."
rm -rf ./tmp_sources