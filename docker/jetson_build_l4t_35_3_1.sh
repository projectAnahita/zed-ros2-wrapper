#!/bin/bash
cd $(dirname $0)

if [ "$#" -lt 1 ]; then
    echo "Please enter ZED SDK version and optional tag. For example:"
    echo "./jetson_build_dockerfile_from_sdk_and_l4T_version.sh zedsdk-4.2.5 [tag]"
    exit 1
fi

# Verify the ZED SDK format (zedsdk-digits.digits.digits)
if ! [[ $1 =~ ^zedsdk-[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid ZED SDK version format."
    exit 1
fi

# Hardcoded L4T version from base image
l4t_major=35
l4t_minor=3
l4t_patch=1

# Get ZED SDK version
ZED_SDK_version=$1
# Remove the prefix 'zedsdk-'
zed_sdk_version_number="${ZED_SDK_version#zedsdk-}"
IFS='.' read -r sdk_major sdk_minor sdk_patch <<< "$zed_sdk_version_number"

# Set image name
IMAGE_NAME="zed_ros2_l4t_${l4t_major}.${l4t_minor}.${l4t_patch}_sdk_${sdk_major}.${sdk_minor}.${sdk_patch}"

# Add tag if provided
if [ ! -z "$2" ]; then
    IMAGE_NAME="${IMAGE_NAME}:$2"
else
    IMAGE_NAME="${IMAGE_NAME}:latest"
fi

# copy the wrapper content
rm -rf ./tmp_sources
mkdir -p ./tmp_sources
cp -r ../zed* ./tmp_sources

echo "Building dockerfile for L4T r${l4t_major}.${l4t_minor}.${l4t_patch} and ZED SDK $1"
docker build -t ${IMAGE_NAME} \
    --build-arg ZED_SDK_MAJOR=${sdk_major} \
    --build-arg ZED_SDK_MINOR=${sdk_minor} \
    --build-arg ZED_SDK_PATCH=${sdk_patch} \
    -f ./Dockerfile.l4t_v35_3_1-zedsdk_v4_2_5_wrapper .

# Remove the temporary folder
rm -rf ./tmp_sources

echo "Build complete! Image name: ${IMAGE_NAME}"