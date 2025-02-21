#!/bin/bash

# Extract version numbers from environment or use defaults
ZED_SDK_MAJOR=${ZED_SDK_MAJOR:-4}
ZED_SDK_MINOR=${ZED_SDK_MINOR:-2}
ZED_SDK_PATCH=${ZED_SDK_PATCH:-5}
UBUNTU_MAJOR=${UBUNTU_MAJOR:-22}
CUDA_MAJOR=${CUDA_MAJOR:-12}
CUDA_MINOR=${CUDA_MINOR:-1}

# Construct the ZED SDK URL
ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/ZED_SDK_Ubuntu${UBUNTU_MAJOR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}_v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}.zstd.run"

echo "Checking ZED SDK URL:"
echo "${ZED_SDK_URL}"
echo

# Use curl to check if the URL is accessible
if curl --output /dev/null --silent --head --fail "$ZED_SDK_URL"; then
    echo "✅ URL is valid and accessible"
    exit 0
else
    echo "❌ URL is not accessible"
    echo "Please verify:"
    echo "- ZED SDK version: ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}"
    echo "- Ubuntu version: ${UBUNTU_MAJOR}"
    echo "- CUDA version: ${CUDA_MAJOR}.${CUDA_MINOR}"
    exit 1
fi
