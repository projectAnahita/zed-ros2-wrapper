#!/bin/bash
cd $(dirname $0)

# Default values
DEFAULT_UBUNTU="ubuntu22.04"
DEFAULT_CUDA="cuda12.1.0"
DEFAULT_ZEDSDK="zedsdk4.1.4"
DEFAULT_DOCKERFILE="Dockerfile.desktop-humble"
DEFAULT_IMAGE_NAME="zed_ros2_desktop_image"
USE_CACHE=true
TAG="latest"

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS] [VERSIONS]"
    echo
    echo "Build Docker image for ZED ROS2 Desktop"
    echo
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  --no-cache             Build without using Docker cache"
    echo "  -t, --tag TAG          Specify Docker image tag (default: latest)"
    echo "  -f, --file DOCKERFILE  Specify Dockerfile to use (default: $DEFAULT_DOCKERFILE)"
    echo
    echo "Versions (optional):"
    echo "  ubuntu=VERSION         Ubuntu version (default: $DEFAULT_UBUNTU)"
    echo "  cuda=VERSION           CUDA version (default: $DEFAULT_CUDA)"
    echo "  zedsdk=VERSION        ZED SDK version (default: $DEFAULT_ZEDSDK)"
    echo
    echo "Examples:"
    echo "  $0 --no-cache -t dev -f Dockerfile.custom"
    echo "  $0 cuda=cuda12.1.0 zedsdk=zedsdk4.1.2"
    echo "  $0 # Uses all defaults"
}

# Parse command line arguments
POSITIONAL_ARGS=()
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
        ubuntu=*)
            DEFAULT_UBUNTU="${1#*=}"
            shift
            ;;
        cuda=*)
            DEFAULT_CUDA="${1#*=}"
            shift
            ;;
        zedsdk=*)
            DEFAULT_ZEDSDK="${1#*=}"
            shift
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

# Check if Dockerfile exists
if [ ! -f "$DEFAULT_DOCKERFILE" ]; then
    echo "Error: Dockerfile '$DEFAULT_DOCKERFILE' not found"
    exit 1
fi

# Validate Ubuntu version
if ! [[ $DEFAULT_UBUNTU =~ ^ubuntu[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid Ubuntu version format: $DEFAULT_UBUNTU"
    exit 1
fi
ubuntu_version_number="${DEFAULT_UBUNTU#ubuntu}"
IFS='.' read -r ubuntu_major ubuntu_minor <<< "$ubuntu_version_number"
echo "Ubuntu $ubuntu_major.$ubuntu_minor detected."

# Validate CUDA version
if ! [[ $DEFAULT_CUDA =~ ^cuda[0-9]+\.[0-9]\.[0-9]$ ]]; then
    echo "Invalid CUDA version format: $DEFAULT_CUDA"
    exit 1
fi
cuda_version_number="${DEFAULT_CUDA#cuda}"
IFS='.' read -r cuda_major cuda_minor cuda_patch <<< "$cuda_version_number"
echo "CUDA $cuda_major.$cuda_minor.$cuda_patch detected."

# Validate ZED SDK version
if ! [[ $DEFAULT_ZEDSDK =~ ^zedsdk[0-9]\.[0-9]\.[0-9]$ ]]; then
    echo "Invalid ZED SDK version format: $DEFAULT_ZEDSDK"
    exit 1
fi
zed_sdk_version_number="${DEFAULT_ZEDSDK#zedsdk}"
IFS='.' read -r major minor patch <<< "$zed_sdk_version_number"
echo "ZED SDK $major.$minor.$patch detected."

# Prepare build context
echo "Preparing build context..."
rm -rf ./tmp_sources
mkdir -p ./tmp_sources
cp -rL ../zed* ./tmp_sources/

# # Ensure dos2unix is installed
# if ! command -v dos2unix &> /dev/null; then
#     echo "dos2unix not found. Installing..."
#     sudo apt-get update && sudo apt-get install -y dos2unix
# fi
# 
# # Convert line endings for scripts (excluding tmp_sources)
# echo "Converting line endings for scripts..."
# find . -maxdepth 1 -type f \( -name "*.sh" -o -name ".bash_aliases" -o -name "Dockerfile*" \) -exec dos2unix {} \;


# Prepare build command
BUILD_CMD="docker build"
if [ "$USE_CACHE" = false ]; then
    BUILD_CMD+=" --no-cache"
fi

echo "Building using Dockerfile: $DEFAULT_DOCKERFILE"
echo "Building with tag: $TAG"
$BUILD_CMD -t ${DEFAULT_IMAGE_NAME}:${TAG} \
--build-arg ZED_SDK_MAJOR=$major \
--build-arg ZED_SDK_MINOR=$minor \
--build-arg ZED_SDK_PATCH=$patch \
--build-arg UBUNTU_MAJOR=$ubuntu_major \
--build-arg UBUNTU_MINOR=$ubuntu_minor \
--build-arg CUDA_MAJOR=$cuda_major \
--build-arg CUDA_MINOR=$cuda_minor \
--build-arg CUDA_PATCH=$cuda_patch \
-f $DEFAULT_DOCKERFILE .

# Clean up temporary files
echo "Cleaning up..."
rm -rf ./tmp_sources
rm -f ./.bash_aliases