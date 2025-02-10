#!/bin/bash

# Set dependency versions (matching Dockerfile.l4t-humble exactly)
XACRO_VERSION=2.0.8
DIAGNOSTICS_VERSION=3.0.0
AMENT_LINT_VERSION=0.12.4
GEOGRAPHIC_INFO_VERSION=1.0.4
ROBOT_LOCALIZATION_VERSION=3.4.2
DRACO_VERSION=1.5.7

# Function to clean workspace
clean_workspace() {
    echo "Cleaning workspace..."
    rm -rf build/ install/ log/
    cd src/
    rm -rf * # Remove all contents in src directory
    cd ..
}

# Install system dependencies
echo "Installing system dependencies..."
apt-get update && apt-get install -y \
    libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Draco compression library
echo "Installing Draco compression library..."
cd /tmp
git clone https://github.com/google/draco.git
cd draco
git checkout ${DRACO_VERSION}
mkdir build
cd build

# Configure Draco with all needed options
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DDRACO_POINT_CLOUD_COMPRESSION=ON \
    -DDRACO_MESH_COMPRESSION=ON \
    -DDRACO_STANDARD_EDGEBREAKER=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DDRACO_ANIMATION_ENCODING=OFF \
    -DDRACO_TRANSCODER_SUPPORTED=OFF \
    -DCMAKE_CXX_FLAGS="-fvisibility=default"

make -j$(nproc)
make install

# Configure library path for Draco
ldconfig

cd /
rm -rf /tmp/draco

# Create workspace
mkdir -p /root/ws_startup/ws_zed/src
cd /root/ws_startup/ws_zed

# Clean the workspace BEFORE cloning repositories
clean_workspace

cd src

# Clone all repositories AFTER cleaning
echo "Cloning all required repositories..."

# Clone ZED ROS2 wrapper (moved here, after cleaning)
echo "Cloning ZED ROS2 wrapper..."
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git --branch humble-v4.1.4

# Clone point_cloud_transport and dependencies
echo "Cloning point_cloud_transport and dependencies..."
git clone https://github.com/ros2/rcpputils.git --branch humble
git clone https://github.com/ros-perception/point_cloud_transport.git --branch humble
git clone https://github.com/ros-perception/point_cloud_transport_plugins.git --branch humble

# Install other dependencies
echo "Installing missing dependencies..."
wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro
wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics
wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint
wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization
wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info
cp -r geographic-info/geographic_msgs/ .
rm -rf geographic-info
git clone https://github.com/ros-drivers/nmea_msgs.git --branch ros2
git clone https://github.com/ros/angles.git --branch humble-devel

cd ..  # Back to workspace root

# Build workspace in stages
source /opt/ros/humble/setup.bash || true

# # Build point cloud related packages first
# echo "Building point cloud related packages..."
# colcon build --symlink-install \
#     --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
#     " -DCMAKE_LIBRARY_PATH=/usr/lib:/usr/local/lib:/usr/lib/aarch64-linux-gnu" \
#     --packages-up-to rcpputils point_cloud_transport draco_point_cloud_transport && \
# source install/setup.bash

# Build the complete workspace
echo "Building the complete workspace..."
# colcon build --parallel-workers $(nproc) --symlink-install \
#     --event-handlers console_direct+ --base-paths src \
#     --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
#     " -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs:/usr/lib:/usr/local/lib:/usr/lib/aarch64-linux-gnu" \
#     " -DCMAKE_CXX_FLAGS='-Wl,--allow-shlib-undefined -Wl,-rpath,/usr/lib:/usr/local/lib:/usr/lib/aarch64-linux-gnu'" \
#     " -DCMAKE_PREFIX_PATH=$(pwd)/install:/usr/" \
#     " --no-warn-unused-cli"
colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src \
    --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
    " -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs" \
    " -DCMAKE_CXX_FLAGS='-Wl,--allow-shlib-undefined'" \
    " --no-warn-unused-cli"
source install/setup.bash

# Add sourcing to .bashrc
if ! grep -q "source /root/ws_startup/ws_zed/install/local_setup.bash" ~/.bashrc; then
    echo "Adding workspace sourcing to ~/.bashrc..."
    echo "source /root/ws_startup/ws_zed/install/local_setup.bash" >> ~/.bashrc
fi

echo "Installation complete! The ZED ROS2 Wrapper is now ready."