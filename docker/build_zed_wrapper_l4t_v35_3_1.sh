#!/bin/bash

# Set dependency versions (matching Dockerfile.l4t-humble exactly)
XACRO_VERSION=2.0.8
DIAGNOSTICS_VERSION=3.0.0
AMENT_LINT_VERSION=0.12.4
GEOGRAPHIC_INFO_VERSION=1.0.4
ROBOT_LOCALIZATION_VERSION=3.4.2
DRACO_VERSION=1.5.6

# Function to clean workspace
clean_workspace() {
    echo "Cleaning workspace..."s
    rm -rf build/ install/ log/
    cd src/
    rm -rf * # Remove all contents in src directory
    cd ..
}

# Clean the workspace
clean_workspace

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
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DDRACO_ANIMATION_ENCODING=OFF \
    -DDRACO_TRANSCODER_SUPPORTED=OFF

make -j$(nproc)
make install

# Configure library path for Draco
echo "/usr/local/lib" > /etc/ld.so.conf.d/draco.conf
ldconfig

cd /
rm -rf /tmp/draco

# Create workspace and clone ZED wrapper
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone ZED ROS2 wrapper with submodules (this includes zed-ros2-interfaces)
echo "Cloning ZED ROS2 wrapper..."
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git --branch humble-v4.1.4

# Clone point_cloud_transport and dependencies (required for Humble)
echo "Cloning point_cloud_transport and dependencies..."
git clone https://github.com/ros2/rcpputils.git --branch humble
git clone https://github.com/ros-perception/point_cloud_transport.git --branch humble
git clone https://github.com/ros-perception/point_cloud_transport_plugins.git --branch humble

# Install missing dependencies (exactly matching Dockerfile.l4t-humble)
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

# Build rcpputils first
cd ~/ros2_ws
source /opt/ros/humble/install/setup.bash || true
echo "Building rcpputils..."
colcon build --symlink-install \
    --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
    --packages-select rcpputils

# Source to make rcpputils available
source install/setup.bash

# Build point_cloud_transport
echo "Building point_cloud_transport..."
colcon build --symlink-install \
    --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
    --packages-select point_cloud_transport point_cloud_transport_plugins

# Source the workspace to make point_cloud_transport available
source install/setup.bash

# Build the complete workspace
echo "Building the complete workspace..."
colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src \
    --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
    " -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs" \
    " -DCMAKE_CXX_FLAGS='-Wl,--allow-shlib-undefined'" \
    " --no-warn-unused-cli"

# Add sourcing to .bashrc (if not already present)
if ! grep -q "source \$(pwd)/install/local_setup.bash" ~/.bashrc; then
    echo "Adding workspace sourcing to ~/.bashrc..."
    echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc
fi

echo "Installation complete! The ZED ROS2 Wrapper is now ready."