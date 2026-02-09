#!/bin/bash
set -e

echo "=== Atlas Scanner Auxiliary Installation ==="
echo "This script installs calibration tools and additional packages"
echo "WARNING: Do NOT run this script with sudo. It will prompt for sudo when needed."
echo ""

if [ "$EUID" -eq 0 ]; then
  echo "ERROR: Please do not run this script as root or with sudo"
  exit 1
fi

# Ensure we're in the correct location
if [ ! -f "$HOME/atlas_ws/src/atlas-scanner/install_aux_deps.sh" ]; then
  echo "ERROR: This script must be run from ~/atlas_ws/src/atlas-scanner/"
  echo "Please ensure the repository is cloned to ~/atlas_ws/src/atlas-scanner"
  exit 1
fi

# Build Sophus for calibration visualization
cd ~/atlas_ws
if [ ! -d "Sophus" ]; then
  git clone https://github.com/strasdat/Sophus.git
fi
cd Sophus
git reset --hard 'd0b7315a0d90fc6143defa54596a3a95d9fa10ec'
mkdir -p build && cd build
cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j2

# Install Sophus (system-wide installation)
sudo make install
sudo ldconfig

# Clone RKO LIO ROS package
cd ~/atlas_ws/src
if [ ! -d "rko_lio" ]; then
  git clone https://github.com/PRBonn/rko_lio.git
fi
cd ~/atlas_ws/src/rko_lio
git reset --hard '2363d1f55d3a7db4ff1a1266e35ad84e81069728'

# Clone Direct Visual Lidar Calibration ROS package
cd ~/atlas_ws/src
if [ ! -d "direct_visual_lidar_calibration" ]; then
  git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
fi
cd ~/atlas_ws/src/direct_visual_lidar_calibration
git reset --hard '02a0dc039f5509708f384be4ff3228e0ae09352d'

# Fix direct_visual_lidar_calibration for ROS2 Humble
bash ~/atlas_ws/src/atlas-scanner/src/install/fix_dvl_calibration.sh

# Clone (optional) SuperGlue (used for auto calibration which isn't currently recommended in the procedure)
cd ~/atlas_ws/
if [ ! -d "SuperGluePretrainedNetwork" ]; then
  git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
fi
cd ~/atlas_ws/SuperGluePretrainedNetwork
git reset --hard 'ddcf11f42e7e0732a0c4607648f9448ea8d73590'

# Replace {/home/orion/} with your home path
echo "export PYTHONPATH=\$PYTHONPATH:$HOME/atlas_ws/SuperGluePretrainedNetwork" >> ~/.bashrc
source ~/.bashrc

# Build GTSAM for calibration
cd ~/atlas_ws
if [ ! -d "gtsam" ]; then
  git clone https://github.com/borglab/gtsam
fi
cd gtsam && git checkout 4.2a9
mkdir -p build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j2

# Install GTSAM (system-wide installation)
sudo make install
sudo ldconfig

# Build Ceres for calibration
cd ~/atlas_ws
if [ ! -d "ceres-solver" ]; then
  git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
fi
cd ceres-solver
git checkout e47a42c2957951c9fafcca9995d9927e15557069
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j2

# Install Ceres (system-wide installation)
sudo make install

# Build Iridescence for calibration visualization 
cd ~/atlas_ws
if [ ! -d "iridescence" ]; then
  git clone https://github.com/koide3/iridescence --recursive
fi
cd ~/atlas_ws/iridescence
git reset --hard 'de083e26c7b27c14f4de37292decca9a18957fef'
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j2

# Install Iridescence (system-wide installation)
sudo make install

# Mark non-ROS packages to be ignored by colcon
cd ~/atlas_ws
touch ceres-solver/COLCON_IGNORE gtsam/COLCON_IGNORE iridescence/COLCON_IGNORE \
      Livox-SDK2/COLCON_IGNORE Sophus/COLCON_IGNORE SuperGluePretrainedNetwork/COLCON_IGNORE

# If you extracted the Insta360 SDK zip in the workspace root, also ignore it
if [ -d "Linux_CameraSDK-2.1.1_MediaSDK-3.1.1" ]; then
  touch Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/COLCON_IGNORE
fi

# Copy RViz config with odometry visualization to livox driver (source location for symlink-install)
cp ~/atlas_ws/src/atlas-scanner/src/install/display_point_cloud_ROS2.rviz \
   ~/atlas_ws/src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz

# Final build of all ROS packages
cd ~/atlas_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Replace {/home/orion/} with your home path
export LD_LIBRARY_PATH=$HOME/atlas_ws/install/direct_visual_lidar_calibration/lib:$HOME/atlas_ws/gtsam/build/lib:$HOME/atlas_ws/ceres-solver/build/lib:$HOME/atlas_ws/iridescence/build/lib:$LD_LIBRARY_PATH

echo ""
echo "=== Auxiliary Installation Complete ==="
echo "Your system should now be setup to run this repo's software"
echo "Next, calibrate your system setup using the Calibration doc"
echo ""
echo "Note: You may want to add the following to your ~/.bashrc:"
echo "export LD_LIBRARY_PATH=$HOME/atlas_ws/install/direct_visual_lidar_calibration/lib:$HOME/atlas_ws/gtsam/build/lib:$HOME/atlas_ws/ceres-solver/build/lib:$HOME/atlas_ws/iridescence/build/lib:\$LD_LIBRARY_PATH"
