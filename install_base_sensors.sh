#!/bin/bash
set -e

echo "=== Atlas Scanner Base Sensors Installation ==="
echo "Note: Only Linux Ubuntu Jammy 22.04 is currently supported"
echo "WARNING: Do NOT run this script with sudo. It will prompt for sudo when needed."
echo ""

if [ "$EUID" -eq 0 ]; then
  echo "ERROR: Please do not run this script as root or with sudo"
  exit 1
fi

# Ensure we're in the correct location
if [ ! -f "$HOME/atlas_ws/src/atlas-scanner/install_base_sensors.sh" ]; then
  echo "ERROR: This script must be run from ~/atlas_ws/src/atlas-scanner/"
  echo "Please ensure the repository is cloned to ~/atlas_ws/src/atlas-scanner"
  exit 1
fi

# Create ROS2 workspace
mkdir -p ~/atlas_ws/src
cd ~/atlas_ws/src

# Install dependencies
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev software-properties-common lsb-release python3-sensor-msgs python3-opencv ros-humble-rosbag2-transport ffmpeg ros-humble-robot-localization ros-humble-topic-tools ros-humble-cartographer ros-humble-cartographer-ros ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan ros-humble-cv-bridge ros-humble-vision-opencv libopencv-* libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev ros-humble-ament-cmake-auto ros-humble-rosidl-default-generators ros-humble-pcl-conversions ros-humble-ament-lint-auto ros-humble-ament-lint-common ros-humble-rosbag2 ros-humble-camera-info-manager ros-humble-imu-tools ros-humble-launch ros-humble-launch-ros ros-humble-launch-xml ros-humble-launch-yaml ros-humble-ros2launch ros-humble-rviz2 ros-humble-ros2topic nlohmann-json3-dev ros-humble-ros2run python3-rosdep ros-humble-rosbag2-cpp

# Install cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install kitware-archive-keyring
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
sudo apt update
sudo apt install -y cmake

# Prepare the Insta360 ROS Driver repo
cd ~/atlas_ws/src
if [ ! -d "insta360_ros_driver" ]; then
  git clone -b humble https://github.com/ai4ce/insta360_ros_driver.git
fi
cd insta360_ros_driver
git reset --hard '9d2d3f51093d906903a6ea57bc5383c39a77ebfb'

echo ""
echo "=== MANUAL STEP REQUIRED ==="
echo "To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the Insta360 website."
echo "Unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip (latest version) and"
echo "find the libCameraSDK.so file and camera/ and stream/ folders"
echo "Then, the Insta360 libraries need to be installed as follows:"
echo "  - chmod 777 camera/ stream/ libCameraSDK.so"
echo "  - copy the camera and stream header files inside the include directory (~/atlas_ws/src/insta360_ros_driver/include)"
echo "  - copy the libCameraSDK.so library under the lib directory (~/atlas_ws/src/insta360_ros_driver/lib)"
echo ""
read -p "Press Enter once you have completed the above steps..."

# Remove any old SDK versions and create symlink to ensure correct version is used
rm -f ~/libCameraSDK.so ~/libCameraSDK.so.old
ln -s ~/atlas_ws/src/insta360_ros_driver/lib/libCameraSDK.so ~/libCameraSDK.so

# Prepare the Livox ROS Driver repo
cd ~/atlas_ws/src
if [ ! -d "livox_ros_driver2" ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git
fi
cd ~/atlas_ws/src/livox_ros_driver2
git reset --hard '6b9356cadf77084619ba406e6a0eb41163b08039'

# Build Livox-SDK2
cd ~/atlas_ws
if [ ! -d "Livox-SDK2" ]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi
cd ~/atlas_ws/Livox-SDK2
git reset --hard '6a940156dd7151c3ab6a52442d86bc83613bd11b'
find ~/atlas_ws/Livox-SDK2 -name "CMakeLists.txt" -exec sed -i 's/cmake_minimum_required(VERSION 3.0)/cmake_minimum_required(VERSION 3.10)/' {} +
mkdir -p build
cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j2

# Install Livox-SDK2 (system-wide installation)
sudo make install
sudo ldconfig

# Mid360 Firmware Update using LivoxViewer2
cd ~/atlas_ws/src
wget --referer="https://www.livoxtech.com/" \
  --user-agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
  "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/LivoxViewer2%20for%20Ubuntu%20v2.3.0.zip"
unzip "LivoxViewer2 for Ubuntu v2.3.0.zip"
cd "LivoxViewer2 for Ubuntu v2.3.0"
chmod 777 LivoxViewer2.sh
rm -rf 'LivoxViewer2 for Ubuntu v2.3.0.zip'
wget --referer="https://www.livoxtech.com/" \
  --user-agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
  "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/20241115/LIVOX_MID360_FW_v13.18.0237.bin"

echo ""
echo "=== MANUAL STEP REQUIRED ==="
echo "To update Mid360 firmware, run: ~/atlas_ws/src/LivoxViewer2\ for\ Ubuntu\ v2.3.0/LivoxViewer2.sh"
echo "Then select the firmware file: LIVOX_MID360_FW_v13.18.0237.bin"
echo ""

# Build ROS2 packages
cd ~/atlas_ws
sudo rosdep init || true
rosdep update
source /opt/ros/humble/setup.bash
ln -sf ~/atlas_ws/src/livox_ros_driver2/package_ROS2.xml ~/atlas_ws/src/livox_ros_driver2/package.xml

# Apply fix to Livox Cmake
python3 ~/atlas_ws/src/atlas-scanner/src/install/fix_livox_cmake.py

# Install ROS dependencies
cd ~/atlas_ws
rosdep install --from-paths src --ignore-src -r -y

# Apply fix for insta360 library (remove unsupported SDK call)
sed -i '/uint64_t utc_time/,/cam->SyncLocalTimeToCamera/d' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp

# Apply camera quality improvements
echo "Applying camera quality improvements..."
# 1. Update resolution to 2560x1280 (mid quality)
sed -i 's/RES_1920_960P30/RES_2560_1280P30/g' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp
# 2. Increase bitrate to 12 Mbps for better quality
sed -i 's/param.video_bitrate = 1024 \* 1024 \/ 2;/param.video_bitrate = 1024 * 1024 * 12;/g' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp
# 3. Use high-quality Lanczos scaling instead of nearest neighbor
sed -i 's/SWS_POINT/SWS_LANCZOS/g' ~/atlas_ws/src/insta360_ros_driver/src/decoder.cpp
# 4. Update equirectangular config for new resolution
sed -i 's/crop_size: 960/crop_size: 1920/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
sed -i 's/out_width: 1920/out_width: 3840/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
sed -i 's/out_height: 960/out_height: 1920/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
# 5. Update atlas-scanner configs for new resolution
sed -i 's/image_width: 1920/image_width: 3840/g' ~/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml
sed -i 's/image_height: 960/image_height: 1920/g' ~/atlas_ws/src/atlas-scanner/src/config/fusion_calibration.yaml
sed -i 's/width=1920, height=960/width=3840, height=1920/g' ~/atlas_ws/src/atlas-scanner/src/calibration/generate_intensity_images.py
# 6. Update JPEG quality to 100 in capture scripts
sed -i 's/cv2.imwrite(img_file, cv_image)/cv2.imwrite(img_file, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])/g' ~/atlas_ws/src/atlas-scanner/src/capture/buffered_camera_capture.py
sed -i "s/cv2.imwrite(output_path, cv_image)/cv2.imwrite(output_path, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])/g" ~/atlas_ws/src/atlas-scanner/src/terrestrial_fusion_with_lio.sh

# Build ROS2 packages with symlink-install
export HUMBLE_ROS=humble
cd ~/atlas_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo ""
echo "It is ok to have warnings and stderr output as long as all three packages finished"
echo "Expected: Summary: 3 packages finished"
echo ""

# Configure MID360_config.json
# Poke hole in firewall for lidar com
sudo ufw allow from 192.168.1.186
sudo apt install net-tools
sudo ifconfig enp2s0 192.168.1.50

echo ""
echo "=== MANUAL STEP REQUIRED ==="
echo "If wired shows 'connecting', apply Network Settings to Wired:"
echo "  - Static IP: 192.168.1.50"
echo "  - Mask: 255.255.255.0"
echo ""
echo "For SDK, change settings in:"
echo "  vi ~/atlas_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
echo ""
echo "Change the lidar IP address to fit your model (default: 192.168.1.186)"
echo ""
read -p "Press Enter once you have completed the network configuration..."

echo ""
echo "=== Testing Livox Viewer ==="
echo "Check with Livox Viewer First to confirm can connect to lidar"
echo "Run: cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
echo ""

echo ""
echo "=== Insta360 Camera Setup ==="
echo "Check Insta360 settings on device:"
echo "  - power on"
echo "  - set in dual camera mode (legacy)"
echo "  - set USB mode to Android"
echo "  - connect to computer using USB-C cable"
echo ""
read -p "Press Enter once camera is connected..."

# Look for camera
echo "Looking for camera (should see: Bus 003 Device 017: ID 2e1a:0002 Arashi Vision Insta360 ONE)"
lsusb | grep -i insta || echo "Warning: Insta360 camera not found"

# Enable Camera Access
bash ~/atlas_ws/src/insta360_ros_driver/setup.sh
ls -l /dev/insta

echo ""
echo "=== Testing Camera Driver ==="
echo "To test camera driver, run:"
echo "  cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true"
echo ""
echo "In another terminal, run:"
echo "  cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 topic list"
echo ""
echo "You should see topics like /dual_fisheye/image, /equirectangular/image, /imu/data, etc."
echo ""

echo ""
echo "=== Base Sensors Installation Complete ==="
echo "At this point you should be able to get both camera and lidar streams running successfully inside your ROS workspace"
echo ""
echo "=== VALIDATION STEPS ==="
echo "Please validate your sensors are working:"
echo ""
echo "1. Test LiDAR (in this terminal):"
echo "   cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
echo ""
echo "2. Test Camera (in a new terminal):"
echo "   cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true"
echo ""
echo "3. Verify topics (in another terminal):"
echo "   cd ~/atlas_ws && source ~/atlas_ws/install/setup.bash && ros2 topic list"
echo "   Expected topics: /dual_fisheye/image, /equirectangular/image, /imu/data, etc."
echo ""
echo "Once validated, proceed with install_aux_deps.sh for calibration tools"
