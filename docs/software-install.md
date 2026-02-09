# Software Installation
> *Note: Only Linux Ubuntu Jammy 22.04 is currently supported*

## Automated Installation

### Initial Setup
First, create the ROS2 workspace and clone this repository:
```bash
mkdir -p ~/atlas_ws/src
cd ~/atlas_ws/src
git clone https://github.com/Eecornwell/atlas-scanner.git
```

### 1. Base Sensors Installation
Installs camera and LiDAR drivers:
```bash
cd ~/atlas_ws/src/atlas-scanner
./install_base_sensors.sh
```

**Manual steps required during script execution:**
- Install Insta360 SDK files (camera/stream headers and libCameraSDK.so)
- Update Mid360 firmware using LivoxViewer2
- Configure network settings for LiDAR (Static IP 192.168.1.50)
- Edit MID360_config.json to match your LiDAR IP address
- Connect and configure Insta360 camera (dual camera mode, USB mode to Android)

**After installation, validate sensors are working:**

Test LiDAR:
```bash
cd ~/atlas_ws
source ~/atlas_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Test Camera (in a new terminal):
```bash
cd ~/atlas_ws
source ~/atlas_ws/install/setup.bash
ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true
```

Verify topics (in another terminal):
```bash
cd ~/atlas_ws
source ~/atlas_ws/install/setup.bash
ros2 topic list
# Should see: /dual_fisheye/image, /equirectangular/image, /imu/data, etc.
```

### 2. Auxiliary Dependencies Installation
Installs calibration tools and additional packages:
```bash
cd ~/atlas_ws/src/atlas-scanner
./install_aux_deps.sh
```

**Important:** Do NOT run these scripts with `sudo`. They will prompt for sudo when needed.

---

## Manual Installation

If you prefer to run commands manually, follow the steps below:

```
# Create ROS2 workspace
mkdir -p ~/atlas_ws/src &&
cd ~/atlas_ws/src

# Copy this repo files/folders into ~/atlas_ws/src
cd ~/atlas_ws/src
git clone https://github.com/Eecornwell/atlas-scanner.git

# Install dependencies
sudo apt-get update &&
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev software-properties-common lsb-release python3-sensor-msgs python3-opencv ros-humble-rosbag2-transport ffmpeg ros-humble-robot-localization ros-humble-topic-tools ros-humble-cartographer ros-humble-cartographer-ros ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan ros-humble-cv-bridge ros-humble-vision-opencv libopencv-* libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev ros-humble-ament-cmake-auto ros-humble-rosidl-default-generators ros-humble-pcl-conversions ros-humble-ament-lint-auto ros-humble-ament-lint-common ros-humble-rosbag2 ros-humble-camera-info-manager ros-humble-imu-tools ros-humble-launch ros-humble-launch-ros ros-humble-launch-xml ros-humble-launch-yaml ros-humble-ros2launch ros-humble-rviz2 ros-humble-ros2topic nlohmann-json3-dev ros-humble-ros2run python3-rosdep ros-humble-rosbag2-cpp

# Install cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null

sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" &&
sudo apt update &&
sudo apt install kitware-archive-keyring &&
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg &&
sudo apt update &&
sudo apt install -y cmake	

# Prepare the Insta360 ROS Driver repo
cd ~/atlas_ws/src &&
git clone -b humble https://github.com/ai4ce/insta360_ros_driver.git &&
cd insta360_ros_driver.git &&
git reset --hard '9d2d3f51093d906903a6ea57bc5383c39a77ebfb'

# To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the Insta360 website.
# Unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip (latest version) and
# find the libCameraSDK.so file and camera/ and stream/ folders
# Then, the Insta360 libraries need to be installed as follows:
    # - chmod 777 camera/ stream/ libCameraSDK.so
    # - copy the camera and stream header files inside the include directory (~/atlas_ws/src/insta360_ros_driver/include)
    # - copy the libCameraSDK.so library under the lib directory (~/atlas_ws/src/insta360_ros_driver/lib)

# Remove any old SDK versions and create symlink to ensure correct version is used
rm -f ~/libCameraSDK.so ~/libCameraSDK.so.old &&
ln -s ~/atlas_ws/src/insta360_ros_driver/lib/libCameraSDK.so ~/libCameraSDK.so

# Prepare the Livox ROS Driver repo
cd ~/atlas_ws/src &&
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/atlas_ws/src/livox_ros_driver2 &&
git reset --hard '6b9356cadf77084619ba406e6a0eb41163b08039'

# Build Livox-SDK2
cd ~/atlas_ws &&
git clone https://github.com/Livox-SDK/Livox-SDK2.git &&
cd ~/atlas_ws/Livox-SDK2 &&
git reset --hard '6a940156dd7151c3ab6a52442d86bc83613bd11b' &&
find ~/atlas_ws/Livox-SDK2 -name "CMakeLists.txt" -exec sed -i 's/cmake_minimum_required(VERSION 3.0)/cmake_minimum_required(VERSION 3.10)/' {} + &&
cd ./Livox-SDK2/ &&
mkdir build &&
cd build &&
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
make -j2

# Install Livox-SDK2 (system-wide installation)
sudo make install &&
sudo ldconfig

# Mid360 Firmware Update using LivoxViewer2
cd ~/atlas_ws/src &&
wget --referer="https://www.livoxtech.com/" \
  --user-agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
  "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/LivoxViewer2%20for%20Ubuntu%20v2.3.0.zip" &&
unzip "LivoxViewer2 for Ubuntu v2.3.0.zip" &&
cd "LivoxViewer2 for Ubuntu v2.3.0.zip" &&
chmod 777 LivoxViewer2.sh &&
rm -rf 'LivoxViewer2 for Ubuntu v2.3.0.zip' &&
wget --referer="https://www.livoxtech.com/" \
  --user-agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
  "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/20241115/LIVOX_MID360_FW_v13.18.0237.bin &&
./LivoxViewer2.sh
# (select the .bin file above to update the firmware)
rm -rf 'LIVOX_MID360_FW_v13.18.0237.bin'

# Build ROS2 packages
cd ~/atlas_ws
sudo rosdep init &&
rosdep update &&
source /opt/ros/humble/setup.bash &&
ln -s /home/orion/atlas_ws/src/livox_ros_driver2/package_ROS2.xml /home/orion/atlas_ws/src/livox_ros_driver2/package.xml

# Apply fix to Livox Cmake
python3 ~/atlas_ws/src/atlas-scanner/src/install/fix_livox_cmake.py

# Install ROS dependencies
cd ~/atlas_ws &&
rosdep install --from-paths src --ignore-src -r -y

# Apply fix for insta360 library (remove unsupported SDK call)
sed -i '/SyncLocalTimeToCamera/d; /offset_time/d; /utc_time/d' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp

# Build ROS2 packages with symlink-install
export HUMBLE_ROS=humble &&
cd ~/atlas_ws &&
source /opt/ros/humble/setup.bash &&
colcon build --symlink-install &&
source install/setup.bash

# It it ok to have warnings and stderr output as long as all three packages finished
Summary: 3 packages finished [6.69s]
  1 package had stderr output: insta360_ros_driver

# Configure MID360_config.json
# Poke hole in firewall for lidar com
sudo ufw allow from 192.168.1.186 &&
sudo apt install net-tools &&
sudo ifconfig enp2s0 192.168.1.50

# If wired shows "connecting",
# Apply Network Settings to Wired:
# Static IP 192.168.1.50
# Mask 255.255.255.0

# For SDK, change settings here, change the lidar IP address to fit your model:
vi ~/atlas_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
{
"lidar_summary_info" : {
    "lidar_type": 8
},
"MID360": {
    "lidar_net_info" : {
    "cmd_data_port": 56100,
    "push_msg_port": 56200,
    "point_data_port": 56300,
    "imu_data_port": 56400,
    "log_data_port": 56500
    },
    "host_net_info" : {
    "cmd_data_ip" : "192.168.1.50",
    "cmd_data_port": 56101,
    "push_msg_ip": "192.168.1.50",
    "push_msg_port": 56201,
    "point_data_ip": "192.168.1.50",
    "point_data_port": 56301,
    "imu_data_ip" : "192.168.1.50",
    "imu_data_port": 56401,
    "log_data_ip" : "",
    "log_data_port": 56501
    }
},
"lidar_configs" : [
    {
    "ip" : "192.168.1.186",
    "pcl_data_type" : 1,
    "pattern_mode" : 0,
    "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
    }
    }
]
}

# Check with Livox Viewer First to confirm can connect to lidar_configs
cd ~/atlas_ws &&
source ~/atlas_ws/install/setup.bash &&
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# Check Insta360 settings on device
# - power on
# - set in dual camera mode (legacy)
# - set USB mode to Android
# - connect to computer using USB-C cable

# Look for: Bus 003 Device 017: ID 2e1a:0002 Arashi Vision Insta360 ONE
lsusb

# Enable Camera Access
bash ~/atlas_ws/src/insta360_ros_driver/setup.sh
ls -l /dev/insta

# Bring up camera to test driver
cd ~/atlas_ws &&
source ~/atlas_ws/install/setup.bash &&
ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true

# While the above is running in terminal, open another terminal and run
cd ~/atlas_ws &&
source ~/atlas_ws/install/setup.bash &&
ros2 topic list

# You should see the topics below
# /dual_fisheye/image
# /dual_fisheye/image/compressed
# /equirectangular/image
# /imu/data
# /imu/data_raw
# /parameter_events
# /rosout
# /tf

### !!! At this point you should be able to get both camera and lidar streams running successfully inside your ROS workspace

# Build Sophus for calibration visualization
cd ~/atlas_ws &&
git clone https://github.com/strasdat/Sophus.git &&
git reset --hard 'd0b7315a0d90fc6143defa54596a3a95d9fa10ec' &&
cd Sophus &&
mkdir build && cd build &&
cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF &&
make -j2

# Install Sophus (system-wide installation)
sudo make install && sudo ldconfig

# Clone RKO LIO ROS package
cd ~/atlas_ws/src &&
git clone https://github.com/PRBonn/rko_lio.git &&
cd ~/atlas_ws/src/rko_lio &&
git reset --hard '2363d1f55d3a7db4ff1a1266e35ad84e81069728'

# Clone Direct Visual Lidar Calibration ROS package
cd ~/atlas_ws/src &&
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive &&
cd ~/atlas_ws/src/direct_visual_lidar_calibration &&
git reset --hard '02a0dc039f5509708f384be4ff3228e0ae09352d'

# Fix direct_visual_lidar_calibration for ROS2 Humble
bash ~/atlas_ws/src/atlas-scanner/src/install/fix_dvl_calibration.sh

# Clone (optional) SuperGlue (used for auto calibration which isn't currently recommended in the procedure)
cd ~/atlas_ws/ &&
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git &&
cd cd ~/atlas_ws/SuperGluePretrainedNetwork &&
git reset --hard 'ddcf11f42e7e0732a0c4607648f9448ea8d73590'

# Replace {/home/orion/} with your home path
echo 'export PYTHONPATH=$PYTHONPATH:/home/orion/atlas_ws/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc

# Build GTSAM for calibration
cd ~/atlas_ws &&
git clone https://github.com/borglab/gtsam &&
cd gtsam && git checkout 4.2a9 &&
mkdir build && cd build &&
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
make -j2

# Install GTSAM (system-wide installation)
sudo make install && sudo ldconfig

# Build Ceres for calibration
cd ~/atlas_ws
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver &&
cd ceres-solver &&
git checkout e47a42c2957951c9fafcca9995d9927e15557069 &&
mkdir build && cd build &&
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
make -j2

# Install Ceres (system-wide installation)
sudo make install

# Build Iridescence for calibration visualization 
cd ~/atlas_ws &&
git clone https://github.com/koide3/iridescence --recursive &&
cd ~/atlas_ws/iridescence &&
git reset --hard 'de083e26c7b27c14f4de37292decca9a18957fef' &&
mkdir iridescence/build && cd iridescence/build &&
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
make -j2

# Install Iridescence (system-wide installation)
sudo make install

# Mark non-ROS packages to be ignored by colcon
cd ~/atlas_ws &&
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
cd ~/atlas_ws &&
source /opt/ros/humble/setup.bash &&
colcon build --symlink-install &&
source install/setup.bash

# Replace {/home/orion/} with your home path
export LD_LIBRARY_PATH=/home/orion/atlas_ws/install/direct_visual_lidar_calibration/lib:/home/orion/atlas_ws/gtsam/build/lib:/home/orion/atlas_ws/ceres-solver/build/lib:/home/orion/atlas_ws/iridescence/build/lib:$LD_LIBRARY_PATH
```

* Now your system should be setup to run this repo's software
* Next, calibrate your system setup using the [Calibration doc](calibration.md)
