# Software Installation
> *Note: Only Linux Ubuntu Jammy 22.04 is currently supported*

After cloning this repo, open a command line window inside of the repo `src` folder

```
# Install dependencies
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev software-properties-common lsb-release python3-sensor-msgs python3-opencv ros-humble-rosbag2-transport ffmpeg ros-humble-robot-localization ros-humble-topic-tools ros-humble-cartographer ros-humble-cartographer-ros ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan ros-humble-cv-bridge ros-humble-vision-opencv libopencv-* libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install kitware-archive-keyring
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
sudo apt update
sudo apt install cmake	

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash

# Build Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
sudo nano /etc/ld.so.conf
add include  /usr/local/lib
sudo ldconfig

# Firmware Update using LivoxViewer2
cd ~/ros2_ws/src
wget https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/LivoxViewer2%20for%20Ubuntu%20v2.3.0.zip
unzip "LivoxViewer2 for Ubuntu v2.3.0.zip"
cd "LivoxViewer2 for Ubuntu v2.3.0.zip"
chmod 777 LivoxViewer2.sh
wget https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/20241115/LIVOX_MID360_FW_v13.18.0237.bin
./LivoxViewer2.sh (select the .bin file above to update the firmware)

# Build and Install Insta360 SDK
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver.git
# Then, the Insta360 libraries need to be installed as follows:
    # - add the camera and stream header files inside the include directory
    # - add the libCameraSDK.so library under the lib directory.
# To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the Insta360 website.
    # - Copy camera/ stream/ libCameraSDK.so to Home directory
    # - chmod 777 camera/ stream/ libCameraSDK.so 

# Replace {your-home-path} with your home path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:{your-home-path}

# Copy source files/folders into ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Eecornwell/atlas-scanner.git

# Create new ROS dependency package
cd ..
sudo rosdep init
rosdep update
cd src/livox_ros_driver2
./build.sh humble
source install/setup.bash

# Configure MID360_config.json
# Poke hole in firewall for lidar com
sudo ufw allow from 192.168.1.186
sudo apt install net-tools
sudo ifconfig enp2s0 192.168.1.50

# If wired shows "connecting",
# Apply Network Settings to Wired:
# Static IP 192.168.1.50
# Mask 255.255.255.0

# For SDK, change settings here:
~/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
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
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# Check Insta360 settings
# - set in dual camera mode
# - set USB mode to Android
# - connect to computer using USB-C cable

# Look for: Bus 003 Device 017: ID 2e1a:0002 Arashi Vision Insta360 ONE
lsusb

# Enable Camera Access
bash ~/ros2_ws/src/insta360_ros_driver/setup.sh
ls -l /dev/insta

# Edit config/general_configuration.yaml
#- lidar_topic: /livox/lidar
#- frame_id: /base_link

# Install RKO LIO
cd ~/ros2_ws/src
git clone https://github.com/PRBonn/rko_lio.git
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+
# Create config file `rko_lio_config_robust.yaml`

# Install Direct Visual Lidar Calibration
cd ~/ros2_ws/src
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd .. && colcon build
# Replace {your-home-path} with your home path
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
echo 'export PYTHONPATH=$PYTHONPATH:{your-home-path}/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc

# Install GTSAM
cd ~/ros2_ws/src
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2a9
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc)
sudo make install

# Install Ceres
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
cd ceres-solver
git checkout e47a42c2957951c9fafcca9995d9927e15557069
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
make -j$(nproc)
sudo make install

# Install Iridescence for visualization
cd ~/ros2_ws/src
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# Replace {your-home-path} with your home path
export LD_LIBRARY_PATH={your-home-path}/ros2_ws/install/direct_visual_lidar_calibration/lib:{your-home-path}/gtsam/build/lib:{your-home-path}/ceres-solver/build/lib:{your-home-path}/iridescence/build/lib:$LD_LIBRARY_PATH

```