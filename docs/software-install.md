# Software Installation
> *Note: Only Linux Ubuntu Jammy 22.04, ROS2 Humble is currently supported*

## Automated Installation

### Initial Setup
First, create the ROS2 workspace and clone this repository:
```bash
mkdir -p ~/atlas_ws/src
cd ~/atlas_ws/src
git clone https://github.com/Eecornwell/atlas-scanner.git
```

### 1. Base Sensors Installation
Installs camera and LiDAR drivers with optimized camera settings:
- **Resolution**: 3840x1920 (high quality)
- **ISO**: 600 (increased light sensitivity)
- **Exposure (shutter speed)**: 1/120 (faster shutter, motion compensation)
- **Bitrate**: 30 Mbps (high quality, matched to 15 fps effective output)
- **Scaling**: Lanczos interpolation (highest quality)
- **JPEG Quality**: 100 (maximum)

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
~/insta360-dev/build/insta360_capture
# Should print: Found camera: <serial>  ...  Camera session open
# Ctrl+C to exit
```

Verify LiDAR topics (in another terminal):
```bash
cd ~/atlas_ws
source ~/atlas_ws/install/setup.bash
ros2 topic list
# Should see: /livox/lidar, /livox/imu, /rko_lio/odometry, etc.
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

<details>
<summary>Click to expand manual installation steps</summary>

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

# Set xfer_format=0 (PointCloud2) — upstream default is 1 (Livox custom format)
# which RKO-LIO cannot deserialize, causing odometry to never publish
sed -i 's/xfer_format   = 1/xfer_format   = 0/' ~/atlas_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

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

# Apply camera quality improvements (automatically applied by install script)
# - Resolution: 3840x1920 (high quality)
# - Bitrate: 12 Mbps (reduced compression)
# - Scaling: Lanczos interpolation (highest quality)
# - JPEG Quality: 100 (maximum)
sed -i 's/RES_1920_960P30/RES_2560_1280P30/g' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp
sed -i 's/param.video_bitrate = 1024 \* 1024 \* [0-9]*/param.video_bitrate = 1024 * 1024 * 30;/g' ~/atlas_ws/src/insta360_ros_driver/src/main.cpp
sed -i "s/dual_fisheye_\${timestamp}.jpg/dual_fisheye_\${timestamp}.png/; s/cv2.imwrite(img_file, frame)/cv2.imwrite(img_file, frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])/" ~/atlas_ws/src/atlas-scanner/src/capture/buffered_camera_capture.py
sed -i 's/SWS_POINT/SWS_LANCZOS/g' ~/atlas_ws/src/insta360_ros_driver/src/decoder.cpp
sed -i 's/crop_size: 960/crop_size: 1920/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
sed -i 's/out_width: 1920/out_width: 3840/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml
sed -i 's/out_height: 960/out_height: 1920/g' ~/atlas_ws/src/insta360_ros_driver/config/equirectangular.yaml

# Apply manual exposure control (MANUAL mode, 1/120s ISO 1600 for indoor use)
# Note: SDK uses SetIso (not SetISO)
python3 - <<'PYEOF'
path = '/home/orion/atlas_ws/src/insta360_ros_driver/src/main.cpp'
content = open(path).read()
patch = '''
        node_->declare_parameter("shutter_speed", 1.0 / 120.0);
        node_->declare_parameter("iso", 1600);
        double shutter = node_->get_parameter("shutter_speed").as_double();
        int iso = node_->get_parameter("iso").as_int();
        if (shutter > 0.0) {
            auto exposure = std::make_shared<ins_camera::ExposureSettings>();
            exposure->SetExposureMode(
                ins_camera::PhotographyOptions_ExposureOptions_Program_MANUAL);
            exposure->SetShutterSpeed(shutter);
            exposure->SetIso(iso);
            if (cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM,
                                         exposure)) {
                RCLCPP_INFO(node_->get_logger(),
                            "Manual exposure: 1/%.0fs ISO %d", 1.0 / shutter, iso);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Failed to set exposure, using auto-exposure");
            }
        }'''
marker = 'RCLCPP_INFO(node_->get_logger(), "Live streaming started.");'
if marker in content and 'shutter_speed' not in content:
    content = content.replace(marker, marker + patch)
    open(path, 'w').write(content)
    print('✓ Manual exposure patch applied')
elif 'shutter_speed' in content:
    print('✓ Manual exposure patch already present')
else:
    print('✗ Marker not found - check main.cpp manually')
PYEOF

# Add shutter_speed arg to bringup.launch.xml
sed -i 's|<arg name="imu_filter" default="true"/>|<arg name="imu_filter" default="true"/>\n    <arg name="shutter_speed" default="0.00833"/>\n    <arg name="iso" default="1600"/>|' \
    ~/atlas_ws/src/insta360_ros_driver/launch/bringup.launch.xml
sed -i 's|exec="insta360_ros_driver" name="insta360_ros_driver" output="log"/>|exec="insta360_ros_driver" name="insta360_ros_driver" output="log">\n        <param name="shutter_speed" value="$(var shutter_speed)"/>\n        <param name="iso" value="$(var iso)"/>\n    </node>|' \
    ~/atlas_ws/src/insta360_ros_driver/launch/bringup.launch.xml

# Rebuild only the camera driver to verify patches compile cleanly
cd ~/atlas_ws &&
source /opt/ros/humble/setup.bash &&
colcon build --packages-select insta360_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release &&
source install/setup.bash
# Warnings about unused parameters are expected; the build must show no errors

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

# Increase UDP receive buffer permanently so the lidar point cloud stream
# (4MB/s at 10Hz) never drops packets between capture node invocations.
# Without this the 208KB default fills in <1s and scans after the first fail.
echo 'net.core.rmem_max=33554432
net.core.rmem_default=33554432' | sudo tee /etc/sysctl.d/99-atlas-udp.conf
sudo sysctl -p /etc/sysctl.d/99-atlas-udp.conf

# If wired shows "connecting",
# Apply Network Settings to Wired:
# Static IP 192.168.1.50
# Mask 255.255.255.0

# For SDK, change settings here, change the lidar IP address to fit your model:
vi ~/atlas_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json

# ── LiDAR Clock Sync ────────────────────────────────────────────────────
# The MID360 hardware clock runs independently from the host system clock,
# producing a fixed ~65ms offset between LiDAR header.stamp values and host
# shutter timestamps. This offset is stable (±0.3ms) and auto-corrected in
# post-processing, but the device's internal wall-clock reference should be
# set to UTC at startup for accurate last_sync_time metadata.
#
# PRIMARY: RMC time sync via Livox SDK2 (livox_time_sync binary)
# Calls SetLivoxLidarRmcSyncTime() with a synthesised GPRMC sentence before
# the ROS driver starts. Runs in ~3s, requires no extra hardware.
# Built automatically by the SDK build script (build.sh).
# Invoked automatically by atlas_fusion_capture.sh before livox_ros_driver2.
#
# SUPPLEMENTARY: PTP grandmaster on enp2s0 (optional, keeps host PHC aligned)
# The MID360 sends PTP peer-delay-request messages but does not slave its
# point cloud timestamp domain to an external PTP master. The ptp4l/phc2sys
# services below keep the host PHC (±0.1ms) aligned to system clock, which
# is useful if other PTP-capable devices are on the same network segment.

# Install linuxptp (required for ptp4l/phc2sys services)
sudo apt install -y linuxptp

# Run the one-time install script (copies configs, installs and enables systemd services)
cd ~/atlas_ws/src/atlas-scanner/src
sudo ./install_ptp.sh

# Verify both services are running
sudo systemctl status ptp4l-livox.service phc2sys-livox.service

# Confirm PHC is locked to system clock (offset should be <1ms)
sudo journalctl -u phc2sys-livox.service -n 5 --no-pager
# Expected output:
#   phc_sync: offset=+0.08ms  PHC=1783396948.404  sys=1783396948.404

# Both services are enabled and start automatically on every boot.
# The livox_time_sync binary (RMC sync) runs automatically at each session start.
# ──────────────────────────────────────────────────────────────────────────────

# Configure FastDDS profiles
# fastdds_atlas.xml  — used by long-running driver processes (LiDAR, RKO-LIO, camera).
# fastdds_capture.xml — used by short-lived capture processes (participantID=50 to avoid
#                       SHM port collisions with the persistent decoder process).
# No IP address substitution needed — all profiles use default SHM+UDP transports.

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
bash ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
ls -l /dev/insta

# Verify camera connection (SDK daemon)
~/insta360-dev/build/insta360_capture
# Should print: Found camera: <serial>  ...  Camera session open
# Ctrl+C to exit

### !!! At this point you should be able to get both camera and lidar running successfully

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

# Patch rko_lio node.cpp to:
# 1. Expose max_lidar_buffer_size as a ROS parameter (value set to 5 in rko_lio_config_robust.yaml)
#    to keep the registration queue small and prevent stale frame replay.
# 2. Pop the oldest frame when the buffer is full so the registration loop never
#    stalls and TF publishing continues uninterrupted.
# 3. Add max_lidar_buffer_size to the launch file's configurable_parameters so
#    it is passed through from the YAML config to the node.
python3 - <<'PYEOF'
import pathlib, re
path = pathlib.Path('/home/orion/atlas_ws/src/rko_lio/rko_lio/ros/node.cpp')
content = path.read_text()
if 'max_lidar_buffer_size' in content:
    print('✓ rko_lio buffer patches already present')
else:
    # Expose parameter
    content = content.replace(
        '  // manually, if, define extrinsics',
        '  max_lidar_buffer_size =\n      static_cast<size_t>(node->declare_parameter<int>("max_lidar_buffer_size", static_cast<int>(max_lidar_buffer_size)));\n\n  // manually, if, define extrinsics')
    # Pop oldest on overflow
    content = content.replace(
        '      RCLCPP_WARN_STREAM(node->get_logger(), "Registration lidar buffer limit reached. Dropping frame.");\n      sync_condition_variable.notify_one();\n      return;',
        '      RCLCPP_WARN_STREAM(node->get_logger(), "Registration lidar buffer limit reached. Dropping frame.");\n      lidar_buffer.pop();\n      sync_condition_variable.notify_one();\n      return;')
    path.write_text(content)
    print('✓ rko_lio buffer patches applied')
PYEOF

# Add max_lidar_buffer_size to rko_lio launch file configurable_parameters
python3 - <<'PYEOF'
import pathlib
path = pathlib.Path('/home/orion/atlas_ws/src/rko_lio/launch/odometry.launch.py')
content = path.read_text()
if 'max_lidar_buffer_size' in content:
    print('✓ rko_lio launch parameter already present')
else:
    content = content.replace(
        '    {\n        "name": "min_beta",',
        '    {\n        "name": "min_beta",\n    },\n    {\n        "name": "max_lidar_buffer_size",\n        "default": "50",\n        "type": "int",\n        "description": "Max lidar frames buffered for registration.",\n    },\n    {')
    path.write_text(content)
    print('✓ rko_lio launch parameter added')
PYEOF

# Patch insta360 decoder to self-recover from corrupt H.264 streams instead of
# requiring an external process restart between scans.
# The decoder now flushes its codec context after 5 consecutive decode errors
# and waits for the next I-frame automatically.
python3 - <<'PYEOF'
import re, pathlib
path = pathlib.Path('/home/orion/atlas_ws/src/insta360_ros_driver/src/decoder.cpp')
content = path.read_text()

# Check if patch already applied
if 'consecutive_errors_' in content:
    print('✓ Decoder self-recovery patch already present')
else:
    # Add fields and ResetDecoder after i_frame_only_
    content = content.replace(
        '    bool i_frame_only_ = false;',
        '''    bool i_frame_only_ = false;
    int consecutive_errors_ = 0;
    static constexpr int kMaxConsecutiveErrors = 5;

    void ResetDecoder() {
        if (codec_ctx_) { avcodec_flush_buffers(codec_ctx_); }
        if (sws_ctx_) { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
        bgr_frame_.release();
        consecutive_errors_ = 0;
        RCLCPP_INFO(this->get_logger(), "Decoder reset — waiting for next I-frame");
    }''')
    # Add error tracking in DecodeAndDisplayPacket
    content = content.replace(
        '        int ret = avcodec_send_packet(codec_ctx_, packet);\n        if (ret < 0) {\n            return;\n        }\n\n        while (ret >= 0) {\n            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);\n            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {\n                return;\n            } else if (ret < 0) {\n                return;\n            }',
        '''        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret < 0) {
            if (++consecutive_errors_ >= kMaxConsecutiveErrors) { ResetDecoder(); }
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) { return; }
            else if (ret < 0) {
                if (++consecutive_errors_ >= kMaxConsecutiveErrors) { ResetDecoder(); }
                return;
            }
            consecutive_errors_ = 0;''')
    path.write_text(content)
    print('✓ Decoder self-recovery patch applied')
PYEOF

# Final build of all ROS packages
cd ~/atlas_ws &&
source /opt/ros/humble/setup.bash &&
colcon build --symlink-install &&
source install/setup.bash

# Replace {/home/orion/} with your home path
export LD_LIBRARY_PATH=/home/orion/atlas_ws/install/direct_visual_lidar_calibration/lib:/home/orion/atlas_ws/gtsam/build/lib:/home/orion/atlas_ws/ceres-solver/build/lib:/home/orion/atlas_ws/iridescence/build/lib:$LD_LIBRARY_PATH
```

</details>

---

## SDK Setup

> **Prerequisites:** Before running any scan sessions, ensure the LiDAR clock sync is set up. The `livox_time_sync` binary (built by `build.sh`) runs automatically at each session start and sets the MID360 wall-clock reference via `SetLivoxLidarRmcSyncTime()`. The supplementary PTP services (`ptp4l-livox`, `phc2sys-livox`) keep the host PHC aligned and are installed via `install_ptp.sh`. See the [LiDAR Clock Sync](#lidar-clock-sync) section in the manual installation steps.

The ATLAS pipeline uses two Insta360 SDK packages:

| Package | Version | Supports |
|---------|---------|----------|
| CameraSDK | 2.1.1 (Aug 2025) | One X, One R, One RS, One X2, X3, X4, **X5**, X4 Air |
| MediaSDK | 3.1.1 (Sep 2025) | `.insp` → ERP stitching for all above models |

Download `Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip` from the Insta360 developer portal and place it in `~/Downloads/`.

### 1. Install CameraSDK 2.1.1

```bash
# Extract the zip (macOS metadata entries are harmless)
cd ~/Downloads
unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip -d Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/

# Extract the CameraSDK tarball
mkdir -p ~/LinuxSDK
tar -xzf ~/Downloads/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/CameraSDK-20250812_192742-2.1.1-Linux.tar_1754998644023.gz \
    -C ~/LinuxSDK/

# Verify
ls ~/LinuxSDK/CameraSDK-20250812_192742-2.1.1-Linux/
# Should show: bin/  example/  include/  lib/
```

### 2. Install MediaSDK 3.1.1

The MediaSDK ships as a `.deb` inside a `.tar.xz` inside the zip:

```bash
# Extract the tar.xz from the zip
unzip -j ~/Downloads/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip \
  "Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.tar_1758540334111.xz" \
  -d /tmp/mediasdk/

# Extract the .deb from the tar.xz
tar -xJf /tmp/mediasdk/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.tar_1758540334111.xz \
    libMediaSDK-dev-3.1.1.0-20250922_191110-amd64/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.deb \
    -C /tmp/mediasdk/

# Install
sudo dpkg -i /tmp/mediasdk/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.deb
sudo ldconfig

# Verify
ldconfig -p | grep MediaSDK   # should show libMediaSDK.so
ls /usr/include/ins_stitcher.h  # MediaSDK 3.x header (was stitcher/stitcher.h in 2.x)
```

> The MediaSDK 3.x `.deb` also installs `libMNN.so`, `libtscsdk_center.so` and several CUDA stubs into `/usr/lib`. These are required at runtime by `libMediaSDK.so` and are included in the package.

### 3. Build insta360_capture and insta360_stitch

```bash
mkdir -p ~/insta360-dev
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/main.cpp ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/stitch.cpp ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/livox_time_sync.cpp ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/CMakeLists.txt ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/build.sh ~/insta360-dev/
cd ~/insta360-dev && bash build.sh

# Verify
ls ~/insta360-dev/build/insta360_capture
ls ~/insta360-dev/build/insta360_stitch
ls ~/insta360-dev/build/insta360_reset_clock
ls ~/insta360-dev/build/livox_time_sync
```

The `CMakeLists.txt` defaults to `~/LinuxSDK/CameraSDK-20250812_192742-2.1.1-Linux`. If you extracted elsewhere:

```bash
# Edit the CAMERA_SDK_DIR line in CMakeLists.txt before building
set(CAMERA_SDK_DIR "/path/to/CameraSDK-20250812_192742-2.1.1-Linux")
```

To rebuild after pulling repo updates:

```bash
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/main.cpp ~/insta360-dev/
cp ~/atlas_ws/src/atlas-scanner/src/capture/sdk/stitch.cpp ~/insta360-dev/
cd ~/insta360-dev && bash build.sh
```

### 4. Camera USB connection (One X2 and X5)

The X5 does **not** have a USB Mode setting in its menus — it uses a direct USB connection protocol that the CameraSDK 2.1.1 handles automatically. Simply:

1. Power on the camera
2. Connect via USB-C
3. Run `lsusb | grep -i insta` — should show Vendor ID `2e1a`
4. Run `sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh`
5. Verify `/dev/insta` exists

For the One X2, the USB Mode → Android setting is still required as before.

### 5. Verify SDK daemon connects

```bash
cd ~/atlas_ws && source install/setup.bash
sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
~/insta360-dev/build/insta360_capture
# Should print: Found camera: <serial>  ...  Camera session open
# Ctrl+C to exit
```

### 6. LiDAR mask

The SDK stitcher places ERP content in different pixel regions than a manual fisheye-to-ERP pipeline. Dedicated masks are included in the repo for each supported camera:

```
~/atlas_ws/src/atlas-scanner/src/lidar_mask_dual_sdk.png    # One X2, dual fisheye
~/atlas_ws/src/atlas-scanner/src/lidar_mask_single.png      # One X2, single fisheye
~/atlas_ws/src/atlas-scanner/src/lidar_mask_dual_x5.png     # X5, dual fisheye (replace placeholder)
~/atlas_ws/src/atlas-scanner/src/lidar_mask_single_x5.png   # X5, single fisheye (replace placeholder)
```

The capture script selects the correct mask automatically based on `CAMERA_HW` and `CAMERA_MODE`.

> The X5 placeholder masks need to be replaced with real masks once you have a sample X5 scan. See [calibration.md](calibration.md) — Lidar Masks section.

---

## Desktop Shortcuts

### ATLAS GUI

```bash
cp ~/atlas_ws/src/atlas-scanner/assets/media/atlas_logo_app.png ~/.local/share/icons/atlas_logo_app.png
cat > ~/Desktop/ATLAS.desktop << 'EOF'
[Desktop Entry]
Name=ATLAS
Exec=bash -c '~/atlas_ws/src/atlas-scanner/src/run_gui.sh'
Icon=atlas_logo_app
Terminal=false
Type=Application
EOF
chmod +x ~/Desktop/ATLAS.desktop
```

### Camera Permissions

The camera requires USB permissions to be set each time it is reconnected. Create a desktop shortcut to run the permissions script without opening a terminal:

```bash
cp ~/atlas_ws/src/atlas-scanner/assets/media/atlas_logo_app.png ~/.local/share/icons/atlas_logo_app.png
cat > ~/Desktop/ATLAS-Camera-Permissions.desktop << 'EOF'
[Desktop Entry]
Name=ATLAS Camera Permissions
Exec=bash -c '~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh'
Icon=atlas_logo_app
Terminal=true
Type=Application
EOF
chmod +x ~/Desktop/ATLAS-Camera-Permissions.desktop
```

> *Note: `Terminal=true` is set so you can see the output and confirm permissions were applied successfully.*

---

# Install pycolmap (required for panorama SfM pipeline, must match system colmap version)
pip3 install pycolmap==3.14.0.dev0

```

</details>

---

## Post-Processing: Panorama SfM (COLMAP)

The recommended pipeline uses ERP images directly with COLMAP's `SPHERICAL` camera model, a single-camera rig, and LiDAR pose priors from odometry.

### New pipeline (recommended)

Uses `panorama_sfm_colmap.py` — no cubemap conversion, ERP fed directly to COLMAP:

```bash
SESSION=~/atlas_ws/data/synchronized_scans/sync_fusion_{TIMESTAMP}
rm -rf $SESSION/colmap
cd ~/atlas_ws/src/atlas-scanner/src/post_processing
python3 panorama_sfm_colmap.py $SESSION
```

Options:
- `--sequential` — use sequential matcher instead of exhaustive (faster, for large sessions)
- `--no-bundle-adjustment` — skip rig-aware bundle adjustment

### Legacy pipeline (deprecated)

The old `export_to_colmap.py` and `erp_to_perspective_colmap.py` scripts have been retired. Use `panorama_sfm_colmap.py` for all COLMAP exports.

---

* Now your system should be setup to run this repo's software
* Next, calibrate your system setup using the [Calibration doc](calibration.md)
