cmake_minimum_required(VERSION 3.16)
project(insta360_capture)

set(CMAKE_CXX_STANDARD 17)

# Path to the extracted CameraSDK directory.
# Override at configure time: cmake -DCAMERA_SDK_DIR=/path/to/CameraSDK ..
if(NOT DEFINED CAMERA_SDK_DIR)
    set(CAMERA_SDK_DIR "$ENV{HOME}/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux")
endif()

# Full capture + stitch binary (requires camera connected)
add_executable(insta360_capture main.cpp)
target_include_directories(insta360_capture PRIVATE ${CAMERA_SDK_DIR}/include /usr/include)
target_link_directories(insta360_capture PRIVATE ${CAMERA_SDK_DIR}/lib /usr/lib)
target_link_libraries(insta360_capture PRIVATE CameraSDK)
set_target_properties(insta360_capture PROPERTIES
    BUILD_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
    INSTALL_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
)

# Stitch-only binary (no camera needed, for post-processing .insp files)
add_executable(insta360_stitch stitch.cpp)
target_include_directories(insta360_stitch PRIVATE /usr/include)
target_link_directories(insta360_stitch PRIVATE /usr/lib)
target_link_libraries(insta360_stitch PRIVATE MediaSDK)
set_target_properties(insta360_stitch PROPERTIES
    BUILD_RPATH "/usr/lib"
    INSTALL_RPATH "/usr/lib"
)
