cmake_minimum_required(VERSION 3.16)
project(insta360_capture)

set(CMAKE_CXX_STANDARD 17)

# Path to the extracted CameraSDK directory.
# Override at configure time: cmake -DCAMERA_SDK_DIR=/path/to/CameraSDK ..
# CameraSDK 2.1.1 (supports X5): CameraSDK-20250812_192742-2.1.1-Linux
if(NOT DEFINED CAMERA_SDK_DIR)
    set(CAMERA_SDK_DIR "$ENV{HOME}/LinuxSDK/CameraSDK-20250812_192742-2.1.1-Linux")
endif()

# Full capture + stitch binary (requires camera connected)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(insta360_capture main.cpp)
target_include_directories(insta360_capture PRIVATE ${CAMERA_SDK_DIR}/include /usr/include ${LIBUSB_INCLUDE_DIRS})
target_link_directories(insta360_capture PRIVATE ${CAMERA_SDK_DIR}/lib /usr/lib)
target_link_libraries(insta360_capture PRIVATE CameraSDK ${LIBUSB_LIBRARIES})
set_target_properties(insta360_capture PROPERTIES
    BUILD_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
    INSTALL_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
)

# Stitch-only binary (no camera needed, for post-processing .insp files)
# MediaSDK 3.x: header is ins_stitcher.h (was stitcher/stitcher.h), namespace is ins (was ins_media)
find_package(OpenCV QUIET)
add_executable(insta360_stitch stitch.cpp)
target_include_directories(insta360_stitch PRIVATE /usr/include ${OpenCV_INCLUDE_DIRS})
target_link_directories(insta360_stitch PRIVATE /usr/lib)
target_link_libraries(insta360_stitch PRIVATE MediaSDK ${OpenCV_LIBS})
set_target_properties(insta360_stitch PROPERTIES
    BUILD_RPATH "/usr/lib"
    INSTALL_RPATH "/usr/lib"
)

# Clock reset utility — syncs host system time to the camera RTC.
add_executable(insta360_reset_clock reset_clock.cpp)
target_include_directories(insta360_reset_clock PRIVATE ${CAMERA_SDK_DIR}/include /usr/include)
target_link_directories(insta360_reset_clock PRIVATE ${CAMERA_SDK_DIR}/lib /usr/lib)
target_link_libraries(insta360_reset_clock PRIVATE CameraSDK)
set_target_properties(insta360_reset_clock PROPERTIES
    BUILD_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
    INSTALL_RPATH "${CAMERA_SDK_DIR}/lib;/usr/lib"
)
