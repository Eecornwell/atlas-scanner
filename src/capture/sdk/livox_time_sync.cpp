// livox_time_sync.cpp
// One-shot utility: connects to the MID360, sends the current system time
// via SetLivoxLidarRmcSyncTime(), then exits immediately.
// Run ONCE before livox_ros_driver2 starts so there is no concurrent SDK2
// instance competing for the LiDAR command port.

#include <iostream>
#include <fstream>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdint>
#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

static std::atomic<uint32_t> g_handle{0};
static std::atomic<bool>     g_ready{false};
static std::atomic<bool>     g_done{false};

static std::string make_gprmc() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&t, &utc);
    // Include centiseconds for sub-second precision (NMEA supports HH:MM:SS.ss)
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    int cs = static_cast<int>(ms / 10);  // centiseconds
    char body[96];
    std::snprintf(body, sizeof(body),
        "GPRMC,%02d%02d%02d.%02d,A,0000.00,N,00000.00,E,0.0,0.0,%02d%02d%02d,,",
        utc.tm_hour, utc.tm_min, utc.tm_sec, cs,
        utc.tm_mday, utc.tm_mon + 1, utc.tm_year % 100);
    uint8_t cksum = 0;
    for (const char* p = body; *p; ++p) cksum ^= static_cast<uint8_t>(*p);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cksum);
    return std::string(buf);
}

static void info_cb(const uint32_t handle, const LivoxLidarInfo* info, void*) {
    if (!info || g_ready.load()) return;
    std::cout << "[livox_time_sync] found: " << info->sn
              << "  handle=" << handle << std::endl;
    g_handle.store(handle);
    g_ready.store(true);
}

int main() {
    // Write config to temp file — use alternate host ports so this utility
    // can coexist briefly with livox_ros_driver2 if needed, though it is
    // intended to run before the ROS driver starts.
    const std::string cfg = R"({
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port":   56100,
      "push_msg_port":   56200,
      "point_data_port": 56300,
      "imu_data_port":   56400,
      "log_data_port":   56500
    },
    "host_net_info": [{
      "host_ip":         "192.168.1.50",
      "multicast_ip":    "224.1.1.5",
      "cmd_data_port":   57101,
      "push_msg_port":   57201,
      "point_data_port": 57301,
      "imu_data_port":   57401,
      "log_data_port":   57501
    }]
  }
})";
    const std::string cfg_path = "/tmp/.livox_time_sync.json";
    { std::ofstream f(cfg_path); f << cfg; }

    if (!LivoxLidarSdkInit(cfg_path.c_str())) {
        std::cerr << "[livox_time_sync] SDK init failed" << std::endl;
        return 1;
    }
    SetLivoxLidarInfoChangeCallback(info_cb, nullptr);

    // Wait up to 10s for LiDAR to appear
    for (int i = 0; i < 100 && !g_ready.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!g_ready.load()) {
        std::cerr << "[livox_time_sync] MID360 not found within 10s" << std::endl;
        LivoxLidarSdkUninit();
        return 1;
    }

    std::string gprmc = make_gprmc();
    uint32_t handle = g_handle.load();

    SetLivoxLidarRmcSyncTime(handle, gprmc.c_str(),
        static_cast<uint16_t>(gprmc.size()),
        [](livox_status status, uint32_t h, LivoxLidarRmcSyncTimeResponse* resp, void*) {
            if (resp && resp->ret == 0)
                std::cout << "[livox_time_sync] OK  handle=" << h << std::endl;
            else
                std::cerr << "[livox_time_sync] FAILED  status=" << status
                          << "  ret=" << (resp ? (int)resp->ret : -1) << std::endl;
            g_done.store(true);
        }, nullptr);

    // Wait for callback (max 3s)
    for (int i = 0; i < 30 && !g_done.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LivoxLidarSdkUninit();
    return g_done.load() ? 0 : 1;
}
