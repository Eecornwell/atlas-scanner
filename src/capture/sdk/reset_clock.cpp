#include <iostream>
#include <chrono>
#include <camera/camera.h>
#include <camera/device_discovery.h>

int main() {
    ins_camera::DeviceDiscovery discovery;
    auto list = discovery.GetAvailableDevices();
    if (list.empty()) { std::cerr << "No camera found\n"; return 1; }

    auto* cam = new ins_camera::Camera(list[0].info);
    cam->SetTimeout(10000);
    if (!cam->Open()) { std::cerr << "Failed to open camera\n"; return 1; }

    // Try seconds (uint64_t seconds since epoch)
    auto now_s = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::cout << "Current time (s):  " << now_s << "\n";
    std::cout << "Current time (ms): " << now_ms << "\n";

    // Try with seconds
    bool ok_s = cam->SyncLocalTimeToCamera((uint64_t)now_s, 0);
    std::cout << "SyncLocalTimeToCamera(seconds): " << (ok_s ? "OK" : "FAILED") << "\n";

    cam->Close();
    delete cam;
    return 0;
}
