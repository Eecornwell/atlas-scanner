#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <filesystem>
#include <future>
#include <chrono>
#include <thread>
#include <atomic>
#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

namespace fs = std::filesystem;

// Keep the SDK session open for the entire ATLAS session — closing and
// reopening between scans leaves the firmware's HTTP server in a bad state
// that causes download stalls on the 2nd+ scan.
//
// Protocol (via files in INSTA360_OUTPUT_DIR):
//   .capture_trigger  — shell writes this to request a photo
//   .capture_done     — we write this when photo+download complete (exit=0)
//   .capture_failed   — we write this when capture failed
//   .session_done     — shell writes this to shut down cleanly

static ins_camera::DeviceDiscovery g_discovery;
static std::vector<ins_camera::DeviceDescriptor> g_list;

static bool open_session(ins_camera::Camera* cam) {
    auto open_future = std::async(std::launch::async, [cam]() {
        return cam->Open();
    });
    if (open_future.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
        std::cerr << "Camera Open() timed out" << std::endl;
        return false;
    }
    return open_future.get();
}

static bool apply_camera_settings(ins_camera::Camera* cam) {
    // Shutter priority: fixed 1/100s shutter, auto ISO.
    // Eliminates overexposure from full-auto and gives the sensor enough
    // light for indoor scanning without motion blur from a long exposure.
    auto exp = std::make_shared<ins_camera::ExposureSettings>();
    exp->SetExposureMode(ins_camera::PhotographyOptions_ExposureOptions_Program_SHUTTER_PRIORITY);
    exp->SetShutterSpeed(1.0 / 100.0);
    exp->SetEVBias(0);
    if (!cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, exp)) {
        std::cerr << "SetExposureSettings failed" << std::endl;
        return false;
    }

    // Fixed 4000K white balance — prevents the blue cast that auto-WB
    // produces under mixed indoor fluorescent/LED lighting.
    auto cap = std::make_shared<ins_camera::CaptureSettings>();
    cap->SetWhiteBalance(ins_camera::PhotographyOptions_WhiteBalance_WB_4000K);
    if (!cam->SetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, cap)) {
        std::cerr << "SetCaptureSettings failed" << std::endl;
        return false;
    }

    std::cout << "Camera settings applied: shutter=1/100s, WB=4000K" << std::endl;
    return true;
}

static bool do_capture(ins_camera::Camera* cam, const std::string& output_dir) {
    if (!cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE)) {
        std::cerr << "SetPhotoSubMode failed (continuing anyway)" << std::endl;
    }

    std::cout << "Taking photo..." << std::endl;
    auto url = cam->TakePhoto();
    if (url.Empty() || !url.IsSingleOrigin()) {
        std::cerr << "Failed to take photo" << std::endl;
        return false;
    }

    std::string remote_path = url.GetSingleOrigin();
    std::cout << "Photo captured: " << remote_path << std::endl;

    std::string filename = fs::path(remote_path).filename().string();
    std::string local_path = output_dir + "/" + filename;

    std::cout << "Downloading..." << std::endl;

    std::atomic<bool> dl_done{false};
    std::atomic<bool> dl_timed_out{false};
    std::thread watchdog([&dl_done, &dl_timed_out, &local_path]() {
        for (int i = 0; i < 240; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (dl_done.load()) return;
        }
        std::cerr << "Download timed out — USB connection likely dropped" << std::endl;
        dl_timed_out.store(true);
        std::remove(local_path.c_str());
    });
    watchdog.detach();

    bool ok = cam->DownloadCameraFile(remote_path, local_path);
    dl_done.store(true);

    if (dl_timed_out.load() || !ok) {
        std::cerr << "Download failed" << std::endl;
        std::remove(local_path.c_str());
        return false;
    }

    std::cout << "Saved: " << local_path << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::string session_dir = "./output";
    if (auto* v = std::getenv("INSTA360_SESSION_DIR")) session_dir = v;
    if (session_dir.empty()) session_dir = "./output";
    // Resolve to absolute path
    if (session_dir[0] != '/') {
        session_dir = fs::current_path().string() + "/" + session_dir;
    }
    fs::create_directories(session_dir);

    // Discovery — once only
    g_list = g_discovery.GetAvailableDevices();
    if (g_list.empty()) {
        std::cerr << "No camera found" << std::endl;
        _exit(1);
    }
    std::cout << "Found camera: " << g_list[0].serial_number
              << " type=" << int(g_list[0].camera_type) << std::endl;

    const ins_camera::DeviceConnectionInfo& conn = g_list[0].info;

    // Open session — retry until firmware is ready
    auto* cam = new ins_camera::Camera(conn);
    cam->SetTimeout(20000);

    bool opened = false;
    for (int attempt = 1; attempt <= 5 && !opened; ++attempt) {
        if (attempt > 1) {
            int wait = (attempt - 1) * 3;
            std::cerr << "Retrying Open() in " << wait << "s (attempt " << attempt << "/5)..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(wait));
        }
        if (open_session(cam)) {
            opened = true;
        } else {
            cam = new ins_camera::Camera(conn);
            cam->SetTimeout(20000);
        }
    }
    if (!opened) {
        std::cerr << "Failed to open camera after 5 attempts" << std::endl;
        _exit(1);
    }

    std::cout << "Camera session open — ready for captures" << std::endl;

    if (!apply_camera_settings(cam)) {
        std::cerr << "Failed to apply camera settings — continuing with defaults" << std::endl;
    }

    // Signal ready
    std::string ready_file = session_dir + "/.sdk_ready";
    { std::ofstream f(ready_file); }

    // Event loop — wait for trigger files
    while (true) {
        std::string trigger = session_dir + "/.sdk_capture_trigger";
        std::string done    = session_dir + "/.sdk_capture_done";
        std::string failed  = session_dir + "/.sdk_capture_failed";
        std::string quit    = session_dir + "/.session_done";
        std::string out_dir = session_dir + "/.capture_output_dir";

        if (fs::exists(quit)) {
            std::cout << "Session done signal received" << std::endl;
        break;
    }

        if (fs::exists(trigger)) {
            // Read per-scan output dir from trigger file
            std::string scan_output;
            {
                std::ifstream f(trigger);
                if (f) std::getline(f, scan_output);
            }
            fs::remove(trigger);

            if (scan_output.empty()) {
                std::cerr << "Empty scan output dir in trigger, skipping" << std::endl;
                std::ofstream ff(failed); ff << "fail";
                continue;
            }

            // Make absolute if relative
            if (!scan_output.empty() && scan_output[0] != '/') {
                scan_output = fs::current_path().string() + "/" + scan_output;
            }

            fs::create_directories(scan_output);
            bool ok = do_capture(cam, scan_output);
            if (ok) {
                std::ofstream df(done); df << "ok";
            } else {
                std::ofstream ff(failed); ff << "fail";
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    cam->Close();
    delete cam;
    return 0;
}
