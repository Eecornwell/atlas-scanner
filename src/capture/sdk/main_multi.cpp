#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <set>
#include <vector>
#include <atomic>
#include <optional>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

namespace fs = std::filesystem;

// Multi-camera continuous mode: host-controlled alternating TakePhoto() loop.
//
// Supports 1-3 cameras. When N cameras are connected, shots are fired in
// round-robin order with interval/N spacing between consecutive shots.
// Each camera gets its own timer thread; the download queue is shared.
//
// Protocol (INSTA360_SESSION_DIR):
//   .sdk_ready / .sdk_capture_trigger / .sdk_capture_done / .sdk_capture_failed
//   .sdk_downloads_pending / .session_done
//   .sdk_camera_count (written at startup: number of active cameras)

static ins_camera::DeviceDiscovery g_discovery;
static std::mutex              g_log_mutex;

#define LOG_OUT(msg) do { std::unique_lock<std::mutex> _lk(g_log_mutex); std::cout << msg << std::endl; } while(0)
#define LOG_ERR(msg) do { std::unique_lock<std::mutex> _lk(g_log_mutex); std::cerr << msg << std::endl; } while(0)

static constexpr int MAX_CAMERAS = 3;

static double now_sec() {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1e6;
}

static std::string sanitise(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
        [](unsigned char c) { return c < 0x20 || c == 0x7f; }), s.end());
    return s;
}

static void write_shutter_event(const std::string& session_dir, int index, double t, int cam_idx) {
    std::string safe_dir = sanitise(session_dir);
    fs::path base = fs::weakly_canonical(fs::path(safe_dir));
    char idx_buf[8];
    std::snprintf(idx_buf, sizeof(idx_buf), "%03d", index + 1);
    fs::path scan = fs::weakly_canonical(base / ("fusion_scan_" + std::string(idx_buf)));
    if (scan.string().rfind(base.string(), 0) != 0) return;
    fs::create_directories(scan);
    fs::path event = scan / ("capture_" + std::to_string(index) + ".shutter_event");
    if (event.string().rfind(base.string(), 0) != 0) return;
    std::ofstream se(event);
    se << std::fixed << std::setprecision(6) << t << " " << cam_idx;
}

static bool http_download(const std::string& url_base, const std::string& remote_path, const std::string& local_path) {
    std::string url = url_base;
    if (!url.empty() && url.back() == '/' && !remote_path.empty() && remote_path.front() == '/')
        url += remote_path.substr(1);
    else
        url += remote_path;

    pid_t pid = fork();
    if (pid < 0) { return false; }
    if (pid == 0) {
        const char* argv[] = {
            "curl", "-sf", "--max-time", "45", "--retry", "2", "--retry-delay", "1",
            "-o", local_path.c_str(),
            url.c_str(),
            nullptr
        };
        execvp("curl", const_cast<char* const*>(argv));
        _exit(127);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    int ret = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (ret != 0) { std::remove(local_path.c_str()); return false; }
    struct stat st;
    if (stat(local_path.c_str(), &st) != 0 || st.st_size < 100000) {
        std::remove(local_path.c_str()); return false;
    }
    return true;
}

static bool open_session(ins_camera::Camera* cam) {
    auto f = std::async(std::launch::async, [cam]() { return cam->Open(); });
    if (f.wait_for(std::chrono::seconds(8)) != std::future_status::ready) { LOG_ERR("Open() timed out"); return false; }
    return f.get();
}

// Cached settings parsed once from env vars, applied uniformly to all cameras.
struct CameraSettings {
    int exposure_mode = 2;
    int wb_mode = 2;
    int photo_size = -1;
    int ev_bias = 0;

    void load_from_env() {
        if (auto* v = std::getenv("INSTA360_EXPOSURE"))
            exposure_mode = std::atoi(v);
        if (auto* v = std::getenv("INSTA360_WB"))
            wb_mode = std::atoi(v);
        if (auto* v = std::getenv("INSTA360_PHOTO_SIZE_DEFAULT"))
            photo_size = std::atoi(v);
        if (auto* v = std::getenv("INSTA360_EV_BIAS"))
            ev_bias = std::atoi(v);
    }
};
static CameraSettings g_settings;

static bool apply_camera_settings(ins_camera::Camera* cam, int cam_idx) {
    bool ok = true;

    // Exposure
    if (g_settings.exposure_mode != 0) {
        auto exp = std::make_shared<ins_camera::ExposureSettings>();
        auto mode = (g_settings.exposure_mode == 4)
            ? ins_camera::PhotographyOptions_ExposureMode::ADAPTIVE
            : ins_camera::PhotographyOptions_ExposureMode::SHUTTER_PRIORITY;
        exp->SetExposureMode(mode);
        exp->SetShutterSpeed(1.0 / 100.0);
        exp->SetEVBias(g_settings.ev_bias);
        if (!cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, exp)) {
            LOG_ERR("Camera [" << cam_idx << "] SetExposureSettings failed");
            ok = false;
        }
    }

    // White balance
    {
        static const ins_camera::PhotographyOptions_WhiteBalance wb_map[] = {
            ins_camera::PhotographyOptions_WhiteBalance::WB_AUTO,
            ins_camera::PhotographyOptions_WhiteBalance::WB_2700K,
            ins_camera::PhotographyOptions_WhiteBalance::WB_4000K,
            ins_camera::PhotographyOptions_WhiteBalance::WB_5000K,
            ins_camera::PhotographyOptions_WhiteBalance::WB_6500K,
            ins_camera::PhotographyOptions_WhiteBalance::WB_7500K,
        };
        auto wb = (g_settings.wb_mode >= 0 && g_settings.wb_mode <= 5)
            ? wb_map[g_settings.wb_mode]
            : ins_camera::PhotographyOptions_WhiteBalance::WB_AUTO;
        auto cap = std::make_shared<ins_camera::CaptureSettings>();
        cap->SetWhiteBalance(wb);
        if (!cam->SetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, cap)) {
            LOG_ERR("Camera [" << cam_idx << "] SetCaptureSettings failed");
            ok = false;
        }
    }

    // Photo size
    if (g_settings.photo_size >= 0) {
        if (!cam->SetPhotoSize(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE,
                              static_cast<ins_camera::PhotoSize>(g_settings.photo_size))) {
            LOG_ERR("Camera [" << cam_idx << "] SetPhotoSize failed");
            ok = false;
        }
    }

    // Photo sub-mode
    if (!cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE)) {
        LOG_ERR("Camera [" << cam_idx << "] SetPhotoSubMode failed");
        ok = false;
    }

    // Sync host clock to camera RTC so all cameras share the same time base
    auto now = std::chrono::system_clock::now();
    uint64_t utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    // offset_time: local timezone offset in minutes (not critical for our use)
    auto tt = std::chrono::system_clock::to_time_t(now);
    struct tm local_tm;
    localtime_r(&tt, &local_tm);
    int32_t tz_offset_min = static_cast<int32_t>(local_tm.tm_gmtoff / 60);
    if (!cam->SyncLocalTimeToCamera(utc_ms, static_cast<uint32_t>(tz_offset_min))) {
        LOG_ERR("Camera [" << cam_idx << "] SyncLocalTimeToCamera failed (non-fatal)");
        // Non-fatal: RTC sync failure doesn't prevent capture
    }

    return ok;
}

static void flush_usb_endpoints() {
    // In multi-camera mode, flushing USB endpoints can interfere with the SDK's
    // ability to discover and open subsequent cameras. The SDK's own Open()
    // retry logic handles stale data adequately. Skip the flush entirely —
    // it was only needed for single-camera recovery from unclean shutdowns,
    // which the shell script's USB reset already handles.
}

// Parse INSTA360_CAMERA_SERIALS env var: comma-separated serial numbers.
// If empty, accept all discovered cameras (up to MAX_CAMERAS).
static std::vector<std::string> parse_serial_filter() {
    std::vector<std::string> serials;
    if (auto* v = std::getenv("INSTA360_CAMERA_SERIALS")) {
        std::string s(v);
        size_t pos = 0;
        while ((pos = s.find(',')) != std::string::npos) {
            std::string tok = s.substr(0, pos);
            if (!tok.empty()) serials.push_back(tok);
            s.erase(0, pos + 1);
        }
        if (!s.empty()) serials.push_back(s);
    }
    return serials;
}

struct CameraSlot {
    ins_camera::Camera* cam = nullptr;
    std::string         serial;
    std::string         http_base;
    int                 index = 0;
    bool                active = false;
};

struct DownloadJob {
    int         global_shot;
    int         cam_idx;
    std::string http_base;
    std::string remote_path;
    std::string local_path;
    double      t_shutter;
    double      t_after;
};

int main(int argc, char* argv[]) {
    // --discover mode: list cameras and exit
    if (argc > 1 && std::string(argv[1]) == "--discover") {
        flush_usb_endpoints();
        auto list = g_discovery.GetAvailableDevices();
        std::cout << "Found " << list.size() << " camera(s):" << std::endl;
        for (size_t i = 0; i < list.size(); ++i) {
            std::cout << "  [" << i << "] serial=" << sanitise(list[i].serial_number)
                      << "  type=" << int(list[i].camera_type)
                      << "  name=" << sanitise(list[i].camera_name) << std::endl;
        }
        return 0;
    }

    std::string session_dir = "./output";
    if (auto* v = std::getenv("INSTA360_SESSION_DIR")) session_dir = sanitise(v);
    if (session_dir.empty()) session_dir = "./output";
    if (session_dir[0] != '/') session_dir = fs::current_path().string() + "/" + session_dir;
    fs::create_directories(session_dir);

    flush_usb_endpoints();

    // The Insta360 SDK's GetAvailableDevices() may only return one device at a
    // time if multiple cameras share the same VID:PID. Work around this by
    // discovering and opening cameras sequentially: discover → open first →
    // re-discover → open second (the already-opened device won't appear again).
    auto serial_filter = parse_serial_filter();

    // When serial_filter is set, we open cameras in the ORDER specified by the
    // filter list. serial_filter[0] = cam_0, serial_filter[1] = cam_1, etc.
    // This guarantees deterministic index assignment regardless of USB discovery order.
    std::vector<CameraSlot> slots;
    std::set<std::string> opened_serials;

    if (!serial_filter.empty()) {
        // Open cameras in the exact order specified by serial_filter
        for (size_t target_idx = 0; target_idx < serial_filter.size(); ++target_idx) {
            const std::string& target_sn = serial_filter[target_idx];
            bool found = false;
            for (int attempt = 0; attempt < 3 && !found; ++attempt) {
                if (attempt > 0)
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                auto dev_list = g_discovery.GetAvailableDevices();
                LOG_OUT("[discover] attempt " << attempt << " for '" << target_sn
                        << "': " << dev_list.size() << " device(s)");
                for (size_t i = 0; i < dev_list.size(); ++i) {
                    std::string sn = sanitise(dev_list[i].serial_number);
                    LOG_OUT("[discover]   [" << i << "] serial='" << sn << "'"
                            << (opened_serials.count(sn) ? " (already opened)" : ""));
                    if (sn != target_sn) continue;
                    if (opened_serials.count(sn)) { found = true; break; }

                    CameraSlot slot;
                    slot.serial = sn;
                    slot.index = static_cast<int>(target_idx);
                    slot.cam = new ins_camera::Camera(dev_list[i].info);
                    slot.cam->SetTimeout(20000);
                    slot.cam->SetServicePort(8090 + slot.index);

                    bool opened = false;
                    for (int retry = 1; retry <= 3 && !opened; ++retry) {
                        if (retry > 1)
                            std::this_thread::sleep_for(std::chrono::seconds(retry * 2));
                        if (open_session(slot.cam)) {
                            opened = true;
                        } else {
                            delete slot.cam;
                            auto fresh = g_discovery.GetAvailableDevices();
                            for (auto& d : fresh) {
                                if (sanitise(d.serial_number) == sn) {
                                    slot.cam = new ins_camera::Camera(d.info);
                                    slot.cam->SetTimeout(20000);
                                    slot.cam->SetServicePort(8090 + slot.index);
                                    break;
                                }
                            }
                        }
                    }
                    if (opened) {
                        slot.active = true;
                        slot.http_base = slot.cam->GetHttpBaseUrl();
                        opened_serials.insert(sn);
                        slots.push_back(slot);
                        LOG_OUT("Camera [" << slot.index << "] " << sn << " opened  HTTP=" << sanitise(slot.http_base));
                        found = true;
                    } else {
                        LOG_ERR("Camera [" << slot.index << "] " << sn << " FAILED to open");
                        delete slot.cam;
                    }
                    break;
                }
            }
            if (!found) {
                LOG_ERR("Camera serial " << target_sn << " not found after retries");
            }
        }
    } else {
        // No serial filter: accept cameras in discovery order (original behavior)
        for (int pass = 0; pass < MAX_CAMERAS; ++pass) {
            auto dev_list = g_discovery.GetAvailableDevices();
            if (dev_list.empty()) {
                if (pass == 0) {
                    LOG_ERR("No camera found on first attempt, retrying in 3s...");
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    dev_list = g_discovery.GetAvailableDevices();
                    if (dev_list.empty()) { LOG_ERR("No camera found"); _exit(1); }
                } else {
                    break;
                }
            }
            for (size_t i = 0; i < dev_list.size() && slots.size() < MAX_CAMERAS; ++i) {
                std::string sn = sanitise(dev_list[i].serial_number);
                if (opened_serials.count(sn)) continue;
                CameraSlot slot;
                slot.serial = sn;
                slot.index = static_cast<int>(slots.size());
                slot.cam = new ins_camera::Camera(dev_list[i].info);
                slot.cam->SetTimeout(20000);
                slot.cam->SetServicePort(8090 + slot.index);

                bool opened = false;
                for (int retry = 1; retry <= 3 && !opened; ++retry) {
                    if (retry > 1)
                        std::this_thread::sleep_for(std::chrono::seconds(retry * 2));
                    if (open_session(slot.cam)) {
                        opened = true;
                    } else {
                        delete slot.cam;
                        auto fresh = g_discovery.GetAvailableDevices();
                        for (auto& d : fresh) {
                            if (sanitise(d.serial_number) == sn) {
                                slot.cam = new ins_camera::Camera(d.info);
                                slot.cam->SetTimeout(20000);
                                slot.cam->SetServicePort(8090 + slot.index);
                                break;
                            }
                        }
                    }
                }
                if (opened) {
                    slot.active = true;
                    slot.http_base = slot.cam->GetHttpBaseUrl();
                    opened_serials.insert(sn);
                    slots.push_back(slot);
                    LOG_OUT("Camera [" << slot.index << "] " << sn << " opened  HTTP=" << sanitise(slot.http_base));
                    break;  // re-discover on next pass
                } else {
                    LOG_ERR("Camera [" << slot.index << "] " << sn << " FAILED to open");
                    delete slot.cam;
                }
            }
        }
    }

    // Remove inactive slots
    slots.erase(std::remove_if(slots.begin(), slots.end(),
        [](const CameraSlot& s) { return !s.active; }), slots.end());
    // Re-index
    for (size_t i = 0; i < slots.size(); ++i) slots[i].index = static_cast<int>(i);

    if (slots.empty()) { LOG_ERR("No cameras opened successfully"); _exit(1); }

    // Apply uniform settings to all opened cameras
    g_settings.load_from_env();
    LOG_OUT("Settings: exposure=" << g_settings.exposure_mode
            << " wb=" << g_settings.wb_mode
            << " photo_size=" << g_settings.photo_size);
    for (auto& slot : slots) {
        if (apply_camera_settings(slot.cam, slot.index))
            LOG_OUT("Camera [" << slot.index << "] " << slot.serial << " configured OK");
        else
            LOG_OUT("Camera [" << slot.index << "] " << slot.serial << " configured with warnings");
    }

    int num_cameras = static_cast<int>(slots.size());
    LOG_OUT(num_cameras << " camera(s) active — alternating interval will be divided by " << num_cameras);

    // Write camera count for shell script
    { std::ofstream f(session_dir + "/.sdk_camera_count"); f << num_cameras; }
    // Write camera serial mapping for post-processing
    {
        std::ofstream f(session_dir + "/.sdk_camera_map");
        for (auto& s : slots)
            f << s.index << " " << s.serial << "\n";
    }

    { std::ofstream f(session_dir + "/.sdk_ready"); }

    const std::string trigger_path = session_dir + "/.sdk_capture_trigger";
    const std::string done_path    = session_dir + "/.sdk_capture_done";
    const std::string failed_path  = session_dir + "/.sdk_capture_failed";
    const std::string quit_path    = session_dir + "/.session_done";
    const std::string pending_path = session_dir + "/.sdk_downloads_pending";

    bool   continuous_active = false;
    int    interval_s        = 5;
    double t_start           = 0.0;

    std::atomic<bool>   timer_stop{false};
    std::atomic<int>    global_shot_counter{0};

    // Download queue (shared across all cameras)
    std::queue<DownloadJob>  dl_queue;
    std::mutex               dl_mutex;
    std::condition_variable  dl_cv;
    std::atomic<bool>        dl_stop{false};
    std::thread              dl_thread;

    // Per-camera timer threads
    std::vector<std::thread> timer_threads;

    static std::atomic<int> stat_dl_pending{0};

    while (true) {

        if (fs::exists(quit_path)) {
            LOG_OUT("Session done — stopping capture...");
            if (continuous_active) {
                timer_stop.store(true);
                for (auto& t : timer_threads) { if (t.joinable()) t.join(); }
                timer_threads.clear();
                continuous_active = false;
                LOG_OUT("Capture stopped. Total shots: " << global_shot_counter.load());
            }
            // Drain stationary download queue before exiting
            while (stat_dl_pending.load() > 0) {
                LOG_OUT("Waiting for " << stat_dl_pending.load() << " download(s) to finish...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            { std::ofstream pf(pending_path); pf << 0; }
            break;
        }

        if (!fs::exists(trigger_path)) {
            // Update pending count for shell to read during download-wait
            static int last_pending = -1;
            int cur = stat_dl_pending.load();
            if (cur != last_pending) {
                { std::ofstream pf(pending_path); pf << cur; }
                last_pending = cur;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::string trigger_content;
        { std::ifstream f(trigger_path); if (f) std::getline(f, trigger_content); }
        fs::remove(trigger_path);

        if (trigger_content.empty()) { std::ofstream ff(failed_path); ff << "fail"; continue; }
        if (trigger_content[0] != '/') trigger_content = fs::current_path().string() + "/" + trigger_content;

        if (trigger_content == session_dir) {
            // --- Continuous mode: alternating TakePhoto() across N cameras ---
            interval_s = 5;
            if (auto* v = std::getenv("INSTA360_INTERVAL_MS"))
                interval_s = std::max(1, std::atoi(v) / 1000);

            // Settings already applied at open time — no per-trigger reconfiguration needed.

            t_start = now_sec();
            continuous_active = true;
            timer_stop.store(false);
            global_shot_counter.store(0);

            // Download worker
            dl_stop.store(false);
            dl_thread = std::thread([&]() {
                while (true) {
                    DownloadJob job;
                    {
                        std::unique_lock<std::mutex> lk(dl_mutex);
                        dl_cv.wait(lk, [&]{ return !dl_queue.empty() || dl_stop.load(); });
                        if (dl_queue.empty()) break;
                        job = dl_queue.front();
                        dl_queue.pop();
                    }
                    LOG_OUT("[dl] cam[" << job.cam_idx << "] downloading shot " << job.global_shot
                            << " <- " << sanitise(fs::path(job.remote_path).filename().string()));
                    bool ok = http_download(job.http_base, job.remote_path, job.local_path);
                    if (ok) {
                        // Write .capture_time: "t_shutter t_after cam_idx"
                        std::ofstream ct(job.local_path + ".capture_time");
                        ct << std::fixed << std::setprecision(6)
                           << job.t_shutter << " " << job.t_after << " " << job.cam_idx;

                        // Also write into fusion_scan_NNN/
                        char idx_buf[8];
                        std::snprintf(idx_buf, sizeof(idx_buf), "%03d", job.global_shot + 1);
                        fs::path scan_dir = fs::path(session_dir) / ("fusion_scan_" + std::string(idx_buf));
                        fs::create_directories(scan_dir);
                        fs::path insp_name = fs::path(job.local_path).filename();
                        std::ofstream ct2(scan_dir / (insp_name.string() + ".capture_time"));
                        ct2 << std::fixed << std::setprecision(6)
                            << job.t_shutter << " " << job.t_after << " " << job.cam_idx;

                        // Write cam_idx marker so post-processing knows which calibration to use
                        std::ofstream ci(scan_dir / ".cam_index");
                        ci << job.cam_idx;

                        LOG_OUT("[dl] saved shot " << job.global_shot << " cam[" << job.cam_idx << "]: "
                                << sanitise(insp_name.string()));

                        // Delete from camera storage to prevent filling SD card
                        if (!slots[job.cam_idx].cam->DeleteCameraFile(job.remote_path))
                            LOG_ERR("[dl] failed to delete " << sanitise(insp_name.string()) << " from cam[" << job.cam_idx << "]");
                    } else {
                        LOG_ERR("[dl] download failed: " << sanitise(job.remote_path));
                    }
                }
                LOG_OUT("[dl] download worker exiting");
            });

            // Single scheduler thread dispatches shots in strict round-robin.
            // Each camera has its own worker thread that blocks on TakePhoto().
            // The scheduler waits for the assigned camera to be idle before
            // dispatching, maintaining consistent stagger regardless of variable
            // TakePhoto() blocking duration.
            static std::atomic<bool> cam_busy[MAX_CAMERAS];
            for (int i = 0; i < MAX_CAMERAS; ++i) cam_busy[i].store(false);
            std::mutex shot_mutex;
            std::condition_variable shot_cv;

            struct ShotRequest {
                int cam_idx = 0;
                int shot_num = 0;
                std::chrono::steady_clock::time_point scheduled_time{};
            };
            static std::queue<ShotRequest> cam_queues[MAX_CAMERAS];
            for (int i = 0; i < MAX_CAMERAS; ++i)
                while (!cam_queues[i].empty()) cam_queues[i].pop();  // clear from prior session

            // Shared: last time a shutter actually fired (any camera).
            // Updated by worker threads, read by scheduler to enforce spacing.
            static std::atomic<double> last_shutter_time{0.0};
            last_shutter_time.store(0.0);

            // Per-camera worker threads: pull from their queue and call TakePhoto()
            for (int ci = 0; ci < num_cameras; ++ci) {
                timer_threads.emplace_back([&, ci]() {
                    while (!timer_stop.load()) {
                        ShotRequest req;
                        {
                            std::unique_lock<std::mutex> lk(shot_mutex);
                            shot_cv.wait(lk, [&]{ return !cam_queues[ci].empty() || timer_stop.load(); });
                            if (timer_stop.load()) break;
                            req = cam_queues[ci].front();
                            cam_queues[ci].pop();
                        }

                        // Wait until scheduled time
                        while (!timer_stop.load() && std::chrono::steady_clock::now() < req.scheduled_time)
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        if (timer_stop.load()) break;

                        cam_busy[ci].store(true);
                        double t_before = now_sec();
                        last_shutter_time.store(t_before);

                        LOG_OUT("[cam" << ci << "] taking shot " << req.shot_num << " at t+"
                                << std::fixed << std::setprecision(1) << (t_before - t_start) << "s");

                        auto url = slots[ci].cam->TakePhoto();
                        double t_after = now_sec();
                        cam_busy[ci].store(false);

                        if (url.Empty() || !url.IsSingleOrigin()) {
                            LOG_ERR("[cam" << ci << "] shot " << req.shot_num << " TakePhoto failed");
                            continue;
                        }

                        double t_shutter = t_before;
                        LOG_OUT("[cam" << ci << "] shot " << req.shot_num
                                << " latency: " << std::fixed << std::setprecision(3)
                                << (t_after - t_before) << "s");
                        write_shutter_event(session_dir, req.shot_num, t_shutter, ci);

                        // Enqueue download
                        std::string remote_path = url.GetSingleOrigin();
                        std::string shot_dir = session_dir + "/.sdk_shot_" + std::to_string(req.shot_num);
                        fs::create_directories(shot_dir);
                        std::string local_path = shot_dir + "/" + fs::path(remote_path).filename().string();
                        {
                            std::unique_lock<std::mutex> lk(dl_mutex);
                            dl_queue.push({req.shot_num, ci, slots[ci].http_base,
                                          remote_path, local_path, t_shutter, t_after});
                        }
                        dl_cv.notify_one();
                    }
                    LOG_OUT("[cam" << ci << "] worker exiting");
                });
            }

            // Each camera gets its own fixed-cadence schedule anchored to t_start.
            // cam_N fires at: t_start + (N * spacing_ms) + (k * per_camera_interval_ms)
            // This prevents convergence regardless of variable TakePhoto() latency.
            // If a camera misses its slot (still processing), it waits for the next one.
            int spacing_ms = interval_s * 1000;  // gap between any two consecutive shots
            int per_cam_ms = spacing_ms * num_cameras;  // each camera's own repeat interval

            for (int ci = 0; ci < num_cameras; ++ci) {
                int offset_ms = ci * spacing_ms;  // stagger: cam0 at 0, cam1 at spacing, cam2 at 2*spacing
                timer_threads.emplace_back([&, ci, offset_ms, per_cam_ms]() {
                    // This camera's first shot time
                    auto epoch = std::chrono::steady_clock::now();
                    auto next_fire = epoch + std::chrono::milliseconds(offset_ms + per_cam_ms);

                    while (!timer_stop.load()) {
                        // Wait until our scheduled fire time
                        while (!timer_stop.load() && std::chrono::steady_clock::now() < next_fire)
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        if (timer_stop.load()) break;

                        // If still busy from previous shot, skip to next slot
                        if (cam_busy[ci].load()) {
                            LOG_OUT("[cam" << ci << "] still busy, skipping slot");
                            next_fire += std::chrono::milliseconds(per_cam_ms);
                            continue;
                        }

                        int shot = global_shot_counter.fetch_add(1);
                        {
                            std::unique_lock<std::mutex> lk(shot_mutex);
                            cam_queues[ci].push({ci, shot, std::chrono::steady_clock::now()});
                        }
                        shot_cv.notify_all();

                        // Advance to next fixed slot for this camera
                        next_fire += std::chrono::milliseconds(per_cam_ms);
                    }
                    shot_cv.notify_all();
                    LOG_OUT("[sched-cam" << ci << "] exiting");
                });
            }

            LOG_OUT("Continuous capture started (" << num_cameras << " cameras, "
                    << "shot every " << interval_s << "s, "
                    << "per-camera every " << interval_s * num_cameras << "s)");
            { std::ofstream df(done_path); df << "ok"; }

        } else {
            // --- Stationary mode: single shot on next camera in round-robin ---
            static int stat_cam_idx = 0;
            auto& slot = slots[stat_cam_idx % num_cameras];
            fs::create_directories(trigger_content);

            int shot = global_shot_counter.fetch_add(1);
            LOG_OUT("Taking photo " << shot << " on cam[" << slot.index << "]...");
            auto url = slot.cam->TakePhoto();
            if (url.Empty() || !url.IsSingleOrigin()) { std::ofstream ff(failed_path); ff << "fail"; stat_cam_idx++; continue; }

            double host_t = now_sec();
            write_shutter_event(session_dir, shot, host_t, slot.index);

            // Signal done immediately so shell can trigger next camera without
            // waiting for the (potentially slow) USB download to complete.
            std::string remote_path = url.GetSingleOrigin();
            std::string local_path = std::string(trigger_content) + "/" + fs::path(remote_path).filename().string();
            LOG_OUT("[dl] cam[" << slot.index << "] URL: " << sanitise(slot.http_base) << sanitise(remote_path)
                    << " -> " << sanitise(fs::path(local_path).filename().string()));

            // Write cam_index marker immediately (doesn't depend on download)
            { std::ofstream ci(fs::path(trigger_content) / ".cam_index"); ci << slot.index; }
            // Write capture_time sidecar immediately
            { std::ofstream ct(local_path + ".capture_time");
              ct << std::fixed << std::setprecision(6) << host_t << " " << host_t << " " << slot.index; }

            { std::ofstream df(done_path); df << "ok"; df.flush(); }
            LOG_OUT("[stat] cam[" << slot.index << "] shot " << shot << " done signaled");
            // Brief pause to ensure shell detects done before next loop iteration
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Download in background — enqueue to shared download worker
            if (!dl_thread.joinable()) {
                dl_stop.store(false);
                dl_thread = std::thread([&]() {
                    while (true) {
                        DownloadJob job;
                        {
                            std::unique_lock<std::mutex> lk(dl_mutex);
                            dl_cv.wait(lk, [&]{ return !dl_queue.empty() || dl_stop.load(); });
                            if (dl_queue.empty()) break;
                            job = dl_queue.front();
                            dl_queue.pop();
                        }
                        LOG_OUT("[dl] downloading cam[" << job.cam_idx << "] shot " << job.global_shot);
                        bool ok = http_download(job.http_base, job.remote_path, job.local_path);
                        if (ok) {
                            LOG_OUT("[dl] saved: " << sanitise(fs::path(job.local_path).filename().string()));
                            if (job.cam_idx < static_cast<int>(slots.size()))
                                if (!slots[job.cam_idx].cam->DeleteCameraFile(job.remote_path))
                                    LOG_ERR("[dl] delete failed: " << sanitise(fs::path(job.remote_path).filename().string()));
                        } else {
                            LOG_ERR("[dl] download FAILED: " << sanitise(fs::path(job.remote_path).filename().string()));
                        }
                        stat_dl_pending.fetch_sub(1);
                        { std::ofstream pf(pending_path); pf << stat_dl_pending.load(); }
                    }
                });
            }
            {
                std::unique_lock<std::mutex> lk(dl_mutex);
                dl_queue.push({shot, slot.index, slot.http_base, remote_path, local_path, host_t, host_t});
            }
            stat_dl_pending.fetch_add(1);
            { std::ofstream pf(pending_path); pf << stat_dl_pending.load(); }
            dl_cv.notify_one();
            stat_cam_idx++;
        }
    }

    timer_stop.store(true);
    for (auto& t : timer_threads) { if (t.joinable()) t.join(); }
    dl_stop.store(true);
    dl_cv.notify_one();
    if (dl_thread.joinable()) dl_thread.join();

    for (auto& slot : slots) {
        slot.cam->Close();
        delete slot.cam;
    }
    return 0;
}
