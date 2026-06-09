#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
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

// Continuous mode: TIMELAPSE_INTERVAL_SHOOTING with in-session HTTP download.
//
// Each shot is downloaded during the active timelapse session using the camera's
// HTTP file server (GetHttpBaseUrl). The firmware only serves files that are
// currently accessible in its HTTP session — during an active timelapse this
// includes the most recently written .insp file.
//
// Timer thread flow per shot:
//  1. Sleep to t_start + (shot+1)*interval_s
//  2. Write shutter_event with timer prediction
//  3. Poll GetCameraFilesCount until it increments (~200ms SD write)
//  4. Call GetCameraFilesList, filter to /DCIM/Camera03/*.insp files only
//     (Camera01 contains background .insv video files — ignore those)
//  5. Find the newest Camera03 .insp not yet seen — download via HTTP GET
//  6. Refine shutter_event timestamp after successful download
//
// After StopTimeLapse: any missed shots recovered via OriginUrls() HTTP GET
// (the last shot is always accessible, others may not be — that's OK since
// in-session downloads handle the earlier shots).
//
// Protocol (INSTA360_SESSION_DIR):
//   .sdk_ready / .sdk_capture_trigger / .sdk_capture_done / .sdk_capture_failed
//   .sdk_downloads_pending / .session_done

static ins_camera::DeviceDiscovery g_discovery;
static std::vector<ins_camera::DeviceDescriptor> g_list;

struct DownloadJob {
    std::string remote_path;  // e.g. /DCIM/Camera03/IMG_xxx.insp
    std::string local_path;
    std::string session_dir;
    int         index;
    double      host_t;
};

static std::mutex              g_dl_mutex;
static std::queue<DownloadJob> g_dl_queue;
static std::atomic<int>        g_dl_pending{0};
static std::atomic<bool>       g_dl_stop{false};
static std::string             g_http_base;
// Serialises all USB/HTTP operations so GetCameraFilesList and http_download
// never run concurrently. Contention caused GetCameraFilesList to block for
// 21s while a download was in progress, starving the timer thread and making
// it appear the firmware had stopped shooting.
static std::mutex              g_usb_mutex;

static double now_sec() {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1e6;
}

static void write_shutter_event(const std::string& session_dir, int index, double t) {
    // Write into the individual scan folder (1-based) so files stay organised.
    std::string scan_dir = session_dir + "/fusion_scan_" +
        [](int n){ char buf[8]; std::snprintf(buf, sizeof(buf), "%03d", n); return std::string(buf); }(index + 1);
    fs::create_directories(scan_dir);
    std::ofstream se(scan_dir + "/capture_" + std::to_string(index) + ".shutter_event");
    se << std::fixed << std::setprecision(6) << t;
}

static bool http_download(const std::string& remote_path, const std::string& local_path) {
    std::string url = g_http_base;
    if (!url.empty() && url.back() == '/' && !remote_path.empty() && remote_path.front() == '/')
        url += remote_path.substr(1);
    else
        url += remote_path;
    std::string cmd = "curl -sf --max-time 60 -o '" + local_path + "' '" + url + "'";
    int ret = std::system(cmd.c_str());
    if (ret != 0) { std::remove(local_path.c_str()); return false; }
    struct stat st;
    if (stat(local_path.c_str(), &st) != 0 || st.st_size < 100000) {
        std::remove(local_path.c_str()); return false;
    }
    return true;
}

static void download_worker() {
    while (!g_dl_stop.load()) {
        DownloadJob job;
        {
            std::unique_lock<std::mutex> lk(g_dl_mutex);
            if (g_dl_queue.empty()) { lk.unlock(); std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
            job = g_dl_queue.front(); g_dl_queue.pop();
        }
        std::cout << "[dl] " << fs::path(job.remote_path).filename().string() << " -> " << job.local_path << std::endl;
        bool ok;
        {
            std::unique_lock<std::mutex> usb_lk(g_usb_mutex);
            ok = http_download(job.remote_path, job.local_path);
        }
        if (!ok) {
            std::cerr << "[dl] failed: " << job.remote_path << std::endl;
        } else {
            std::cout << "[dl] saved: " << job.local_path << std::endl;
            std::ofstream ct(job.local_path + ".capture_time");
            ct << std::fixed << std::setprecision(6) << job.host_t;
            write_shutter_event(job.session_dir, job.index, job.host_t);
        }
        g_dl_pending.fetch_sub(1);
    }
}

static void queue_download(const std::string& remote_path, const std::string& session_dir,
                            int index, double host_t) {
    std::string shot_dir  = session_dir + "/.sdk_shot_" + std::to_string(index);
    fs::create_directories(shot_dir);
    std::string local_path = shot_dir + "/" + fs::path(remote_path).filename().string();
    std::unique_lock<std::mutex> lk(g_dl_mutex);
    g_dl_queue.push({remote_path, local_path, session_dir, index, host_t});
    g_dl_pending.fetch_add(1);
}

// Return only .insp files from Camera03 (our timelapse photos).
// Filters out background .insv files from Camera01 and other directories.
static std::vector<std::string> filter_photo_files(const std::vector<std::string>& all_files) {
    std::vector<std::string> result;
    for (const auto& f : all_files) {
        std::string ext = fs::path(f).extension().string();
        bool is_insp = (ext == ".insp");
        bool is_cam03 = (f.find("Camera03") != std::string::npos ||
                         f.find("camera03") != std::string::npos);
        if (is_insp && is_cam03)
            result.push_back(f);
    }
    return result;
}

static bool open_session(ins_camera::Camera* cam) {
    auto f = std::async(std::launch::async, [cam]() { return cam->Open(); });
    if (f.wait_for(std::chrono::seconds(8)) != std::future_status::ready) { std::cerr << "Open() timed out" << std::endl; return false; }
    return f.get();
}

static bool apply_camera_settings(ins_camera::Camera* cam) {
    auto exp = std::make_shared<ins_camera::ExposureSettings>();
    exp->SetExposureMode(ins_camera::PhotographyOptions_ExposureOptions_Program_SHUTTER_PRIORITY);
    exp->SetShutterSpeed(1.0 / 100.0);
    exp->SetEVBias(0);
    if (!cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, exp)) return false;
    auto cap = std::make_shared<ins_camera::CaptureSettings>();
    cap->SetWhiteBalance(ins_camera::PhotographyOptions_WhiteBalance_WB_4000K);
    if (!cam->SetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_IMAGE, cap)) return false;
    std::cout << "Camera settings applied: shutter=1/100s, WB=4000K" << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::string session_dir = "./output";
    if (auto* v = std::getenv("INSTA360_SESSION_DIR")) session_dir = v;
    if (session_dir.empty()) session_dir = "./output";
    if (session_dir[0] != '/') session_dir = fs::current_path().string() + "/" + session_dir;
    fs::create_directories(session_dir);

    g_list = g_discovery.GetAvailableDevices();
    if (g_list.empty()) { std::cerr << "No camera found" << std::endl; _exit(1); }
    std::cout << "Found camera: " << g_list[0].serial_number << " type=" << int(g_list[0].camera_type) << std::endl;

    auto* cam = new ins_camera::Camera(g_list[0].info);
    cam->SetTimeout(20000);

    bool opened = false;
    for (int attempt = 1; attempt <= 5 && !opened; ++attempt) {
        if (attempt > 1) {
            int wait = (attempt - 1) * 3;
            std::cerr << "Retrying Open() in " << wait << "s (attempt " << attempt << "/5)..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(wait));
        }
        if (open_session(cam)) { opened = true; }
        else { delete cam; cam = new ins_camera::Camera(g_list[0].info); cam->SetTimeout(20000); }
    }
    if (!opened) { std::cerr << "Failed to open camera" << std::endl; _exit(1); }
    std::cout << "Camera session open" << std::endl;

    if (!apply_camera_settings(cam))
        std::cerr << "Failed to apply settings — continuing with defaults" << std::endl;

    g_http_base = cam->GetHttpBaseUrl();
    std::cout << "HTTP base: " << g_http_base << std::endl;

    std::thread dl_thread(download_worker);
    { std::ofstream f(session_dir + "/.sdk_ready"); }

    const std::string trigger_path = session_dir + "/.sdk_capture_trigger";
    const std::string done_path    = session_dir + "/.sdk_capture_done";
    const std::string failed_path  = session_dir + "/.sdk_capture_failed";
    const std::string quit_path    = session_dir + "/.session_done";
    const std::string pending_path = session_dir + "/.sdk_downloads_pending";

    bool   timelapse_active = false;
    int    capture_index    = 0;
    int    interval_s       = 5;
    double t_start          = 0.0;

    std::atomic<bool>   timer_stop{false};
    std::mutex          timer_mutex;
    std::vector<double> timer_timestamps;
    std::thread         timer_thread;

    while (true) {
        { std::ofstream pf(pending_path); pf << g_dl_pending.load(); }

        if (fs::exists(quit_path)) {
            std::cout << "Session done — stopping timelapse..." << std::endl;
            if (timelapse_active) {
                timer_stop.store(true);
                if (timer_thread.joinable()) timer_thread.join();

                // Inter-shot window wait
                {
                    std::unique_lock<std::mutex> lk(timer_mutex);
                    if (!timer_timestamps.empty()) {
                        double t_wait = (timer_timestamps.back() + 2.5) - now_sec();
                        if (t_wait > 0 && t_wait < (double)interval_s) {
                            lk.unlock();
                            std::cout << "Waiting " << std::fixed << std::setprecision(1) << t_wait << "s for SD flush..." << std::endl;
                            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(t_wait * 1e6)));
                        }
                    }
                }

                cam->SetTimeout(30000);
                auto result = cam->StopTimeLapse(ins_camera::CameraTimelapseMode::TIMELAPSE_INTERVAL_SHOOTING);
                cam->SetTimeout(20000);
                timelapse_active = false;

                std::vector<double> timestamps;
                int n_shots;
                { std::unique_lock<std::mutex> lk(timer_mutex); timestamps = timer_timestamps; n_shots = (int)timestamps.size(); }
                std::cout << "Timer counted " << n_shots << " shots, OriginUrls has " << result.OriginUrls().size() << std::endl;

                // Recover any shots not already downloaded in-session via OriginUrls HTTP GET.
                // Only the last shot is accessible post-stop; earlier shots were handled in-session.
                if (!result.OriginUrls().empty()) {
                    std::string last_url = result.OriginUrls().back();
                    // Check if already downloaded
                    std::string fname = fs::path(last_url).filename().string();
                    bool already = false;
                    for (int i = 0; i < n_shots; ++i) {
                        std::string shot_dir = session_dir + "/.sdk_shot_" + std::to_string(capture_index + i);
                        if (fs::exists(shot_dir)) {
                            for (auto& e : fs::directory_iterator(shot_dir)) {
                                if (e.path().filename().string() == fname && fs::file_size(e.path()) > 100000) { already = true; break; }
                            }
                        }
                        if (already) break;
                    }
                    if (!already) {
                        // Last shot not downloaded yet — get it now (HTTP session still open)
                        int last_idx = capture_index + n_shots - 1;
                        double host_t = n_shots > 0 ? timestamps.back() : t_start + interval_s;
                        write_shutter_event(session_dir, last_idx, host_t);
                        queue_download(last_url, session_dir, last_idx, host_t);
                        std::cout << "Recovery: last shot " << last_idx << " queued" << std::endl;
                    }
                }
                capture_index += n_shots;
            }
            while (g_dl_pending.load() > 0) {
                { std::ofstream pf(pending_path); pf << g_dl_pending.load(); }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            { std::ofstream pf(pending_path); pf << 0; }
            break;
        }

        if (!fs::exists(trigger_path)) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }

        std::string trigger_content;
        { std::ifstream f(trigger_path); if (f) std::getline(f, trigger_content); }
        fs::remove(trigger_path);

        if (trigger_content.empty()) { std::ofstream ff(failed_path); ff << "fail"; continue; }
        if (trigger_content[0] != '/') trigger_content = fs::current_path().string() + "/" + trigger_content;

        if (trigger_content == session_dir) {
            // --- Continuous mode: start timelapse ---
            interval_s = 5;
            if (auto* v = std::getenv("INSTA360_INTERVAL_MS"))
                interval_s = std::max(3, std::atoi(v) / 1000);

            // Snapshot Camera03 .insp files before session starts
            std::set<std::string> pre_session_fnames;
            {
                auto files = cam->GetCameraFilesList();
                for (const auto& f : filter_photo_files(files))
                    pre_session_fnames.insert(fs::path(f).filename().string());
            }
            std::cout << "Pre-session Camera03 .insp count: " << pre_session_fnames.size() << std::endl;

            int base_count = 0;
            cam->GetCameraFilesCount(base_count);

            ins_camera::TimelapseParam params{};
            params.mode      = ins_camera::CameraTimelapseMode::TIMELAPSE_INTERVAL_SHOOTING;
            params.duration  = 0;
            params.lapseTime = interval_s * 1000;
            if (!cam->SetTimeLapseOption(params)) { std::ofstream ff(failed_path); ff << "fail"; continue; }

            double t_before = now_sec();
            if (!cam->StartTimeLapse(ins_camera::CameraTimelapseMode::TIMELAPSE_INTERVAL_SHOOTING)) {
                std::ofstream ff(failed_path); ff << "fail"; continue;
            }
            t_start = t_before;
            timelapse_active = true;
            timer_stop.store(false);
            { std::unique_lock<std::mutex> lk(timer_mutex); timer_timestamps.clear(); }
            capture_index = 0;

            timer_thread = std::thread([&, base_count, pre_session_fnames]() mutable {
                int shot = 0;
                int last_count = base_count;

                while (!timer_stop.load()) {
                    // Poll file count every 200ms — no fixed sleep based on interval.
                    // The firmware fires at its own rate; we just detect each new file.
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    if (timer_stop.load()) break;

                    int new_count = last_count;
                    cam->GetCameraFilesCount(new_count);
                    if (new_count <= last_count) continue;

                    // Record timestamp immediately when count increments —
                    // BEFORE GetCameraFilesList which takes ~1.2s to return.
                    // Actual shutter = t_count_increment - SD_write_latency (~0.2s).
                    double t_count = now_sec();
                    double host_t  = t_count - 0.20;

                    // New file detected — get the Camera03 .insp path.
                    // Hold g_usb_mutex so this doesn't race with http_download
                    // in the download worker — concurrent USB/HTTP calls caused
                    // GetCameraFilesList to block for 21s, starving the timer.
                    std::vector<std::string> all_files;
                    {
                        std::unique_lock<std::mutex> usb_lk(g_usb_mutex);
                        all_files = cam->GetCameraFilesList();
                    }
                    auto photo_files = filter_photo_files(all_files);
                    std::string new_remote;
                    for (auto it = photo_files.rbegin(); it != photo_files.rend(); ++it) {
                        std::string fname = fs::path(*it).filename().string();
                        if (pre_session_fnames.find(fname) == pre_session_fnames.end()) {
                            new_remote = *it;
                            pre_session_fnames.insert(fname);
                            break;
                        }
                    }
                    if (new_remote.empty()) continue;  // not a Camera03 .insp

                    { std::unique_lock<std::mutex> lk(timer_mutex); timer_timestamps.push_back(host_t); }
                    write_shutter_event(session_dir, shot, host_t);

                    double t_det = now_sec();  // after GetCameraFilesList — for logging only
                    std::cout << "[timer] shot " << shot << " t=" << std::fixed << std::setprecision(3)
                              << host_t << " count_t=" << t_count << " gfls_dt=" << (t_det - t_count)
                              << " url=" << fs::path(new_remote).filename().string() << std::endl;

                    queue_download(new_remote, session_dir, shot, host_t);
                    last_count = new_count;
                    ++shot;
                }
            });

            std::cout << "Timelapse started (interval=" << interval_s << "s, t_start="
                      << std::fixed << std::setprecision(3) << t_start << ")" << std::endl;
            { std::ofstream df(done_path); df << "ok"; }

        } else {
            // --- Stationary mode: single shot ---
            fs::create_directories(trigger_content);
            if (!cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE))
                std::cerr << "SetPhotoSubMode failed (continuing)" << std::endl;
            std::cout << "Taking photo " << capture_index << "..." << std::endl;
            auto url = cam->TakePhoto();
            if (url.Empty() || !url.IsSingleOrigin()) { std::ofstream ff(failed_path); ff << "fail"; continue; }
            double host_t = now_sec();
            write_shutter_event(session_dir, capture_index, host_t);
            std::string remote_path = url.GetSingleOrigin();
            queue_download(remote_path, session_dir, capture_index, host_t);
            std::cout << "Shot " << capture_index << " queued" << std::endl;
            { std::ofstream df(done_path); df << "ok"; }
            ++capture_index;
        }
    }

    g_dl_stop.store(true);
    timer_stop.store(true);
    if (timer_thread.joinable()) timer_thread.join();
    dl_thread.join();
    cam->Close();
    delete cam;
    return 0;
}
