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

static std::string             g_http_base;
// Serialises all log writes so multi-statement << chains from different
// threads never interleave on stdout/stderr (CWE-362).
static std::mutex              g_log_mutex;
static std::mutex              g_usb_mutex;     // serialises USB commands
static std::mutex              g_dl_mutex;      // serialises curl downloads (HTTP tunnel, not USB)

// Thread-safe log helpers.
#define LOG_OUT(msg) do { std::unique_lock<std::mutex> _lk(g_log_mutex); std::cout << msg << std::endl; } while(0)
#define LOG_ERR(msg) do { std::unique_lock<std::mutex> _lk(g_log_mutex); std::cerr << msg << std::endl; } while(0)

static double now_sec() {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1e6;
}

// Strip newlines, carriage returns and other ASCII control characters from
// device-supplied strings before writing them to the log so a malicious
// camera response cannot inject fake log lines (CWE-117).
static std::string sanitise(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
        [](unsigned char c) { return c < 0x20 || c == 0x7f; }), s.end());
    return s;
}

static void write_shutter_event(const std::string& session_dir, int index, double t) {
    std::string safe_dir = sanitise(session_dir);
    // Validate scan_dir stays within session_dir to prevent path traversal.
    fs::path base  = fs::weakly_canonical(fs::path(safe_dir));
    char idx_buf[8];
    std::snprintf(idx_buf, sizeof(idx_buf), "%03d", index + 1);
    fs::path scan  = fs::weakly_canonical(base / ("fusion_scan_" + std::string(idx_buf)));
    if (scan.string().rfind(base.string(), 0) != 0) return;  // escapes session root
    fs::create_directories(scan);
    fs::path event = scan / ("capture_" + std::to_string(index) + ".shutter_event");
    if (event.string().rfind(base.string(), 0) != 0) return;  // escapes session root
    std::ofstream se(event);
    se << std::fixed << std::setprecision(6) << t;
}

static bool http_download(const std::string& remote_path, const std::string& local_path) {
    std::string url = g_http_base;
    if (!url.empty() && url.back() == '/' && !remote_path.empty() && remote_path.front() == '/')
        url += remote_path.substr(1);
    else
        url += remote_path;

    // Use fork+execvp instead of std::system so url and local_path are passed
    // as discrete argv entries and never interpreted by a shell (CWE-78/117).
    pid_t pid = fork();
    if (pid < 0) { return false; }
    if (pid == 0) {
        // Child: exec curl directly — no shell, no injection risk.
        const char* argv[] = {
            "curl", "-sf", "--max-time", "60",
            "-o", local_path.c_str(),
            url.c_str(),
            nullptr
        };
        execvp("curl", const_cast<char* const*>(argv));
        _exit(127);  // execvp failed
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

// Return only .insp files from Camera directories (timelapse photos).
// Filters out background .insv video files. Accepts Camera03, Camera04, etc.
static std::vector<std::string> filter_photo_files(const std::vector<std::string>& all_files) {
    std::vector<std::string> result;
    for (const auto& f : all_files) {
        std::string ext = fs::path(f).extension().string();
        if (ext != ".insp") continue;
        // Accept any CameraNN directory (timelapse uses Camera03 or Camera04
        // depending on firmware version)
        std::string fl = f;
        std::transform(fl.begin(), fl.end(), fl.begin(), ::tolower);
        if (fl.find("camera") != std::string::npos)
            result.push_back(f);
    }
    return result;
}

static bool open_session(ins_camera::Camera* cam) {
    auto f = std::async(std::launch::async, [cam]() { return cam->Open(); });
    if (f.wait_for(std::chrono::seconds(8)) != std::future_status::ready) { LOG_ERR("Open() timed out"); return false; }
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
    LOG_OUT("Camera settings applied: shutter=1/100s, WB=4000K");
    return true;
}

int main(int argc, char* argv[]) {
    std::string session_dir = "./output";
    if (auto* v = std::getenv("INSTA360_SESSION_DIR")) session_dir = sanitise(v);
    if (session_dir.empty()) session_dir = "./output";
    if (session_dir[0] != '/') session_dir = fs::current_path().string() + "/" + session_dir;
    fs::create_directories(session_dir);

    g_list = g_discovery.GetAvailableDevices();
    if (g_list.empty()) { LOG_ERR("No camera found"); _exit(1); }
    LOG_OUT("Found camera: " << sanitise(g_list[0].serial_number) << " type=" << int(g_list[0].camera_type));

    auto* cam = new ins_camera::Camera(g_list[0].info);
    cam->SetTimeout(20000);

    bool opened = false;
    for (int attempt = 1; attempt <= 5 && !opened; ++attempt) {
        if (attempt > 1) {
            int wait = (attempt - 1) * 3;
            LOG_ERR("Retrying Open() in " << wait << "s (attempt " << attempt << "/5)...");
            std::this_thread::sleep_for(std::chrono::seconds(wait));
        }
        if (open_session(cam)) { opened = true; }
        else { delete cam; cam = new ins_camera::Camera(g_list[0].info); cam->SetTimeout(20000); }
    }
    if (!opened) { LOG_ERR("Failed to open camera"); _exit(1); }
    LOG_OUT("Camera session open");

    if (!apply_camera_settings(cam))
        LOG_ERR("Failed to apply settings — continuing with defaults");

    g_http_base = cam->GetHttpBaseUrl();
    LOG_OUT("HTTP base: " << sanitise(g_http_base));

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
    // Limit concurrent in-session downloads to prevent USB saturation.
    // With unlimited detached threads, 10+ concurrent GetCameraFilesCount
    // polls starve off the USB bus causing 20s command timeouts on the camera.
    std::mutex              dl_sem_mutex;
    std::condition_variable dl_sem_cv;
    int                     dl_active{0};
    constexpr int           DL_MAX{2};

    while (true) {
        { std::ofstream pf(pending_path); pf << 0; }

        if (fs::exists(quit_path)) {
            LOG_OUT("Session done — stopping timelapse...");
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
                            LOG_OUT("Waiting " << std::fixed << std::setprecision(1) << t_wait << "s for SD flush...");
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
                LOG_OUT("Timer counted " << n_shots << " shots, OriginUrls has " << result.OriginUrls().size());

                // Recover shots using OriginUrls() for the last shot, then
                // reconstruct earlier shot URLs by decrementing the sequence
                // number. Within a session all shots share the same date prefix
                // and the sequence decrements: last=NNN, prev=NNN-1, etc.
                if (!result.OriginUrls().empty()) {
                    std::string last_url  = result.OriginUrls().back();
                    std::string last_path = fs::path(last_url).parent_path().string(); // /DCIM/Camera03
                    std::string last_name = fs::path(last_url).stem().string();        // IMG_YYYYMMDD_HHMMSS_00_NNN
                    std::string last_ext  = fs::path(last_url).extension().string();   // .insp

                    // Parse sequence number from last filename
                    // Format: IMG_YYYYMMDD_HHMMSS_00_NNN
                    int last_seq = -1;
                    auto sep = last_name.rfind('_');
                    if (sep != std::string::npos) {
                        try { last_seq = std::stoi(last_name.substr(sep + 1)); }
                        catch (...) {}
                    }
                    std::string prefix = (sep != std::string::npos) ? last_name.substr(0, sep + 1) : "";

                    int total = std::max(n_shots, 1);
                    LOG_OUT("Recovery: " << total << " shots via URL reconstruction (last_seq=" << last_seq << ")");

                    for (int i = 0; i < total; ++i) {
                        int abs_shot = capture_index + i;
                        // Check if already downloaded in-session
                        bool already = false;
                        std::string sd = session_dir + "/.sdk_shot_" + std::to_string(abs_shot);
                        if (fs::exists(sd)) {
                            for (auto& e : fs::directory_iterator(sd))
                                if (fs::file_size(e.path()) > 100000) { already = true; break; }
                        }
                        if (already) continue;

                        // Construct URL: earlier shots have lower seq numbers
                        // shot i=0 is oldest, i=total-1 is last_seq
                        int seq = -1;
                        std::string url;
                        if (last_seq >= 0 && !prefix.empty()) {
                            seq = (last_seq - (total - 1 - i) + 1000) % 1000;
                            char seq_buf[8];
                            std::snprintf(seq_buf, sizeof(seq_buf), "%03d", seq);
                            std::string fname = prefix + seq_buf + last_ext;
                            url = last_path + "/" + fname;
                        } else {
                            if (i == total - 1) url = last_url; // fallback: only last shot
                            else continue;
                        }

                        double host_t = (i < (int)timestamps.size())
                            ? timestamps[i]
                            : t_start + interval_s * (i + 1);

                        fs::create_directories(sd);
                        std::string local_path = sd + "/" + fs::path(url).filename().string();
                        LOG_OUT("[recovery] shot " << abs_shot << " seq=" << seq << " <- " << sanitise(fs::path(url).filename().string()));
                        bool ok = http_download(url, local_path);
                        if (ok && fs::exists(local_path) && fs::file_size(local_path) > 100000) {
                            std::ofstream ct(local_path + ".capture_time");
                            ct << std::fixed << std::setprecision(6) << host_t;
                            write_shutter_event(session_dir, abs_shot, host_t);
                            LOG_OUT("[recovery] saved: " << sanitise(local_path));
                        } else {
                            LOG_ERR("[recovery] failed seq=" << seq << ": " << sanitise(url));
                            if (fs::exists(local_path)) fs::remove(local_path);
                        }
                    }
                }
                capture_index += std::max(n_shots, !result.OriginUrls().empty() ? 1 : 0);
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

            // Snapshot all existing .insp paths before the timelapse starts so
            // we can identify the new file written for each shot by exclusion.
            std::set<std::string> pre_session_paths;
            {
                std::unique_lock<std::mutex> usb_lk(g_usb_mutex);
                auto seed = cam->GetCameraFilesList();
                for (const auto& f : filter_photo_files(seed))
                    pre_session_paths.insert(f);
            }
            LOG_OUT("Pre-session .insp count: " << pre_session_paths.size());

            int base_count = 0;
            cam->GetCameraFilesCount(base_count);

            ins_camera::TimelapseParam params{};
            params.mode      = ins_camera::CameraTimelapseMode::TIMELAPSE_INTERVAL_SHOOTING;
            params.duration  = 86400;  // 24h ceiling — StopTimeLapse ends it early
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

            timer_thread = std::thread([&, base_count, pre_session_paths]() mutable {
                int shot = 0;
                int last_count = base_count;
                std::set<std::string> seen_paths = pre_session_paths;
                double sd_latency_estimate = 2.0;
                int    sd_latency_samples  = 0;

                while (!timer_stop.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    if (timer_stop.load()) break;

                    int new_count = last_count;
                    cam->GetCameraFilesCount(new_count);
                    if (new_count <= last_count) continue;

                    double t_count = now_sec();
                    // Estimate the true shutter time from the SD-write detection
                    // time minus a rolling estimate of SD-write latency.
                    // This anchors each shot to a real measurement rather than
                    // the purely mathematical t_expected = t_start + N*interval,
                    // which drifts from the firmware's actual firing cadence
                    // (firmware uses its own oscillator, not the host clock) and
                    // accumulates error linearly across shots.
                    double t_expected = t_start + (shot + 1) * (double)interval_s;

                    // Ignore increments before expected shutter time —
                    // background camera writes (video, housekeeping) not shots.
                    if (t_count < t_expected - 0.5) {
                        LOG_OUT("[timer] ignoring early count increment at t=" << std::fixed
                            << std::setprecision(3) << t_count << " (expected shot " << shot
                            << " at " << t_expected << ")");
                        continue;
                    }

                    // Back-calculate true shutter from t_count using the rolling
                    // SD-write latency estimate. For the first two shots, bootstrap
                    // from t_expected (clocks agree within ~10ms at session start).
                    double host_t;
                    if (sd_latency_samples < 2) {
                        // Use t_expected to seed the latency estimate.
                        double measured_latency = t_count - t_expected;
                        if (measured_latency >= 0.0 && measured_latency < 30.0) {
                            sd_latency_estimate = (sd_latency_estimate * sd_latency_samples
                                                   + measured_latency) / (sd_latency_samples + 1);
                            ++sd_latency_samples;
                        }
                        host_t = t_expected;
                    } else {
                        // Use t_count - sd_latency_estimate as the shutter time.
                        // This tracks the firmware's actual firing cadence rather
                        // than the drifting mathematical prediction.
                        host_t = t_count - sd_latency_estimate;
                        // Update latency estimate: assume host_t is correct,
                        // so latency = t_count - host_t.
                        double measured_latency = t_count - host_t;
                        if (measured_latency >= 0.0 && measured_latency < 30.0)
                            sd_latency_estimate = sd_latency_estimate * 0.8 + measured_latency * 0.2;
                    }
                    LOG_OUT("[timer] shot " << shot << " t_count=" << std::fixed << std::setprecision(3)
                            << t_count << " sd_lat=" << std::setprecision(3) << sd_latency_estimate
                            << "s host_t=" << std::setprecision(3) << host_t);

                    // Find the new .insp file for this shot via file list.
                    // Uses g_usb_mutex (USB commands) separately from g_dl_mutex
                    // (curl downloads) so file discovery never blocks on downloads.
                    std::string new_remote;
                    for (int attempt = 0; attempt < 10 && new_remote.empty(); ++attempt) {
                        if (attempt > 0) std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        std::vector<std::string> all_files;
                        {
                            std::unique_lock<std::mutex> usb_lk(g_usb_mutex);
                            all_files = cam->GetCameraFilesList();
                        }
                        for (const auto& f : filter_photo_files(all_files)) {
                            if (seen_paths.find(f) == seen_paths.end()) {
                                new_remote = f;
                                seen_paths.insert(f);
                                break;
                            }
                        }
                    }

                    { std::unique_lock<std::mutex> lk(timer_mutex); timer_timestamps.push_back(host_t); }
                    write_shutter_event(session_dir, shot, host_t);

                    if (!new_remote.empty()) {
                        // Download in a detached thread so the detection loop is never
                        // blocked by curl — subsequent shots' GetCameraFilesCount polls
                        // would deadlock on g_usb_mutex if download held it synchronously.
                        std::string shot_dir   = session_dir + "/.sdk_shot_" + std::to_string(shot);
                        fs::create_directories(shot_dir);
                        std::string local_path = shot_dir + "/" + fs::path(new_remote).filename().string();
                        LOG_OUT("[timer] queuing download shot " << shot << " <- " << sanitise(fs::path(new_remote).filename().string()));
                        std::thread([new_remote, local_path, host_t, shot, &session_dir,
                                     &dl_sem_mutex, &dl_sem_cv, &dl_active]() {
                            {
                                std::unique_lock<std::mutex> lk(dl_sem_mutex);
                                dl_sem_cv.wait(lk, [&]{ return dl_active < DL_MAX; });
                                ++dl_active;
                            }
                            bool ok;
                            {
                                std::unique_lock<std::mutex> dl_lk(g_dl_mutex);
                                ok = http_download(new_remote, local_path);
                            }
                            {
                                std::unique_lock<std::mutex> lk(dl_sem_mutex);
                                --dl_active;
                                dl_sem_cv.notify_all();
                            }
                            if (ok) {
                                std::ofstream ct(local_path + ".capture_time");
                                ct << std::fixed << std::setprecision(6) << host_t;
                                write_shutter_event(session_dir, shot, host_t);
                                LOG_OUT("[timer] saved: " << sanitise(local_path));
                            } else {
                                LOG_ERR("[timer] download failed: " << sanitise(new_remote));
                            }
                        }).detach();
                    } else {
                        LOG_OUT("[timer] shot " << shot << " no new file found — deferred to recovery");
                    }
                    last_count = new_count;
                    ++shot;
                }
            });

            LOG_OUT("Timelapse started (interval=" << interval_s << "s, t_start="
                      << std::fixed << std::setprecision(3) << t_start << ")");
            { std::ofstream df(done_path); df << "ok"; }

        } else {
            // --- Stationary mode: single shot ---
            fs::create_directories(trigger_content);
            if (!cam->SetPhotoSubMode(ins_camera::SubPhotoMode::PHOTO_SINGLE))
                LOG_ERR("SetPhotoSubMode failed (continuing)");
            LOG_OUT("Taking photo " << capture_index << "...");
            auto url = cam->TakePhoto();
            if (url.Empty() || !url.IsSingleOrigin()) { std::ofstream ff(failed_path); ff << "fail"; continue; }
            double host_t = now_sec();
            write_shutter_event(session_dir, capture_index, host_t);
            std::string remote_path = url.GetSingleOrigin();
            std::string shot_dir   = session_dir + "/.sdk_shot_" + std::to_string(capture_index);
            fs::create_directories(shot_dir);
            std::string local_path = shot_dir + "/" + fs::path(remote_path).filename().string();
            LOG_OUT("Taking photo " << capture_index << " — downloading...");
            bool ok;
            {
                std::unique_lock<std::mutex> usb_lk(g_usb_mutex);
                ok = http_download(remote_path, local_path);
            }
            if (ok) {
                std::ofstream ct(local_path + ".capture_time");
                ct << std::fixed << std::setprecision(6) << host_t;
                write_shutter_event(session_dir, capture_index, host_t);
                LOG_OUT("Shot " << capture_index << " saved");
            } else {
                LOG_ERR("Shot " << capture_index << " download failed: " << sanitise(remote_path));
            }
            { std::ofstream df(done_path); df << "ok"; }
            ++capture_index;
        }
    }

    timer_stop.store(true);
    if (timer_thread.joinable()) timer_thread.join();
    cam->Close();
    delete cam;
    return 0;
}
