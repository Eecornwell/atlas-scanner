#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <unistd.h>
#include <filesystem>
#include <ins_stitcher.h>
#include <ins_common.h>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

static std::string sanitise(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
        [](unsigned char c) { return c < 0x20 || c == 0x7f; }), s.end());
    return s;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: insta360_stitch <input.insp> <output.jpg> [--single] [--ai] [--model-dir /path/to/models]" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];
    bool single_fisheye = false;
    bool use_ai = false;
    bool denoise = false;
    std::string model_dir;
    for (int i = 3; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--single")         single_fisheye = true;
        else if (a == "--ai")        use_ai = true;
        else if (a == "--denoise")   denoise = true;
        else if (a == "--model-dir" && i + 1 < argc) model_dir = argv[++i];
    }
    // Allow env var overrides
    if (!use_ai && std::getenv("INSTA360_AI_STITCH"))
        use_ai = std::string(std::getenv("INSTA360_AI_STITCH")) == "1";
    if (model_dir.empty() && std::getenv("INSTA360_MODEL_DIR"))
        model_dir = std::getenv("INSTA360_MODEL_DIR");

    int erp_w = 11520, erp_h = 5760;
    if (auto* v = std::getenv("INSTA360_ERP_WIDTH")) {
        try {
            int val = std::stoi(v);
            if (val > 0) erp_w = val;
            else { std::cerr << "Invalid INSTA360_ERP_WIDTH: must be > 0" << std::endl; return 1; }
        } catch (const std::exception&) {
            std::cerr << "Invalid INSTA360_ERP_WIDTH: not a valid integer" << std::endl; return 1;
        }
    }
    if (auto* v = std::getenv("INSTA360_ERP_HEIGHT")) {
        try {
            int val = std::stoi(v);
            if (val > 0) erp_h = val;
            else { std::cerr << "Invalid INSTA360_ERP_HEIGHT: must be > 0" << std::endl; return 1; }
        } catch (const std::exception&) {
            std::cerr << "Invalid INSTA360_ERP_HEIGHT: not a valid integer" << std::endl; return 1;
        }
    }

    // Set model dir before stitcher is used (required for AIFLOW)
    if (!model_dir.empty()) {
        ins::SetModelFileRootDir(model_dir);
        std::cout << "Model dir: " << sanitise(model_dir) << std::endl;
    }

    ins::ImageStitcher stitcher;
    std::vector<std::string> inputs = {input_path};
    stitcher.SetInputPath(inputs);
    stitcher.SetOutputPath(output_path);
    stitcher.SetOutputSize(erp_w, erp_h);

    // Stitch type: INSTA360_STITCH_TYPE env var selects the algorithm.
    //   0 = TEMPLATE      — pure geometric projection, no optical flow, no seam blending.
    //                       Fastest; preserves raw lens geometry but shows hard seam ghosting.
    //   1 = OPTFLOW       — DIS (Dense Inverse Search) static optical flow at the seam. Best quality.
    //   2 = DYNAMICSTITCH — dynamic optical flow (recomputed per-frame). Similar speed to OPTFLOW.
    //   3 = AIFLOW        — AI-based (requires --ai and --model-dir).
    // Default is OPTFLOW (1): best seam quality at the same cost as DYNAMICSTITCH.
    int stitch_type_int = 1;
    if (auto* v = std::getenv("INSTA360_STITCH_TYPE"))
        stitch_type_int = std::atoi(v);
    if (use_ai && !single_fisheye) {
        if (model_dir.empty()) {
            std::cerr << "--ai requires --model-dir or INSTA360_MODEL_DIR to be set" << std::endl;
            return 1;
        }
        stitcher.SetStitchType(ins::STITCH_TYPE::AIFLOW);
        std::cout << "Stitch mode: AIFLOW" << std::endl;
    } else {
        ins::STITCH_TYPE stype;
        const char* sname;
        switch (stitch_type_int) {
            case 1:  stype = ins::STITCH_TYPE::OPTFLOW;       sname = "OPTFLOW";       break;
            case 2:  stype = ins::STITCH_TYPE::DYNAMICSTITCH; sname = "DYNAMICSTITCH"; break;
            default: stype = ins::STITCH_TYPE::TEMPLATE;      sname = "TEMPLATE";      break;
        }
        stitcher.SetStitchType(stype);
        if (single_fisheye)
            stitcher.EnableStitchFusion(false);
        std::cout << "Stitch mode: " << sname
                  << (single_fisheye ? " (single fisheye — fusion disabled)" : "") << std::endl;
    }
    // FlowState = gyro-based automatic horizon leveling (EIS/stabilization).
    // Always disabled — the LiDAR pose drives orientation, not the camera IMU.
    stitcher.EnableFlowState(false);
    stitcher.EnableCuda(false);
    stitcher.SetImageProcessingAccelType(ins::ImageProcessingAccel::kCPU);

    // Pre-read input file bytes for fallback use — must happen before Stitch()
    // since the SDK may modify internal state that interferes with cv::imdecode.
    std::vector<uchar> input_buf;
    {
        std::ifstream ifs(input_path, std::ios::binary);
        input_buf.assign((std::istreambuf_iterator<char>(ifs)),
                          std::istreambuf_iterator<char>());
    }

    std::cout << "Stitching " << sanitise(input_path) << " -> " << sanitise(output_path)
              << " (" << erp_w << "x" << erp_h << ")" << std::endl;

    if (!stitcher.Stitch()) {
        // SDK stitch failed — attempt OpenCV fisheye-to-ERP fallback.
        // This handles HDR single-fisheye .insp files (square aspect ratio)
        // which the SDK rejects with ErrorCode:11.
        // cv::imread uses file extension to select codec; .insp is not
        // recognised so use imdecode with raw bytes instead.
        std::ifstream ifs(input_path, std::ios::binary);
        std::vector<uchar> buf((std::istreambuf_iterator<char>(ifs)),
                                std::istreambuf_iterator<char>());
        cv::Mat fisheye = cv::imdecode(input_buf, cv::IMREAD_COLOR);
        if (fisheye.empty()) {
            std::cerr << "Stitching failed (fallback: could not decode image, buf=" << input_buf.size() << ")" << std::endl;
            return 1;
        }
        int fw = fisheye.cols, fh = fisheye.rows;
        std::cout << "Fallback: decoded " << fw << "x" << fh << " from " << input_buf.size() << " bytes" << std::endl;
        // Only attempt fallback for square (single-fisheye) images
        if (std::abs(fw - fh) > fw / 10) {
            std::cerr << "Stitching failed (fallback: not square: " << fw << "x" << fh << ")" << std::endl;
            return 1;
        }
        std::cout << "SDK stitch failed — using OpenCV fisheye-to-ERP fallback ("
                  << fw << "x" << fh << ")" << std::endl;
        // Equidistant fisheye projection: r = f * theta
        double cx = fw / 2.0, cy = fh / 2.0;
        double f  = fw / M_PI;  // FOV ~180deg: at edge r=fw/2, theta=pi/2 -> f=fw/pi
        cv::Mat map_x(erp_h, erp_w, CV_32F);
        cv::Mat map_y(erp_h, erp_w, CV_32F);
        for (int ey = 0; ey < erp_h; ++ey) {
            double lat = (0.5 - (ey + 0.5) / erp_h) * M_PI;
            for (int ex = 0; ex < erp_w; ++ex) {
                double lon = ((ex + 0.5) / erp_w - 0.5) * 2.0 * M_PI;
                double X = std::cos(lat) * std::sin(lon);
                double Y = std::sin(lat);
                double Z = std::cos(lat) * std::cos(lon);
                double theta = std::acos(std::max(-1.0, std::min(1.0, Z)));
                double phi   = std::atan2(Y, X);
                double r     = f * theta;
                float sx = static_cast<float>(cx + r * std::cos(phi));
                float sy = static_cast<float>(cy + r * std::sin(phi));
                // Only map front hemisphere (theta < pi/2)
                if (theta < M_PI / 2.0 && sx >= 0 && sx < fw && sy >= 0 && sy < fh) {
                    map_x.at<float>(ey, ex) = sx;
                    map_y.at<float>(ey, ex) = sy;
                } else {
                    map_x.at<float>(ey, ex) = -1;
                    map_y.at<float>(ey, ex) = -1;
                }
            }
        }
        cv::Mat erp;
        cv::remap(fisheye, erp, map_x, map_y, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
        cv::imwrite(output_path, erp, params);
        std::cout << "Done (fisheye fallback)" << std::endl;
        return 0;
    }

    // The SDK's internal JPEG encoder uses ~q60 which causes visible blocking.
    // Re-encode the output at q95 to eliminate compression artifacts.
    // Only applies when the output path is .jpg/.jpeg.
    std::string ext = output_path.size() >= 4 ? output_path.substr(output_path.rfind('.')) : "";
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".jpg" || ext == ".jpeg") {
        cv::Mat img = cv::imread(output_path, cv::IMREAD_COLOR);
        if (!img.empty()) {
            cv::Mat processed;
            if (denoise) {
                cv::fastNlMeansDenoisingColored(img, processed, 4.0f, 4.0f, 7, 21);
            } else {
                processed = img;
            }
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
            cv::imwrite(output_path, processed, params);
        }
    }

    std::cout << "Done" << std::endl;
    // Use _exit() to skip the SDK's OpenGL offscreen context destructor which
    // takes ~500ms per instance on Intel GPU (no CUDA/Vulkan). With many scans
    // running in parallel this is the dominant source of post-processing latency.
    _exit(0);
}
