#include <iostream>
#include <string>
#include <algorithm>
#include <cstdlib>
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

    std::cout << "Stitching " << sanitise(input_path) << " -> " << sanitise(output_path)
              << " (" << erp_w << "x" << erp_h << ")" << std::endl;

    if (!stitcher.Stitch()) {
        std::cerr << "Stitching failed" << std::endl;
        return 1;
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
                // X3 sensor noise reduction: NL-means denoising removes
                // high-frequency sensor noise (std ~3x higher than X5).
                // h=8: filter strength — more aggressive than default h=4.
                // templateWindowSize=7, searchWindowSize=21: standard values.
                cv::fastNlMeansDenoisingColored(img, processed, 8.0f, 8.0f, 7, 21);
            } else {
                processed = img;
            }
            // Mild bilateral filter to smooth residual JPEG DCT block artifacts.
            cv::Mat deblocked;
            cv::bilateralFilter(processed, deblocked, 5, 8.0, 5.0);
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
            cv::imwrite(output_path, deblocked, params);
        }
    }

    std::cout << "Done" << std::endl;
    return 0;
}
