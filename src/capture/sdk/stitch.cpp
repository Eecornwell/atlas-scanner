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
    std::string model_dir;
    for (int i = 3; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--single")    single_fisheye = true;
        else if (a == "--ai")   use_ai = true;
        else if (a == "--model-dir" && i + 1 < argc) model_dir = argv[++i];
    }
    // Allow env var overrides
    if (!use_ai && std::getenv("INSTA360_AI_STITCH"))
        use_ai = std::string(std::getenv("INSTA360_AI_STITCH")) == "1";
    if (model_dir.empty() && std::getenv("INSTA360_MODEL_DIR"))
        model_dir = std::getenv("INSTA360_MODEL_DIR");

    int erp_w = 5760, erp_h = 2880;
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

    if (use_ai) {
        if (model_dir.empty()) {
            std::cerr << "--ai requires --model-dir or INSTA360_MODEL_DIR to be set" << std::endl;
            return 1;
        }
        stitcher.SetStitchType(ins::STITCH_TYPE::AIFLOW);
        std::cout << "Stitch mode: AI (AIFLOW)" << std::endl;
    } else {
        stitcher.SetStitchType(ins::STITCH_TYPE::DYNAMICSTITCH);
        std::cout << "Stitch mode: DYNAMICSTITCH" << std::endl;
    }
    stitcher.EnableFlowState(false);
    stitcher.EnableCuda(false);
    stitcher.SetImageProcessingAccelType(ins::ImageProcessingAccel::kCPU);

    std::cout << "Stitching " << sanitise(input_path) << " -> " << sanitise(output_path)
              << " (" << erp_w << "x" << erp_h << ")" << std::endl;

    if (!stitcher.Stitch()) {
        std::cerr << "Stitching failed" << std::endl;
        return 1;
    }

    if (single_fisheye) {
        // Single fisheye: the full ERP is output as-is (rear hemisphere is blank).
        // The mask is applied downstream by regenerate_masked_images.py to zero
        // out the blank region + scanner body before coloring.
        std::cout << "Single fisheye mode: full ERP with blank rear hemisphere" << std::endl;
    }

    std::cout << "Done" << std::endl;
    return 0;
}
