#include <iostream>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <stitcher/stitcher.h>

static std::string sanitise(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
        [](unsigned char c) { return c < 0x20 || c == 0x7f; }), s.end());
    return s;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: insta360_stitch <input.insp> <output.jpg> [--single]" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];
    bool single_fisheye = false;
    for (int i = 3; i < argc; ++i)
        if (std::string(argv[i]) == "--single") single_fisheye = true;

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

    ins_media::ImageStitcher stitcher;
    std::vector<std::string> inputs = {input_path};
    stitcher.SetInputPath(inputs);
    stitcher.SetOutputPath(output_path);
    stitcher.SetOutputSize(erp_w, erp_h);
    stitcher.SetStitchType(STITCH_TYPE::TEMPLATE);
    stitcher.EnableFlowState(false);
    stitcher.EnableCuda(false);

    std::cout << "Stitching " << sanitise(input_path) << " -> " << sanitise(output_path)
              << " (" << erp_w << "x" << erp_h << ")" << std::endl;

    if (!stitcher.Stitch()) {
        std::cerr << "Stitching failed" << std::endl;
        return 1;
    }

    if (single_fisheye) {
        // Crop the back-hemisphere half (right side of ERP = 180°–360° longitude).
        // This is the LiDAR-facing lens used in single_fisheye mode.
        // Uses OpenCV via libjpeg round-trip since MediaSDK writes the full ERP to disk.
#if __has_include(<opencv2/opencv.hpp>)
        cv::Mat erp = cv::imread(output_path);
        if (erp.empty()) { std::cerr << "Crop: failed to read stitched ERP" << std::endl; return 1; }
        cv::Mat crop = erp(cv::Rect(erp.cols / 2, 0, erp.cols / 2, erp.rows));
        cv::imwrite(output_path, crop, {cv::IMWRITE_JPEG_QUALITY, 95});
        std::cout << "Cropped to single fisheye (" << crop.cols << "x" << crop.rows << ")" << std::endl;
#else
        std::cerr << "--single requires OpenCV (not available at build time)" << std::endl;
        return 1;
#endif
    }

    std::cout << "Done" << std::endl;
    return 0;
}
