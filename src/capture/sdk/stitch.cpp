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
        std::cerr << "Usage: insta360_stitch <input.insp> <output.jpg>" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];

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
    std::cout << "Done" << std::endl;
    return 0;
}
