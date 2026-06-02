#include <iostream>
#include <string>
#include <cstdlib>
#include <stitcher/stitcher.h>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: insta360_stitch <input.insp> <output.jpg>" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];

    int erp_w = 3840, erp_h = 1920;
    if (auto* v = std::getenv("INSTA360_ERP_WIDTH")) erp_w = std::atoi(v);
    if (auto* v = std::getenv("INSTA360_ERP_HEIGHT")) erp_h = std::atoi(v);

    ins_media::ImageStitcher stitcher;
    std::vector<std::string> inputs = {input_path};
    stitcher.SetInputPath(inputs);
    stitcher.SetOutputPath(output_path);
    stitcher.SetOutputSize(erp_w, erp_h);
    stitcher.SetStitchType(STITCH_TYPE::TEMPLATE);
    stitcher.EnableFlowState(false);
    stitcher.EnableCuda(false);

    std::cout << "Stitching " << input_path << " -> " << output_path
              << " (" << erp_w << "x" << erp_h << ")" << std::endl;

    if (!stitcher.Stitch()) {
        std::cerr << "Stitching failed" << std::endl;
        return 1;
    }
    std::cout << "Done" << std::endl;
    return 0;
}
