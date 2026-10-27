#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace hikquad {

struct StitchInput {
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> mapx;
    std::vector<cv::Mat> mapy;
    std::vector<cv::Mat> weight;
    int out_width = 0;
    int out_height = 0;
};

cv::Mat stitchFrames(const StitchInput& input);

}  // namespace hikquad
