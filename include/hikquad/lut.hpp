#pragma once

#include <opencv2/core.hpp>
#include <string>

#include "calibration.hpp"

namespace hikquad {

struct Lut {
    int width = 0;
    int height = 0;
    cv::Mat mapx;
    cv::Mat mapy;
    cv::Mat weight;
};

Lut buildCylindricalLut(const Intrinsics& intrinsics, const Extrinsics& extrinsics, int out_width,
                        int out_height, double fov_h_deg, double blend_sigma_deg);
void writeLutBin(const std::string& path, const Lut& lut, int camera_index);
void writeWeightBin(const std::string& path, const cv::Mat& weight, int camera_index);
Lut readLutBin(const std::string& path, int camera_index = -1);
cv::Mat readWeightBin(const std::string& path, int camera_index = -1);

}  // namespace hikquad
