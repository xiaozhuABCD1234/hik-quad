#include "hikquad/stitcher.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace hikquad {

cv::Mat stitchWithLuts(const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& mapx,
                       const std::vector<cv::Mat>& mapy, const std::vector<cv::Mat>& weight,
                       int out_width, int out_height) {
    StitchInput input;
    input.images = images;
    input.mapx = mapx;
    input.mapy = mapy;
    input.weight = weight;
    input.out_width = out_width;
    input.out_height = out_height;
    return stitchFrames(input);
}

cv::Mat stitchFrames(const StitchInput& input) {
    const size_t count = input.images.size();
    if (count == 0) {
        throw std::invalid_argument("stitchFrames: no input images");
    }
    if (input.mapx.size() != count || input.mapy.size() != count || input.weight.size() != count) {
        throw std::invalid_argument("stitchFrames: LUT/weight count does not match image count");
    }
    if (input.out_width <= 0 || input.out_height <= 0) {
        throw std::invalid_argument("stitchFrames: invalid output size");
    }

    cv::Mat accum = cv::Mat::zeros(input.out_height, input.out_width, CV_32FC3);
    cv::Mat weight_sum = cv::Mat::zeros(input.out_height, input.out_width, CV_32F);

    for (size_t i = 0; i < count; ++i) {
        if (input.images[i].empty() || input.mapx[i].empty() || input.mapy[i].empty() ||
            input.weight[i].empty()) {
            continue;
        }
        if (input.mapx[i].size() != cv::Size(input.out_width, input.out_height) ||
            input.mapy[i].size() != input.mapx[i].size() ||
            input.weight[i].size() != input.mapx[i].size()) {
            throw std::invalid_argument("stitchFrames: LUT/weight size mismatch");
        }
        cv::Mat warped;
        cv::remap(input.images[i], warped, input.mapx[i], input.mapy[i], cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        if (warped.channels() == 1) {
            cv::cvtColor(warped, warped, cv::COLOR_GRAY2BGR);
        }
        cv::Mat warped_float;
        warped.convertTo(warped_float, CV_32FC3);
        cv::Mat weight3;
        cv::Mat channels[] = {input.weight[i], input.weight[i], input.weight[i]};
        cv::merge(channels, 3, weight3);
        cv::Mat weighted;
        cv::multiply(warped_float, weight3, weighted);
        accum += weighted;
        weight_sum += input.weight[i];
    }

    cv::Mat panorama = cv::Mat::zeros(input.out_height, input.out_width, CV_8UC3);
    for (int row = 0; row < input.out_height; ++row) {
        const float* sum_row = weight_sum.ptr<float>(row);
        const cv::Vec3f* accum_row = accum.ptr<cv::Vec3f>(row);
        cv::Vec3b* out_row = panorama.ptr<cv::Vec3b>(row);
        for (int column = 0; column < input.out_width; ++column) {
            const float sum = sum_row[column];
            if (sum <= 1e-8F) {
                continue;
            }
            const cv::Vec3f value = accum_row[column] / sum;
            out_row[column] = cv::Vec3b(cv::saturate_cast<unsigned char>(value[0]),
                                        cv::saturate_cast<unsigned char>(value[1]),
                                        cv::saturate_cast<unsigned char>(value[2]));
        }
    }
    return panorama;
}

}  // namespace hikquad
