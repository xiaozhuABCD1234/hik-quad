#include <opencv2/imgproc.hpp>

#include "hikquad/lut.hpp"
#include "hikquad/stitcher.hpp"
#include "test_framework.hpp"

namespace {

hikquad::Intrinsics makeIntrinsics() {
    hikquad::Intrinsics intrinsics;
    intrinsics.model = hikquad::CameraModel::Fisheye;
    intrinsics.K = (cv::Mat_<double>(3, 3) << 300.0, 0.0, 720.0, 0.0, 300.0, 540.0, 0.0, 0.0, 1.0);
    intrinsics.D = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 0.0);
    intrinsics.width = 1440;
    intrinsics.height = 1080;
    intrinsics.circle_center = cv::Point2d(720.0, 540.0);
    intrinsics.circle_radius = 520.0;
    intrinsics.fold_radius = 520.0 / 300.0;
    return intrinsics;
}

}  // namespace

HIK_TEST(lut_has_valid_pixels) {
    hikquad::Extrinsics extrinsics;
    extrinsics.R_cw = cv::Mat::eye(3, 3, CV_64F);
    extrinsics.T_cw = cv::Mat::zeros(3, 1, CV_64F);
    const hikquad::Lut lut =
        hikquad::buildCylindricalLut(makeIntrinsics(), extrinsics, 512, 256, 120.0, 35.0);
    CHECK_EQ(lut.width, 512);
    CHECK_EQ(lut.height, 256);
    CHECK_TRUE(cv::countNonZero(lut.weight > 0.0F) > 0);
    CHECK_TRUE(cv::countNonZero(lut.mapx >= 0.0F) > 0);
    CHECK_NEAR(lut.mapx.at<float>(128, 256), 720.0, 8.0);
}

HIK_TEST(stitch_two_identity_cameras) {
    cv::Mat left(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat right(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));
    std::vector<cv::Mat> images = {left, right};
    std::vector<cv::Mat> mapx;
    std::vector<cv::Mat> mapy;
    std::vector<cv::Mat> weights;
    for (int i = 0; i < 2; ++i) {
        mapx.push_back(cv::Mat(64, 64, CV_32F, cv::Scalar(320.0F)));
        mapy.push_back(cv::Mat(64, 64, CV_32F, cv::Scalar(240.0F)));
        weights.push_back(cv::Mat(64, 64, CV_32F, cv::Scalar(0.5F)));
    }
    const cv::Mat panorama = hikquad::stitchWithLuts(images, mapx, mapy, weights, 64, 64);
    CHECK_EQ(panorama.rows, 64);
    CHECK_EQ(panorama.cols, 64);
    const cv::Vec3b pixel = panorama.at<cv::Vec3b>(32, 32);
    CHECK_NEAR(static_cast<int>(pixel[0]), 128, 1);
    CHECK_NEAR(static_cast<int>(pixel[2]), 128, 1);
}
