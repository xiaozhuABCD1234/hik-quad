#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "hikquad/lut.hpp"
#include "hikquad/stitcher.hpp"
#include "test_framework.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

hikquad::Intrinsics makeIntrinsics() {
    hikquad::Intrinsics intrinsics;
    intrinsics.model = hikquad::CameraModel::Fisheye;
    intrinsics.K = (cv::Mat_<double>(3, 3) << 225.0, 0.0, 360.0, 0.0, 225.0, 270.0, 0.0, 0.0, 1.0);
    intrinsics.D = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 0.0);
    intrinsics.width = 720;
    intrinsics.height = 540;
    intrinsics.circle_center = cv::Point2d(360.0, 270.0);
    intrinsics.circle_radius = 270.0;
    intrinsics.fold_radius = 270.0 / 225.0;
    return intrinsics;
}

cv::Mat makeTexture(int width, int height) {
    const double focal = static_cast<double>(width) / (2.0 * kPi);
    cv::Mat texture(height, width, CV_8UC3);
    const cv::Scalar colors[] = {{40, 40, 220},  {40, 200, 40},  {220, 80, 40},  {40, 220, 220},
                                 {220, 40, 220}, {220, 220, 40}, {120, 60, 240}, {60, 240, 120}};
    for (int row = 0; row < height; ++row) {
        const double h = (static_cast<double>(row) + 0.5 - 0.5 * height) / focal;
        const double shade = 0.55 + 0.45 * std::exp(-h * h);
        for (int column = 0; column < width; ++column) {
            const double theta = (static_cast<double>(column) + 0.5 - 0.5 * width) / focal;
            int bar = static_cast<int>(std::floor((theta + kPi) / (kPi / 4.0)));
            bar = ((bar % 8) + 8) % 8;
            cv::Scalar color = colors[bar] * shade;
            texture.at<cv::Vec3b>(row, column) =
                cv::Vec3b(cv::saturate_cast<unsigned char>(color[0]),
                          cv::saturate_cast<unsigned char>(color[1]),
                          cv::saturate_cast<unsigned char>(color[2]));
        }
    }
    cv::putText(texture, "360", cv::Point(40, 90), cv::FONT_HERSHEY_SIMPLEX, 1.5,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    return texture;
}

cv::Mat renderCamera(const cv::Mat& texture, const hikquad::Intrinsics& intrinsics,
                     const hikquad::Extrinsics& extrinsics) {
    const int width = intrinsics.width;
    const int height = intrinsics.height;
    std::vector<cv::Point2f> pixels;
    pixels.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            pixels.emplace_back(static_cast<float>(column), static_cast<float>(row));
        }
    }
    std::vector<cv::Point2f> normalized;
    cv::fisheye::undistortPoints(pixels, normalized, intrinsics.K, intrinsics.D);

    const double focal = static_cast<double>(texture.cols) / (2.0 * kPi);
    cv::Mat mapx(height, width, CV_32F);
    cv::Mat mapy(height, width, CV_32F);
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            const cv::Point2f& point =
                normalized[static_cast<size_t>(row) * static_cast<size_t>(width) +
                           static_cast<size_t>(column)];
            cv::Mat ray = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0);
            ray /= cv::norm(ray);
            const cv::Mat world = extrinsics.R_cw * ray;
            const double x = world.at<double>(0, 0);
            const double y = world.at<double>(1, 0);
            const double z = world.at<double>(2, 0);
            const double theta = std::atan2(x, z);
            const double horizontal = std::hypot(x, z);
            const double h = horizontal > 1e-12 ? y / horizontal : 0.0;
            double u = 0.5 * texture.cols + theta * focal;
            while (u < 0.0) {
                u += texture.cols;
            }
            while (u >= texture.cols) {
                u -= texture.cols;
            }
            const double v = 0.5 * texture.rows + h * focal;
            mapx.at<float>(row, column) = static_cast<float>(u);
            mapy.at<float>(row, column) = static_cast<float>(v);
        }
    }
    cv::Mat image;
    cv::remap(texture, image, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar(0, 0, 0));
    return image;
}

}  // namespace

HIK_TEST(synthetic_four_camera_pipeline) {
    constexpr int out_width = 1024;
    constexpr int out_height = 256;
    const cv::Mat texture = makeTexture(out_width, out_height);
    const hikquad::Intrinsics intrinsics = makeIntrinsics();

    std::vector<hikquad::Extrinsics> extrinsics(4);
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> mapx;
    std::vector<cv::Mat> mapy;
    std::vector<cv::Mat> weights;
    for (int camera = 0; camera < 4; ++camera) {
        extrinsics[static_cast<size_t>(camera)].R_cw =
            hikquad::nominalRotationY(static_cast<double>(camera) * 90.0);
        extrinsics[static_cast<size_t>(camera)].T_cw = cv::Mat::zeros(3, 1, CV_64F);
        extrinsics[static_cast<size_t>(camera)].center_angle_deg =
            static_cast<double>(camera) * 90.0;
        images.push_back(
            renderCamera(texture, intrinsics, extrinsics[static_cast<size_t>(camera)]));
        const hikquad::Lut lut =
            hikquad::buildCylindricalLut(intrinsics, extrinsics[static_cast<size_t>(camera)],
                                         out_width, out_height, 360.0, 35.0);
        mapx.push_back(lut.mapx);
        mapy.push_back(lut.mapy);
        weights.push_back(lut.weight);
    }

    const cv::Mat panorama =
        hikquad::stitchWithLuts(images, mapx, mapy, weights, out_width, out_height);
    CHECK_EQ(panorama.size(), texture.size());
    const double mse =
        cv::norm(panorama, texture, cv::NORM_L2SQR) / (static_cast<double>(panorama.total()) * 3.0);
    const double psnr = 10.0 * std::log10(255.0 * 255.0 / std::max(mse, 1e-12));
    CHECK_TRUE(psnr > 22.0);
    cv::imwrite("/tmp/hik_quad_synthetic_panorama.jpg", panorama);
}
