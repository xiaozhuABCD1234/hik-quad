#include "hikquad/lut.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace hikquad {
namespace {

constexpr double kPi = 3.14159265358979323846;

#pragma pack(push, 1)
struct BinHeader {
    char magic[4];
    uint16_t version;
    uint16_t dtype;
    int32_t width;
    int32_t height;
    int32_t camera;
    int32_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(BinHeader) == 24, "LUT binary header must stay 24 bytes");

bool projectFisheye(const cv::Mat& R_cw, const cv::Mat& direction, const Intrinsics& intrinsics,
                    double* u, double* v, double* theta_d) {
    const cv::Mat x_cam = R_cw.t() * direction;
    const double x = x_cam.at<double>(0, 0);
    const double y = x_cam.at<double>(1, 0);
    const double z = x_cam.at<double>(2, 0);
    const double radius = std::hypot(x, y);
    const double theta = std::atan2(radius, z);
    if (!std::isfinite(theta) || theta > kPi) {
        return false;
    }
    const double t2 = theta * theta;
    const double k1 = intrinsics.D.at<double>(0, 0);
    const double k2 = intrinsics.D.at<double>(0, 1);
    const double k3 = intrinsics.D.at<double>(0, 2);
    const double k4 = intrinsics.D.at<double>(0, 3);
    const double distortion =
        1.0 + k1 * t2 + k2 * t2 * t2 + k3 * t2 * t2 * t2 + k4 * t2 * t2 * t2 * t2;
    const double theta_distorted = theta * distortion;
    const double fx = intrinsics.K.at<double>(0, 0);
    const double fy = intrinsics.K.at<double>(1, 1);
    const double cx = intrinsics.K.at<double>(0, 2);
    const double cy = intrinsics.K.at<double>(1, 2);
    if (radius < 1e-12) {
        *u = cx;
        *v = cy;
    } else {
        *u = fx * theta_distorted * x / radius + cx;
        *v = fy * theta_distorted * y / radius + cy;
    }
    *theta_d = theta_distorted;
    return true;
}

bool projectPinhole(const cv::Mat& R_cw, const cv::Mat& direction, const Intrinsics& intrinsics,
                    double* u, double* v) {
    const cv::Mat x_cam = R_cw.t() * direction;
    const double x = x_cam.at<double>(0, 0);
    const double y = x_cam.at<double>(1, 0);
    const double z = x_cam.at<double>(2, 0);
    if (z <= 1e-9) {
        return false;
    }
    const double x_n = x / z;
    const double y_n = y / z;
    const double k1 = intrinsics.D.total() > 0 ? intrinsics.D.at<double>(0, 0) : 0.0;
    const double k2 = intrinsics.D.total() > 1 ? intrinsics.D.at<double>(0, 1) : 0.0;
    const double p1 = intrinsics.D.total() > 2 ? intrinsics.D.at<double>(0, 2) : 0.0;
    const double p2 = intrinsics.D.total() > 3 ? intrinsics.D.at<double>(0, 3) : 0.0;
    const double k3 = intrinsics.D.total() > 4 ? intrinsics.D.at<double>(0, 4) : 0.0;
    const double r2 = x_n * x_n + y_n * y_n;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double x_d = x_n * radial + 2.0 * p1 * x_n * y_n + p2 * (r2 + 2.0 * x_n * x_n);
    const double y_d = y_n * radial + p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * p2 * x_n * y_n;
    *u = intrinsics.K.at<double>(0, 0) * x_d + intrinsics.K.at<double>(0, 2);
    *v = intrinsics.K.at<double>(1, 1) * y_d + intrinsics.K.at<double>(1, 2);
    return true;
}

bool insideImage(const Intrinsics& intrinsics, double u, double v, double theta_d) {
    if (u < 0.0 || v < 0.0 || u >= intrinsics.width || v >= intrinsics.height) {
        return false;
    }
    if (intrinsics.circle_radius > 0.0) {
        const double dx = u - intrinsics.circle_center.x;
        const double dy = v - intrinsics.circle_center.y;
        if (dx * dx + dy * dy > intrinsics.circle_radius * intrinsics.circle_radius) {
            return false;
        }
    }
    if (intrinsics.fold_radius > 0.0 && theta_d > intrinsics.fold_radius) {
        return false;
    }
    return true;
}

}  // namespace

Lut buildCylindricalLut(const Intrinsics& intrinsics, const Extrinsics& extrinsics, int out_width,
                        int out_height, double fov_h_deg, double blend_sigma_deg) {
    if (intrinsics.K.empty() || extrinsics.R_cw.empty()) {
        throw std::invalid_argument("LUT needs intrinsics.K and extrinsics.R_cw");
    }
    if (out_width <= 0 || out_height <= 0 || fov_h_deg <= 0.0) {
        throw std::invalid_argument("invalid panorama size / field of view");
    }
    const double fov = fov_h_deg * kPi / 180.0;
    const double focal = static_cast<double>(out_width) / fov;
    const double sigma = std::max(1e-3, blend_sigma_deg * kPi / 180.0);
    const cv::Mat forward = extrinsics.R_cw * (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);

    Lut lut;
    lut.width = out_width;
    lut.height = out_height;
    lut.mapx = cv::Mat(out_height, out_width, CV_32F, cv::Scalar(-1.0F));
    lut.mapy = cv::Mat(out_height, out_width, CV_32F, cv::Scalar(-1.0F));
    lut.weight = cv::Mat(out_height, out_width, CV_32F, cv::Scalar(0.0F));

    for (int row = 0; row < out_height; ++row) {
        const double y = (static_cast<double>(row) + 0.5 - 0.5 * out_height) / focal;
        for (int column = 0; column < out_width; ++column) {
            const double theta = (static_cast<double>(column) + 0.5 - 0.5 * out_width) / focal;
            cv::Mat direction = (cv::Mat_<double>(3, 1) << std::sin(theta), y, std::cos(theta));
            direction /= cv::norm(direction);

            double u = 0.0;
            double v = 0.0;
            double theta_d = 0.0;
            bool ok = false;
            if (intrinsics.model == CameraModel::Fisheye) {
                ok = projectFisheye(extrinsics.R_cw, direction, intrinsics, &u, &v, &theta_d);
            } else {
                ok = projectPinhole(extrinsics.R_cw, direction, intrinsics, &u, &v);
            }
            if (!ok || !insideImage(intrinsics, u, v, theta_d)) {
                continue;
            }
            const double cosine = std::clamp(forward.dot(direction), -1.0, 1.0);
            const double angle = std::acos(cosine);
            const double weight = std::exp(-0.5 * (angle / sigma) * (angle / sigma));
            if (weight < 1e-6) {
                continue;
            }
            lut.mapx.at<float>(row, column) = static_cast<float>(u);
            lut.mapy.at<float>(row, column) = static_cast<float>(v);
            lut.weight.at<float>(row, column) = static_cast<float>(weight);
        }
    }
    return lut;
}

void writeLutBin(const std::string& path, const Lut& lut, int camera_index) {
    if (lut.mapx.empty() || lut.mapy.empty() || lut.mapx.size() != lut.mapy.size()) {
        throw std::invalid_argument("LUT mapx/mapy are empty or have different sizes");
    }
    BinHeader header{};
    std::memcpy(header.magic, "PLUT", 4);
    header.version = 1;
    header.dtype = 0;
    header.width = lut.mapx.cols;
    header.height = lut.mapx.rows;
    header.camera = camera_index;
    header.reserved = 0;

    std::ofstream stream(path, std::ios::binary);
    if (!stream.good()) {
        throw std::runtime_error("cannot write LUT: " + path);
    }
    stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
    const cv::Mat mapx = lut.mapx.isContinuous() ? lut.mapx : lut.mapx.clone();
    const cv::Mat mapy = lut.mapy.isContinuous() ? lut.mapy : lut.mapy.clone();
    stream.write(reinterpret_cast<const char*>(mapx.data),
                 static_cast<std::streamsize>(mapx.total() * sizeof(float)));
    stream.write(reinterpret_cast<const char*>(mapy.data),
                 static_cast<std::streamsize>(mapy.total() * sizeof(float)));
    if (!stream.good()) {
        throw std::runtime_error("failed while writing LUT: " + path);
    }
}

void writeWeightBin(const std::string& path, const cv::Mat& weight, int camera_index) {
    if (weight.empty() || weight.type() != CV_32F) {
        throw std::invalid_argument("weight map must be non-empty CV_32F");
    }
    BinHeader header{};
    std::memcpy(header.magic, "PWGT", 4);
    header.version = 1;
    header.dtype = 0;
    header.width = weight.cols;
    header.height = weight.rows;
    header.camera = camera_index;
    header.reserved = 0;

    std::ofstream stream(path, std::ios::binary);
    if (!stream.good()) {
        throw std::runtime_error("cannot write weight map: " + path);
    }
    stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
    const cv::Mat continuous = weight.isContinuous() ? weight : weight.clone();
    stream.write(reinterpret_cast<const char*>(continuous.data),
                 static_cast<std::streamsize>(continuous.total() * sizeof(float)));
    if (!stream.good()) {
        throw std::runtime_error("failed while writing weight map: " + path);
    }
}

Lut readLutBin(const std::string& path, int camera_index) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream.good()) {
        throw std::runtime_error("cannot open LUT: " + path);
    }
    BinHeader header{};
    stream.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!stream.good() || std::memcmp(header.magic, "PLUT", 4) != 0) {
        throw std::runtime_error("invalid LUT header: " + path);
    }
    if (header.dtype != 0 || header.width <= 0 || header.height <= 0) {
        throw std::runtime_error("unsupported LUT header (only float32 is supported): " + path);
    }
    if (camera_index >= 0 && header.camera != camera_index) {
        throw std::runtime_error("LUT camera index mismatch: " + path);
    }
    const size_t count = static_cast<size_t>(header.width) * static_cast<size_t>(header.height);
    std::vector<float> buffer(count * 2);
    stream.read(reinterpret_cast<char*>(buffer.data()),
                static_cast<std::streamsize>(buffer.size() * sizeof(float)));
    if (!stream.good()) {
        throw std::runtime_error("truncated LUT payload: " + path);
    }
    Lut lut;
    lut.width = header.width;
    lut.height = header.height;
    cv::Mat mapx(header.height, header.width, CV_32F, buffer.data());
    cv::Mat mapy(header.height, header.width, CV_32F, buffer.data() + count);
    lut.mapx = mapx.clone();
    lut.mapy = mapy.clone();
    return lut;
}

cv::Mat readWeightBin(const std::string& path, int camera_index) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream.good()) {
        throw std::runtime_error("cannot open weight map: " + path);
    }
    BinHeader header{};
    stream.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!stream.good() || std::memcmp(header.magic, "PWGT", 4) != 0) {
        throw std::runtime_error("invalid weight header: " + path);
    }
    if (header.dtype != 0 || header.width <= 0 || header.height <= 0) {
        throw std::runtime_error("unsupported weight header: " + path);
    }
    if (camera_index >= 0 && header.camera != camera_index) {
        throw std::runtime_error("weight camera index mismatch: " + path);
    }
    cv::Mat weight(header.height, header.width, CV_32F);
    stream.read(reinterpret_cast<char*>(weight.data),
                static_cast<std::streamsize>(weight.total() * sizeof(float)));
    if (!stream.good()) {
        throw std::runtime_error("truncated weight payload: " + path);
    }
    return weight;
}

}  // namespace hikquad
