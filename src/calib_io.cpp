#include "hikquad/calib_io.hpp"

#include <filesystem>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

namespace hikquad {
namespace fs = std::filesystem;

namespace {

std::string modelName(CameraModel model) {
    return model == CameraModel::Fisheye ? "fisheye" : "pinhole";
}

void ensureParentDirectory(const std::string& path) {
    const fs::path parent = fs::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        fs::create_directories(parent, ec);
    }
}

}  // namespace

void writeIntrinsicsYml(const std::string& path, const Intrinsics& intrinsics) {
    ensureParentDirectory(path);
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot write intrinsics: " + path);
    }
    cv::Mat K = intrinsics.K;
    cv::Mat D = intrinsics.D.reshape(1, 1);
    K.convertTo(K, CV_64F);
    D.convertTo(D, CV_64F);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);
    const cv::Point2d center =
        intrinsics.circle_center.x == 0.0 && intrinsics.circle_center.y == 0.0
            ? cv::Point2d(cx, cy)
            : intrinsics.circle_center;
    storage << "model" << modelName(intrinsics.model);
    storage << "K" << K;
    storage << "D" << D;
    storage << "image_width" << intrinsics.width;
    storage << "image_height" << intrinsics.height;
    storage << "pattern_cols" << intrinsics.board.cols;
    storage << "pattern_rows" << intrinsics.board.rows;
    storage << "square_mm" << intrinsics.board.square_mm;
    storage << "circle_center" << (cv::Mat_<double>(1, 2) << center.x, center.y);
    storage << "circle_radius" << intrinsics.circle_radius;
    storage << "fold_radius" << intrinsics.fold_radius;
    storage << "rms" << intrinsics.rms;
    storage << "mean_error" << intrinsics.mean_error;
    storage.release();
}

void writeExtrinsicsYml(const std::string& path, const Extrinsics& extrinsics) {
    ensureParentDirectory(path);
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot write extrinsics: " + path);
    }
    cv::Mat R = extrinsics.R_cw;
    cv::Mat T = extrinsics.T_cw.reshape(1, 3);
    R.convertTo(R, CV_64F);
    T.convertTo(T, CV_64F);
    storage << "R_cw" << R;
    storage << "T_cw" << T;
    storage << "center_angle_deg" << extrinsics.center_angle_deg;
    storage.release();
}

Intrinsics readIntrinsicsYml(const std::string& path) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot read intrinsics: " + path);
    }
    Intrinsics intrinsics;
    std::string model;
    storage["model"] >> model;
    intrinsics.model = model == "fisheye" ? CameraModel::Fisheye : CameraModel::Pinhole;
    storage["K"] >> intrinsics.K;
    storage["D"] >> intrinsics.D;
    if (intrinsics.K.empty() || intrinsics.D.empty()) {
        throw std::runtime_error("intrinsics file is missing K or D: " + path);
    }
    intrinsics.K.convertTo(intrinsics.K, CV_64F);
    intrinsics.D = intrinsics.D.reshape(1, 1).clone();
    intrinsics.D.convertTo(intrinsics.D, CV_64F);
    storage["image_width"] >> intrinsics.width;
    storage["image_height"] >> intrinsics.height;
    storage["pattern_cols"] >> intrinsics.board.cols;
    storage["pattern_rows"] >> intrinsics.board.rows;
    storage["square_mm"] >> intrinsics.board.square_mm;
    cv::Mat center;
    storage["circle_center"] >> center;
    if (!center.empty()) {
        center = center.reshape(1, 1);
        intrinsics.circle_center = cv::Point2d(center.at<double>(0, 0), center.at<double>(0, 1));
    }
    storage["circle_radius"] >> intrinsics.circle_radius;
    storage["fold_radius"] >> intrinsics.fold_radius;
    storage["rms"] >> intrinsics.rms;
    storage["mean_error"] >> intrinsics.mean_error;
    if (intrinsics.width <= 0 || intrinsics.height <= 0) {
        throw std::runtime_error("intrinsics file has invalid image size: " + path);
    }
    return intrinsics;
}

Extrinsics readExtrinsicsYml(const std::string& path) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot read extrinsics: " + path);
    }
    Extrinsics extrinsics;
    storage["R_cw"] >> extrinsics.R_cw;
    storage["T_cw"] >> extrinsics.T_cw;
    if (extrinsics.R_cw.empty() || extrinsics.T_cw.empty()) {
        throw std::runtime_error("extrinsics file is missing R_cw or T_cw: " + path);
    }
    extrinsics.R_cw.convertTo(extrinsics.R_cw, CV_64F);
    extrinsics.T_cw = extrinsics.T_cw.reshape(1, 3).clone();
    extrinsics.T_cw.convertTo(extrinsics.T_cw, CV_64F);
    storage["center_angle_deg"] >> extrinsics.center_angle_deg;
    return extrinsics;
}

}  // namespace hikquad
