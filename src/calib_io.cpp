#include "hikquad/calib_io.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <opencv2/core.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "hikquad/config.hpp"

namespace hikquad {
namespace fs = std::filesystem;

namespace {

std::string nowIso8601() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    std::ostringstream out;
    out << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return out.str();
}

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
    extrinsics.has_angle = true;
    return extrinsics;
}

void writeCalibMetaJson(const std::string& path, int camera_count, const BoardSpec& board,
                        const PanoramaConfig& panorama, const CalibrationResult& result) {
    ensureParentDirectory(path);
    cv::FileStorage storage(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot write calibration metadata: " + path);
    }
    storage << "version" << 1;
    storage << "created" << nowIso8601();
    storage << "count" << camera_count;
    storage << "out_size" << "[" << panorama.out_width << panorama.out_height << "]";
    storage << "fov_h_deg" << panorama.fov_h_deg;
    storage << "lut_dtype" << "float32";
    storage << "board" << "{";
    storage << "inner_cols" << board.cols;
    storage << "inner_rows" << board.rows;
    storage << "square_mm" << board.square_mm;
    storage << "}";
    storage << "cameras" << "[";
    for (int i = 0; i < camera_count; ++i) {
        const auto& camera = result.cameras[static_cast<size_t>(i)];
        storage << "{";
        storage << "index" << i;
        storage << "intrinsics_rms" << camera.intrinsics.rms;
        storage << "has_lut" << (!camera.intrinsics.K.empty());
        storage << "has_weight" << (!camera.intrinsics.K.empty());
        storage << "}";
    }
    storage << "]";
    storage.release();
}

void writeCalibrationReportJson(const std::string& path, const CalibrationResult& result) {
    ensureParentDirectory(path);
    cv::FileStorage storage(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot write calibration report: " + path);
    }
    storage << "board" << "{";
    storage << "cols" << result.board.cols;
    storage << "rows" << result.board.rows;
    storage << "square_mm" << result.board.square_mm;
    storage << "}";
    storage << "camera_count" << result.camera_count;
    storage << "cameras" << "[";
    for (size_t i = 0; i < result.cameras.size(); ++i) {
        const auto& camera = result.cameras[i];
        storage << "{";
        storage << "index" << static_cast<int>(i);
        storage << "views" << camera.intrinsics.view_count;
        storage << "rms" << camera.intrinsics.rms;
        storage << "mean_error" << camera.intrinsics.mean_error;
        storage << "K" << camera.intrinsics.K;
        storage << "D" << camera.intrinsics.D;
        storage << "center_angle_deg" << camera.extrinsics.center_angle_deg;
        storage << "T_cw" << camera.extrinsics.T_cw;
        storage << "}";
    }
    storage << "]";
    storage << "edges" << "[";
    for (const auto& edge : result.edges) {
        storage << "{";
        storage << "from" << edge.from;
        storage << "to" << edge.to;
        storage << "common_views" << edge.common_views;
        storage << "baseline_m" << cv::norm(edge.t);
        storage << "translation_spread_m" << edge.translation_spread_m;
        storage << "}";
    }
    storage << "]";
    storage << "warnings" << "[";
    for (const auto& warning : result.warnings) {
        storage << warning;
    }
    storage << "]";
    storage.release();
}

}  // namespace hikquad
