#include "hikquad/calibration.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <map>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hikquad/log.hpp"

namespace hikquad {
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool isImageFile(const fs::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
           extension == ".bmp" || extension == ".tif" || extension == ".tiff";
}

std::vector<fs::path> listImages(const fs::path& directory) {
    std::vector<fs::path> files;
    std::error_code ec;
    if (!fs::is_directory(directory, ec)) {
        return files;
    }
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (entry.is_regular_file() && isImageFile(entry.path())) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::string findCameraImage(const fs::path& directory, int camera_index) {
    const std::vector<std::string> stems = {
        "cam" + std::to_string(camera_index), "camera_" + std::to_string(camera_index),
        std::to_string(camera_index), "cam_" + std::to_string(camera_index)};
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (!entry.is_regular_file() || !isImageFile(entry.path())) {
            continue;
        }
        const std::string stem = entry.path().stem().string();
        for (const auto& candidate : stems) {
            if (stem == candidate) {
                return entry.path().string();
            }
        }
    }
    return {};
}

cv::Mat orthonormalize(const cv::Mat& rotation) {
    cv::Mat u;
    cv::Mat w;
    cv::Mat vt;
    cv::SVD::compute(rotation, w, u, vt);
    cv::Mat result = u * vt;
    if (cv::determinant(result) < 0.0) {
        cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
        correction.at<double>(2, 2) = -1.0;
        result = u * correction * vt;
    }
    return result;
}

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Quaternion quaternionFromMatrix(const cv::Mat& rotation) {
    Quaternion q;
    const double trace =
        rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2);
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / s;
        q.y = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / s;
        q.z = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / s;
    } else if (rotation.at<double>(0, 0) > rotation.at<double>(1, 1) &&
               rotation.at<double>(0, 0) > rotation.at<double>(2, 2)) {
        const double s = std::sqrt(1.0 + rotation.at<double>(0, 0) - rotation.at<double>(1, 1) -
                                   rotation.at<double>(2, 2)) *
                         2.0;
        q.w = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / s;
        q.z = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / s;
    } else if (rotation.at<double>(1, 1) > rotation.at<double>(2, 2)) {
        const double s = std::sqrt(1.0 + rotation.at<double>(1, 1) - rotation.at<double>(0, 0) -
                                   rotation.at<double>(2, 2)) *
                         2.0;
        q.w = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / s;
        q.x = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + rotation.at<double>(2, 2) - rotation.at<double>(0, 0) -
                                   rotation.at<double>(1, 1)) *
                         2.0;
        q.w = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / s;
        q.x = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / s;
        q.y = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / s;
        q.z = 0.25 * s;
    }
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 0.0) {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    return q;
}

cv::Mat matrixFromQuaternion(const Quaternion& q) {
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;
    return (cv::Mat_<double>(3, 3) << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
            2.0 * (x * z + w * y), 2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x), 2.0 * (x * z - w * y), 2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y));
}

cv::Mat averageRotation(const std::vector<cv::Mat>& rotations) {
    if (rotations.empty()) {
        return cv::Mat::eye(3, 3, CV_64F);
    }
    std::vector<Quaternion> quaternions;
    quaternions.reserve(rotations.size());
    for (const auto& rotation : rotations) {
        quaternions.push_back(quaternionFromMatrix(rotation));
    }
    // q and -q describe the same rotation.  Align every quaternion to the first
    // one before averaging so a 180-degree relative yaw does not cancel.
    const Quaternion reference = quaternions.front();
    Quaternion sum;
    sum.w = 0.0;
    sum.x = 0.0;
    sum.y = 0.0;
    sum.z = 0.0;
    for (const auto& q : quaternions) {
        const double dot =
            q.w * reference.w + q.x * reference.x + q.y * reference.y + q.z * reference.z;
        const double sign = dot < 0.0 ? -1.0 : 1.0;
        sum.w += sign * q.w;
        sum.x += sign * q.x;
        sum.y += sign * q.y;
        sum.z += sign * q.z;
    }
    const double norm = std::sqrt(sum.w * sum.w + sum.x * sum.x + sum.y * sum.y + sum.z * sum.z);
    if (norm < 1e-12) {
        return orthonormalize(rotations.front());
    }
    sum.w /= norm;
    sum.x /= norm;
    sum.y /= norm;
    sum.z /= norm;
    return orthonormalize(matrixFromQuaternion(sum));
}

cv::Mat averageTranslation(const std::vector<cv::Mat>& translations, double* spread) {
    if (translations.empty()) {
        *spread = 0.0;
        return cv::Mat::zeros(3, 1, CV_64F);
    }
    cv::Mat median(3, 1, CV_64F);
    for (int row = 0; row < 3; ++row) {
        std::vector<double> values;
        for (const auto& translation : translations) {
            values.push_back(translation.at<double>(row, 0));
        }
        std::sort(values.begin(), values.end());
        const size_t middle = values.size() / 2;
        median.at<double>(row, 0) =
            values.size() % 2 == 0 ? 0.5 * (values[middle - 1] + values[middle]) : values[middle];
    }
    double max_spread = 0.0;
    for (const auto& translation : translations) {
        max_spread = std::max(max_spread, cv::norm(translation - median));
    }
    *spread = max_spread;
    return median;
}

std::vector<cv::Point3f> boardObjectPoints(const BoardSpec& board) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(board.cols) * static_cast<size_t>(board.rows));
    const double square = board.squareMeters();
    for (int row = 0; row < board.rows; ++row) {
        for (int column = 0; column < board.cols; ++column) {
            points.emplace_back(static_cast<float>(column * square),
                                static_cast<float>(row * square), 0.0F);
        }
    }
    return points;
}

double reprojectionError(const std::vector<cv::Point3f>& object_points,
                         const std::vector<cv::Point2f>& image_points, const cv::Mat& K,
                         const cv::Mat& D, CameraModel model, const Pose& pose) {
    std::vector<cv::Point2f> projected;
    cv::Vec3d rvec;
    cv::Rodrigues(pose.R, rvec);
    cv::Vec3d tvec(pose.t.at<double>(0, 0), pose.t.at<double>(1, 0), pose.t.at<double>(2, 0));
    if (model == CameraModel::Fisheye) {
        cv::fisheye::projectPoints(object_points, projected, rvec, tvec, K, D);
    } else {
        cv::projectPoints(object_points, rvec, tvec, K, D, projected);
    }
    if (projected.size() != image_points.size() || image_points.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        sum += cv::norm(image_points[i] - projected[i]);
    }
    return sum / static_cast<double>(image_points.size());
}

}  // namespace

std::vector<CalibrationView> discoverViews(const std::string& root, int camera_count) {
    const fs::path base(root);
    std::error_code ec;
    if (!fs::is_directory(base, ec)) {
        throw std::runtime_error("calibration input is not a directory: " + root);
    }

    std::vector<CalibrationView> views;

    // Layout A: root/cam0 ... root/camN, paired by sorted file index.
    bool per_camera_layout = true;
    for (int i = 0; i < camera_count; ++i) {
        if (!fs::is_directory(base / ("cam" + std::to_string(i)), ec)) {
            per_camera_layout = false;
            break;
        }
    }
    if (per_camera_layout) {
        std::vector<std::vector<fs::path>> files;
        size_t count = 0;
        for (int i = 0; i < camera_count; ++i) {
            files.push_back(listImages(base / ("cam" + std::to_string(i))));
            count = std::max(count, files.back().size());
        }
        for (size_t index = 0; index < count; ++index) {
            CalibrationView view;
            view.index = static_cast<int>(index);
            view.image_paths.resize(static_cast<size_t>(camera_count));
            for (int camera = 0; camera < camera_count; ++camera) {
                if (index < files[static_cast<size_t>(camera)].size()) {
                    view.image_paths[static_cast<size_t>(camera)] =
                        files[static_cast<size_t>(camera)][index].string();
                }
            }
            views.push_back(std::move(view));
        }
        return views;
    }

    // Layout B: root/view_000/cam0.png ... camN.png
    std::vector<fs::path> directories;
    for (const auto& entry : fs::directory_iterator(base, ec)) {
        if (entry.is_directory()) {
            directories.push_back(entry.path());
        }
    }
    std::sort(directories.begin(), directories.end());
    for (const auto& directory : directories) {
        CalibrationView view;
        view.index = static_cast<int>(views.size());
        view.image_paths.resize(static_cast<size_t>(camera_count));
        int found = 0;
        for (int camera = 0; camera < camera_count; ++camera) {
            view.image_paths[static_cast<size_t>(camera)] = findCameraImage(directory, camera);
            if (!view.image_paths[static_cast<size_t>(camera)].empty()) {
                ++found;
            }
        }
        if (found > 0) {
            views.push_back(std::move(view));
        }
    }
    if (views.empty()) {
        throw std::runtime_error("no calibration views found under '" + root +
                                 "'. Expected root/view_000/cam0.png ... or root/cam0/*.png");
    }
    return views;
}

bool detectBoard(const cv::Mat& bgr, const BoardSpec& board, std::vector<cv::Point2f>* corners) {
    if (bgr.empty()) {
        return false;
    }
    cv::Mat gray;
    if (bgr.channels() == 3) {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = bgr;
    }
    std::vector<cv::Point2f> detected;
    bool ok = cv::findChessboardCornersSB(gray, board.patternSize(), detected,
                                          cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
    if (!ok) {
        ok = cv::findChessboardCorners(
            gray, board.patternSize(), detected,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        if (ok) {
            cv::cornerSubPix(
                gray, detected, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        }
    }
    if (ok && corners != nullptr) {
        *corners = std::move(detected);
    }
    return ok;
}

CameraCalibrationResult calibrateCamera(const std::vector<ViewObservation>& observations,
                                        const BoardSpec& board, CameraModel model, int image_width,
                                        int image_height) {
    std::vector<ViewObservation> detected;
    for (const auto& observation : observations) {
        if (observation.detected && !observation.image_points.empty()) {
            detected.push_back(observation);
        }
    }
    if (detected.size() < 3) {
        throw std::runtime_error("at least 3 views with a detected board are required, got " +
                                 std::to_string(detected.size()));
    }

    const std::vector<cv::Point3f> object_points = boardObjectPoints(board);
    std::vector<std::vector<cv::Point3f>> object_views(detected.size(), object_points);
    std::vector<std::vector<cv::Point2f>> image_views;
    image_views.reserve(detected.size());
    for (const auto& observation : detected) {
        image_views.push_back(observation.image_points);
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = 0.5 * std::max(image_width, image_height);
    K.at<double>(1, 1) = K.at<double>(0, 0);
    K.at<double>(0, 2) = 0.5 * image_width;
    K.at<double>(1, 2) = 0.5 * image_height;

    cv::Mat D;
    double rms = 0.0;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    try {
        if (model == CameraModel::Fisheye) {
            D = cv::Mat::zeros(4, 1, CV_64F);
            rms = cv::fisheye::calibrate(
                object_views, image_views, cv::Size(image_width, image_height), K, D, rvecs, tvecs,
                cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-8));
        } else {
            D = cv::Mat::zeros(5, 1, CV_64F);
            rms = cv::calibrateCamera(
                object_views, image_views, cv::Size(image_width, image_height), K, D, rvecs, tvecs,
                0, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-8));
        }
    } catch (const cv::Exception& error) {
        throw std::runtime_error(std::string("OpenCV calibration failed: ") + error.what());
    }

    K.convertTo(K, CV_64F);
    D = D.reshape(1, 1).clone();
    D.convertTo(D, CV_64F);

    CameraCalibrationResult result;
    result.intrinsics.model = model;
    result.intrinsics.K = K;
    result.intrinsics.D = D;
    result.intrinsics.width = image_width;
    result.intrinsics.height = image_height;
    result.intrinsics.board = board;
    result.intrinsics.rms = rms;
    result.intrinsics.view_count = static_cast<int>(detected.size());
    result.observations = detected;
    result.used_views = static_cast<int>(detected.size());

    double sum_error = 0.0;
    for (size_t i = 0; i < detected.size(); ++i) {
        Pose pose;
        cv::Rodrigues(rvecs[i], pose.R);
        pose.R.convertTo(pose.R, CV_64F);
        pose.t = cv::Mat(tvecs[i]).reshape(1, 3).clone();
        pose.t.convertTo(pose.t, CV_64F);
        result.board_poses.push_back(pose);
        result.observations[i].reprojection_error =
            reprojectionError(object_points, detected[i].image_points, K, D, model, pose);
        sum_error += result.observations[i].reprojection_error;
    }
    result.intrinsics.mean_error = sum_error / static_cast<double>(detected.size());
    finalizeIntrinsics(result.intrinsics);
    return result;
}

void finalizeIntrinsics(Intrinsics& intrinsics) {
    if (intrinsics.K.empty()) {
        return;
    }
    intrinsics.K.convertTo(intrinsics.K, CV_64F);
    intrinsics.D = intrinsics.D.reshape(1, 1).clone();
    intrinsics.D.convertTo(intrinsics.D, CV_64F);
    const double fx = intrinsics.K.at<double>(0, 0);
    const double fy = intrinsics.K.at<double>(1, 1);
    const double cx = intrinsics.K.at<double>(0, 2);
    const double cy = intrinsics.K.at<double>(1, 2);
    if (intrinsics.circle_center.x == 0.0 && intrinsics.circle_center.y == 0.0) {
        intrinsics.circle_center = cv::Point2d(cx, cy);
    }
    if (intrinsics.circle_radius <= 0.0 && intrinsics.width > 0 && intrinsics.height > 0) {
        intrinsics.circle_radius = std::min({cx, cy, static_cast<double>(intrinsics.width) - cx,
                                             static_cast<double>(intrinsics.height) - cy});
    }
    const double focal = 0.5 * (fx + fy);
    if (intrinsics.fold_radius <= 0.0 && focal > 0.0 && intrinsics.circle_radius > 0.0) {
        intrinsics.fold_radius = intrinsics.circle_radius / focal;
    }
}

double opticalAxisAngleDeg(const cv::Mat& R_cw) {
    cv::Mat axis = R_cw * (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    double angle = std::atan2(axis.at<double>(0, 0), axis.at<double>(2, 0)) * 180.0 / kPi;
    while (angle < 0.0) {
        angle += 360.0;
    }
    while (angle >= 360.0) {
        angle -= 360.0;
    }
    return angle;
}

cv::Mat nominalRotationY(double angle_deg) {
    const double angle = angle_deg * kPi / 180.0;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return (cv::Mat_<double>(3, 3) << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
}

void estimateExtrinsics(CalibrationResult& result, int camera_count, double nominal_step_deg) {
    result.camera_count = camera_count;
    result.cameras.resize(static_cast<size_t>(camera_count));
    for (int i = 0; i < camera_count; ++i) {
        result.cameras[static_cast<size_t>(i)].intrinsics.board = result.board;
    }

    // Index board poses by capture view id for O(1) pairwise lookup.
    std::vector<std::map<int, Pose>> poses(static_cast<size_t>(camera_count));
    for (int camera = 0; camera < camera_count; ++camera) {
        const auto& calibration = result.cameras[static_cast<size_t>(camera)];
        for (size_t i = 0;
             i < calibration.observations.size() && i < calibration.board_poses.size(); ++i) {
            poses[static_cast<size_t>(camera)][calibration.observations[i].view_index] =
                calibration.board_poses[i];
        }
    }

    struct Edge {
        int a = -1;
        int b = -1;
        int common = 0;
        cv::Mat R_ab;  // X_a = R_ab * X_b + t_ab
        cv::Mat t_ab;
        double spread = 0.0;
    };
    std::vector<Edge> edges;
    for (int a = 0; a < camera_count; ++a) {
        for (int b = a + 1; b < camera_count; ++b) {
            std::vector<cv::Mat> rotations;
            std::vector<cv::Mat> translations;
            for (const auto& entry : poses[static_cast<size_t>(a)]) {
                const auto found = poses[static_cast<size_t>(b)].find(entry.first);
                if (found == poses[static_cast<size_t>(b)].end()) {
                    continue;
                }
                const Pose& pose_a = entry.second;
                const Pose& pose_b = found->second;
                const cv::Mat R_ab = pose_a.R * pose_b.R.t();
                const cv::Mat t_ab = pose_a.t - R_ab * pose_b.t;
                rotations.push_back(R_ab);
                translations.push_back(t_ab);
            }
            if (rotations.size() < 2) {
                continue;
            }
            Edge edge;
            edge.a = a;
            edge.b = b;
            edge.common = static_cast<int>(rotations.size());
            edge.R_ab = averageRotation(rotations);
            edge.t_ab = averageTranslation(translations, &edge.spread);
            edges.push_back(edge);
            PairEdge public_edge;
            public_edge.from = a;
            public_edge.to = b;
            public_edge.common_views = edge.common;
            public_edge.R = edge.R_ab;
            public_edge.t = edge.t_ab;
            public_edge.translation_spread_m = edge.spread;
            result.edges.push_back(public_edge);
        }
    }

    if (edges.empty()) {
        result.warnings.push_back(
            "no camera pair shared enough board views; using nominal 90-degree yaw and zero "
            "baseline");
    }

    std::vector<bool> known(static_cast<size_t>(camera_count), false);
    // cv::Mat's copy constructor shares data; build distinct matrices explicitly.
    std::vector<cv::Mat> R(static_cast<size_t>(camera_count));
    std::vector<cv::Mat> t(static_cast<size_t>(camera_count));
    for (int camera = 0; camera < camera_count; ++camera) {
        R[static_cast<size_t>(camera)] = cv::Mat::eye(3, 3, CV_64F);
        t[static_cast<size_t>(camera)] = cv::Mat::zeros(3, 1, CV_64F);
    }
    known[0] = true;
    std::vector<int> queue{0};
    while (!queue.empty()) {
        const int current = queue.front();
        queue.erase(queue.begin());
        for (const auto& edge : edges) {
            int next = -1;
            cv::Mat R_current_next;
            cv::Mat t_current_next;
            if (edge.a == current && !known[static_cast<size_t>(edge.b)]) {
                next = edge.b;
                R_current_next = edge.R_ab;
                t_current_next = edge.t_ab;
                // X_current = R * X_next + t  =>  X_next = R^T (X_current - t)
                R[static_cast<size_t>(next)] = R_current_next.t() * R[static_cast<size_t>(current)];
                t[static_cast<size_t>(next)] =
                    R_current_next.t() * (t[static_cast<size_t>(current)] - t_current_next);
            } else if (edge.b == current && !known[static_cast<size_t>(edge.a)]) {
                next = edge.a;
                R_current_next = edge.R_ab.t();
                t_current_next = -edge.R_ab.t() * edge.t_ab;
                R[static_cast<size_t>(next)] = R_current_next * R[static_cast<size_t>(current)];
                t[static_cast<size_t>(next)] =
                    R_current_next * t[static_cast<size_t>(current)] + t_current_next;
            }
            if (next >= 0) {
                known[static_cast<size_t>(next)] = true;
                queue.push_back(next);
            }
        }
    }

    for (int camera = 0; camera < camera_count; ++camera) {
        Extrinsics extrinsics;
        if (camera == 0) {
            extrinsics.R_cw = cv::Mat::eye(3, 3, CV_64F);
            extrinsics.T_cw = cv::Mat::zeros(3, 1, CV_64F);
        } else if (known[static_cast<size_t>(camera)]) {
            // BFS produced X_cam = R_i0 * X_world + t_i0 (world -> camera).
            // quad-camera stores X_world = R_cw * X_cam + T_cw (camera -> world),
            // where T_cw is the camera centre.
            const cv::Mat R_i0 = orthonormalize(R[static_cast<size_t>(camera)]);
            const cv::Mat t_i0 = t[static_cast<size_t>(camera)];
            extrinsics.R_cw = R_i0.t();
            extrinsics.T_cw = -R_i0.t() * t_i0;
        } else {
            extrinsics.R_cw = nominalRotationY(camera * nominal_step_deg);
            extrinsics.T_cw = cv::Mat::zeros(3, 1, CV_64F);
            result.warnings.push_back("camera " + std::to_string(camera) +
                                      " has no connected pair; using nominal yaw");
        }
        extrinsics.center_angle_deg = opticalAxisAngleDeg(extrinsics.R_cw);
        extrinsics.has_angle = true;
        result.cameras[static_cast<size_t>(camera)].extrinsics = extrinsics;
    }
}

std::string formatCalibrationSummary(const CalibrationResult& result) {
    std::ostringstream out;
    out << "board " << result.board.cols << "x" << result.board.rows
        << " square=" << result.board.square_mm << "mm\n";
    for (size_t i = 0; i < result.cameras.size(); ++i) {
        const auto& camera = result.cameras[i];
        const auto& intr = camera.intrinsics;
        out << "camera " << i << ": views=" << intr.view_count << " rms=" << intr.rms
            << " mean=" << intr.mean_error << " fx=" << intr.K.at<double>(0, 0)
            << " fy=" << intr.K.at<double>(1, 1) << " cx=" << intr.K.at<double>(0, 2)
            << " cy=" << intr.K.at<double>(1, 2) << " angle=" << camera.extrinsics.center_angle_deg
            << "deg\n";
    }
    for (const auto& edge : result.edges) {
        out << "edge " << edge.from << "<-" << edge.to << " views=" << edge.common_views
            << " baseline=" << cv::norm(edge.t) << "m spread=" << edge.translation_spread_m
            << "m\n";
    }
    for (const auto& warning : result.warnings) {
        out << "warning: " << warning << "\n";
    }
    return out.str();
}

}  // namespace hikquad
