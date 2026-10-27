#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace hikquad {

enum class CameraModel { Fisheye, Pinhole };

struct BoardSpec {
    int cols = 8;
    int rows = 11;
    double square_mm = 100.0;

    cv::Size patternSize() const {
        return cv::Size(cols, rows);
    }
    double squareMeters() const {
        return square_mm / 1000.0;
    }
};

struct Intrinsics {
    CameraModel model = CameraModel::Fisheye;
    cv::Mat K;
    cv::Mat D;
    int width = 0;
    int height = 0;
    BoardSpec board;
    cv::Point2d circle_center{0.0, 0.0};
    double circle_radius = 0.0;
    double fold_radius = 0.0;
    double rms = 0.0;
    double mean_error = 0.0;
    int view_count = 0;
};

struct Extrinsics {
    cv::Mat R_cw;  // world = R_cw * camera + T_cw
    cv::Mat T_cw;  // camera centre in world coordinates
    double center_angle_deg = 0.0;
};

struct Pose {
    cv::Mat R;
    cv::Mat t;
};

struct ViewObservation {
    int view_index = -1;
    std::string image_path;
    std::vector<cv::Point2f> image_points;
    bool detected = false;
    double reprojection_error = 0.0;
};

struct CalibrationView {
    int index = -1;
    std::vector<std::string> image_paths;
};

struct CameraCalibrationResult {
    Intrinsics intrinsics;
    Extrinsics extrinsics;
    std::vector<ViewObservation> observations;
    std::vector<Pose> board_poses;
};

struct PairEdge {
    int from = -1;
    int to = -1;
    int common_views = 0;
    cv::Mat R;
    cv::Mat t;
    double translation_spread_m = 0.0;
};

struct CalibrationResult {
    BoardSpec board;
    int camera_count = 0;
    std::vector<CameraCalibrationResult> cameras;
    std::vector<PairEdge> edges;
    std::vector<std::string> warnings;
};

std::vector<CalibrationView> discoverViews(const std::string& root, int camera_count);
bool detectBoard(const cv::Mat& bgr, const BoardSpec& board, std::vector<cv::Point2f>* corners);
CameraCalibrationResult calibrateCamera(const std::vector<ViewObservation>& observations,
                                        const BoardSpec& board, CameraModel model, int image_width,
                                        int image_height);
void estimateExtrinsics(CalibrationResult& result, int camera_count,
                        double nominal_step_deg = 90.0);
void finalizeIntrinsics(Intrinsics& intrinsics);
cv::Mat nominalRotationY(double angle_deg);
double opticalAxisAngleDeg(const cv::Mat& R_cw);

/// Projects an extrinsics rotation onto the world Y axis (yaw only) and puts
/// the camera centre on the world XZ plane.  Use this for a mechanically
/// horizontal 4-camera ring where pitch/roll are known to be zero.
Extrinsics horizontalizeExtrinsics(const Extrinsics& extrinsics);
void applyHorizontalConstraint(CalibrationResult& result);

std::string formatCalibrationSummary(const CalibrationResult& result);

}  // namespace hikquad
