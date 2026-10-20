#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "hikquad/calibration.hpp"
#include "test_framework.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

cv::Mat rotationY(double degrees) {
    const double angle = degrees * kPi / 180.0;
    return (cv::Mat_<double>(3, 3) << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0, 0.0,
            -std::sin(angle), 0.0, std::cos(angle));
}

}  // namespace

HIK_TEST(nominal_rotation_is_consistent) {
    const cv::Mat identity = hikquad::nominalRotationY(0.0);
    CHECK_NEAR(cv::norm(identity - cv::Mat::eye(3, 3, CV_64F)), 0.0, 1e-12);
    const cv::Mat right = hikquad::nominalRotationY(90.0);
    CHECK_NEAR(hikquad::opticalAxisAngleDeg(right), 90.0, 1e-9);
    CHECK_NEAR(hikquad::opticalAxisAngleDeg(hikquad::nominalRotationY(180.0)), 180.0, 1e-9);
    CHECK_NEAR(hikquad::opticalAxisAngleDeg(hikquad::nominalRotationY(270.0)), 270.0, 1e-9);
}

HIK_TEST(intrinsics_finalize_derives_circle) {
    hikquad::Intrinsics intrinsics;
    intrinsics.K = (cv::Mat_<double>(3, 3) << 450.0, 0.0, 720.0, 0.0, 450.0, 540.0, 0.0, 0.0, 1.0);
    intrinsics.D = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 0.0);
    intrinsics.width = 1440;
    intrinsics.height = 1080;
    hikquad::finalizeIntrinsics(intrinsics);
    CHECK_NEAR(intrinsics.circle_radius, 540.0, 1e-9);
    CHECK_NEAR(intrinsics.fold_radius, 540.0 / 450.0, 1e-9);
    CHECK_NEAR(intrinsics.circle_center.x, 720.0, 1e-9);
}

HIK_TEST(calibrate_recovers_fisheye_intrinsics) {
    const hikquad::BoardSpec board{8, 11, 100.0};
    const cv::Mat true_K =
        (cv::Mat_<double>(3, 3) << 330.0, 0.0, 720.0, 0.0, 332.0, 540.0, 0.0, 0.0, 1.0);
    const cv::Mat true_D = (cv::Mat_<double>(1, 4) << -0.045, -0.008, 0.0002, -0.0001);
    std::vector<cv::Point3f> object;
    for (int row = 0; row < board.rows; ++row) {
        for (int column = 0; column < board.cols; ++column) {
            object.emplace_back(static_cast<float>(column * board.squareMeters()),
                                static_cast<float>(row * board.squareMeters()), 0.0F);
        }
    }

    std::vector<hikquad::ViewObservation> observations;
    for (int view = 0; view < 16; ++view) {
        const double tilt_x = 0.35 * std::sin(0.7 * view);
        const double tilt_y = 0.35 * std::cos(0.9 * view);
        const cv::Vec3d rvec(tilt_x, tilt_y, 0.1 * std::sin(0.3 * view));
        const cv::Vec3d tvec(-0.10 + 0.02 * view, -0.05 + 0.01 * view, 0.65 + 0.02 * view);
        std::vector<cv::Point2f> image;
        cv::fisheye::projectPoints(object, image, rvec, tvec, true_K, true_D);
        bool inside = true;
        for (const auto& point : image) {
            if (point.x < 10.0F || point.y < 10.0F || point.x > 1430.0F || point.y > 1070.0F) {
                inside = false;
                break;
            }
        }
        if (!inside) {
            continue;
        }
        hikquad::ViewObservation observation;
        observation.view_index = view;
        observation.detected = true;
        observation.image_points = std::move(image);
        observations.push_back(std::move(observation));
    }
    CHECK_TRUE(observations.size() >= 8);
    const hikquad::CameraCalibrationResult result =
        hikquad::calibrateCamera(observations, board, hikquad::CameraModel::Fisheye, 1440, 1080);
    CHECK_NEAR(result.intrinsics.K.at<double>(0, 0), 330.0, 12.0);
    CHECK_NEAR(result.intrinsics.K.at<double>(1, 1), 332.0, 12.0);
    CHECK_NEAR(result.intrinsics.K.at<double>(0, 2), 720.0, 8.0);
    CHECK_NEAR(result.intrinsics.K.at<double>(1, 2), 540.0, 8.0);
    CHECK_TRUE(result.intrinsics.rms < 0.5);
}

HIK_TEST(estimate_extrinsics_recovers_ring) {
    constexpr int count = 4;
    hikquad::CalibrationResult result;
    result.board = hikquad::BoardSpec{8, 11, 100.0};
    result.camera_count = count;
    result.cameras.resize(count);

    std::vector<cv::Mat> true_R(count);
    std::vector<cv::Mat> true_t(count);
    const cv::Mat center0 = (cv::Mat_<double>(3, 1) << 0.08, 0.01, 0.0);
    for (size_t camera = 0; camera < static_cast<size_t>(count); ++camera) {
        true_R[camera] = rotationY(static_cast<double>(camera) * 90.0);
        const double angle = static_cast<double>(camera) * kPi / 2.0;
        const cv::Mat center =
            (cv::Mat_<double>(3, 1) << 0.08 * std::cos(angle), 0.01, 0.08 * std::sin(angle));
        // The world frame is camera 0, so camera 0's centre is the origin.
        true_t[camera] = center - center0;
        result.cameras[camera].intrinsics.K = cv::Mat::eye(3, 3, CV_64F);
    }

    int view_index = 0;
    for (int yaw = 0; yaw < 360; yaw += 15) {
        const cv::Mat R_yaw = rotationY(yaw);
        const cv::Mat board_center = R_yaw * (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.9);
        const cv::Mat flip =
            (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
        const cv::Mat R_board = R_yaw * flip;
        for (size_t camera = 0; camera < static_cast<size_t>(count); ++camera) {
            // true_R/true_t are camera -> world.  The board pose fed into the
            // estimator is board -> camera.
            const cv::Mat R_ic = true_R[camera].t();
            const cv::Mat t_ic = -R_ic * true_t[camera];
            hikquad::Pose pose;
            pose.R = R_ic * R_board;
            pose.t = R_ic * board_center + t_ic;
            hikquad::ViewObservation observation;
            observation.view_index = view_index;
            observation.detected = true;
            result.cameras[camera].observations.push_back(observation);
            result.cameras[camera].board_poses.push_back(pose);
        }
        ++view_index;
    }

    hikquad::estimateExtrinsics(result, count, 90.0);
    for (size_t camera = 0; camera < static_cast<size_t>(count); ++camera) {
        const cv::Mat delta = true_R[camera].t() * result.cameras[camera].extrinsics.R_cw;
        CHECK_NEAR(cv::norm(delta - cv::Mat::eye(3, 3, CV_64F)), 0.0, 1e-8);
        CHECK_NEAR(cv::norm(result.cameras[camera].extrinsics.T_cw - true_t[camera]), 0.0, 1e-8);
        CHECK_NEAR(result.cameras[camera].extrinsics.center_angle_deg,
                   static_cast<double>(camera) * 90.0, 1e-6);
    }
}
