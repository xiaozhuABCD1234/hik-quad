#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>

#include "hikquad/calib_io.hpp"
#include "hikquad/calibration.hpp"
#include "hikquad/lut.hpp"
#include "test_framework.hpp"

HIK_TEST(calibration_yml_roundtrip) {
    hikquad::Intrinsics intrinsics;
    intrinsics.model = hikquad::CameraModel::Fisheye;
    intrinsics.K = (cv::Mat_<double>(3, 3) << 450.0, 0.0, 720.0, 0.0, 451.0, 540.0, 0.0, 0.0, 1.0);
    intrinsics.D = (cv::Mat_<double>(1, 4) << -0.04, -0.007, 0.0001, 0.0002);
    intrinsics.width = 1440;
    intrinsics.height = 1080;
    intrinsics.board = hikquad::BoardSpec{8, 11, 100.0};
    intrinsics.circle_center = cv::Point2d(720.0, 540.0);
    intrinsics.circle_radius = 520.0;
    intrinsics.fold_radius = 1.15;
    intrinsics.rms = 0.42;
    intrinsics.mean_error = 0.39;

    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "hikquad_test_calib_io";
    std::filesystem::create_directories(directory);
    const std::string path = (directory / "intrinsics.yml").string();
    hikquad::writeIntrinsicsYml(path, intrinsics);
    const hikquad::Intrinsics loaded = hikquad::readIntrinsicsYml(path);
    CHECK_EQ(loaded.model, hikquad::CameraModel::Fisheye);
    CHECK_EQ(loaded.width, 1440);
    CHECK_EQ(loaded.board.cols, 8);
    CHECK_NEAR(loaded.K.at<double>(0, 0), 450.0, 1e-9);
    CHECK_NEAR(loaded.D.at<double>(0, 0), -0.04, 1e-9);
    CHECK_NEAR(loaded.circle_radius, 520.0, 1e-9);

    hikquad::Extrinsics extrinsics;
    extrinsics.R_cw = hikquad::nominalRotationY(90.0);
    extrinsics.T_cw = (cv::Mat_<double>(3, 1) << 0.1, 0.2, 0.3);
    extrinsics.center_angle_deg = 90.0;
    extrinsics.has_angle = true;
    const std::string extrinsics_path = (directory / "extrinsics.yml").string();
    hikquad::writeExtrinsicsYml(extrinsics_path, extrinsics);
    const hikquad::Extrinsics loaded_extrinsics = hikquad::readExtrinsicsYml(extrinsics_path);
    CHECK_NEAR(loaded_extrinsics.center_angle_deg, 90.0, 1e-9);
    CHECK_NEAR(loaded_extrinsics.T_cw.at<double>(2, 0), 0.3, 1e-9);
    std::filesystem::remove_all(directory);
}

HIK_TEST(lut_binary_roundtrip) {
    hikquad::Lut lut;
    lut.width = 16;
    lut.height = 8;
    lut.mapx = cv::Mat(8, 16, CV_32F, cv::Scalar(-1.0F));
    lut.mapy = cv::Mat(8, 16, CV_32F, cv::Scalar(-1.0F));
    lut.mapx.at<float>(2, 3) = 12.5F;
    lut.mapy.at<float>(2, 3) = 4.25F;
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "hikquad_test_lut.bin";
    hikquad::writeLutBin(path.string(), lut, 2);
    const hikquad::Lut loaded = hikquad::readLutBin(path.string(), 2);
    CHECK_EQ(loaded.width, 16);
    CHECK_EQ(loaded.height, 8);
    CHECK_NEAR(loaded.mapx.at<float>(2, 3), 12.5, 1e-6);
    CHECK_NEAR(loaded.mapy.at<float>(2, 3), 4.25, 1e-6);
    std::filesystem::remove(path);
}

HIK_TEST(discover_views_ignores_non_view_directories) {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "hikquad_test_discover_views";
    std::filesystem::remove_all(root);
    for (const char* directory : {"view_000", "frames"}) {
        const std::filesystem::path path = root / directory;
        std::filesystem::create_directories(path);
        for (int camera = 0; camera < 4; ++camera) {
            std::ofstream stream(path / ("cam" + std::to_string(camera) + ".png"));
            stream << "placeholder";
        }
    }
    const std::vector<hikquad::CalibrationView> views = hikquad::discoverViews(root.string(), 4);
    CHECK_EQ(views.size(), size_t(1));
    CHECK_TRUE(views[0].image_paths[0].find("view_000") != std::string::npos);
    std::filesystem::remove_all(root);
}
