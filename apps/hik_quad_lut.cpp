#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "app_common.hpp"
#include "hikquad/calib_io.hpp"
#include "hikquad/calibration.hpp"
#include "hikquad/config.hpp"
#include "hikquad/lut.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    hikquad::CliParser cli(argc, argv);
    const std::vector<std::string> allowed = {
        "--config", "--calib",        "--output",  "--count", "--pano",      "--fov",
        "--sigma",  "--nominal-step", "--verbose", "--quiet", "--log-level", "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Rebuild cylindrical lut.bin / weight.bin from existing intrinsics.yml and "
            "extrinsics.yml.\n\n"
            "  --calib DIR          calibration directory (default: data/calib)\n"
            "  --output DIR         output directory (default: same as --calib)\n"
            "  --count N            camera count (default: config)\n"
            "  --pano WxH           panorama size (default: 2048x512)\n"
            "  --fov DEG            horizontal field of view (default: 360)\n"
            "  --sigma DEG          angular blend sigma (default: 35)\n"
            "  --nominal-step DEG   yaw step when extrinsics.yml is missing (default: 90)\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        hikquad::AppConfig config = hikquad::loadConfig(hikquad::app::resolveConfigOrThrow(cli));
        std::string calib = cli.get("--calib");
        if (calib.empty()) {
            calib = hikquad::resolveConfigPath(config.source_path, "data/calib");
        }
        calib = fs::absolute(calib).string();
        std::string output = cli.get("--output", calib);
        output = fs::absolute(output).string();
        hikquad::app::ensureDirectory(output);

        const int count = cli.getInt("--count", config.array.camera_count);
        const cv::Size pano = hikquad::app::parseSize(
            cli.get("--pano"), cv::Size(config.panorama.out_width, config.panorama.out_height));
        const double fov = cli.getDouble("--fov", config.panorama.fov_h_deg);
        const double sigma = cli.getDouble("--sigma", config.panorama.blend_sigma_deg);
        const double step = cli.getDouble("--nominal-step", 90.0);

        for (int camera = 0; camera < count; ++camera) {
            const fs::path source_dir = fs::path(calib) / ("camera_" + std::to_string(camera));
            const fs::path target_dir = fs::path(output) / ("camera_" + std::to_string(camera));
            hikquad::app::ensureDirectory(target_dir.string());

            hikquad::Intrinsics intrinsics =
                hikquad::readIntrinsicsYml((source_dir / "intrinsics.yml").string());
            hikquad::Extrinsics extrinsics;
            const fs::path extrinsics_path = source_dir / "extrinsics.yml";
            if (fs::exists(extrinsics_path)) {
                extrinsics = hikquad::readExtrinsicsYml(extrinsics_path.string());
            } else {
                extrinsics.R_cw = hikquad::nominalRotationY(camera * step);
                extrinsics.T_cw = cv::Mat::zeros(3, 1, CV_64F);
                extrinsics.center_angle_deg = hikquad::opticalAxisAngleDeg(extrinsics.R_cw);
                extrinsics.has_angle = true;
                std::cerr << "warning: camera " << camera
                          << " has no extrinsics.yml; using nominal yaw " << camera * step
                          << " deg\n";
            }

            // Keep the output directory self-contained so hik_quad_stitch can
            // validate the source image size even when --output differs from --calib.
            hikquad::writeIntrinsicsYml((target_dir / "intrinsics.yml").string(), intrinsics);
            hikquad::writeExtrinsicsYml((target_dir / "extrinsics.yml").string(), extrinsics);

            const hikquad::Lut lut = hikquad::buildCylindricalLut(
                intrinsics, extrinsics, pano.width, pano.height, fov, sigma);
            hikquad::writeLutBin((target_dir / "lut.bin").string(), lut, camera);
            hikquad::writeWeightBin((target_dir / "weight.bin").string(), lut.weight, camera);
            const double valid = static_cast<double>(cv::countNonZero(lut.weight > 0.0F));
            std::cout << "camera " << camera << " -> " << target_dir.string() << " coverage "
                      << (100.0 * valid / (pano.width * pano.height)) << "%\n";
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
