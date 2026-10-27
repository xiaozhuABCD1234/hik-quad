#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
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
        "--config", "--input", "--output", "--pattern", "--square-mm", "--model",
        "--min-views", "--max-rms", "--pano", "--fov", "--sigma", "--horizontal",
        "--verbose", "--quiet", "--log-level", "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Calibrate the four cameras and write intrinsics, extrinsics and stitching LUTs.\n\n"
            "  --input DIR          view_000/cam0.png ... or cam0/*.png\n"
            "  --output DIR         calibration output (default: data/calib)\n"
            "  --pattern COLSxROWS  inner corners (default: 8x11)\n"
            "  --square-mm MM       square size (default: 100)\n"
            "  --model fisheye      fisheye or pinhole\n"
            "  --min-views N        warn below this many views (default: 10)\n"
            "  --max-rms PX         warn above this RMS (default: 1.0)\n"
            "  --pano WxH           panorama size (default: 2048x512)\n"
            "  --fov DEG            horizontal FOV (default: 360)\n"
            "  --sigma DEG          blend angular sigma (default: 35)\n"
            "  --horizontal         force yaw-only extrinsics for a horizontal ring\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        const hikquad::AppConfig config = hikquad::app::loadAppConfig(cli);

        hikquad::BoardSpec board = hikquad::app::parseBoard(
            cli.get("--pattern"),
            hikquad::BoardSpec{config.calibration.pattern_cols, config.calibration.pattern_rows,
                               config.calibration.square_mm});
        board.square_mm = cli.getDouble("--square-mm", board.square_mm);

        const std::string model_name = cli.get("--model", config.calibration.model);
        hikquad::CameraModel model = hikquad::CameraModel::Fisheye;
        if (model_name == "fisheye") {
            model = hikquad::CameraModel::Fisheye;
        } else if (model_name == "pinhole") {
            model = hikquad::CameraModel::Pinhole;
        } else {
            throw std::runtime_error("--model must be fisheye or pinhole");
        }

        const int min_views = cli.getInt("--min-views", config.calibration.min_views);
        const double max_rms = cli.getDouble("--max-rms", config.calibration.max_rms);
        const cv::Size pano_size = hikquad::app::parseSize(
            cli.get("--pano"), cv::Size(config.panorama.out_width, config.panorama.out_height));
        const double fov = cli.getDouble("--fov", config.panorama.fov_h_deg);
        const double sigma = cli.getDouble("--sigma", config.panorama.blend_sigma_deg);

        std::string input = cli.get("--input");
        if (input.empty()) {
            throw std::runtime_error("--input DIR is required");
        }
        input = fs::absolute(input).string();
        const std::string output =
            fs::absolute(hikquad::app::pathOption(cli, "--output", "data/calib")).string();

        const int camera_count = config.array.camera_count;
        std::cout << "board: " << board.cols << "x" << board.rows << " inner corners, "
                  << board.square_mm << "mm squares\n";
        std::cout << "input: " << input << "\n";
        std::cout << "output: " << output << "\n";

        const std::vector<hikquad::CalibrationView> views =
            hikquad::discoverViews(input, camera_count);
        std::cout << "discovered " << views.size() << " view(s)\n";

        std::vector<std::vector<hikquad::ViewObservation>> observations(
            static_cast<size_t>(camera_count));
        std::vector<cv::Size> image_sizes(static_cast<size_t>(camera_count), cv::Size());
        for (const auto& view : views) {
            int detected = 0;
            for (int camera = 0; camera < camera_count; ++camera) {
                const std::string& path = view.image_paths[static_cast<size_t>(camera)];
                if (path.empty()) {
                    continue;
                }
                const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
                if (image.empty()) {
                    std::cerr << "warning: cannot read " << path << "\n";
                    continue;
                }
                if (image_sizes[static_cast<size_t>(camera)].empty()) {
                    image_sizes[static_cast<size_t>(camera)] = image.size();
                }
                hikquad::ViewObservation observation;
                observation.view_index = view.index;
                observation.image_path = path;
                observation.detected =
                    hikquad::detectBoard(image, board, &observation.image_points);
                if (observation.detected) {
                    ++detected;
                }
                observations[static_cast<size_t>(camera)].push_back(std::move(observation));
            }
            std::cout << "  view " << std::setw(3) << std::setfill('0') << view.index << ": "
                      << detected << "/" << camera_count << " cameras detected\n";
        }

        hikquad::CalibrationResult result;
        result.board = board;
        result.camera_count = camera_count;
        result.cameras.resize(static_cast<size_t>(camera_count));

        for (int camera = 0; camera < camera_count; ++camera) {
            const auto& camera_observations = observations[static_cast<size_t>(camera)];
            int detected = 0;
            for (const auto& observation : camera_observations) {
                detected += observation.detected ? 1 : 0;
            }
            if (detected < 3) {
                throw std::runtime_error("camera " + std::to_string(camera) + " has only " +
                                         std::to_string(detected) +
                                         " usable view(s); capture more chessboard views");
            }
            if (detected < min_views) {
                std::cerr << "warning: camera " << camera << " has only " << detected << " views ("
                          << min_views << " recommended)\n";
            }
            if (image_sizes[static_cast<size_t>(camera)].empty()) {
                throw std::runtime_error("no readable image for camera " + std::to_string(camera));
            }
            try {
                result.cameras[static_cast<size_t>(camera)] =
                    hikquad::calibrateCamera(camera_observations, board, model,
                                             image_sizes[static_cast<size_t>(camera)].width,
                                             image_sizes[static_cast<size_t>(camera)].height);
            } catch (const std::exception& error) {
                throw std::runtime_error("camera " + std::to_string(camera) +
                                         " calibration failed: " + error.what());
            }
            if (result.cameras[static_cast<size_t>(camera)].intrinsics.rms > max_rms) {
                std::cerr << "warning: camera " << camera << " RMS "
                          << result.cameras[static_cast<size_t>(camera)].intrinsics.rms
                          << " exceeds --max-rms " << max_rms << "\n";
            }
        }

        hikquad::estimateExtrinsics(result, camera_count);
        if (cli.has("--horizontal")) {
            hikquad::applyHorizontalConstraint(result);
        }
        hikquad::app::ensureDirectory(output);

        for (int camera = 0; camera < camera_count; ++camera) {
            const fs::path camera_dir = fs::path(output) / ("camera_" + std::to_string(camera));
            hikquad::app::ensureDirectory(camera_dir.string());
            const auto& calibration = result.cameras[static_cast<size_t>(camera)];
            hikquad::writeIntrinsicsYml((camera_dir / "intrinsics.yml").string(),
                                        calibration.intrinsics);
            hikquad::writeExtrinsicsYml((camera_dir / "extrinsics.yml").string(),
                                        calibration.extrinsics);
            const hikquad::Lut lut =
                hikquad::buildCylindricalLut(calibration.intrinsics, calibration.extrinsics,
                                             pano_size.width, pano_size.height, fov, sigma);
            hikquad::writeLutBin((camera_dir / "lut.bin").string(), lut, camera);
            hikquad::writeWeightBin((camera_dir / "weight.bin").string(), lut.weight, camera);
            const double valid = static_cast<double>(cv::countNonZero(lut.weight > 0.0F));
            std::cout << "camera " << camera << " lut coverage "
                      << (100.0 * valid / (pano_size.width * pano_size.height)) << "%\n";
        }

        std::cout << "\n" << hikquad::formatCalibrationSummary(result);
        std::cout << "wrote calibration to " << output << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
