#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
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
        "--config",  "--input",     "--output",       "--pattern",   "--square-mm",
        "--model",   "--min-views", "--max-rms",      "--no-lut",    "--pano",
        "--fov",     "--sigma",     "--nominal-step", "--debug-dir", "--check",
        "--verbose", "--quiet",     "--log-level",    "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Calibrate a multi-camera ring from chessboard views and emit quad-camera compatible "
            "products.\n\n"
            "  --input DIR          view_000/cam0.png ... or cam0/*.png\n"
            "  --output DIR         calibration output (default: data/calib)\n"
            "  --pattern COLSxROWS  inner corners (default: 8x11)\n"
            "  --square-mm MM       square size (default: 100)\n"
            "  --model fisheye      fisheye or pinhole\n"
            "  --min-views N        quality gate (default: 10)\n"
            "  --max-rms PX         warn above this RMS (default: 1.0)\n"
            "  --no-lut             do not emit lut.bin / weight.bin\n"
            "  --pano WxH           panorama size (default: 2048x512)\n"
            "  --fov DEG            horizontal FOV (default: 360)\n"
            "  --sigma DEG          blend angular sigma (default: 35)\n"
            "  --nominal-step DEG   fallback yaw step (default: 90)\n"
            "  --debug-dir DIR      save corner overlay images\n"
            "  --check              only detect boards and print coverage\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        const std::string config_path = hikquad::app::resolveConfigOrThrow(cli);
        hikquad::AppConfig config = hikquad::loadConfig(config_path);

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
        const bool emit_lut = !cli.has("--no-lut") && config.calibration.emit_lut;
        const cv::Size pano_size = hikquad::app::parseSize(
            cli.get("--pano"), cv::Size(config.panorama.out_width, config.panorama.out_height));
        const double fov = cli.getDouble("--fov", config.panorama.fov_h_deg);
        const double sigma = cli.getDouble("--sigma", config.panorama.blend_sigma_deg);
        const double nominal_step = cli.getDouble("--nominal-step", 90.0);

        std::string input = cli.get("--input");
        if (input.empty()) {
            throw std::runtime_error("--input DIR is required");
        }
        input = fs::absolute(input).string();
        std::string output = cli.get("--output");
        if (output.empty()) {
            output = hikquad::resolveConfigPath(config_path, "data/calib");
        }
        output = fs::absolute(output).string();
        const std::string debug_dir = cli.get("--debug-dir");
        if (!debug_dir.empty()) {
            hikquad::app::ensureDirectory(fs::absolute(debug_dir).string());
        }

        const int camera_count = config.array.camera_count;
        if (camera_count <= 0) {
            throw std::runtime_error("camera_count must be positive");
        }
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
                    if (!debug_dir.empty()) {
                        cv::Mat overlay = image.clone();
                        cv::drawChessboardCorners(overlay, board.patternSize(),
                                                  observation.image_points, true);
                        std::ostringstream name;
                        name << "view_" << std::setw(3) << std::setfill('0') << view.index << "_cam"
                             << camera << ".png";
                        cv::imwrite((fs::path(debug_dir) / name.str()).string(), overlay);
                    }
                }
                observations[static_cast<size_t>(camera)].push_back(std::move(observation));
            }
            std::cout << "  view " << std::setw(3) << std::setfill('0') << view.index << ": "
                      << detected << "/" << camera_count << " cameras detected\n";
        }

        if (cli.has("--check")) {
            int usable = 0;
            for (int camera = 0; camera < camera_count; ++camera) {
                int detected = 0;
                for (const auto& observation : observations[static_cast<size_t>(camera)]) {
                    detected += observation.detected ? 1 : 0;
                }
                usable += detected;
                std::cout << "camera " << camera << ": " << detected << " detected view(s)\n";
            }
            std::cout << "check complete: " << usable << " detection(s)\n";
            return usable > 0 ? 0 : 1;
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
            const auto& intrinsics = result.cameras[static_cast<size_t>(camera)].intrinsics;
            std::cout << "camera " << camera << ": views=" << intrinsics.view_count
                      << " rms=" << intrinsics.rms << " fx=" << intrinsics.K.at<double>(0, 0)
                      << " fy=" << intrinsics.K.at<double>(1, 1)
                      << " cx=" << intrinsics.K.at<double>(0, 2)
                      << " cy=" << intrinsics.K.at<double>(1, 2) << "\n";
            if (intrinsics.rms > max_rms) {
                std::cerr << "warning: camera " << camera << " RMS " << intrinsics.rms
                          << " exceeds --max-rms " << max_rms << "\n";
            }
        }

        hikquad::estimateExtrinsics(result, camera_count, nominal_step);
        hikquad::app::ensureDirectory(output);

        for (int camera = 0; camera < camera_count; ++camera) {
            const fs::path camera_dir = fs::path(output) / ("camera_" + std::to_string(camera));
            hikquad::app::ensureDirectory(camera_dir.string());
            const auto& calibration = result.cameras[static_cast<size_t>(camera)];
            hikquad::writeIntrinsicsYml((camera_dir / "intrinsics.yml").string(),
                                        calibration.intrinsics);
            hikquad::writeExtrinsicsYml((camera_dir / "extrinsics.yml").string(),
                                        calibration.extrinsics);
            if (emit_lut) {
                const hikquad::Lut lut =
                    hikquad::buildCylindricalLut(calibration.intrinsics, calibration.extrinsics,
                                                 pano_size.width, pano_size.height, fov, sigma);
                hikquad::writeLutBin((camera_dir / "lut.bin").string(), lut, camera);
                hikquad::writeWeightBin((camera_dir / "weight.bin").string(), lut.weight, camera);
                const double valid = static_cast<double>(cv::countNonZero(lut.weight > 0.0F));
                std::cout << "camera " << camera << " lut coverage "
                          << (100.0 * valid / (pano_size.width * pano_size.height)) << "%\n";
            }
        }

        hikquad::PanoramaConfig panorama;
        panorama.out_width = pano_size.width;
        panorama.out_height = pano_size.height;
        panorama.fov_h_deg = fov;
        panorama.blend_sigma_deg = sigma;
        hikquad::writeCalibMetaJson((fs::path(output) / "calib_meta.json").string(), camera_count,
                                    board, panorama, result);
        hikquad::writeCalibrationReportJson((fs::path(output) / "report.json").string(), result);

        std::cout << "\n" << hikquad::formatCalibrationSummary(result);
        std::cout << "wrote quad-camera compatible calibration to " << output << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
