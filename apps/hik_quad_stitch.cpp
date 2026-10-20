#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "app_common.hpp"
#include "hikquad/calib_io.hpp"
#include "hikquad/config.hpp"
#include "hikquad/lut.hpp"
#include "hikquad/stitcher.hpp"

namespace fs = std::filesystem;

namespace {

std::string findImage(const std::string& directory, int camera) {
    const std::vector<std::string> stems = {"cam" + std::to_string(camera), std::to_string(camera),
                                            "cam_" + std::to_string(camera)};
    const std::vector<std::string> extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"};
    for (const auto& stem : stems) {
        for (const auto& extension : extensions) {
            const fs::path candidate = fs::path(directory) / (stem + extension);
            if (fs::exists(candidate)) {
                return candidate.string();
            }
        }
    }
    return {};
}

}  // namespace

int main(int argc, char** argv) {
    hikquad::CliParser cli(argc, argv);
    const std::vector<std::string> allowed = {"--config",    "--calib",   "--frames",  "--out",
                                              "--count",     "--quality", "--verbose", "--quiet",
                                              "--log-level", "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Stitch one synchronized frame set with the LUTs produced by hik_quad_calib / "
            "hik_quad_lut.\n\n"
            "  --calib DIR     calibration directory (default: data/calib)\n"
            "  --frames DIR    source frames: cam0.png ... or 0.bmp ...\n"
            "  --out FILE      output image (default: output/panorama.jpg)\n"
            "  --count N       camera count (default: config)\n"
            "  --quality N     JPEG quality (default: 95)\n");
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
        std::string frames = cli.get("--frames");
        if (frames.empty()) {
            throw std::runtime_error("--frames DIR is required");
        }
        frames = fs::absolute(frames).string();
        std::string output = cli.get("--out");
        if (output.empty()) {
            output = hikquad::resolveConfigPath(config.source_path, "output/panorama.jpg");
        }
        output = fs::absolute(output).string();
        hikquad::app::ensureDirectory(fs::path(output).parent_path().string());

        const int count = cli.getInt("--count", config.array.camera_count);
        const int quality = cli.getInt("--quality", 95);
        std::vector<cv::Mat> images;
        std::vector<cv::Mat> mapx;
        std::vector<cv::Mat> mapy;
        std::vector<cv::Mat> weights;
        int width = 0;
        int height = 0;

        for (int camera = 0; camera < count; ++camera) {
            const fs::path camera_dir = fs::path(calib) / ("camera_" + std::to_string(camera));
            const hikquad::Intrinsics intrinsics =
                hikquad::readIntrinsicsYml((camera_dir / "intrinsics.yml").string());
            hikquad::Lut lut = hikquad::readLutBin((camera_dir / "lut.bin").string(), camera);
            if (width == 0) {
                width = lut.width;
                height = lut.height;
            } else if (width != lut.width || height != lut.height) {
                throw std::runtime_error("LUT sizes differ between cameras");
            }
            cv::Mat weight;
            const fs::path weight_path = camera_dir / "weight.bin";
            if (fs::exists(weight_path)) {
                weight = hikquad::readWeightBin(weight_path.string(), camera);
            } else {
                weight = cv::Mat(lut.height, lut.width, CV_32F, cv::Scalar(0.0F));
                for (int row = 0; row < lut.height; ++row) {
                    for (int column = 0; column < lut.width; ++column) {
                        if (lut.mapx.at<float>(row, column) >= 0.0F) {
                            weight.at<float>(row, column) = 1.0F;
                        }
                    }
                }
            }
            const std::string image_path = findImage(frames, camera);
            if (image_path.empty()) {
                throw std::runtime_error("no image for camera " + std::to_string(camera) + " in " +
                                         frames);
            }
            cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
            if (image.empty()) {
                throw std::runtime_error("cannot read image: " + image_path);
            }
            if (image.cols != intrinsics.width || image.rows != intrinsics.height) {
                throw std::runtime_error(
                    "image " + image_path + " is " + std::to_string(image.cols) + "x" +
                    std::to_string(image.rows) + " but calibration expects " +
                    std::to_string(intrinsics.width) + "x" + std::to_string(intrinsics.height));
            }
            images.push_back(std::move(image));
            mapx.push_back(std::move(lut.mapx));
            mapy.push_back(std::move(lut.mapy));
            weights.push_back(std::move(weight));
        }

        const auto start = std::chrono::steady_clock::now();
        const cv::Mat panorama =
            hikquad::stitchWithLuts(images, mapx, mapy, weights, width, height);
        const double elapsed =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();

        std::vector<int> parameters = {cv::IMWRITE_JPEG_QUALITY, quality};
        if (!cv::imwrite(output, panorama, parameters)) {
            throw std::runtime_error("cannot write panorama: " + output);
        }
        std::cout << "stitched " << count << " cameras into " << width << "x" << height << " in "
                  << elapsed << " ms\n";
        std::cout << "saved " << output << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
