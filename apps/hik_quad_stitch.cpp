#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "app_common.hpp"
#include "hikquad/calib_io.hpp"
#include "hikquad/config.hpp"
#include "hikquad/lut.hpp"
#include "hikquad/media.hpp"
#include "hikquad/stitcher.hpp"

namespace fs = std::filesystem;

namespace {

int videoFourcc(const std::string& path) {
    if (hikquad::lowerExtension(path) == ".avi") {
        return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    }
    return cv::VideoWriter::fourcc('m', 'p', '4', 'v');
}

/// Fallback for calibration directories written without weight.bin.
cv::Mat weightFromLut(const hikquad::Lut& lut) {
    const cv::Mat mask = lut.mapx >= 0.0F;  // 0 or 255
    cv::Mat weight;
    mask.convertTo(weight, CV_32F, 1.0 / 255.0);
    return weight;
}

void requireSize(const std::string& path, const cv::Size& actual, const cv::Size& expected) {
    if (actual == expected) {
        return;
    }
    throw std::runtime_error(path + " is " + std::to_string(actual.width) + "x" +
                             std::to_string(actual.height) + " but calibration expects " +
                             std::to_string(expected.width) + "x" +
                             std::to_string(expected.height));
}

}  // namespace

int main(int argc, char** argv) {
    hikquad::CliParser cli(argc, argv);
    const std::vector<std::string> allowed = {"--config", "--calib", "--input", "--out",
                                              "--fps",    "--verbose", "--quiet", "--log-level",
                                              "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Stitch four cameras into one panorama.\n\n"
            "  --calib DIR   calibration directory written by hik_quad_calib (default: data/calib)\n"
            "  --input DIR   cam0.png ... (single image) or cam0.mp4 ... (whole video)\n"
            "  --out FILE    panorama.jpg or panorama.mp4\n"
            "  --fps FPS     output video frame rate (default: source video rate)\n\n"
            "The output type follows --out: an image extension stitches one frame, a video\n"
            "extension stitches every frame of the four input videos.\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        const hikquad::AppConfig config = hikquad::app::loadAppConfig(cli);
        const int count = config.array.camera_count;
        if (count <= 0) {
            throw std::runtime_error("config has no cameras");
        }

        const std::string calib =
            fs::absolute(hikquad::app::pathOption(cli, "--calib", "data/calib")).string();

        std::string input = cli.get("--input");
        if (input.empty()) {
            throw std::runtime_error("--input DIR is required");
        }
        input = fs::absolute(input).string();

        std::vector<hikquad::CameraMedia> sources(static_cast<size_t>(count));
        hikquad::MediaKind kind = hikquad::MediaKind::None;
        for (int camera = 0; camera < count; ++camera) {
            hikquad::CameraMedia& source = sources[static_cast<size_t>(camera)];
            source = hikquad::findCameraMedia(input, camera);
            if (source.empty()) {
                throw std::runtime_error("no cam" + std::to_string(camera) +
                                         ".{png,jpg,mp4,...} in " + input);
            }
            if (kind == hikquad::MediaKind::None) {
                kind = source.kind;
            } else if (source.kind != kind) {
                throw std::runtime_error(
                    "input mixes images and videos; use one type for all four cameras");
            }
        }
        const bool video_input = kind == hikquad::MediaKind::Video;

        const std::string output = fs::absolute(hikquad::app::pathOption(
                                      cli, "--out",
                                      video_input ? "output/panorama.mp4"
                                                  : "output/panorama.jpg"))
                                       .string();
        const bool video_output =
            hikquad::mediaKind(output) == hikquad::MediaKind::Video;
        if (video_input != video_output) {
            throw std::runtime_error(video_input ? "video input requires a video --out (.mp4 or .avi)"
                                                 : "image input requires an image --out (.jpg or .png)");
        }
        hikquad::app::ensureDirectory(fs::path(output).parent_path().string());

        hikquad::StitchInput stitch_input;
        std::vector<cv::Size> expected_sizes(static_cast<size_t>(count));
        for (int camera = 0; camera < count; ++camera) {
            const fs::path camera_dir = fs::path(calib) / ("camera_" + std::to_string(camera));
            const hikquad::Intrinsics intrinsics =
                hikquad::readIntrinsicsYml((camera_dir / "intrinsics.yml").string());
            expected_sizes[static_cast<size_t>(camera)] =
                cv::Size(intrinsics.width, intrinsics.height);

            hikquad::Lut lut = hikquad::readLutBin((camera_dir / "lut.bin").string(), camera);
            if (camera == 0) {
                stitch_input.out_width = lut.width;
                stitch_input.out_height = lut.height;
            } else if (stitch_input.out_width != lut.width ||
                       stitch_input.out_height != lut.height) {
                throw std::runtime_error("LUT sizes differ between cameras");
            }

            const fs::path weight_path = camera_dir / "weight.bin";
            cv::Mat weight = fs::exists(weight_path)
                                 ? hikquad::readWeightBin(weight_path.string(), camera)
                                 : weightFromLut(lut);
            stitch_input.mapx.push_back(std::move(lut.mapx));
            stitch_input.mapy.push_back(std::move(lut.mapy));
            stitch_input.weight.push_back(std::move(weight));
        }

        if (!video_input) {
            std::vector<cv::Mat> images(static_cast<size_t>(count));
            for (int camera = 0; camera < count; ++camera) {
                const hikquad::CameraMedia& source = sources[static_cast<size_t>(camera)];
                images[static_cast<size_t>(camera)] = cv::imread(source.path, cv::IMREAD_COLOR);
                if (images[static_cast<size_t>(camera)].empty()) {
                    throw std::runtime_error("cannot read image: " + source.path);
                }
                requireSize(source.path, images[static_cast<size_t>(camera)].size(),
                            expected_sizes[static_cast<size_t>(camera)]);
            }
            stitch_input.images = std::move(images);
            const auto start = std::chrono::steady_clock::now();
            const cv::Mat panorama = hikquad::stitchFrames(stitch_input);
            const double elapsed = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - start)
                                       .count();
            const std::vector<int> parameters = {cv::IMWRITE_JPEG_QUALITY, 95};
            if (!cv::imwrite(output, panorama, parameters)) {
                throw std::runtime_error("cannot write panorama: " + output);
            }
            std::cout << "stitched " << count << " cameras into " << stitch_input.out_width << "x"
                      << stitch_input.out_height << " in " << elapsed << " ms\n";
            std::cout << "saved " << output << "\n";
            return 0;
        }

        std::vector<cv::VideoCapture> captures(static_cast<size_t>(count));
        for (int camera = 0; camera < count; ++camera) {
            const std::string& path = sources[static_cast<size_t>(camera)].path;
            if (!captures[static_cast<size_t>(camera)].open(path)) {
                throw std::runtime_error("cannot open video: " + path);
            }
        }
        double fps = cli.getDouble("--fps", 0.0);
        if (fps <= 0.0) {
            fps = captures.front().get(cv::CAP_PROP_FPS);
        }
        if (fps <= 0.0) {
            fps = 20.0;
        }
        cv::VideoWriter writer(output, videoFourcc(output), fps,
                               cv::Size(stitch_input.out_width, stitch_input.out_height));
        if (!writer.isOpened()) {
            throw std::runtime_error("cannot open video writer: " + output);
        }

        int frames = 0;
        const auto start = std::chrono::steady_clock::now();
        while (true) {
            std::vector<cv::Mat> images(static_cast<size_t>(count));
            bool complete = true;
            for (int camera = 0; camera < count; ++camera) {
                if (!captures[static_cast<size_t>(camera)].read(images[static_cast<size_t>(camera)]) ||
                    images[static_cast<size_t>(camera)].empty()) {
                    complete = false;
                    break;
                }
            }
            if (!complete) {
                break;
            }
            if (frames == 0) {
                for (int camera = 0; camera < count; ++camera) {
                    requireSize(sources[static_cast<size_t>(camera)].path,
                                images[static_cast<size_t>(camera)].size(),
                                expected_sizes[static_cast<size_t>(camera)]);
                }
            }
            stitch_input.images = std::move(images);
            writer.write(hikquad::stitchFrames(stitch_input));
            ++frames;
            if (frames % 30 == 0) {
                std::cout << "stitched " << frames << " frame(s)\r" << std::flush;
            }
        }
        writer.release();
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "stitched " << frames << " frame(s) into " << stitch_input.out_width << "x"
                  << stitch_input.out_height << " at " << fps << " fps in " << elapsed << " s\n";
        std::cout << "saved " << output << "\n";
        return frames > 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
