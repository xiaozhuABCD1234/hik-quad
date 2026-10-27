#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "app_common.hpp"
#include "hikquad/camera_array.hpp"
#include "hikquad/config.hpp"

namespace fs = std::filesystem;

namespace {

void saveFrameSet(const hikquad::FrameSet& set, const std::string& directory,
                  const std::string& image_format, bool save_raw,
                  const std::vector<hikquad::CameraSettings>& settings) {
    hikquad::app::ensureDirectory(directory);
    const std::string extension = hikquad::app::imageExtension(image_format);

    for (const auto& frame : set.frames) {
        if (!frame.bgr.empty()) {
            const std::string path =
                (fs::path(directory) / ("cam" + std::to_string(frame.camera_index) + extension))
                    .string();
            if (!cv::imwrite(path, frame.bgr)) {
                throw std::runtime_error("cannot write image: " + path);
            }
        }
        if (save_raw && !frame.raw.empty()) {
            std::string raw_path;
            if (frame.raw.type() == CV_8UC1 || frame.raw.type() == CV_8UC3 ||
                frame.raw.type() == CV_16UC1) {
                raw_path = (fs::path(directory) /
                            ("cam" + std::to_string(frame.camera_index) + "_raw.png"))
                               .string();
                if (!cv::imwrite(raw_path, frame.raw)) {
                    throw std::runtime_error("cannot write raw image: " + raw_path);
                }
            } else {
                raw_path = (fs::path(directory) /
                            ("cam" + std::to_string(frame.camera_index) + "_raw.bin"))
                               .string();
                std::ofstream stream(raw_path, std::ios::binary);
                stream.write(
                    reinterpret_cast<const char*>(frame.raw.data),
                    static_cast<std::streamsize>(frame.raw.total() * frame.raw.elemSize()));
                if (!stream.good()) {
                    throw std::runtime_error("cannot write raw buffer: " + raw_path);
                }
            }
        }
    }

    const std::string meta_path = (fs::path(directory) / "meta.json").string();
    cv::FileStorage storage(meta_path,
                            cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot write metadata: " + meta_path);
    }
    storage << "index" << static_cast<int>(set.index);
    storage << "skew_ms" << set.skew_ms;
    storage << "frames" << "[";
    for (const auto& frame : set.frames) {
        storage << "{";
        storage << "camera_index" << frame.camera_index;
        storage << "serial" << frame.serial;
        storage << "width" << static_cast<int>(frame.width);
        storage << "height" << static_cast<int>(frame.height);
        storage << "pixel_format" << frame.source_pixel_format;
        storage << "host_timestamp_ms" << static_cast<double>(frame.host_timestamp_ms);
        storage << "device_timestamp_ticks" << static_cast<double>(frame.device_timestamp_ticks);
        storage << "exposure_us" << frame.exposure_us;
        storage << "gain_db" << frame.gain_db;
        storage << "transport_bytes" << static_cast<double>(frame.transport_size);
        storage << "expected_bytes" << static_cast<double>(frame.expected_size);
        storage << "lost_packets" << static_cast<int>(frame.lost_packets);
        storage << "complete" << (frame.complete ? 1 : 0);
        storage << "image" << ("cam" + std::to_string(frame.camera_index) + extension);
        storage << "}";
    }
    storage << "]";
    storage << "camera_settings" << "[";
    for (const auto& camera : settings) {
        storage << "{";
        storage << "role" << camera.role;
        storage << "serial" << camera.serial;
        storage << "model" << camera.model;
        storage << "width" << camera.width;
        storage << "height" << camera.height;
        storage << "pixel_format" << camera.pixel_format;
        storage << "exposure_us" << camera.exposure_us;
        storage << "gain_db" << camera.gain_db;
        storage << "frame_rate" << camera.frame_rate;
        storage << "resulting_frame_rate" << camera.resulting_frame_rate;
        storage << "usb_transfer_ways" << static_cast<int>(camera.usb_transfer_ways);
        storage << "usb_transfer_size" << static_cast<int>(camera.usb_transfer_size);
        storage << "}";
    }
    storage << "]";
    storage.release();
}

std::string frameDirectory(const std::string& output, int index) {
    std::ostringstream name;
    name << "frame_" << std::setw(3) << std::setfill('0') << index;
    return (fs::path(output) / name.str()).string();
}

}  // namespace

int main(int argc, char** argv) {
    hikquad::CliParser cli(argc, argv);
    const std::vector<std::string> allowed = {
        "--config", "--frames", "--interval", "--out", "--exposure", "--gain", "--fps",
        "--pixel-format", "--width", "--height", "--format", "--raw", "--video", "--serial",
        "--software-trigger", "--no-save", "--verbose", "--quiet", "--log-level", "--help"};
    std::string cli_error;
    if (!cli.validate(allowed, &cli_error)) {
        std::cerr << cli_error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "Capture frames or four synchronized videos from the configured cameras.\n\n"
            "  --config FILE        config file (default: config/default.json)\n"
            "  --frames N           frame sets to capture (default: config)\n"
            "  --interval SEC       delay between frame sets (images only)\n"
            "  --out DIR            output directory\n"
            "  --video              write cam0.mp4 ... cam3.mp4 instead of images\n"
            "  --fps FPS            camera and video frame rate\n"
            "  --exposure US        exposure time for every camera\n"
            "  --gain DB            gain for every camera\n"
            "  --pixel-format NAME  BayerRG8 (default), BGR8, RGB8, ...\n"
            "  --width W --height H override ROI\n"
            "  --serial S           select camera by serial (repeatable, fixes order)\n"
            "  --software-trigger   trigger all cameras through TriggerSoftware\n"
            "  --raw                also save the transport buffer (images only)\n"
            "  --format png|jpg|bmp output image encoding (images only)\n"
            "  --no-save            grab and report without writing files\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        hikquad::AppConfig config = hikquad::app::loadAppConfig(cli);

        const std::vector<std::string> serials = cli.getAll("--serial");
        if (!serials.empty()) {
            config.array.camera_count = static_cast<int>(serials.size());
            config.array.cameras.resize(serials.size());
            for (size_t i = 0; i < serials.size(); ++i) {
                config.array.cameras[i].serial = serials[i];
                config.array.cameras[i].index = -1;
            }
        }

        const int frames = cli.getInt("--frames", config.capture.frames);
        if (frames <= 0) {
            throw std::runtime_error("--frames must be positive");
        }
        const double interval = cli.getDouble("--interval", config.capture.interval_s);
        const double exposure = cli.getDouble("--exposure", -1.0);
        const double gain = cli.getDouble("--gain", -1.0);
        const double fps = cli.getDouble("--fps", -1.0);
        const int width = cli.getInt("--width", 0);
        const int height = cli.getInt("--height", 0);
        const std::string pixel_format = cli.get("--pixel-format");
        const bool raw = cli.has("--raw") || config.capture.save_raw;
        const bool video = cli.has("--video");
        const bool software_trigger =
            cli.has("--software-trigger") || config.capture.software_trigger;
        const std::string format = cli.get("--format", config.capture.image_format);
        const bool save = !cli.has("--no-save");

        if (video && raw) {
            std::cerr << "warning: --raw is ignored with --video\n";
        }
        if (video && interval > 0.0) {
            std::cerr << "warning: --interval is ignored with --video\n";
        }

        for (auto& camera : config.array.cameras) {
            if (exposure > 0.0) {
                camera.exposure_us = exposure;
            }
            if (gain >= 0.0) {
                camera.gain_db = gain;
            }
            if (fps > 0.0) {
                camera.frame_rate_enable = true;
                camera.frame_rate = fps;
            }
            if (width > 0 && height > 0) {
                camera.width = width;
                camera.height = height;
            }
            if (!pixel_format.empty()) {
                camera.pixel_format = pixel_format;
            }
            camera.keep_raw = raw && !video;
            if (software_trigger) {
                camera.trigger_mode = "On";
                camera.trigger_source = "Software";
            }
        }
        config.array.software_trigger = software_trigger;
        config.array.camera_count = static_cast<int>(config.array.cameras.size());
        if (software_trigger) {
            HIKQUAD_LOG(Warn)
                << "software trigger is timing sensitive on a shared USB3 hub; if frames are "
                   "reported incomplete, use free-run at 20 fps or a hardware trigger";
        }

        const std::string output =
            fs::absolute(hikquad::app::pathOption(cli, "--out", config.capture.output_dir))
                .string();
        if (save) {
            hikquad::app::ensureDirectory(output);
        }

        hikquad::CameraArray array;
        array.open(config.array);
        const std::vector<hikquad::CameraSettings> settings = array.settings();
        double video_fps = 20.0;
        for (const auto& camera : settings) {
            std::cout << "camera " << camera.role << " [" << camera.serial << "]: " << camera.width
                      << "x" << camera.height << " " << camera.pixel_format
                      << " exp=" << camera.exposure_us << "us gain=" << camera.gain_db
                      << "dB fps=" << camera.frame_rate
                      << " resulting=" << camera.resulting_frame_rate
                      << " usb_ways=" << camera.usb_transfer_ways
                      << " usb_transfer=" << camera.usb_transfer_size << "\n";
            if (camera.resulting_frame_rate > 0.0) {
                video_fps = camera.resulting_frame_rate;
            } else if (camera.frame_rate > 0.0) {
                video_fps = camera.frame_rate;
            }
        }

        std::vector<std::unique_ptr<cv::VideoWriter>> writers;
        for (int i = 0; i < frames; ++i) {
            const hikquad::FrameSet set = array.captureOnce();
            std::cout << hikquad::formatFrameSetSummary(set) << "\n";

            if (save && video) {
                if (writers.empty()) {
                    for (const auto& frame : set.frames) {
                        const std::string path = (fs::path(output) /
                                                  ("cam" + std::to_string(frame.camera_index) +
                                                   ".mp4"))
                                                     .string();
                        auto writer = std::make_unique<cv::VideoWriter>(
                            path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), video_fps,
                            frame.bgr.size());
                        if (!writer->isOpened()) {
                            throw std::runtime_error("cannot open video writer: " + path);
                        }
                        writers.push_back(std::move(writer));
                    }
                }
                for (const auto& frame : set.frames) {
                    if (frame.camera_index < 0 ||
                        static_cast<size_t>(frame.camera_index) >= writers.size() ||
                        frame.bgr.empty()) {
                        continue;
                    }
                    writers[static_cast<size_t>(frame.camera_index)]->write(frame.bgr);
                }
            } else if (save) {
                const std::string directory =
                    frames == 1 ? output : frameDirectory(output, i);
                saveFrameSet(set, directory, format, raw, settings);
                std::cout << "saved " << directory << "\n";
            }

            if (!video && interval > 0.0 && i + 1 < frames) {
                std::this_thread::sleep_for(std::chrono::duration<double>(interval));
            }
        }

        array.stop();
        writers.clear();
        if (save && video) {
            std::cout << "saved " << settings.size() << " videos to " << output << " at "
                      << video_fps << " fps\n";
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
