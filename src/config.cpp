#include "hikquad/config.hpp"

#include <filesystem>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

namespace hikquad {
namespace fs = std::filesystem;

namespace {

void readInt(const cv::FileNode& node, int* value) {
    if (node.empty() || !(node.isInt() || node.isReal())) {
        return;
    }
    double parsed = 0.0;
    node >> parsed;
    *value = static_cast<int>(parsed);
}

void readDouble(const cv::FileNode& node, double* value) {
    if (node.empty() || !(node.isInt() || node.isReal())) {
        return;
    }
    node >> *value;
}

void readBool(const cv::FileNode& node, bool* value) {
    if (node.empty()) {
        return;
    }
    if (node.isString()) {
        std::string text;
        node >> text;
        *value = text == "true" || text == "1" || text == "True";
        return;
    }
    if (!node.isInt()) {
        return;
    }
    int parsed = 0;
    node >> parsed;
    *value = parsed != 0;
}

void readString(const cv::FileNode& node, std::string* value) {
    if (node.empty() || !node.isString()) {
        return;
    }
    node >> *value;
}

void readCamera(const cv::FileNode& node, CameraConfig* camera) {
    if (node.empty() || !node.isMap()) {
        return;
    }
    readString(node["serial"], &camera->serial);
    readString(node["role"], &camera->role);
    readString(node["pixel_format"], &camera->pixel_format);
    readString(node["adc_bit_depth"], &camera->adc_bit_depth);
    readString(node["exposure_auto"], &camera->exposure_auto);
    readString(node["gain_auto"], &camera->gain_auto);
    readString(node["white_balance_auto"], &camera->white_balance_auto);
    readString(node["trigger_mode"], &camera->trigger_mode);
    readString(node["trigger_source"], &camera->trigger_source);
    readString(node["trigger_activation"], &camera->trigger_activation);
    readString(node["grab_strategy"], &camera->grab_strategy);
    readInt(node["width"], &camera->width);
    readInt(node["height"], &camera->height);
    readInt(node["offset_x"], &camera->offset_x);
    readInt(node["offset_y"], &camera->offset_y);
    readInt(node["binning"], &camera->binning);
    readInt(node["decimation"], &camera->decimation);
    readInt(node["bayer_conversion_quality"], &camera->bayer_conversion_quality);
    readInt(node["image_node_num"], &camera->image_node_num);
    readInt(node["usb_transfer_ways"], &camera->usb_transfer_ways);
    readInt(node["usb_transfer_size"], &camera->usb_transfer_size);
    readInt(node["transfer_size"], &camera->transfer_size);
    readInt(node["grab_timeout_ms"], &camera->grab_timeout_ms);
    readInt(node["incomplete_frame_retries"], &camera->incomplete_frame_retries);
    readBool(node["reverse_x"], &camera->reverse_x);
    readBool(node["reverse_y"], &camera->reverse_y);
    readBool(node["keep_raw"], &camera->keep_raw);
    readBool(node["frame_rate_enable"], &camera->frame_rate_enable);
    readBool(node["gamma_enable"], &camera->gamma_enable);
    readBool(node["throughput_limit_mode"], &camera->throughput_limit_mode);
    readDouble(node["frame_rate"], &camera->frame_rate);
    readDouble(node["exposure_us"], &camera->exposure_us);
    readDouble(node["gain_db"], &camera->gain_db);
    readDouble(node["balance_ratio_red"], &camera->balance_ratio_red);
    readDouble(node["balance_ratio_green"], &camera->balance_ratio_green);
    readDouble(node["balance_ratio_blue"], &camera->balance_ratio_blue);
    readDouble(node["throughput_limit_mbps"], &camera->throughput_limit_mbps);
}

}  // namespace

AppConfig loadConfig(const std::string& path) {
    AppConfig config;
    config.array.camera_count = 4;

    cv::FileStorage storage;
    try {
        storage.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    } catch (const cv::Exception& error) {
        throw std::runtime_error("cannot parse config '" + path + "': " + error.what());
    }
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot open config: " + path);
    }

    readInt(storage["camera_count"], &config.array.camera_count);
    readString(storage["device_order"], &config.array.device_order);
    readInt(storage["capture"]["frames"], &config.capture.frames);
    readDouble(storage["capture"]["interval_s"], &config.capture.interval_s);
    readString(storage["capture"]["output_dir"], &config.capture.output_dir);
    readString(storage["capture"]["image_format"], &config.capture.image_format);
    readBool(storage["capture"]["save_raw"], &config.capture.save_raw);
    readBool(storage["capture"]["software_trigger"], &config.capture.software_trigger);

    readInt(storage["usb"]["image_node_num"], &config.usb.image_node_num);
    readInt(storage["usb"]["usb_transfer_ways"], &config.usb.usb_transfer_ways);
    readInt(storage["usb"]["usb_transfer_size"], &config.usb.usb_transfer_size);
    readInt(storage["usb"]["incomplete_frame_retries"], &config.usb.incomplete_frame_retries);
    readBool(storage["usb"]["throughput_limit_mode"], &config.usb.throughput_limit_mode);
    readDouble(storage["usb"]["throughput_limit_mbps"], &config.usb.throughput_limit_mbps);
    readInt(storage["usb"]["transfer_size"], &config.usb.transfer_size);
    readInt(storage["usb"]["grab_timeout_ms"], &config.usb.grab_timeout_ms);
    readString(storage["usb"]["grab_strategy"], &config.usb.grab_strategy);

    readInt(storage["calibration"]["pattern_cols"], &config.calibration.pattern_cols);
    readInt(storage["calibration"]["pattern_rows"], &config.calibration.pattern_rows);
    readDouble(storage["calibration"]["square_mm"], &config.calibration.square_mm);
    readString(storage["calibration"]["model"], &config.calibration.model);
    readInt(storage["calibration"]["min_views"], &config.calibration.min_views);
    readDouble(storage["calibration"]["max_rms"], &config.calibration.max_rms);

    readInt(storage["panorama"]["out_width"], &config.panorama.out_width);
    readInt(storage["panorama"]["out_height"], &config.panorama.out_height);
    readDouble(storage["panorama"]["fov_h_deg"], &config.panorama.fov_h_deg);
    readDouble(storage["panorama"]["blend_sigma_deg"], &config.panorama.blend_sigma_deg);

    CameraConfig defaults;
    defaults.image_node_num = config.usb.image_node_num;
    defaults.usb_transfer_ways = config.usb.usb_transfer_ways;
    defaults.usb_transfer_size = config.usb.usb_transfer_size;
    defaults.incomplete_frame_retries = config.usb.incomplete_frame_retries;
    defaults.grab_timeout_ms = config.usb.grab_timeout_ms;
    defaults.grab_strategy = config.usb.grab_strategy;
    defaults.throughput_limit_mode = config.usb.throughput_limit_mode;
    defaults.throughput_limit_mbps = config.usb.throughput_limit_mbps;
    defaults.transfer_size = config.usb.transfer_size;
    readCamera(storage["defaults"], &defaults);

    const cv::FileNode cameras = storage["cameras"];
    if (!cameras.empty() && cameras.isSeq()) {
        int index = 0;
        for (auto it = cameras.begin(); it != cameras.end(); ++it) {
            CameraConfig camera = defaults;
            camera.index = index;
            readCamera(*it, &camera);
            config.array.cameras.push_back(std::move(camera));
            ++index;
        }
    }
    if (config.array.cameras.empty()) {
        for (int i = 0; i < config.array.camera_count; ++i) {
            CameraConfig camera = defaults;
            camera.index = i;
            config.array.cameras.push_back(std::move(camera));
        }
    }
    config.array.camera_count = static_cast<int>(config.array.cameras.size());
    config.array.software_trigger = config.capture.software_trigger;
    config.array.grab_timeout_ms = config.usb.grab_timeout_ms;

    if (config.array.camera_count <= 0) {
        throw std::runtime_error("config has no cameras");
    }
    if (config.calibration.pattern_cols < 3 || config.calibration.pattern_rows < 3) {
        throw std::runtime_error("calibration board must have at least 3x3 inner corners");
    }
    if (config.calibration.square_mm <= 0.0) {
        throw std::runtime_error("calibration.square_mm must be positive");
    }
    if (config.panorama.out_width <= 0 || config.panorama.out_height <= 0) {
        throw std::runtime_error("panorama output size must be positive");
    }
    if (config.panorama.fov_h_deg <= 0.0) {
        throw std::runtime_error("panorama.fov_h_deg must be positive");
    }

    return config;
}

std::string findDefaultConfig() {
    std::error_code ec;
    if (fs::exists("config/default.json", ec)) {
        return "config/default.json";
    }
    const fs::path executable = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const fs::path candidate = executable.parent_path().parent_path() / "config" / "default.json";
        if (fs::exists(candidate, ec)) {
            return candidate.string();
        }
    }
    return {};
}

}  // namespace hikquad
