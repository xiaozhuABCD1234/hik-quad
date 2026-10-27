#pragma once

#include <string>
#include <vector>

#include "camera.hpp"
#include "camera_array.hpp"

namespace hikquad {

struct CaptureConfig {
    int frames = 1;
    double interval_s = 0.0;
    std::string output_dir = "data/captures";
    std::string image_format = "png";
    bool save_raw = false;
    bool software_trigger = false;
};

struct UsbConfig {
    int image_node_num = 8;
    int usb_transfer_ways = 8;
    int usb_transfer_size = 1048576;
    bool throughput_limit_mode = false;
    double throughput_limit_mbps = 0.0;
    int transfer_size = 0;
    int grab_timeout_ms = 3000;
    int incomplete_frame_retries = 3;
    std::string grab_strategy = "OldestFirst";
};

struct CalibrationConfig {
    int pattern_cols = 8;
    int pattern_rows = 11;
    double square_mm = 100.0;
    std::string model = "fisheye";
    int min_views = 10;
    double max_rms = 1.0;
};

struct PanoramaConfig {
    int out_width = 2048;
    int out_height = 512;
    double fov_h_deg = 360.0;
    double blend_sigma_deg = 35.0;
};

struct AppConfig {
    ArrayConfig array;
    CaptureConfig capture;
    UsbConfig usb;
    CalibrationConfig calibration;
    PanoramaConfig panorama;
};

/// Loads the JSON configuration.  Relative paths inside the config are
/// interpreted against the current working directory.
AppConfig loadConfig(const std::string& path);

/// config/default.json next to the working directory, then next to the
/// executable (build/../config).  Empty when neither exists.
std::string findDefaultConfig();

}  // namespace hikquad
