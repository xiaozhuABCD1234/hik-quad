#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "device_info.hpp"
#include "pixel_format.hpp"

namespace hikquad {

struct CameraConfig {
    std::string serial;
    std::string role;  // e.g. cam1..cam4, clockwise physical order
    int index = -1;

    int width = 1440;
    int height = 1080;
    int offset_x = 0;
    int offset_y = 0;
    std::string pixel_format = "BayerRG8";
    std::string adc_bit_depth = "Bits_12";
    int binning = 1;
    int decimation = 1;
    bool reverse_x = false;
    bool reverse_y = false;
    int bayer_conversion_quality = 1;
    bool keep_raw = false;

    bool frame_rate_enable = true;
    double frame_rate = 30.0;
    double exposure_us = 5000.0;
    std::string exposure_auto = "Off";
    double gain_db = 0.0;
    std::string gain_auto = "Off";
    std::string white_balance_auto = "Off";
    double balance_ratio_red = 0.0;
    double balance_ratio_green = 0.0;
    double balance_ratio_blue = 0.0;
    bool gamma_enable = false;

    std::string trigger_mode = "Off";
    std::string trigger_source = "Software";
    std::string trigger_activation = "RisingEdge";

    int image_node_num = 8;
    int usb_transfer_ways = 8;        // MV_USB_SetTransferWays, max 8 on this model
    int usb_transfer_size = 1048576;  // MV_USB_SetTransferSize, bytes
    bool throughput_limit_mode = false;
    double throughput_limit_mbps = 0.0;
    int transfer_size = 0;
    int grab_timeout_ms = 3000;
    int incomplete_frame_retries = 3;
    std::string grab_strategy = "OldestFirst";
};

/// State reported by the camera after the configuration was applied.
struct CameraSettings {
    std::string serial;
    std::string role;
    std::string model;
    int width = 0;
    int height = 0;
    std::string pixel_format;
    double frame_rate = 0.0;
    double resulting_frame_rate = 0.0;
    double exposure_us = 0.0;
    double gain_db = 0.0;
    unsigned int usb_transfer_ways = 0;
    unsigned int usb_transfer_size = 0;
};

struct Frame {
    int camera_index = -1;
    std::string serial;
    uint64_t sequence = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    MvGvspPixelType source_pixel_type = PixelType_Gvsp_Undefined;
    std::string source_pixel_format;
    cv::Mat bgr;
    cv::Mat raw;
    std::chrono::system_clock::time_point host_time;
    int64_t host_timestamp_ms = 0;
    uint64_t device_timestamp_ticks = 0;
    double exposure_us = 0.0;
    double gain_db = 0.0;
    uint64_t transport_size = 0;
    uint64_t expected_size = 0;
    uint32_t lost_packets = 0;
    bool complete = false;
};

class Camera {
public:
    Camera();
    ~Camera();

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    void open(const CameraConfig& config);
    void close() noexcept;

    void start();
    void stop() noexcept;
    bool grab(Frame& frame, int timeout_ms);
    bool grab(Frame& frame) {
        return grab(frame, config_.grab_timeout_ms);
    }
    bool triggerSoftware();

    CameraSettings settings() const;
    const DeviceInfo& deviceInfo() const noexcept {
        return device_;
    }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    CameraConfig config_;
    DeviceInfo device_;
};

}  // namespace hikquad
