#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "camera.hpp"

namespace hikquad {

struct FrameSet {
    uint64_t index = 0;
    std::chrono::system_clock::time_point host_time;
    std::vector<Frame> frames;
    double skew_ms = 0.0;
};

struct ArrayConfig {
    int camera_count = 4;
    std::vector<CameraConfig> cameras;
    /// "usb_port" (topology order, default) or "serial" (lexicographic).
    std::string device_order = "usb_port";
    bool software_trigger = false;
    int grab_timeout_ms = 3000;
};

class CameraArray {
public:
    CameraArray();
    ~CameraArray();

    CameraArray(const CameraArray&) = delete;
    CameraArray& operator=(const CameraArray&) = delete;

    void open(const ArrayConfig& config);
    void close() noexcept;
    void stop() noexcept;

    FrameSet captureOnce();
    std::vector<CameraSettings> settings() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ArrayConfig config_;
};

std::string formatFrameSetSummary(const FrameSet& set);

}  // namespace hikquad
