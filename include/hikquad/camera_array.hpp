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
    bool isOpen() const noexcept;

    void start();
    void stop() noexcept;

    FrameSet captureOnce();
    std::vector<FrameSet> capture(int count, double interval_s);

    std::vector<CameraSettings> settings() const;
    std::vector<DeviceInfo> devices() const;
    const ArrayConfig& config() const noexcept {
        return config_;
    }
    size_t size() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ArrayConfig config_;
};

std::string formatFrameSetSummary(const FrameSet& set);

}  // namespace hikquad
