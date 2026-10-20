#include "hikquad/camera_array.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "hikquad/error.hpp"
#include "hikquad/log.hpp"

namespace hikquad {

struct CameraArray::Impl {
    std::vector<std::unique_ptr<Camera>> cameras;
    std::vector<DeviceInfo> devices;
    bool started = false;
    uint64_t sequence = 0;
    mutable std::mutex mutex;

    void closeAll() noexcept {
        for (auto& camera : cameras) {
            if (camera) {
                camera->close();
            }
        }
        cameras.clear();
        devices.clear();
        started = false;
    }
};

CameraArray::CameraArray() : impl_(std::make_unique<Impl>()) {}

CameraArray::~CameraArray() {
    close();
}

void CameraArray::open(const ArrayConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->closeAll();

    if (config.camera_count <= 0) {
        throw std::invalid_argument("camera_count must be positive");
    }

    const std::vector<DeviceInfo> available = enumerateUsbDevices(false);
    if (available.size() < static_cast<size_t>(config.camera_count)) {
        std::ostringstream message;
        message << "requested " << config.camera_count << " cameras but only " << available.size()
                << " physical USB3 Vision camera(s) are connected";
        throw std::runtime_error(message.str());
    }

    std::vector<CameraConfig> configs = config.cameras;
    configs.resize(static_cast<size_t>(config.camera_count));
    std::set<std::string> used_serials;
    for (int i = 0; i < config.camera_count; ++i) {
        CameraConfig& camera_config = configs[static_cast<size_t>(i)];
        if (camera_config.serial.empty()) {
            camera_config.index = i;
        }
        if (camera_config.serial.empty() && camera_config.index < 0) {
            camera_config.index = i;
        }
        if (config.grab_timeout_ms > 0) {
            camera_config.grab_timeout_ms = config.grab_timeout_ms;
        }
        if (config.software_trigger && camera_config.trigger_mode == "Off") {
            camera_config.trigger_mode = "On";
            camera_config.trigger_source = "Software";
        }
        const std::string key = camera_config.serial.empty()
                                    ? ("index:" + std::to_string(camera_config.index))
                                    : ("serial:" + camera_config.serial);
        if (!used_serials.insert(key).second) {
            throw std::invalid_argument("duplicate camera selector: " + key);
        }
        impl_->cameras.push_back(std::make_unique<Camera>());
        try {
            impl_->cameras.back()->open(camera_config);
        } catch (...) {
            impl_->cameras.pop_back();
            impl_->closeAll();
            throw;
        }
        impl_->devices.push_back(impl_->cameras.back()->deviceInfo());
    }

    try {
        for (auto& camera : impl_->cameras) {
            camera->start();
        }
    } catch (...) {
        impl_->closeAll();
        throw;
    }
    impl_->started = true;

    HIKQUAD_LOG(Info) << "camera array ready: " << impl_->cameras.size()
                      << " cameras, software_trigger=" << (config.software_trigger ? "on" : "off");
    config_ = config;
    config_.cameras = std::move(configs);
}

void CameraArray::close() noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->closeAll();
}

bool CameraArray::isOpen() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->started;
}

void CameraArray::start() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->cameras.empty()) {
        throw std::runtime_error("start() called on an empty camera array");
    }
    for (auto& camera : impl_->cameras) {
        camera->start();
    }
    impl_->started = true;
}

void CameraArray::stop() noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (auto& camera : impl_->cameras) {
        camera->stop();
    }
    impl_->started = false;
}

FrameSet CameraArray::captureOnce() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->cameras.empty()) {
        throw std::runtime_error("captureOnce() called on an empty camera array");
    }
    if (!impl_->started) {
        for (auto& camera : impl_->cameras) {
            camera->start();
        }
        impl_->started = true;
    }

    const bool software_trigger = config_.software_trigger;
    if (software_trigger) {
        for (auto& camera : impl_->cameras) {
            camera->triggerSoftware();
        }
    }

    FrameSet set;
    set.index = ++impl_->sequence;
    set.frames.resize(impl_->cameras.size());

    std::vector<std::future<void>> futures;
    futures.reserve(impl_->cameras.size());
    for (size_t i = 0; i < impl_->cameras.size(); ++i) {
        futures.push_back(std::async(std::launch::async, [this, i, &set]() {
            Frame frame;
            if (!impl_->cameras[i]->grab(frame)) {
                throw std::runtime_error("timeout while waiting for a frame from " +
                                         impl_->cameras[i]->deviceInfo().displayName());
            }
            set.frames[i] = std::move(frame);
        }));
    }
    std::exception_ptr first_error;
    for (auto& future : futures) {
        try {
            future.get();
        } catch (...) {
            if (first_error == nullptr) {
                first_error = std::current_exception();
            }
        }
    }
    if (first_error != nullptr) {
        std::rethrow_exception(first_error);
    }

    set.host_time = std::chrono::system_clock::now();
    if (!set.frames.empty()) {
        auto min_time = set.frames.front().host_time;
        auto max_time = set.frames.front().host_time;
        for (const auto& frame : set.frames) {
            min_time = std::min(min_time, frame.host_time);
            max_time = std::max(max_time, frame.host_time);
        }
        set.skew_ms = std::chrono::duration<double, std::milli>(max_time - min_time).count();
    }
    return set;
}

std::vector<FrameSet> CameraArray::capture(int count, double interval_s) {
    if (count <= 0) {
        return {};
    }
    std::vector<FrameSet> sets;
    sets.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        sets.push_back(captureOnce());
        if (interval_s > 0.0 && i + 1 < count) {
            std::this_thread::sleep_for(std::chrono::duration<double>(interval_s));
        }
    }
    return sets;
}

std::vector<CameraSettings> CameraArray::settings() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<CameraSettings> out;
    out.reserve(impl_->cameras.size());
    for (const auto& camera : impl_->cameras) {
        out.push_back(camera->settings());
    }
    return out;
}

std::vector<DeviceInfo> CameraArray::devices() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->devices;
}

size_t CameraArray::size() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->cameras.size();
}

std::string formatFrameSetSummary(const FrameSet& set) {
    std::ostringstream out;
    out << "frameset " << set.index << " skew=" << set.skew_ms << "ms";
    for (const auto& frame : set.frames) {
        out << "\n  cam" << frame.camera_index << " " << frame.serial << " " << frame.width << "x"
            << frame.height << " " << frame.source_pixel_format << " seq=" << frame.sequence
            << " host=" << frame.host_timestamp_ms << " device_ts=" << frame.device_timestamp_ticks;
    }
    return out.str();
}

}  // namespace hikquad
