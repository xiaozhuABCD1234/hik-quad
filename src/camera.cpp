#include "hikquad/camera.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "hikquad/error.hpp"
#include "hikquad/log.hpp"

#ifndef HIK_QUAD_MVS_ROOT
#define HIK_QUAD_MVS_ROOT "/opt/MVS"
#endif
#ifndef HIK_QUAD_MVS_LIB_DIR
#define HIK_QUAD_MVS_LIB_DIR "/opt/MVS/lib/64"
#endif

namespace hikquad {
namespace {

std::mutex g_sdk_mutex;
int g_sdk_users = 0;

/// Environment variables already set by the user take precedence.
void setEnvIfUnset(const char* key, const std::string& value) {
    if (std::getenv(key) != nullptr) {
        return;
    }
#if defined(_WIN32)
    _putenv_s(key, value.c_str());
#else
    setenv(key, value.c_str(), 1);
#endif
}

/// The MVS runtime resolves its data files relative to these variables, so set
/// them before the first SDK call instead of requiring set_env_path.sh.
void prepareSdkEnvironment() {
    const std::string root = HIK_QUAD_MVS_ROOT;
    const std::string lib = HIK_QUAD_MVS_LIB_DIR;
    setEnvIfUnset("MVCAM_SDK_PATH", root);
    setEnvIfUnset("MVCAM_COMMON_RUNENV", lib);
    setEnvIfUnset("MVCAM_SOFTWARE_LIBENV", lib);
    setEnvIfUnset("MVCAM_GENICAM_CLPROTOCOL", lib + "/CLProtocol");
    setEnvIfUnset("ALLUSERSPROFILE", root + "/MVFG");
}

void acquireSdk() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_users == 0) {
        prepareSdkEnvironment();
        const int result = MV_CC_Initialize();
        if (result != MV_OK) {
            throw SdkError(result, "MV_CC_Initialize",
                           "check the MVS installation and LD_LIBRARY_PATH; the SDK loads its "
                           "format-conversion and GenICam producer libraries at runtime");
        }
    }
    ++g_sdk_users;
}

void releaseSdk() noexcept {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_users > 0) {
        --g_sdk_users;
        if (g_sdk_users == 0) {
            MV_CC_Finalize();
        }
    }
}

std::string enumSymbolic(void* handle, const char* key) {
    MVCC_ENUMVALUE value{};
    if (MV_CC_GetEnumValue(handle, key, &value) != MV_OK) {
        return {};
    }
    MVCC_ENUMENTRY entry{};
    entry.nValue = value.nCurValue;
    if (MV_CC_GetEnumEntrySymbolic(handle, key, &entry) != MV_OK) {
        return {};
    }
    return std::string(entry.chSymbolic);
}

double readFloat(void* handle, const char* key, double fallback) {
    MVCC_FLOATVALUE value{};
    if (MV_CC_GetFloatValue(handle, key, &value) != MV_OK) {
        return fallback;
    }
    return value.fCurValue;
}

int64_t readInt(void* handle, const char* key, int64_t fallback) {
    MVCC_INTVALUE_EX value{};
    if (MV_CC_GetIntValueEx(handle, key, &value) != MV_OK) {
        return fallback;
    }
    return value.nCurValue;
}

MV_GRAB_STRATEGY grabStrategy(const std::string& name) {
    if (name == "LatestImages") {
        return MV_GrabStrategy_LatestImages;
    }
    if (name == "LatestOnly" || name == "LatestImagesOnly") {
        return MV_GrabStrategy_LatestImagesOnly;
    }
    if (name == "NextFrame" || name == "UpcomingImage") {
        return MV_GrabStrategy_UpcomingImage;
    }
    return MV_GrabStrategy_OneByOne;
}

void reportIgnored(const char* operation, int result, const std::string& detail = {}) {
    HIKQUAD_LOG(Warn) << operation << " ignored: " << mvsErrorText(result) << detail;
}

std::string joinAvailable(const std::vector<UsbDevice>& devices) {
    std::ostringstream out;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << devices[i].info.displayName();
    }
    return out.str();
}

/// Resolves a configured serial (or index into the serial-sorted list) to the
/// raw MVS record needed by MV_CC_CreateHandle().
void findRawDevice(const CameraConfig& config, MV_CC_DEVICE_INFO* raw, DeviceInfo* info) {
    const std::vector<UsbDevice> devices = enumerateUsbDeviceRecords();
    for (size_t i = 0; i < devices.size(); ++i) {
        const UsbDevice& device = devices[i];
        const bool serial_match = !config.serial.empty() && device.info.serial == config.serial;
        const bool index_match =
            config.serial.empty() && config.index >= 0 && config.index == static_cast<int>(i);
        if (serial_match || index_match) {
            *raw = device.raw;
            *info = device.info;
            return;
        }
    }
    throw std::runtime_error("camera not found (requested serial='" + config.serial + "', index=" +
                             std::to_string(config.index) + "). Available: " +
                             joinAvailable(devices));
}

}  // namespace

struct Camera::Impl {
    void* handle = nullptr;
    bool grabbing = false;
    uint64_t sequence = 0;
    mutable std::mutex mutex;

    Impl() {
        acquireSdk();
    }
    ~Impl() {
        closeLocked();
        releaseSdk();
    }

    void closeLocked() noexcept {
        if (handle == nullptr) {
            return;
        }
        if (grabbing) {
            MV_CC_StopGrabbing(handle);
            grabbing = false;
        }
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
    }

    void setEnum(const char* key, const std::string& value, bool required, const char* operation) {
        const int result = MV_CC_SetEnumValueByString(handle, key, value.c_str());
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + value);
        }
        reportIgnored(operation, result);
    }

    void setInt(const char* key, int64_t value, bool required, const char* operation) {
        const int result = MV_CC_SetIntValueEx(handle, key, value);
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + std::to_string(value));
        }
        reportIgnored(operation, result);
    }

    void setFloat(const char* key, double value, bool required, const char* operation) {
        const int result = MV_CC_SetFloatValue(handle, key, static_cast<float>(value));
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + std::to_string(value));
        }
        reportIgnored(operation, result);
    }

    void setBool(const char* key, bool value, bool required, const char* operation) {
        const int result = MV_CC_SetBoolValue(handle, key, value);
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + (value ? "true" : "false"));
        }
        reportIgnored(operation, result);
    }

    void apply(const CameraConfig& config) {
        if (handle == nullptr) {
            throw std::runtime_error("camera is not open");
        }

        if (pixelFormatByName(config.pixel_format) == nullptr) {
            throw std::invalid_argument("unknown pixel format: " + config.pixel_format);
        }

        setEnum("AcquisitionMode", "Continuous", false, "AcquisitionMode");
        setInt("Width", config.width, true, "Width");
        setInt("Height", config.height, true, "Height");
        setInt("OffsetX", config.offset_x, false, "OffsetX");
        setInt("OffsetY", config.offset_y, false, "OffsetY");

        if (config.binning != 1) {
            setEnum("BinningHorizontal", "BinningHorizontal" + std::to_string(config.binning),
                    false, "BinningHorizontal");
            setEnum("BinningVertical", "BinningVertical" + std::to_string(config.binning), false,
                    "BinningVertical");
        }
        if (config.decimation != 1) {
            setEnum("DecimationHorizontal",
                    "DecimationHorizontal" + std::to_string(config.decimation), false,
                    "DecimationHorizontal");
            setEnum("DecimationVertical", "DecimationVertical" + std::to_string(config.decimation),
                    false, "DecimationVertical");
        }

        setEnum("PixelFormat", config.pixel_format, true, "PixelFormat");
        if (!config.adc_bit_depth.empty()) {
            setEnum("ADCBitDepth", config.adc_bit_depth, false, "ADCBitDepth");
        }

        setBool("ReverseX", config.reverse_x, false, "ReverseX");
        setBool("ReverseY", config.reverse_y, false, "ReverseY");

        setBool("AcquisitionFrameRateEnable", config.frame_rate_enable, false,
                "AcquisitionFrameRateEnable");
        if (config.frame_rate_enable && config.frame_rate > 0.0) {
            setFloat("AcquisitionFrameRate", config.frame_rate, false, "AcquisitionFrameRate");
        }

        setEnum("TriggerMode", config.trigger_mode, false, "TriggerMode");
        if (config.trigger_mode == "On") {
            setEnum("TriggerSource", config.trigger_source, true, "TriggerSource");
            setEnum("TriggerActivation", config.trigger_activation, false, "TriggerActivation");
        }

        setEnum("ExposureAuto", config.exposure_auto, false, "ExposureAuto");
        if (config.exposure_auto == "Off" && config.exposure_us > 0.0) {
            setFloat("ExposureTime", config.exposure_us, true, "ExposureTime");
        }
        setEnum("GainAuto", config.gain_auto, false, "GainAuto");
        if (config.gain_auto == "Off") {
            setFloat("Gain", config.gain_db, true, "Gain");
        }

        setEnum("BalanceWhiteAuto", config.white_balance_auto, false, "BalanceWhiteAuto");
        if (config.white_balance_auto == "Off") {
            const std::pair<const char*, double> ratios[] = {{"Red", config.balance_ratio_red},
                                                             {"Green", config.balance_ratio_green},
                                                             {"Blue", config.balance_ratio_blue}};
            for (const auto& ratio : ratios) {
                if (ratio.second <= 0.0) {
                    continue;
                }
                setEnum("BalanceRatioSelector", ratio.first, false, "BalanceRatioSelector");
                setFloat("BalanceRatio", ratio.second, false, "BalanceRatio");
            }
        }

        setBool("GammaEnable", config.gamma_enable, false, "GammaEnable");

        // USB3 Vision-specific transport knobs.  They are distinct from the
        // GenICam FeatureTree nodes probed by setInt() above.
        if (config.usb_transfer_ways > 0) {
            const int result =
                MV_USB_SetTransferWays(handle, static_cast<unsigned int>(config.usb_transfer_ways));
            if (result != MV_OK) {
                HIKQUAD_LOG(Warn) << "MV_USB_SetTransferWays(" << config.usb_transfer_ways
                                  << ") ignored: " << mvsErrorText(result)
                                  << "; this model supports up to 8";
            }
        }
        if (config.usb_transfer_size > 0) {
            const int size = config.usb_transfer_size;
            if (size < 0x8000 || size > 0x200000 || size % 4 != 0) {
                HIKQUAD_LOG(Warn) << "MV_USB_SetTransferSize(" << size
                                  << ") skipped: Linux range is [32768, 2097152] and must be "
                                     "a multiple of 4";
            } else {
                const int result = MV_USB_SetTransferSize(handle, static_cast<unsigned int>(size));
                if (result != MV_OK) {
                    HIKQUAD_LOG(Warn) << "MV_USB_SetTransferSize(" << size
                                      << ") ignored: " << mvsErrorText(result);
                }
            }
        }

        if (config.image_node_num > 0) {
            const int result =
                MV_CC_SetImageNodeNum(handle, static_cast<unsigned int>(config.image_node_num));
            if (result != MV_OK) {
                HIKQUAD_LOG(Warn) << "MV_CC_SetImageNodeNum(" << config.image_node_num
                                  << ") ignored: " << mvsErrorText(result);
            }
        }

        const int strategy_result =
            MV_CC_SetGrabStrategy(handle, grabStrategy(config.grab_strategy));
        if (strategy_result != MV_OK) {
            HIKQUAD_LOG(Warn) << "MV_CC_SetGrabStrategy ignored: " << mvsErrorText(strategy_result);
        }

        if (config.throughput_limit_mode || config.throughput_limit_mbps > 0.0) {
            setBool("DeviceLinkThroughputLimitMode", true, false, "DeviceLinkThroughputLimitMode");
            if (config.throughput_limit_mbps > 0.0) {
                const int64_t bytes_per_second =
                    static_cast<int64_t>(config.throughput_limit_mbps * 1000.0 * 1000.0);
                setInt("DeviceLinkThroughputLimit", bytes_per_second, false,
                       "DeviceLinkThroughputLimit (not exposed on every USB3 Vision model)");
            }
        }
        if (config.transfer_size > 0) {
            setInt("TransferSize", config.transfer_size, false,
                   "TransferSize (not exposed on every USB3 Vision model)");
        }

        const int quality_result = MV_CC_SetBayerCvtQuality(
            handle, static_cast<unsigned int>(std::max(0, config.bayer_conversion_quality)));
        if (quality_result != MV_OK) {
            HIKQUAD_LOG(Debug) << "MV_CC_SetBayerCvtQuality ignored: "
                               << mvsErrorText(quality_result);
        }
    }

    CameraSettings settings(const CameraConfig& config) const {
        std::lock_guard<std::mutex> lock(mutex);
        CameraSettings out;
        if (handle == nullptr) {
            return out;
        }
        out.width = static_cast<int>(readInt(handle, "Width", config.width));
        out.height = static_cast<int>(readInt(handle, "Height", config.height));
        out.pixel_format = enumSymbolic(handle, "PixelFormat");
        if (out.pixel_format.empty()) {
            out.pixel_format = config.pixel_format;
        }
        out.frame_rate = readFloat(handle, "AcquisitionFrameRate", config.frame_rate);
        out.resulting_frame_rate = readFloat(handle, "ResultingFrameRate", 0.0);
        out.exposure_us = readFloat(handle, "ExposureTime", config.exposure_us);
        out.gain_db = readFloat(handle, "Gain", config.gain_db);
        unsigned int ways = 0;
        unsigned int transfer_size = 0;
        if (MV_USB_GetTransferWays(handle, &ways) == MV_OK) {
            out.usb_transfer_ways = ways;
        }
        if (MV_USB_GetTransferSize(handle, &transfer_size) == MV_OK) {
            out.usb_transfer_size = transfer_size;
        }
        return out;
    }

    bool grab(const CameraConfig& config, const DeviceInfo& device, Frame* frame, int timeout_ms) {
        if (handle == nullptr || !grabbing) {
            throw std::runtime_error("grab() called on a camera that is not grabbing");
        }
        MV_FRAME_OUT out{};
        uint64_t expected_size = 0;
        uint64_t transport_size = 0;
        int incomplete_attempts = 0;
        uint32_t width = 0;
        uint32_t height = 0;

        // A truncated USB3 Vision frame is still returned as MV_OK by the SDK.
        // Validate nFrameLen/LostPacket and retry, so callers never receive a
        // half image that would silently corrupt calibration or stitching.
        while (true) {
            out = MV_FRAME_OUT{};
            const int result = MV_CC_GetImageBuffer(
                handle, &out, static_cast<unsigned int>(std::max(1, timeout_ms)));
            if (static_cast<unsigned int>(result) == MV_E_NODATA) {
                return false;
            }
            checkMvs(result, "MV_CC_GetImageBuffer");

            const MV_FRAME_OUT_INFO_EX& frame_info = out.stFrameInfo;
            width = frame_info.nExtendWidth > 0 ? frame_info.nExtendWidth : frame_info.nWidth;
            height = frame_info.nExtendHeight > 0 ? frame_info.nExtendHeight : frame_info.nHeight;
            transport_size =
                frame_info.nFrameLenEx > 0 ? frame_info.nFrameLenEx : frame_info.nFrameLen;
            expected_size = expectedFrameBytes(frame_info.enPixelType, width, height);
            const bool complete =
                out.pBufAddr != nullptr && width > 0 && height > 0 && frame_info.nLostPacket == 0 &&
                (expected_size == 0 ? transport_size > 0 : transport_size >= expected_size);
            if (complete) {
                break;
            }

            HIKQUAD_LOG(Warn) << "incomplete frame from " << device.serial
                              << ": transport=" << transport_size << " expected=" << expected_size
                              << " lost_packets=" << frame_info.nLostPacket;
            MV_CC_FreeImageBuffer(handle, &out);
            if (incomplete_attempts++ >= config.incomplete_frame_retries) {
                std::ostringstream message;
                message << "incomplete frame from " << device.serial << " after "
                        << (config.incomplete_frame_retries + 1)
                        << " attempts (transport=" << transport_size
                        << ", expected=" << expected_size
                        << ", lost_packets=" << frame_info.nLostPacket
                        << "). Lower the frame rate to 30-60 fps on a shared USB hub, move "
                           "cameras to different USB controllers, or check usbfs_memory_mb.";
                throw std::runtime_error(message.str());
            }
        }

        struct BufferGuard {
            void* handle;
            MV_FRAME_OUT* out;
            ~BufferGuard() {
                if (handle != nullptr && out != nullptr) {
                    MV_CC_FreeImageBuffer(handle, out);
                }
            }
        } guard{handle, &out};

        const MV_FRAME_OUT_INFO_EX& info = out.stFrameInfo;
        if (width == 0 || height == 0 || out.pBufAddr == nullptr) {
            throw std::runtime_error("SDK returned an empty frame");
        }
        const int width_i = static_cast<int>(width);
        const int height_i = static_cast<int>(height);

        Frame result_frame;
        result_frame.camera_index = config.index >= 0 ? config.index : 0;
        result_frame.serial = device.serial;
        result_frame.sequence = ++sequence;
        result_frame.width = width;
        result_frame.height = height;
        result_frame.source_pixel_type = info.enPixelType;
        result_frame.source_pixel_format = pixelFormatName(info.enPixelType);
        result_frame.host_time = std::chrono::system_clock::now();
        result_frame.host_timestamp_ms = info.nHostTimeStamp;
        result_frame.device_timestamp_ticks =
            (static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) | info.nDevTimeStampLow;
        result_frame.exposure_us = info.fExposureTime > 0.0F
                                       ? static_cast<double>(info.fExposureTime)
                                       : config.exposure_us;
        result_frame.gain_db = info.fGain > 0.0F ? static_cast<double>(info.fGain) : config.gain_db;
        result_frame.transport_size = transport_size;
        result_frame.expected_size = expected_size;
        result_frame.lost_packets = info.nLostPacket;
        result_frame.complete = true;

        if (config.keep_raw) {
            if (isEightBitContainer(info.enPixelType)) {
                const int channels = (info.enPixelType == PixelType_Gvsp_RGB8_Packed ||
                                      info.enPixelType == PixelType_Gvsp_BGR8_Packed)
                                         ? 3
                                         : 1;
                cv::Mat raw(height_i, width_i, CV_8UC(channels), out.pBufAddr);
                result_frame.raw = raw.clone();
            } else if (isSixteenBitContainer(info.enPixelType)) {
                cv::Mat raw(height_i, width_i, CV_16UC1, out.pBufAddr);
                result_frame.raw = raw.clone();
            } else {
                HIKQUAD_LOG(Warn) << "raw buffer for packed format "
                                  << result_frame.source_pixel_format
                                  << " is stored as an opaque byte stream";
                result_frame.raw =
                    cv::Mat(1, static_cast<int>(transport_size), CV_8UC1, out.pBufAddr).clone();
            }
        }

        const bool source_is_bgr = info.enPixelType == PixelType_Gvsp_BGR8_Packed;
        const bool source_is_rgb = info.enPixelType == PixelType_Gvsp_RGB8_Packed;
        const bool source_is_mono8 = info.enPixelType == PixelType_Gvsp_Mono8;

        if (source_is_bgr) {
            cv::Mat bgr(height_i, width_i, CV_8UC3, out.pBufAddr);
            result_frame.bgr = bgr.clone();
        } else {
            result_frame.bgr.create(height_i, width_i, CV_8UC3);
            MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
            convert.nWidth = width;
            convert.nHeight = height;
            convert.enSrcPixelType = info.enPixelType;
            convert.pSrcData = out.pBufAddr;
            convert.nSrcDataLen = static_cast<unsigned int>(transport_size);
            convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            convert.pDstBuffer = result_frame.bgr.data;
            convert.nDstBufferSize = static_cast<unsigned int>(result_frame.bgr.total() *
                                                               result_frame.bgr.elemSize());
            const int convert_result = MV_CC_ConvertPixelTypeEx(handle, &convert);
            if (convert_result != MV_OK) {
                if (isBayer(info.enPixelType) && bytesPerPixel(info.enPixelType) == 1) {
                    const int code = openCvBayerCode(info.enPixelType);
                    if (code >= 0) {
                        cv::Mat mosaic(height_i, width_i, CV_8UC1, out.pBufAddr);
                        cv::cvtColor(mosaic, result_frame.bgr, code);
                        HIKQUAD_LOG(Debug) << "SDK conversion failed; used OpenCV Bayer fallback";
                    } else {
                        checkMvs(convert_result, "MV_CC_ConvertPixelTypeEx");
                    }
                } else if (source_is_rgb) {
                    cv::Mat rgb(height_i, width_i, CV_8UC3, out.pBufAddr);
                    cv::cvtColor(rgb, result_frame.bgr, cv::COLOR_RGB2BGR);
                } else if (source_is_mono8) {
                    cv::Mat gray(height_i, width_i, CV_8UC1, out.pBufAddr);
                    cv::cvtColor(gray, result_frame.bgr, cv::COLOR_GRAY2BGR);
                } else {
                    checkMvs(convert_result, "MV_CC_ConvertPixelTypeEx",
                             "pixel format " + result_frame.source_pixel_format);
                }
            }
        }

        *frame = std::move(result_frame);
        return true;
    }
};

Camera::Camera() : impl_(std::make_unique<Impl>()) {}

Camera::~Camera() {
    close();
}

void Camera::open(const CameraConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle != nullptr) {
        impl_->closeLocked();
    }
    MV_CC_DEVICE_INFO raw{};
    DeviceInfo device;
    findRawDevice(config, &raw, &device);
    const int create_result = MV_CC_CreateHandle(&impl_->handle, &raw);
    if (create_result != MV_OK) {
        impl_->handle = nullptr;
        throw SdkError(create_result, "MV_CC_CreateHandle", device.displayName());
    }
    const int open_result = MV_CC_OpenDevice(impl_->handle, MV_ACCESS_Exclusive, 0);
    if (open_result != MV_OK) {
        MV_CC_DestroyHandle(impl_->handle);
        impl_->handle = nullptr;
        throw SdkError(
            open_result, "MV_CC_OpenDevice",
            device.displayName() + " may be opened by MVS, another process, or another node");
    }
    try {
        impl_->apply(config);
    } catch (...) {
        impl_->closeLocked();
        throw;
    }
    config_ = config;
    device_ = device;
    HIKQUAD_LOG(Info) << "opened " << device_.displayName() << " via USB "
                      << (device_.link.found ? device_.link.port_path : std::string("?")) << " at "
                      << (device_.link.found ? std::to_string(device_.link.speed_mbps) + " Mbps"
                                             : std::string("unknown speed"));
}

void Camera::close() noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->closeLocked();
}

void Camera::start() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle == nullptr) {
        throw std::runtime_error("start() called on a closed camera");
    }
    if (impl_->grabbing) {
        return;
    }
    checkMvs(MV_CC_StartGrabbing(impl_->handle), "MV_CC_StartGrabbing", device_.displayName());
    impl_->grabbing = true;
}

void Camera::stop() noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle != nullptr && impl_->grabbing) {
        MV_CC_StopGrabbing(impl_->handle);
        impl_->grabbing = false;
    }
}

bool Camera::grab(Frame& frame, int timeout_ms) {
    return impl_->grab(config_, device_, &frame, timeout_ms);
}

bool Camera::triggerSoftware() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle == nullptr) {
        return false;
    }
    const int result = MV_CC_SetCommandValue(impl_->handle, "TriggerSoftware");
    if (result != MV_OK) {
        HIKQUAD_LOG(Warn) << "TriggerSoftware failed: " << mvsErrorText(result);
        return false;
    }
    return true;
}

CameraSettings Camera::settings() const {
    CameraSettings out = impl_->settings(config_);
    out.serial = device_.serial;
    out.role = config_.role;
    out.model = device_.model;
    return out;
}

}  // namespace hikquad
