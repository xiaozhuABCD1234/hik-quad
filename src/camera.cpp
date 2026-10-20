#include "hikquad/camera.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
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

void prepareSdkEnvironment() {
    const char* root = HIK_QUAD_MVS_ROOT;
    const char* lib = HIK_QUAD_MVS_LIB_DIR;
    setenv("MVCAM_SDK_PATH", root, 0);
    setenv("MVCAM_COMMON_RUNENV", lib, 0);
    setenv("MVCAM_SOFTWARE_LIBENV", lib, 0);
    std::string cl_protocol = std::string(lib) + "/CLProtocol";
    setenv("MVCAM_GENICAM_CLPROTOCOL", cl_protocol.c_str(), 0);
    std::string mvfg = std::string(root) + "/MVFG";
    setenv("ALLUSERSPROFILE", mvfg.c_str(), 0);
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

std::string decode(const unsigned char* data, size_t capacity) {
    size_t length = 0;
    while (length < capacity && data[length] != 0) {
        ++length;
    }
    return std::string(reinterpret_cast<const char*>(data), length);
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

std::string joinAvailable(const std::vector<DeviceInfo>& devices) {
    std::ostringstream out;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << devices[i].displayName();
    }
    return out.str();
}

bool findRawDevice(const CameraConfig& config, MV_CC_DEVICE_INFO* raw, DeviceInfo* info) {
    MV_CC_DEVICE_INFO_LIST list{};
    int result = MV_CC_EnumDevicesEx2(MV_USB_DEVICE, &list, "Hikrobot", SortMethod_SerialNumber);
    if (result != MV_OK) {
        result = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    }
    if (result != MV_OK) {
        throw SdkError(result, "MV_CC_EnumDevicesEx2");
    }

    struct Candidate {
        MV_CC_DEVICE_INFO raw;
        DeviceInfo info;
    };
    std::vector<Candidate> candidates;
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        if (list.pDeviceInfo[i] == nullptr || list.pDeviceInfo[i]->nTLayerType != MV_USB_DEVICE) {
            continue;
        }
        const MV_USB3_DEVICE_INFO& usb = list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
        const std::string serial = decode(usb.chSerialNumber, sizeof(usb.chSerialNumber));
        if (serial.rfind("Vir", 0) == 0) {
            continue;
        }
        Candidate candidate;
        candidate.raw = *list.pDeviceInfo[i];
        candidate.info.serial = serial;
        candidate.info.model = decode(usb.chModelName, sizeof(usb.chModelName));
        candidate.info.vendor = decode(usb.chVendorName, sizeof(usb.chVendorName));
        candidate.info.family = decode(usb.chFamilyName, sizeof(usb.chFamilyName));
        candidate.info.firmware = decode(usb.chDeviceVersion, sizeof(usb.chDeviceVersion));
        candidate.info.usb_address = usb.nDeviceAddress;
        candidate.info.bcd_usb = usb.nbcdUSB;
        candidate.info.link = findUsbLink(serial);
        candidates.push_back(std::move(candidate));
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.info.serial < b.info.serial; });

    for (size_t i = 0; i < candidates.size(); ++i) {
        candidates[i].info.index = static_cast<int>(i);
    }

    for (const auto& candidate : candidates) {
        const bool serial_match = !config.serial.empty() && candidate.info.serial == config.serial;
        const bool index_match =
            config.serial.empty() && config.index >= 0 && candidate.info.index == config.index;
        if (serial_match || index_match) {
            *raw = candidate.raw;
            *info = candidate.info;
            return true;
        }
    }
    throw std::runtime_error("camera not found (requested serial='" + config.serial + "', index=" +
                             std::to_string(config.index) + "). Available: " + joinAvailable([&]() {
                                 std::vector<DeviceInfo> devices;
                                 for (const auto& candidate : candidates) {
                                     devices.push_back(candidate.info);
                                 }
                                 return devices;
                             }()));
}

bool isEightBit(MvGvspPixelType type) {
    return type == PixelType_Gvsp_Mono8 || type == PixelType_Gvsp_BayerGR8 ||
           type == PixelType_Gvsp_BayerRG8 || type == PixelType_Gvsp_BayerGB8 ||
           type == PixelType_Gvsp_BayerBG8 || type == PixelType_Gvsp_RGB8_Packed ||
           type == PixelType_Gvsp_BGR8_Packed;
}

bool isSixteenBitContainer(MvGvspPixelType type) {
    return type == PixelType_Gvsp_Mono10 || type == PixelType_Gvsp_Mono12 ||
           type == PixelType_Gvsp_BayerGR10 || type == PixelType_Gvsp_BayerRG10 ||
           type == PixelType_Gvsp_BayerGB10 || type == PixelType_Gvsp_BayerBG10 ||
           type == PixelType_Gvsp_BayerGR12 || type == PixelType_Gvsp_BayerRG12 ||
           type == PixelType_Gvsp_BayerGB12 || type == PixelType_Gvsp_BayerBG12;
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
        HIKQUAD_LOG(Warn) << operation << " ignored: 0x" << std::hex
                          << static_cast<unsigned int>(result) << " (" << mvsErrorName(result)
                          << ")";
    }

    void setInt(const char* key, int64_t value, bool required, const char* operation) {
        const int result = MV_CC_SetIntValueEx(handle, key, value);
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + std::to_string(value));
        }
        HIKQUAD_LOG(Warn) << operation << " ignored: 0x" << std::hex
                          << static_cast<unsigned int>(result) << " (" << mvsErrorName(result)
                          << ")";
    }

    void setFloat(const char* key, double value, bool required, const char* operation) {
        const int result = MV_CC_SetFloatValue(handle, key, static_cast<float>(value));
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + std::to_string(value));
        }
        HIKQUAD_LOG(Warn) << operation << " ignored: 0x" << std::hex
                          << static_cast<unsigned int>(result) << " (" << mvsErrorName(result)
                          << ")";
    }

    void setBool(const char* key, bool value, bool required, const char* operation) {
        const int result = MV_CC_SetBoolValue(handle, key, value);
        if (result == MV_OK) {
            return;
        }
        if (required) {
            checkMvs(result, operation, std::string(key) + "=" + (value ? "true" : "false"));
        }
        HIKQUAD_LOG(Warn) << operation << " ignored: 0x" << std::hex
                          << static_cast<unsigned int>(result) << " (" << mvsErrorName(result)
                          << ")";
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

        if (config.image_node_num > 0) {
            const int result =
                MV_CC_SetImageNodeNum(handle, static_cast<unsigned int>(config.image_node_num));
            if (result != MV_OK) {
                HIKQUAD_LOG(Warn) << "MV_CC_SetImageNodeNum(" << config.image_node_num
                                  << ") ignored: 0x" << std::hex
                                  << static_cast<unsigned int>(result) << " ("
                                  << mvsErrorName(result) << ")";
            }
        }

        const int strategy = MV_CC_SetGrabStrategy(handle, grabStrategy(config.grab_strategy));
        if (strategy != MV_OK) {
            HIKQUAD_LOG(Warn) << "MV_CC_SetGrabStrategy ignored: 0x" << std::hex
                              << static_cast<unsigned int>(strategy) << " ("
                              << mvsErrorName(strategy) << ")";
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

        const int quality = MV_CC_SetBayerCvtQuality(
            handle, static_cast<unsigned int>(std::max(0, config.bayer_conversion_quality)));
        if (quality != MV_OK) {
            HIKQUAD_LOG(Debug) << "MV_CC_SetBayerCvtQuality ignored: 0x" << std::hex
                               << static_cast<unsigned int>(quality) << " ("
                               << mvsErrorName(quality) << ")";
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
        const PixelFormatInfo* format_info = pixelFormatByName(out.pixel_format);
        out.pixel_type = format_info != nullptr ? format_info->type : PixelType_Gvsp_Undefined;
        out.frame_rate = readFloat(handle, "AcquisitionFrameRate", config.frame_rate);
        out.resulting_frame_rate = readFloat(handle, "ResultingFrameRate", 0.0);
        out.exposure_us = readFloat(handle, "ExposureTime", config.exposure_us);
        out.gain_db = readFloat(handle, "Gain", config.gain_db);
        out.trigger_mode = enumSymbolic(handle, "TriggerMode");
        out.trigger_source = enumSymbolic(handle, "TriggerSource");
        out.image_node_num = config.image_node_num;
        return out;
    }

    bool grab(const CameraConfig& config, const DeviceInfo& device, Frame* frame, int timeout_ms) {
        if (handle == nullptr || !grabbing) {
            throw std::runtime_error("grab() called on a camera that is not grabbing");
        }
        MV_FRAME_OUT out{};
        const int result =
            MV_CC_GetImageBuffer(handle, &out, static_cast<unsigned int>(std::max(1, timeout_ms)));
        if (static_cast<unsigned int>(result) == MV_E_NODATA) {
            return false;
        }
        checkMvs(result, "MV_CC_GetImageBuffer");
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
        const uint32_t width = info.nExtendWidth > 0 ? info.nExtendWidth : info.nWidth;
        const uint32_t height = info.nExtendHeight > 0 ? info.nExtendHeight : info.nHeight;
        if (width == 0 || height == 0 || out.pBufAddr == nullptr) {
            throw std::runtime_error("SDK returned an empty frame");
        }
        const int width_i = static_cast<int>(width);
        const int height_i = static_cast<int>(height);

        Frame result_frame;
        result_frame.camera_index = device.index;
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

        if (config.keep_raw) {
            if (isEightBit(info.enPixelType)) {
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
                    cv::Mat(1, static_cast<int>(info.nFrameLen), CV_8UC1, out.pBufAddr).clone();
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
            convert.nSrcDataLen = info.nFrameLen;
            convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            convert.pDstBuffer = result_frame.bgr.data;
            convert.nDstBufferSize = static_cast<unsigned int>(result_frame.bgr.total() * 3);
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
    if (!findRawDevice(config, &raw, &device)) {
        throw std::runtime_error("camera selection failed");
    }
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

bool Camera::isOpen() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->handle != nullptr;
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
        HIKQUAD_LOG(Warn) << "TriggerSoftware failed: 0x" << std::hex
                          << static_cast<unsigned int>(result) << " (" << mvsErrorName(result)
                          << ")";
        return false;
    }
    return true;
}

CameraSettings Camera::settings() const {
    CameraSettings out = impl_->settings(config_);
    out.serial = device_.serial;
    out.model = device_.model;
    return out;
}

void Camera::setExposure(double microseconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle == nullptr) {
        throw std::runtime_error("setExposure() called on a closed camera");
    }
    checkMvs(MV_CC_SetFloatValue(impl_->handle, "ExposureTime", static_cast<float>(microseconds)),
             "MV_CC_SetFloatValue(ExposureTime)");
    config_.exposure_us = microseconds;
}

void Camera::setGain(double db) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle == nullptr) {
        throw std::runtime_error("setGain() called on a closed camera");
    }
    checkMvs(MV_CC_SetFloatValue(impl_->handle, "Gain", static_cast<float>(db)),
             "MV_CC_SetFloatValue(Gain)");
    config_.gain_db = db;
}

void Camera::setFrameRate(double fps) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle == nullptr) {
        throw std::runtime_error("setFrameRate() called on a closed camera");
    }
    checkMvs(MV_CC_SetBoolValue(impl_->handle, "AcquisitionFrameRateEnable", true),
             "MV_CC_SetBoolValue(AcquisitionFrameRateEnable)");
    checkMvs(MV_CC_SetFloatValue(impl_->handle, "AcquisitionFrameRate", static_cast<float>(fps)),
             "MV_CC_SetFloatValue(AcquisitionFrameRate)");
    config_.frame_rate = fps;
}

std::string sdkVersion() {
    const unsigned int version = MV_CC_GetSDKVersion();
    std::ostringstream out;
    out << ((version >> 24) & 0xFFu) << '.' << ((version >> 16) & 0xFFu) << '.'
        << ((version >> 8) & 0xFFu) << '.' << (version & 0xFFu);
    return out.str();
}

std::string toString(const CameraSettings& settings) {
    std::ostringstream out;
    out << settings.serial << " " << settings.width << "x" << settings.height << " "
        << settings.pixel_format << " fps=" << settings.frame_rate
        << " resulting=" << settings.resulting_frame_rate << " exp=" << settings.exposure_us
        << "us gain=" << settings.gain_db << "dB trigger=" << settings.trigger_mode;
    return out.str();
}

}  // namespace hikquad
