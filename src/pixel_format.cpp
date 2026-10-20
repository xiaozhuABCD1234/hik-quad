#include "hikquad/pixel_format.hpp"

#include <opencv2/imgproc.hpp>

namespace hikquad {
namespace {

constexpr PixelFormatInfo kFormats[] = {
    {PixelType_Gvsp_Mono8, "Mono8", 1, false, false, true},
    {PixelType_Gvsp_Mono10, "Mono10", 2, false, false, true},
    {PixelType_Gvsp_Mono12, "Mono12", 2, false, false, true},
    {PixelType_Gvsp_Mono10_Packed, "Mono10_Packed", 0, false, true, true},
    {PixelType_Gvsp_Mono12_Packed, "Mono12_Packed", 0, false, true, true},
    {PixelType_Gvsp_BayerGR8, "BayerGR8", 1, true, false, false},
    {PixelType_Gvsp_BayerRG8, "BayerRG8", 1, true, false, false},
    {PixelType_Gvsp_BayerGB8, "BayerGB8", 1, true, false, false},
    {PixelType_Gvsp_BayerBG8, "BayerBG8", 1, true, false, false},
    {PixelType_Gvsp_BayerGR10, "BayerGR10", 2, true, false, false},
    {PixelType_Gvsp_BayerRG10, "BayerRG10", 2, true, false, false},
    {PixelType_Gvsp_BayerGB10, "BayerGB10", 2, true, false, false},
    {PixelType_Gvsp_BayerBG10, "BayerBG10", 2, true, false, false},
    {PixelType_Gvsp_BayerGR12, "BayerGR12", 2, true, false, false},
    {PixelType_Gvsp_BayerRG12, "BayerRG12", 2, true, false, false},
    {PixelType_Gvsp_BayerGB12, "BayerGB12", 2, true, false, false},
    {PixelType_Gvsp_BayerBG12, "BayerBG12", 2, true, false, false},
    {PixelType_Gvsp_BayerGR10_Packed, "BayerGR10_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerRG10_Packed, "BayerRG10_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerGB10_Packed, "BayerGB10_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerBG10_Packed, "BayerBG10_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerGR12_Packed, "BayerGR12_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerRG12_Packed, "BayerRG12_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerGB12_Packed, "BayerGB12_Packed", 0, true, true, false},
    {PixelType_Gvsp_BayerBG12_Packed, "BayerBG12_Packed", 0, true, true, false},
    {PixelType_Gvsp_RGB8_Packed, "RGB8Packed", 3, false, true, false},
    {PixelType_Gvsp_BGR8_Packed, "BGR8Packed", 3, false, true, false},
    {PixelType_Gvsp_YUV422_Packed, "YUV422Packed", 2, false, true, false},
    {PixelType_Gvsp_YUV422_YUYV_Packed, "YUV422_YUYV_Packed", 2, false, true, false},
};

}  // namespace

const PixelFormatInfo* pixelFormatByName(const std::string& name) noexcept {
    for (const auto& entry : kFormats) {
        if (name == entry.name) {
            return &entry;
        }
    }
    return nullptr;
}

const PixelFormatInfo* pixelFormatByType(MvGvspPixelType type) noexcept {
    for (const auto& entry : kFormats) {
        if (entry.type == type) {
            return &entry;
        }
    }
    return nullptr;
}

std::string pixelFormatName(MvGvspPixelType type) {
    const PixelFormatInfo* info = pixelFormatByType(type);
    return info != nullptr ? std::string(info->name) : std::string("Unknown");
}

bool isBayer(MvGvspPixelType type) noexcept {
    const PixelFormatInfo* info = pixelFormatByType(type);
    return info != nullptr && info->bayer;
}

bool isMono(MvGvspPixelType type) noexcept {
    const PixelFormatInfo* info = pixelFormatByType(type);
    return info != nullptr && info->mono;
}

bool isColor(MvGvspPixelType type) noexcept {
    return !isMono(type) && type != PixelType_Gvsp_Undefined;
}

int bytesPerPixel(MvGvspPixelType type) noexcept {
    const PixelFormatInfo* info = pixelFormatByType(type);
    return info != nullptr ? info->bytes_per_pixel : 0;
}

size_t expectedFrameBytes(MvGvspPixelType type, uint32_t width, uint32_t height) noexcept {
    const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
    switch (type) {
        case PixelType_Gvsp_Mono8:
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_BayerBG8:
            return pixels;
        case PixelType_Gvsp_Mono10:
        case PixelType_Gvsp_Mono12:
        case PixelType_Gvsp_BayerGR10:
        case PixelType_Gvsp_BayerRG10:
        case PixelType_Gvsp_BayerGB10:
        case PixelType_Gvsp_BayerBG10:
        case PixelType_Gvsp_BayerGR12:
        case PixelType_Gvsp_BayerRG12:
        case PixelType_Gvsp_BayerGB12:
        case PixelType_Gvsp_BayerBG12:
        case PixelType_Gvsp_YUV422_Packed:
        case PixelType_Gvsp_YUV422_YUYV_Packed:
            return pixels * 2;
        case PixelType_Gvsp_RGB8_Packed:
        case PixelType_Gvsp_BGR8_Packed:
            return pixels * 3;
        default:
            // Packed 10/12-bit layouts: trust the SDK frame length.
            return 0;
    }
}

int bayerPatternIndex(MvGvspPixelType type) noexcept {
    switch (type) {
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_BayerRG10:
        case PixelType_Gvsp_BayerRG12:
        case PixelType_Gvsp_BayerRG10_Packed:
        case PixelType_Gvsp_BayerRG12_Packed:
            return 0;  // RGGB
        case PixelType_Gvsp_BayerBG8:
        case PixelType_Gvsp_BayerBG10:
        case PixelType_Gvsp_BayerBG12:
        case PixelType_Gvsp_BayerBG10_Packed:
        case PixelType_Gvsp_BayerBG12_Packed:
            return 1;  // BGGR
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_BayerGB10:
        case PixelType_Gvsp_BayerGB12:
        case PixelType_Gvsp_BayerGB10_Packed:
        case PixelType_Gvsp_BayerGB12_Packed:
            return 2;  // GBRG
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_BayerGR10:
        case PixelType_Gvsp_BayerGR12:
        case PixelType_Gvsp_BayerGR10_Packed:
        case PixelType_Gvsp_BayerGR12_Packed:
            return 3;  // GRBG
        default:
            return -1;
    }
}

int openCvBayerCode(MvGvspPixelType type) noexcept {
    switch (bayerPatternIndex(type)) {
        case 0:
            return cv::COLOR_BayerBG2BGR;  // OpenCV naming is inverted for RGGB
        case 1:
            return cv::COLOR_BayerRG2BGR;
        case 2:
            return cv::COLOR_BayerGR2BGR;
        case 3:
            return cv::COLOR_BayerGB2BGR;
        default:
            return -1;
    }
}

}  // namespace hikquad
