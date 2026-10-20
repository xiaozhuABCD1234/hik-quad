#pragma once

#include <string>

#include "PixelType.h"

namespace hikquad {

struct PixelFormatInfo {
    MvGvspPixelType type;
    const char* name;
    int bytes_per_pixel;
    bool bayer;
    bool packed;
    bool mono;
};

const PixelFormatInfo* pixelFormatByName(const std::string& name) noexcept;
const PixelFormatInfo* pixelFormatByType(MvGvspPixelType type) noexcept;
std::string pixelFormatName(MvGvspPixelType type);

bool isBayer(MvGvspPixelType type) noexcept;
bool isMono(MvGvspPixelType type) noexcept;
bool isColor(MvGvspPixelType type) noexcept;
int bytesPerPixel(MvGvspPixelType type) noexcept;

/// OpenCV demosaic code for an 8-bit Bayer MVS pixel type, or -1.
/// MVS BayerRG8 is RGGB, which OpenCV calls COLOR_BayerBG2BGR.
int openCvBayerCode(MvGvspPixelType type) noexcept;

/// 0 = RGGB, 1 = BGGR, 2 = GBRG, 3 = GRBG.
int bayerPatternIndex(MvGvspPixelType type) noexcept;

}  // namespace hikquad
