#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "PixelType.h"

namespace hikquad {

struct PixelFormatInfo {
    MvGvspPixelType type;
    const char* name;
    /// Bytes per pixel; 0 for packed 10/12-bit layouts.
    int bytes_per_pixel;
    bool bayer;
    bool mono;
};

const PixelFormatInfo* pixelFormatByName(const std::string& name) noexcept;
const PixelFormatInfo* pixelFormatByType(MvGvspPixelType type) noexcept;
std::string pixelFormatName(MvGvspPixelType type);

bool isBayer(MvGvspPixelType type) noexcept;
int bytesPerPixel(MvGvspPixelType type) noexcept;

/// 8-bit transport layout: Mono8, Bayer*8, RGB8, BGR8.
bool isEightBitContainer(MvGvspPixelType type) noexcept;

/// 16-bit container layout: Mono10/12 and Bayer*10/12.
bool isSixteenBitContainer(MvGvspPixelType type) noexcept;

/// Expected transport payload in bytes for one full frame.  Returns 0 for
/// packed layouts where the SDK frame length should be trusted.
size_t expectedFrameBytes(MvGvspPixelType type, uint32_t width, uint32_t height) noexcept;

/// OpenCV demosaic code for an 8-bit Bayer MVS pixel type, or -1.
/// MVS BayerRG8 is RGGB, which OpenCV calls COLOR_BayerBG2BGR.
int openCvBayerCode(MvGvspPixelType type) noexcept;

/// 0 = RGGB, 1 = BGGR, 2 = GBRG, 3 = GRBG.
int bayerPatternIndex(MvGvspPixelType type) noexcept;

}  // namespace hikquad
