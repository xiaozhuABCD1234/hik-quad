#pragma once

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hikquad/calibration.hpp"
#include "hikquad/cli.hpp"
#include "hikquad/config.hpp"
#include "hikquad/log.hpp"

namespace hikquad {
namespace app {

inline void initLogging(const CliParser& cli) {
    LogLevel level = LogLevel::Info;
    if (cli.has("--verbose")) {
        level = LogLevel::Debug;
    }
    if (cli.has("--quiet")) {
        level = LogLevel::Warn;
    }
    const std::string text = cli.get("--log-level");
    if (!text.empty()) {
        LogLevel parsed = level;
        if (parseLogLevel(text, &parsed)) {
            level = parsed;
        }
    }
    setLogLevel(level);
}

inline std::string resolveConfigOrThrow(const CliParser& cli) {
    std::string path = cli.get("--config");
    if (path.empty()) {
        path = findDefaultConfig();
    }
    if (path.empty()) {
        throw std::runtime_error(
            "no config file found. Pass --config config/default.json or run from the project root");
    }
    return path;
}

inline AppConfig loadAppConfig(const CliParser& cli) {
    return loadConfig(resolveConfigOrThrow(cli));
}

inline cv::Size parseSize(const std::string& text, const cv::Size& fallback) {
    if (text.empty()) {
        return fallback;
    }
    const auto separator = text.find_first_of("xX,");
    if (separator == std::string::npos) {
        return fallback;
    }
    try {
        const int width = std::stoi(text.substr(0, separator));
        const int height = std::stoi(text.substr(separator + 1));
        if (width <= 0 || height <= 0) {
            return fallback;
        }
        return cv::Size(width, height);
    } catch (...) {
        return fallback;
    }
}

inline BoardSpec parseBoard(const std::string& text, const BoardSpec& fallback) {
    if (text.empty()) {
        return fallback;
    }
    const auto separator = text.find_first_of("xX,");
    if (separator == std::string::npos) {
        return fallback;
    }
    try {
        BoardSpec board = fallback;
        board.cols = std::stoi(text.substr(0, separator));
        board.rows = std::stoi(text.substr(separator + 1));
        if (board.cols < 3 || board.rows < 3) {
            return fallback;
        }
        return board;
    } catch (...) {
        return fallback;
    }
}

inline bool parseOnOff(const std::string& text, bool fallback) {
    if (text.empty()) {
        return fallback;
    }
    if (text == "1" || text == "true" || text == "on" || text == "yes" || text == "On") {
        return true;
    }
    if (text == "0" || text == "false" || text == "off" || text == "no" || text == "Off") {
        return false;
    }
    return fallback;
}

inline std::string nowStamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &time);
#else
    localtime_r(&time, &tm_buf);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buffer);
}

inline int usbMemoryMb() {
    std::ifstream stream("/sys/module/usbcore/parameters/usbfs_memory_mb");
    int value = 0;
    if (stream.good()) {
        stream >> value;
    }
    return value;
}

inline bool udevRuleInstalled() {
    return std::filesystem::exists("/etc/udev/rules.d/80-drivers-SDK-2bdf.rules");
}

inline void ensureDirectory(const std::string& path) {
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    if (ec) {
        throw std::runtime_error("cannot create directory " + path + ": " + ec.message());
    }
}

inline std::string imageExtension(const std::string& format) {
    if (format == "jpg" || format == "jpeg") {
        return ".jpg";
    }
    if (format == "bmp") {
        return ".bmp";
    }
    return ".png";
}

}  // namespace app
}  // namespace hikquad
