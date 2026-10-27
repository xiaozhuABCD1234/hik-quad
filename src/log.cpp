#include "hikquad/log.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>

namespace hikquad {
namespace {

std::atomic<LogLevel> g_level{LogLevel::Info};
std::mutex g_mutex;

}  // namespace

void setLogLevel(LogLevel level) noexcept {
    g_level.store(level, std::memory_order_relaxed);
}

const char* logLevelName(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Trace:
            return "TRACE";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Off:
            return "OFF";
    }
    return "?";
}

bool parseLogLevel(const std::string& text, LogLevel* level) noexcept {
    if (level == nullptr) {
        return false;
    }
    std::string value;
    value.reserve(text.size());
    for (char ch : text) {
        if (ch >= 'A' && ch <= 'Z') {
            value.push_back(static_cast<char>(ch - 'A' + 'a'));
        } else {
            value.push_back(ch);
        }
    }
    if (value == "trace") {
        *level = LogLevel::Trace;
    } else if (value == "debug") {
        *level = LogLevel::Debug;
    } else if (value == "info") {
        *level = LogLevel::Info;
    } else if (value == "warn" || value == "warning") {
        *level = LogLevel::Warn;
    } else if (value == "error") {
        *level = LogLevel::Error;
    } else if (value == "off" || value == "none") {
        *level = LogLevel::Off;
    } else {
        return false;
    }
    return true;
}

void logMessage(LogLevel level, const std::string& message) {
    if (level < g_level.load(std::memory_order_relaxed)) {
        return;
    }
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &time);
#else
    localtime_r(&time, &tm_buf);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%H:%M:%S", &tm_buf);

    std::lock_guard<std::mutex> lock(g_mutex);
    std::fprintf(stderr, "[%s.%03lld] [%s] %s\n", stamp, static_cast<long long>(ms.count()),
                 logLevelName(level), message.c_str());
    std::fflush(stderr);
}

}  // namespace hikquad
