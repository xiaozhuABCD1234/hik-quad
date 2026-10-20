#pragma once

#include <sstream>
#include <string>

namespace hikquad {

enum class LogLevel { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4, Off = 5 };

void setLogLevel(LogLevel level) noexcept;
LogLevel logLevel() noexcept;
bool parseLogLevel(const std::string& text, LogLevel* level) noexcept;
const char* logLevelName(LogLevel level) noexcept;
void logMessage(LogLevel level, const std::string& message);

class LogMessage {
public:
    explicit LogMessage(LogLevel level) : level_(level) {}
    LogMessage(const LogMessage&) = delete;
    LogMessage& operator=(const LogMessage&) = delete;
    ~LogMessage() {
        logMessage(level_, stream_.str());
    }

    template <typename T>
    LogMessage& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    LogLevel level_;
    std::ostringstream stream_;
};

}  // namespace hikquad

#define HIKQUAD_LOG(level) ::hikquad::LogMessage(::hikquad::LogLevel::level)
