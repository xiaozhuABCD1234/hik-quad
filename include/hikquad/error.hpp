#pragma once

#include <stdexcept>
#include <string>

namespace hikquad {

class SdkError : public std::runtime_error {
public:
    SdkError(int code, std::string operation, std::string hint = {});

    int code() const noexcept {
        return code_;
    }
    const std::string& operation() const noexcept {
        return operation_;
    }
    const std::string& hint() const noexcept {
        return hint_;
    }

private:
    int code_;
    std::string operation_;
    std::string hint_;
};

std::string mvsErrorName(int code);
bool isTimeoutError(int code) noexcept;
void checkMvs(int code, const std::string& operation, const std::string& hint = {});

}  // namespace hikquad
