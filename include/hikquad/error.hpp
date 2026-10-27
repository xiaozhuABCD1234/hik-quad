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

/// Human-readable MVS status, e.g. "0x80000005 MV_E_NODATA (timeout / no data)".
std::string mvsErrorText(int code);

/// Throws SdkError unless the code is MV_OK.
void checkMvs(int code, const std::string& operation, const std::string& hint = {});

}  // namespace hikquad
