#include "hikquad/error.hpp"

#include <sstream>
#include <utility>

#include "MvErrorDefine.h"

namespace hikquad {
namespace {

const char* mvsErrorName(int code) {
    switch (static_cast<unsigned int>(code)) {
        case MV_OK:
            return "MV_OK";
        case MV_E_HANDLE:
            return "MV_E_HANDLE (invalid handle)";
        case MV_E_SUPPORT:
            return "MV_E_SUPPORT (feature not supported)";
        case MV_E_BUFOVER:
            return "MV_E_BUFOVER (buffer overflow)";
        case MV_E_CALLORDER:
            return "MV_E_CALLORDER (wrong call order)";
        case MV_E_PARAMETER:
            return "MV_E_PARAMETER (invalid parameter)";
        case MV_E_RESOURCE:
            return "MV_E_RESOURCE (resource allocation failed)";
        case MV_E_NODATA:
            return "MV_E_NODATA (timeout / no data)";
        case MV_E_PRECONDITION:
            return "MV_E_PRECONDITION (precondition failed)";
        case MV_E_VERSION:
            return "MV_E_VERSION (version mismatch)";
        case MV_E_NOENOUGH_BUF:
            return "MV_E_NOENOUGH_BUF (output buffer too small)";
        case MV_E_ABNORMAL_IMAGE:
            return "MV_E_ABNORMAL_IMAGE (incomplete image / packet loss)";
        case MV_E_LOAD_LIBRARY:
            return "MV_E_LOAD_LIBRARY (runtime library missing)";
        case MV_E_NOOUTBUF:
            return "MV_E_NOOUTBUF (no free internal buffer)";
        case MV_E_OPENFILE:
            return "MV_E_OPENFILE (file open failed)";
        case MV_E_GC_GENERIC:
            return "MV_E_GC_GENERIC (GenICam generic error)";
        case MV_E_GC_ACCESS:
            return "MV_E_GC_ACCESS (GenICam access error)";
        case MV_E_GC_TIMEOUT:
            return "MV_E_GC_TIMEOUT (GenICam timeout)";
        default:
            return "unknown MVS error";
    }
}

}  // namespace

std::string mvsErrorText(int code) {
    std::ostringstream out;
    out << "0x" << std::hex << static_cast<unsigned int>(code) << " " << mvsErrorName(code);
    return out.str();
}

SdkError::SdkError(int code, std::string operation, std::string hint)
    : std::runtime_error([&]() {
          std::ostringstream out;
          out << operation << " failed: " << mvsErrorText(code);
          if (!hint.empty()) {
              out << "\n  hint: " << hint;
          }
          return out.str();
      }()),
      code_(code),
      operation_(std::move(operation)),
      hint_(std::move(hint)) {}

void checkMvs(int code, const std::string& operation, const std::string& hint) {
    if (code != MV_OK) {
        throw SdkError(code, operation, hint);
    }
}

}  // namespace hikquad
