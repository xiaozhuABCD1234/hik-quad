#include "hikquad/version.hpp"

namespace hikquad {

const char* versionString() noexcept {
    return HIK_QUAD_VERSION;
}

const char* buildType() noexcept {
#ifdef HIK_QUAD_BUILD_TYPE
    return HIK_QUAD_BUILD_TYPE;
#else
    return "unknown";
#endif
}

const char* compilerString() noexcept {
#if defined(__clang__)
    return "clang " __clang_version__;
#elif defined(__GNUC__)
    return "gcc " __VERSION__;
#elif defined(_MSC_VER)
    return "msvc";
#else
    return "unknown";
#endif
}

}  // namespace hikquad
