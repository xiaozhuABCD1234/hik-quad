#include "hikquad/paths.hpp"

#include <cstdlib>
#include <filesystem>
#include <system_error>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace hikquad {
namespace fs = std::filesystem;

std::string executableDirectory() {
#if defined(__linux__)
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        return exe.parent_path().string();
    }
#endif
    return {};
}

std::string joinPath(const std::string& base, const std::string& path) {
    if (path.empty()) {
        return base;
    }
    const fs::path p(path);
    if (p.is_absolute()) {
        return p.string();
    }
    return (fs::path(base) / p).string();
}

std::string projectRoot() {
    std::error_code ec;
    fs::path dir = fs::current_path(ec);
    const fs::path exe_dir = executableDirectory();
    for (int i = 0; i < 8; ++i) {
        if (fs::exists(dir / "config" / "default.json", ec)) {
            return dir.string();
        }
        if (!dir.has_parent_path() || dir.parent_path() == dir) {
            break;
        }
        dir = dir.parent_path();
    }
    if (!exe_dir.empty()) {
        dir = exe_dir;
        for (int i = 0; i < 8; ++i) {
            if (fs::exists(dir / "config" / "default.json", ec)) {
                return dir.string();
            }
            if (!dir.has_parent_path() || dir.parent_path() == dir) {
                break;
            }
            dir = dir.parent_path();
        }
    }
    return {};
}

}  // namespace hikquad
