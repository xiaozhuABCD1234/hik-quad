#pragma once

#include <string>

namespace hikquad {

std::string executableDirectory();
std::string projectRoot();
std::string joinPath(const std::string& base, const std::string& path);

}  // namespace hikquad
