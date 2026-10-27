#pragma once

#include <string>

#include "calibration.hpp"

namespace hikquad {

void writeIntrinsicsYml(const std::string& path, const Intrinsics& intrinsics);
void writeExtrinsicsYml(const std::string& path, const Extrinsics& extrinsics);
Intrinsics readIntrinsicsYml(const std::string& path);
Extrinsics readExtrinsicsYml(const std::string& path);

}  // namespace hikquad
