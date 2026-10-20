#pragma once

#include <string>

#include "calibration.hpp"

namespace hikquad {

struct PanoramaConfig;

void writeIntrinsicsYml(const std::string& path, const Intrinsics& intrinsics);
void writeExtrinsicsYml(const std::string& path, const Extrinsics& extrinsics);
Intrinsics readIntrinsicsYml(const std::string& path);
Extrinsics readExtrinsicsYml(const std::string& path);

void writeCalibMetaJson(const std::string& path, int camera_count, const BoardSpec& board,
                        const PanoramaConfig& panorama, const CalibrationResult& result);
void writeCalibrationReportJson(const std::string& path, const CalibrationResult& result);
std::string formatCalibrationSummary(const CalibrationResult& result);

}  // namespace hikquad
