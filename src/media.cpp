#include "hikquad/media.hpp"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cstddef>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace hikquad {
namespace fs = std::filesystem;
namespace {

const char* const kImageExtensions[] = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"};
const char* const kVideoExtensions[] = {".mp4", ".avi", ".mkv", ".mov", ".m4v"};

template <size_t N>
int rankOf(const char* const (&values)[N], const std::string& value) {
    for (size_t i = 0; i < N; ++i) {
        if (value == values[i]) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

constexpr int kVideoRankOffset = 100;  // images rank first

}  // namespace

std::string lowerExtension(const std::string& path) {
    std::string extension = fs::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension;
}

MediaKind mediaKind(const std::string& path) {
    const std::string extension = lowerExtension(path);
    if (rankOf(kImageExtensions, extension) >= 0) {
        return MediaKind::Image;
    }
    if (rankOf(kVideoExtensions, extension) >= 0) {
        return MediaKind::Video;
    }
    return MediaKind::None;
}

bool isImagePath(const std::string& path) {
    return mediaKind(path) == MediaKind::Image;
}

bool isVideoPath(const std::string& path) {
    return mediaKind(path) == MediaKind::Video;
}

std::vector<std::string> cameraFileStems(int camera_index) {
    const std::string index = std::to_string(camera_index);
    return {"cam" + index, "camera_" + index, "cam_" + index, index};
}

CameraMedia findCameraMedia(const std::string& directory, int camera_index) {
    CameraMedia media;
    std::error_code ec;
    if (!fs::is_directory(directory, ec)) {
        return media;
    }

    const std::vector<std::string> stems = cameraFileStems(camera_index);
    int best_stem = INT_MAX;
    int best_extension = INT_MAX;
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string extension = lowerExtension(entry.path().string());
        bool video = false;
        int extension_rank = rankOf(kImageExtensions, extension);
        if (extension_rank < 0) {
            extension_rank = rankOf(kVideoExtensions, extension);
            if (extension_rank < 0) {
                continue;
            }
            extension_rank += kVideoRankOffset;
            video = true;
        }
        const std::string stem = entry.path().stem().string();
        const auto found = std::find(stems.begin(), stems.end(), stem);
        if (found == stems.end()) {
            continue;
        }
        const int stem_rank = static_cast<int>(found - stems.begin());
        if (stem_rank < best_stem || (stem_rank == best_stem && extension_rank < best_extension)) {
            best_stem = stem_rank;
            best_extension = extension_rank;
            media.path = entry.path().string();
            media.kind = video ? MediaKind::Video : MediaKind::Image;
        }
    }
    return media;
}

}  // namespace hikquad
