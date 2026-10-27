#pragma once

#include <string>
#include <vector>

namespace hikquad {

enum class MediaKind { None, Image, Video };

/// Lowercase extension including the dot (".PNG" becomes ".png").  Empty when
/// the path has no extension.
std::string lowerExtension(const std::string& path);

MediaKind mediaKind(const std::string& path);
bool isImagePath(const std::string& path);
bool isVideoPath(const std::string& path);

/// Candidate file stems for one camera, most specific first:
/// cam0, camera_0, cam_0, 0.
std::vector<std::string> cameraFileStems(int camera_index);

struct CameraMedia {
    std::string path;
    MediaKind kind = MediaKind::None;

    bool empty() const {
        return path.empty();
    }
};

/// Locates the media file for one camera inside a directory.  When several
/// candidates exist the stem order above wins, then images over videos, then
/// the extension order used by the tools.
CameraMedia findCameraMedia(const std::string& directory, int camera_index);

}  // namespace hikquad
