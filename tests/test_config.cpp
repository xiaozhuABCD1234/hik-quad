#include <filesystem>
#include <fstream>

#include "hikquad/config.hpp"
#include "test_framework.hpp"

HIK_TEST(config_loads_overrides) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "hikquad_test_config.json";
    {
        std::ofstream stream(path);
        stream << R"({
      "camera_count": 2,
      "usb": {"image_node_num": 4, "grab_timeout_ms": 1500, "grab_strategy": "LatestImages"},
      "defaults": {"width": 640, "height": 480, "exposure_us": 2500.0},
      "cameras": [
        {"serial": "CAM-A", "gain_db": 3.0},
        {"serial": "CAM-B", "exposure_us": 9000.0}
      ],
      "panorama": {"out_width": 1024, "out_height": 256, "fov_h_deg": 180.0}
    })";
    }
    const hikquad::AppConfig config = hikquad::loadConfig(path.string());
    CHECK_EQ(config.array.camera_count, 2);
    CHECK_EQ(config.array.cameras.size(), size_t(2));
    CHECK_EQ(config.array.cameras[0].serial, std::string("CAM-A"));
    CHECK_EQ(config.array.cameras[0].width, 640);
    CHECK_EQ(config.array.cameras[0].height, 480);
    CHECK_NEAR(config.array.cameras[0].gain_db, 3.0, 1e-9);
    CHECK_NEAR(config.array.cameras[1].exposure_us, 9000.0, 1e-9);
    CHECK_EQ(config.usb.image_node_num, 4);
    CHECK_EQ(config.usb.grab_timeout_ms, 1500);
    CHECK_EQ(config.panorama.out_width, 1024);
    CHECK_NEAR(config.panorama.fov_h_deg, 180.0, 1e-9);
    std::filesystem::remove(path);
}

HIK_TEST(config_parses_booleans) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "hikquad_test_bool_config.json";
    {
        std::ofstream stream(path);
        stream << R"({
      "camera_count": 1,
      "capture": {"software_trigger": true},
      "defaults": {"frame_rate_enable": false, "gamma_enable": true, "keep_raw": true}
    })";
    }
    const hikquad::AppConfig config = hikquad::loadConfig(path.string());
    CHECK_FALSE(config.array.cameras[0].frame_rate_enable);
    CHECK_TRUE(config.array.cameras[0].gamma_enable);
    CHECK_TRUE(config.array.cameras[0].keep_raw);
    CHECK_TRUE(config.capture.software_trigger);
    std::filesystem::remove(path);
}

HIK_TEST(config_rejects_bad_board) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "hikquad_test_bad_config.json";
    {
        std::ofstream stream(path);
        stream << R"({"camera_count": 1, "calibration": {"pattern_cols": 2}})";
    }
    bool threw = false;
    try {
        (void)hikquad::loadConfig(path.string());
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK_TRUE(threw);
    std::filesystem::remove(path);
}
