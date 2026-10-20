#include <opencv2/imgproc.hpp>

#include "hikquad/pixel_format.hpp"
#include "test_framework.hpp"

HIK_TEST(pixel_format_names) {
    const auto* bayer = hikquad::pixelFormatByName("BayerRG8");
    CHECK_TRUE(bayer != nullptr);
    CHECK_EQ(bayer->type, PixelType_Gvsp_BayerRG8);
    CHECK_TRUE(hikquad::isBayer(bayer->type));
    CHECK_EQ(hikquad::bytesPerPixel(bayer->type), 1);
    CHECK_EQ(hikquad::pixelFormatName(PixelType_Gvsp_BGR8_Packed), std::string("BGR8Packed"));
    CHECK_TRUE(hikquad::pixelFormatByName("does-not-exist") == nullptr);
}

HIK_TEST(pixel_format_expected_frame_bytes) {
    CHECK_EQ(hikquad::expectedFrameBytes(PixelType_Gvsp_BayerRG8, 1440, 1080), size_t(1555200));
    CHECK_EQ(hikquad::expectedFrameBytes(PixelType_Gvsp_BayerRG12, 1440, 1080), size_t(3110400));
    CHECK_EQ(hikquad::expectedFrameBytes(PixelType_Gvsp_BGR8_Packed, 640, 480), size_t(921600));
    CHECK_EQ(hikquad::expectedFrameBytes(PixelType_Gvsp_BayerRG10_Packed, 1440, 1080), size_t(0));
}

HIK_TEST(pixel_format_bayer_mapping) {
    CHECK_EQ(hikquad::bayerPatternIndex(PixelType_Gvsp_BayerRG8), 0);
    CHECK_EQ(hikquad::bayerPatternIndex(PixelType_Gvsp_BayerBG8), 1);
    CHECK_EQ(hikquad::bayerPatternIndex(PixelType_Gvsp_BayerGB8), 2);
    CHECK_EQ(hikquad::bayerPatternIndex(PixelType_Gvsp_BayerGR8), 3);
    CHECK_EQ(hikquad::openCvBayerCode(PixelType_Gvsp_BayerRG8), cv::COLOR_BayerBG2BGR);
    CHECK_EQ(hikquad::openCvBayerCode(PixelType_Gvsp_BayerBG8), cv::COLOR_BayerRG2BGR);
    CHECK_EQ(hikquad::openCvBayerCode(PixelType_Gvsp_BayerGB8), cv::COLOR_BayerGR2BGR);
    CHECK_EQ(hikquad::openCvBayerCode(PixelType_Gvsp_BayerGR8), cv::COLOR_BayerGB2BGR);
}
