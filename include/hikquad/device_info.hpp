#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikquad {

struct UsbLinkInfo {
    bool found = false;
    int speed_mbps = 0;
    std::string port_path;
};

struct DeviceInfo {
    std::string serial;
    std::string model;
    UsbLinkInfo link;

    std::string displayName() const;
};

/// A physical camera together with the raw MVS record needed to open it.
struct UsbDevice {
    DeviceInfo info;
    MV_CC_DEVICE_INFO raw{};
};

/// Decodes a NUL-terminated, fixed-width SDK string and trims trailing blanks.
std::string decodeSdkString(const unsigned char* data, size_t capacity);

/// Enumerates physical USB3 Vision cameras (the MVS virtual camera is filtered
/// out), ordered by serial number.  empty on enumeration failure.
std::vector<UsbDevice> enumerateUsbDeviceRecords();

/// Convenience view of enumerateUsbDeviceRecords() without the raw records.
std::vector<DeviceInfo> enumerateUsbDevices();

/// Reads /sys/bus/usb/devices for the serial. Empty record when unavailable.
UsbLinkInfo findUsbLink(const std::string& serial);

}  // namespace hikquad
