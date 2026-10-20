#pragma once

#include <string>
#include <vector>

namespace hikquad {

struct UsbLinkInfo {
    bool found = false;
    int bus = 0;
    int device = 0;
    int speed_mbps = 0;
    std::string version;
    std::string port_path;
    std::string sysfs_path;
};

struct DeviceInfo {
    int index = 0;
    std::string serial;
    std::string model;
    std::string vendor;
    std::string family;
    std::string firmware;
    unsigned int usb_address = 0;
    unsigned int bcd_usb = 0;
    bool virtual_device = false;
    UsbLinkInfo link;

    std::string displayName() const;
};

/// Enumerates USB3 Vision cameras through MV_CC_EnumDevicesEx2().
/// Ex2 excludes the persistent MVS virtual camera (serial starts with "Vir").
std::vector<DeviceInfo> enumerateUsbDevices(bool include_virtual = false);

/// Reads /sys/bus/usb/devices for the serial.  Empty record when unavailable.
UsbLinkInfo findUsbLink(const std::string& serial);

std::string formatDeviceTable(const std::vector<DeviceInfo>& devices);

}  // namespace hikquad
