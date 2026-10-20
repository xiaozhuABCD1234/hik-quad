#include "hikquad/device_info.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

#include "MvCameraControl.h"

namespace hikquad {
namespace fs = std::filesystem;

namespace {

std::string decode(const unsigned char* data, size_t capacity) {
    size_t length = 0;
    while (length < capacity && data[length] != 0) {
        ++length;
    }
    std::string value(reinterpret_cast<const char*>(data), length);
    while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
        value.pop_back();
    }
    return value;
}

bool readFile(const fs::path& path, std::string* value) {
    std::ifstream stream(path);
    if (!stream.good()) {
        return false;
    }
    std::getline(stream, *value);
    while (!value->empty() && (value->back() == '\n' || value->back() == '\r')) {
        value->pop_back();
    }
    return true;
}

int readInt(const fs::path& path, int fallback) {
    std::string text;
    if (!readFile(path, &text)) {
        return fallback;
    }
    try {
        return std::stoi(text);
    } catch (...) {
        return fallback;
    }
}

DeviceInfo fromSdk(const MV_CC_DEVICE_INFO& raw, int index, bool include_virtual) {
    DeviceInfo info;
    info.index = index;
    if (raw.nTLayerType != MV_USB_DEVICE) {
        return info;
    }
    const MV_USB3_DEVICE_INFO& usb = raw.SpecialInfo.stUsb3VInfo;
    info.serial = decode(usb.chSerialNumber, sizeof(usb.chSerialNumber));
    info.model = decode(usb.chModelName, sizeof(usb.chModelName));
    info.vendor = decode(usb.chVendorName, sizeof(usb.chVendorName));
    info.family = decode(usb.chFamilyName, sizeof(usb.chFamilyName));
    info.firmware = decode(usb.chDeviceVersion, sizeof(usb.chDeviceVersion));
    info.usb_address = usb.nDeviceAddress;
    info.bcd_usb = usb.nbcdUSB;
    info.virtual_device = info.serial.rfind("Vir", 0) == 0;
    if (!include_virtual && info.virtual_device) {
        info.serial.clear();
        return info;
    }
    info.link = findUsbLink(info.serial);
    return info;
}

}  // namespace

std::string DeviceInfo::displayName() const {
    std::ostringstream out;
    out << model;
    if (!serial.empty()) {
        out << " [" << serial << "]";
    }
    return out.str();
}

UsbLinkInfo findUsbLink(const std::string& serial) {
    UsbLinkInfo link;
    if (serial.empty()) {
        return link;
    }
    const fs::path base("/sys/bus/usb/devices");
    std::error_code ec;
    if (!fs::is_directory(base, ec)) {
        return link;
    }
    for (const auto& entry : fs::directory_iterator(base, ec)) {
        const fs::path serial_path = entry.path() / "serial";
        std::string candidate;
        if (!readFile(serial_path, &candidate) || candidate != serial) {
            continue;
        }
        link.found = true;
        link.sysfs_path = entry.path().string();
        link.port_path = entry.path().filename().string();
        link.bus = readInt(entry.path() / "busnum", 0);
        link.device = readInt(entry.path() / "devnum", 0);
        link.speed_mbps = readInt(entry.path() / "speed", 0);
        readFile(entry.path() / "version", &link.version);
        return link;
    }
    return link;
}

std::vector<DeviceInfo> enumerateUsbDevices(bool include_virtual) {
    MV_CC_DEVICE_INFO_LIST list{};
    int result = static_cast<int>(MV_E_SUPPORT);
    if (include_virtual) {
        result = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    } else {
        result = MV_CC_EnumDevicesEx2(MV_USB_DEVICE, &list, "Hikrobot", SortMethod_SerialNumber);
    }
    if (result != MV_OK) {
        // Older SDKs do not provide Ex2; fall back to the compatible enumeration and
        // filter virtual devices by serial prefix below.
        result = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    }
    if (result != MV_OK) {
        return {};
    }

    std::vector<DeviceInfo> devices;
    devices.reserve(list.nDeviceNum);
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        if (list.pDeviceInfo[i] == nullptr) {
            continue;
        }
        DeviceInfo info = fromSdk(*list.pDeviceInfo[i], static_cast<int>(i), include_virtual);
        if (info.serial.empty() && !info.model.empty() && !include_virtual) {
            continue;  // virtual camera filtered by fromSdk()
        }
        if (info.model.empty() && info.serial.empty()) {
            continue;
        }
        devices.push_back(std::move(info));
    }
    std::stable_sort(devices.begin(), devices.end(),
                     [](const DeviceInfo& a, const DeviceInfo& b) { return a.serial < b.serial; });
    for (size_t i = 0; i < devices.size(); ++i) {
        devices[i].index = static_cast<int>(i);
    }
    return devices;
}

std::string formatDeviceTable(const std::vector<DeviceInfo>& devices) {
    std::ostringstream out;
    out << std::left << std::setw(4) << "#" << std::setw(14) << "Serial" << std::setw(24) << "Model"
        << std::setw(14) << "USB port" << std::setw(10) << "Speed";
    out << "\n";
    out << std::string(66, '-') << "\n";
    for (const auto& device : devices) {
        std::string port = "-";
        std::string speed = "-";
        if (device.link.found) {
            port = device.link.port_path;
            speed = std::to_string(device.link.speed_mbps) + "M";
        }
        out << std::left << std::setw(4) << device.index << std::setw(14) << device.serial
            << std::setw(24) << device.model << std::setw(14) << port << std::setw(10) << speed;
        if (device.virtual_device) {
            out << " virtual";
        }
        out << "\n";
    }
    return out.str();
}

}  // namespace hikquad
