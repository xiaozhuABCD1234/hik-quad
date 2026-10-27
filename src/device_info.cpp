#include "hikquad/device_info.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

namespace hikquad {
namespace fs = std::filesystem;

namespace {

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

}  // namespace

std::string decodeSdkString(const unsigned char* data, size_t capacity) {
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

std::string DeviceInfo::displayName() const {
    std::string name = model;
    if (!serial.empty()) {
        name += " [" + serial + "]";
    }
    return name;
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
        std::string candidate;
        if (!readFile(entry.path() / "serial", &candidate) || candidate != serial) {
            continue;
        }
        link.found = true;
        link.port_path = entry.path().filename().string();
        link.speed_mbps = readInt(entry.path() / "speed", 0);
        return link;
    }
    return link;
}

std::vector<UsbDevice> enumerateUsbDeviceRecords() {
    MV_CC_DEVICE_INFO_LIST list{};
    int result = MV_CC_EnumDevicesEx2(MV_USB_DEVICE, &list, "Hikrobot", SortMethod_SerialNumber);
    if (result != MV_OK) {
        // Older SDKs do not provide Ex2; the virtual camera is filtered below.
        result = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    }
    if (result != MV_OK) {
        return {};
    }

    std::vector<UsbDevice> devices;
    devices.reserve(list.nDeviceNum);
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        const MV_CC_DEVICE_INFO* raw = list.pDeviceInfo[i];
        if (raw == nullptr || raw->nTLayerType != MV_USB_DEVICE) {
            continue;
        }
        const MV_USB3_DEVICE_INFO& usb = raw->SpecialInfo.stUsb3VInfo;
        UsbDevice device;
        device.info.serial = decodeSdkString(usb.chSerialNumber, sizeof(usb.chSerialNumber));
        if (device.info.serial.empty() || device.info.serial.rfind("Vir", 0) == 0) {
            continue;  // unreadable serial or the MVS virtual camera
        }
        device.raw = *raw;
        device.info.model = decodeSdkString(usb.chModelName, sizeof(usb.chModelName));
        device.info.link = findUsbLink(device.info.serial);
        devices.push_back(std::move(device));
    }
    std::stable_sort(devices.begin(), devices.end(), [](const UsbDevice& a, const UsbDevice& b) {
        return a.info.serial < b.info.serial;
    });
    return devices;
}

std::vector<DeviceInfo> enumerateUsbDevices() {
    const std::vector<UsbDevice> devices = enumerateUsbDeviceRecords();
    std::vector<DeviceInfo> infos;
    infos.reserve(devices.size());
    for (const auto& device : devices) {
        infos.push_back(device.info);
    }
    return infos;
}

}  // namespace hikquad
