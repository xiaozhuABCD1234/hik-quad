#include <iostream>
#include <string>
#include <vector>

#include "app_common.hpp"
#include "hikquad/camera.hpp"
#include "hikquad/device_info.hpp"
#include "hikquad/version.hpp"

namespace {

void printEnvironment() {
    const int usb_memory = hikquad::app::usbMemoryMb();
    std::cout << "MVS SDK      : " << hikquad::sdkVersion() << "\n";
    std::cout << "hik-quad     : " << hikquad::versionString() << " (" << hikquad::buildType()
              << ", " << hikquad::compilerString() << ")\n";
    std::cout << "usbfs_memory : "
              << (usb_memory > 0 ? std::to_string(usb_memory) + " MB" : "unknown") << "\n";
    std::cout << "udev rule    : " << (hikquad::app::udevRuleInstalled() ? "present" : "missing")
              << "\n";
    if (usb_memory > 0 && usb_memory < 256) {
        std::cout << "warning      : usbfs_memory_mb is small; four 1440x1080 USB3 cameras "
                     "usually need >= 1000 MB (see docs/usb-tuning.md)\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    hikquad::CliParser cli(argc, argv);
    const std::vector<std::string> allowed = {"--json",  "--include-virtual", "--verbose",
                                              "--quiet", "--log-level",       "--help"};
    std::string error;
    if (!cli.validate(allowed, &error)) {
        std::cerr << error << "\n";
        return 2;
    }
    if (cli.has("--help")) {
        hikquad::CliParser::printUsage(
            argv[0],
            "List Hikrobot USB3 Vision cameras and USB link information.\n\n"
            "  --json               machine readable output\n"
            "  --include-virtual    also show the MVS virtual camera\n"
            "  --verbose            debug logging\n");
        return 0;
    }
    hikquad::app::initLogging(cli);

    try {
        const bool include_virtual = cli.has("--include-virtual");
        const std::vector<hikquad::DeviceInfo> devices =
            hikquad::enumerateUsbDevices(include_virtual);
        if (cli.has("--json")) {
            std::cout << "{\"sdk_version\":\"" << hikquad::sdkVersion() << "\",\"devices\":[";
            for (size_t i = 0; i < devices.size(); ++i) {
                if (i != 0) {
                    std::cout << ",";
                }
                std::cout << hikquad::toJson(devices[i]);
            }
            std::cout << "]}\n";
        } else {
            printEnvironment();
            std::cout << "\nphysical USB3 Vision cameras: " << devices.size() << "\n\n";
            std::cout << hikquad::formatDeviceTable(devices);
            if (devices.size() < 4) {
                std::cout << "\nnote: fewer than 4 physical cameras are visible. Check USB cables, "
                             "power and docs/usb-tuning.md.\n";
            }
        }
        return devices.empty() ? 1 : 0;
    } catch (const std::exception& exception) {
        std::cerr << "error: " << exception.what() << "\n";
        return 1;
    }
}
