#!/usr/bin/env bash
# One-time USB tuning for Hikrobot USB3 Vision cameras.  Run with sudo:
#   sudo ./scripts/install_usb_tuning.sh
#
# It is safe to re-run.  The 4-camera rig on this machine shares one 5 Gbps hub
# for two cameras, so usbfs memory and USB autosuspend are worth setting even
# though the application also enforces frame integrity.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run with sudo: sudo $0" >&2
  exit 1
fi

MODPROBE_FILE=/etc/modprobe.d/99-hik-quad-usb.conf
UDEV_FILE=/etc/udev/rules.d/99-hik-quad-usb.rules

cat > "$MODPROBE_FILE" <<'EOF'
# hik-quad: four 1440x1080 USB3 Vision cameras
options usbcore usbfs_memory_mb=4000
EOF

cat > "$UDEV_FILE" <<'EOF'
# hik-quad: keep Hikrobot USB3 Vision cameras powered, no runtime autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", ATTR{power/control}="on", MODE="0666"
EOF

# Apply the usbfs limit immediately (module parameter is writable at runtime).
if [ -w /sys/module/usbcore/parameters/usbfs_memory_mb ]; then
  echo 4000 > /sys/module/usbcore/parameters/usbfs_memory_mb
fi

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb || true

echo "usbfs_memory_mb = $(cat /sys/module/usbcore/parameters/usbfs_memory_mb)"
echo "wrote $MODPROBE_FILE"
echo "wrote $UDEV_FILE"
echo "replug the cameras (or reboot) to apply the udev power rule."
echo
echo "Verify with:"
echo "  ./scripts/check_usb.sh"
echo "  ./build/release/apps/hik_quad_capture --frames 1 --fps 20 --no-save"
