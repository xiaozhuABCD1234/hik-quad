#!/usr/bin/env bash
# Print the USB topology, kernel usbfs limits and a bandwidth budget for the
# four 1440x1080 BayerRG8 cameras.  Read-only: no sudo, no system changes.
set -u

echo "== host =="
uname -a

echo
echo "== usbfs memory =="
if [ -r /sys/module/usbcore/parameters/usbfs_memory_mb ]; then
  value=$(cat /sys/module/usbcore/parameters/usbfs_memory_mb)
  echo "usbfs_memory_mb = ${value}"
  if [ "${value}" -lt 1024 ]; then
    echo "WARNING: < 1024 MB; four 1440x1080 USB3 cameras may fail to start."
    echo "         See docs/usb-tuning.md for the persistent modprobe setting."
  fi
else
  echo "usbfs_memory_mb: not available"
fi

echo
echo "== udev rule =="
if [ -e /etc/udev/rules.d/80-drivers-SDK-2bdf.rules ]; then
  echo "present: /etc/udev/rules.d/80-drivers-SDK-2bdf.rules"
else
  echo "MISSING: reinstall MVS or add the Hikrobot udev rule."
fi

echo
echo "== lsusb -t =="
if command -v lsusb >/dev/null 2>&1; then
  lsusb -t
else
  echo "lsusb not installed (apt install usbutils)"
fi

echo
echo "== Hikrobot USB devices (sysfs) =="
found=0
declare -A per_bus
for dir in /sys/bus/usb/devices/*; do
  [ -f "${dir}/idVendor" ] || continue
  [ "$(cat "${dir}/idVendor")" = "2bdf" ] || continue
  found=$((found + 1))
  bus=$(cat "${dir}/busnum" 2>/dev/null || echo "?")
  speed=$(cat "${dir}/speed" 2>/dev/null || echo "?")
  serial=$(cat "${dir}/serial" 2>/dev/null || echo "?")
  echo "  bus=${bus} port=${dir##*/} speed=${speed}M serial=${serial}"
  per_bus["${bus}"]=$(( ${per_bus["${bus}"]:-0} + 1 ))
done
echo "physical Hikrobot cameras: ${found}"

if [ "${found}" -eq 0 ]; then
  echo "No camera found. Check cable/power and 'lsusb'."
fi

echo
for bus in "${!per_bus[@]}"; do
  count=${per_bus["${bus}"]}
  echo "bus ${bus}: ${count} camera(s)"
  if [ "${count}" -ge 3 ]; then
    echo "  WARNING: 3+ cameras on one USB3 bus.  165 fps x 3 is ~6.2 Gbps,"
    echo "           above a single 5 Gbps link.  Lower --fps or re-cable."
  fi
done

echo
echo "== bandwidth budget (1440x1080, BayerRG8 = 1 byte/px) =="
awk 'BEGIN {
  printf "  %6s %12s %10s %12s\n", "fps", "MB/s/cam", "Gbps/cam", "3 cams Gbps";
  split("30 60 100 165", rates, " ");
  for (i = 1; i <= 4; ++i) {
    fps = rates[i];
    mb = 1440 * 1080 * fps / 1000000.0;
    gbps = mb * 8 / 1000.0;
    printf "  %6d %12.1f %10.2f %12.2f\n", fps, mb, gbps, gbps * 3;
  }
}'

echo
echo "Camera node availability on MV-CB016-10UC-S (from MVS SDK):"
echo "  DeviceLinkThroughputLimitMode : yes"
echo "  DeviceLinkThroughputLimit     : no  (0x80000106)"
echo "  TransferSize                 : no  (0x80000109)"
echo "  AcquisitionFrameRate         : yes (use this to fit the bus)"
