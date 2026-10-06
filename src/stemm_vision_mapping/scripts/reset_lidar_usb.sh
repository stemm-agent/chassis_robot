#!/bin/bash
# reset_lidar_usb.sh -- force the LIDAR's USB link to re-enumerate, then restart its driver.
#
#   sudo bash ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/reset_lidar_usb.sh
#
# Why by serial and not by bus/device number: /dev/bus/usb/BBB/DDD numbers change on
# every re-enumeration (that is why `usbreset /dev/bus/usb/001/009` says "No such device").
# The udev rules map  1a86:55d4 serial 0001 -> /dev/wheeltec_lidar   (lidar)
#                     1a86:55d4 serial 0002 -> /dev/wheeltec_controller (chassis)
# so this script only ever touches serial 0001 and leaves the chassis alone.
set -u
TARGET_SERIAL="${1:-0001}"

echo "=== USB devices matching 1a86:55d4 ==="
PORT=""
for d in /sys/bus/usb/devices/*/; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor" 2>/dev/null)" = "1a86" ] || continue
  [ "$(cat "$d/idProduct" 2>/dev/null)" = "55d4" ] || continue
  name=$(basename "$d")
  ser=$(cat "$d/serial" 2>/dev/null || echo "?")
  printf "  port %-8s serial=%-5s authorized=%s tty=%s\n" "$name" "$ser" \
         "$(cat "$d/authorized" 2>/dev/null)" \
         "$(ls -1 "$d" 2>/dev/null | grep -E '^ttyACM[0-9]+$' | head -1)"
  [ "$ser" = "$TARGET_SERIAL" ] && PORT="$name"
done

if [ -z "$PORT" ]; then
  echo
  echo "!! no 1a86:55d4 device with serial $TARGET_SERIAL is on the USB bus."
  echo "   The bridge chip itself is gone -> re-plug the lidar USB cable (or power-cycle the lidar)."
  exit 2
fi

echo
echo "=== re-enumerating port $PORT (this does NOT touch serial 0002 / the chassis) ==="
if [ -w "/sys/bus/usb/devices/$PORT/authorized" ]; then
  echo 0 > "/sys/bus/usb/devices/$PORT/authorized"; sleep 2
  echo 1 > "/sys/bus/usb/devices/$PORT/authorized"; sleep 3
  echo "  authorized toggle done"
else
  echo "  !! authorized not writable, falling back to unbind/bind"
  echo -n "$PORT" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null || true
  sleep 2
  echo -n "$PORT" > /sys/bus/usb/drivers/usb/bind 2>/dev/null || true
  sleep 3
fi
echo "  symlink now: $(ls -l /dev/wheeltec_lidar 2>&1)"

echo
echo "=== restarting the lidar driver ==="
systemctl restart wheeltec-lidar.service
sleep 6
printf "  service: "; systemctl is-active wheeltec-lidar.service

echo
echo "=== verify as the normal user (not root) ==="
echo "    ros2 topic hz /scan            # expect ~12 Hz"
echo "    if still nothing: the sensor side is wedged -> re-plug the lidar USB / power-cycle the robot"
