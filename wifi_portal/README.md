# Wi-Fi Portal Deployment Notes

This directory implements step 5 from the single-NIC Wi-Fi provisioning plan:

1. Boot the robot into AP mode.
2. Start the local provisioning portal on `http://10.42.0.1:8080`.
3. Let the phone submit target Wi-Fi credentials.
4. Launch the switch script to move from AP to STA mode.
5. If STA fails, roll back to AP mode.

## Files

- `wifi_portal.py`: web portal that accepts Wi-Fi credentials.
- `wifi_switch.sh`: AP-to-STA switch and rollback script.
- `wifi-portal.service`: systemd unit for auto-starting the portal.

## Install the systemd unit

Copy the service file into systemd and reload the daemon:

```bash
sudo cp /home/wheeltec/wheeltec_ros2/wifi_portal/wifi-portal.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wifi-portal.service
```

## Start and inspect the service

```bash
sudo systemctl start wifi-portal.service
sudo systemctl status wifi-portal.service
journalctl -u wifi-portal.service -n 50 --no-pager
```

## Expected runtime behavior

- The service runs as `root` so it can later trigger `nmcli` network changes.
- The portal stores runtime state under `/home/wheeltec/wheeltec_ros2/wifi_portal/data/`.
- The phone connects to the robot hotspot and opens `http://10.42.0.1:8080`.
- After submit, the portal launches `wifi_switch.sh`.
- On success, the status switches to `sta_connected`.
- On failure, the status switches back to `ap_ready` and AP mode is restored.

## Manual verification checklist

1. Ensure the AP connection `my-robot-ap` is up before starting the portal service.
2. Open `http://10.42.0.1:8080` from a phone connected to the hotspot.
3. Submit a valid Wi-Fi SSID and password.
4. Confirm that `data/current_status.json` changes from `switching` to `sta_connected` or back to `ap_ready`.
5. If AP disconnects, reconnect the phone to the target Wi-Fi and find the robot IP from the router DHCP list.

## Notes

- This service file intentionally matches the earlier plan's step 5: `systemd` starts the web portal after `NetworkManager` is available.
- The service does not create the AP itself. AP setup remains a separate prerequisite.
- If you later change interface names or AP settings, update the `Environment=` lines in `wifi-portal.service`.
