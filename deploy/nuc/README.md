# NUC deployment staging

Prepared on 2026-09-10 for STEMM Jetson-to-x86 migration.

This directory is a staging area only. No service unit in `systemd-units-disabled/` has been copied to `/etc/systemd/system`, enabled, or started.

Applied NUC substitutions:
- service user: `hnh`
- ROS workspace: `/home/hnh/wheeltec_ros2`
- deployment scripts: `/opt/demo-control/deploy/nuc`
- Wi-Fi interface: `wlp4s0`

Deferred until the corresponding ROS packages build and hardware smoke tests pass:
- all robot motion and follow services
- automatic recharge services
- MQTT command bridges and map services
- boot enablement

Deliberately excluded from this core staging set:
- Foxglove
- VNC/noVNC
- Wi-Fi portal
- IP-to-serial helper
- privileged device-restart orchestrator

- `stemm-astra-camera.service`: headless Astra S color/depth driver with registered depth; staged and disabled.
