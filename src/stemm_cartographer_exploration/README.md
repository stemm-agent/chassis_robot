# stemm_cartographer_exploration

Independent Cartographer 2D autonomous-exploration stack for the Wheeltec robot.
It deliberately does not include or modify `stemm_21_auto_mapping.launch.py` or
the legacy Wheeltec Cartographer Lua file.

## Public entry points

- External/session entry: `stemm_cartographer_session.launch.py`
- Inner mapping stack: `stemm_cartographer_auto_mapping.launch.py`
- ROS services:
  - `/stemm_cartographer/start_mapping`
  - `/stemm_cartographer/save_map`
  - `/stemm_cartographer/stop_navigation`
  - `/stemm_cartographer/set_mapping_done`
  - `/stemm_cartographer/save_state`

The session entry must be used for normal operation. It stops only the fixed
functional-service allowlist through the root-owned service guard, starts the
inner launch in a separate process group, and restarts the fixed services on
every normal or abnormal exit.

## Frames and sensors

- Cartographer inputs: `/scan`, raw wheel odometry `/odom`, and compensated IMU
  `/imu/data_bias_compensated`.
- Cartographer publishes `map -> odom_combined`.
- The existing EKF remains the only publisher of
  `odom_combined -> base_footprint`.
- Nav2 continues to use `/odom_combined`. Its configuration and behavior tree
  are private to this package; the original STEMM/bodyfollow Nav2 files remain
  unchanged.

## Nav2 readiness resilience

This package launches its own `stemm_cartographer_nav2_manager` subclass; the
existing `stemm_nav2_manager` package is not modified. Lifecycle `GetState`
requests time out after 1.5 seconds, remove their stale pending request, and
retry with a bounded 0.5-to-2.0-second backoff. The independent Nav2 readiness
guard uses the same policy.

While Nav2 is unavailable, incoming RRT frontier goals are ignored before map
validation so a transient lifecycle-service failure cannot pollute the failed
goal blacklist. A second frontier is also ignored while an action goal is still
waiting for Nav2 acceptance.

## Bounded exploration recovery

Only RRT and manager-generated exploration goals use the package-specific
recovery policy. Manual goals and return-home retain the inherited policy. The
private Nav2 progress checker requires 0.05 m of movement within 10 seconds, so
Nav2 begins recovery at the requested timescale without the manager canceling
its behavior tree first.

The exploration behavior tree handles failures in stages. Planner failures
first clear the global costmap, controller failures first clear the local
costmap, and repeated failures advance through clear, collision-checked Spin,
and Wait actions. BackUp remains disabled because rear clearance is not trusted.

The manager is a later safety fallback, not a competing recovery owner. Its
rolling no-progress window is 18 seconds with a 0.12 m remaining-distance
threshold. Valid commanded TF motion also counts when translation reaches
0.05 m or rotation reaches 0.12 rad; TF noise without a motion command cannot
refresh the window. Nav2 recovery feedback rebases the rolling window, while a
50-second hard limit remains absolute. In addition, an exploration goal that
stays inside a 0.10 m translational bubble for 7 seconds while more than 0.30 m
from the target is abandoned immediately; in-place turning does not hide this
condition. The failed target and blocked location are blacklisted for the
existing 240-second window. The manager requests exact cancellation, asserts
zero velocity until the action reaches a terminal result, runs one
collision-checked Spin fallback, and then selects a different frontier. This
7-second rule applies only to exploration goals, not return-home navigation.

RRT and the manager selector now run in parallel: the fixed five-second RRT
priority wait is disabled, so every eligible one-second manager tick may select
a local frontier while valid RRT goals can still arrive at any time. Fallback
cells are checked for target clearance and path density before dispatch.

The existing map-service adapter also publishes a dedicated legacy-RRT map on
`/stemm_cartographer/rrt_map` and serves it through
`/stemm_cartographer/rrt_dynamic_map`. Cartographer probabilities are normalized
to `-1/0/100` with occupied values beginning at 50; Nav2 and the manager continue
to consume the original `/map`. An RRT boundary sample is projected within
1.0 m to the nearest known-free cell that passes blacklist, clearance, and path
checks rather than weakening the manager to accept an unknown target. The
Cartographer occupancy grid publishes every 2 seconds to reduce global-costmap
resize churn.


## Deployment

Files under `deploy/` are templates. Install the helper as
`/usr/local/sbin/stemm-cartographer-service-guard`, validate and install the
sudoers fragment, and install the two companion service units. Do not enable or
start them until the stack has passed static and sensor-only validation.

Install `deploy/wheeltec-recharge-manager-fast-stop.conf` as
`/etc/systemd/system/wheeltec-recharge-manager.service.d/20-stemm-cartographer-fast-stop.conf`
and replace `/opt/demo-control/deploy/jetson/start_recharge_manager.sh` with the
reviewed `deploy/start_recharge_manager.sh` template. After `systemctl
daemon-reload`, systemd delivers SIGINT to the whole service cgroup and its
MainPID is the real recharge-manager node rather than the `ros2 run` wrapper.
The recharge node's normal KeyboardInterrupt cleanup remains in control. The
existing 45-second hard fallback is deliberately retained until stopping during
an active recharge has been validated with the wheels safely raised.

Before stopping that unit, `stemm-cartographer-service-guard` calls the existing
`/auto_recharge/stop` service through the package's bounded helper. This performs
the current task's exact goal cancellation and zero-motion publication while the
ROS context is still valid; it does not launch the legacy `cancel_charge` node.
If the service does not confirm within the bounded timeout, mapping startup is
aborted and the session wrapper restores the fixed functional-service allowlist.

`wheeltec-mic.service` is deliberately excluded from both the stop and restart
allowlists, and the system preflight permits it to remain active. This keeps the
five mapping voice commands available throughout a session. The voice command
adapter must reject unrelated motion, follow, and recharge commands while mapping
is active, and it must start the blocking session launch through an independent
systemd supervisor or persistent bridge rather than as an unmanaged mic-service
child process.

`deploy/stemm-cartographer-session.service` is that independent supervisor.
Install `deploy/stemm-cartographer-session-control` as
`/usr/local/sbin/stemm-cartographer-session-control` and add its exact `start`
command from `deploy/stemm-cartographer-sudoers` to the validated sudoers
fragment. The voice bridge only requests that fixed start operation; pause,
continue, save, and end remain asynchronous ROS service calls. Restarting
`wheeltec-mic.service` therefore does not terminate a live mapping session.

The companion MQTT bridge reuses the existing bridge program with separate
topics and a strict allowlist. The companion map HTTP service listens on
`127.0.0.1:8093`, leaving the existing service on port 8092 untouched.
