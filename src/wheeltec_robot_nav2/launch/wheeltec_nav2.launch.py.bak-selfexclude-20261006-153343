import errno
import fcntl
import os
import stat
import time
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


_NAV2_SINGLETON_FD = None
_NAV2_SINGLETON_PATH = None


def _classify_navigation_process(argv):
    """Return the role of a process that makes a fresh Nav2 launch unsafe."""
    if not argv:
        return None

    executable = os.path.basename(argv[0])
    if executable in {'amcl_tf_guard', 'nav2_guard_interlock',
                      'amcl_odom_limiter'}:
        return executable

    # A stale component container is the most dangerous case: a new
    # LoadComposableNodes action can otherwise attach to the old service.
    container_node_names = {
        '__node:=nav2_container',
        '__node:=/nav2_container',
        '__node:=localization_container',
        '__node:=/localization_container',
        '__node:=navigation_container',
        '__node:=/navigation_container',
    }
    if executable.startswith('component_container') and any(
            arg in container_node_names for arg in argv):
        return next(
            arg.split(':=', 1)[1].lstrip('/')
            for arg in argv if arg in container_node_names)

    # Match argv elements, not a shell/diagnostic command containing the same
    # text.  This keeps read-only process inspection from looking like a launch.
    for index, arg in enumerate(argv):
        if arg != 'launch':
            continue
        tail = argv[index + 1:]
        if ('wheeltec_nav2' in tail and
                'wheeltec_nav2.launch.py' in tail and
                tail.index('wheeltec_nav2') <
                tail.index('wheeltec_nav2.launch.py')):
            return 'wheeltec_nav2_launch'

    return None


def _read_process_argv(proc_dir):
    try:
        raw = (proc_dir / 'cmdline').read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return []
    except OSError:
        # Processes may disappear while /proc is being enumerated.
        return []
    return [
        part.decode('utf-8', errors='replace')
        for part in raw.rstrip(b'\0').split(b'\0') if part
    ]


def _find_stale_navigation_processes():
    proc_root = Path('/proc')
    try:
        proc_dirs = list(proc_root.iterdir())
    except OSError as exc:
        raise RuntimeError(
            f'Cannot inspect /proc to confirm Nav2 cleanup: {exc}') from exc

    current_pid = os.getpid()
    stale = []
    for proc_dir in proc_dirs:
        if not proc_dir.name.isdigit() or int(proc_dir.name) == current_pid:
            continue
        role = _classify_navigation_process(_read_process_argv(proc_dir))
        if role is not None:
            stale.append((int(proc_dir.name), role))
    return sorted(stale)


def _nav2_lock_path():
    # Use one inode for this real UID regardless of the caller's environment or
    # whether a login-scoped /run/user directory currently exists.
    return Path('/tmp') / f'wheeltec_nav2-{os.getuid()}.launch.lock'


_NAV2_ZOMBIE_STATE_PATH = None


def _nav2_zombie_state_path():
    global _NAV2_ZOMBIE_STATE_PATH
    if _NAV2_ZOMBIE_STATE_PATH is None:
        _NAV2_ZOMBIE_STATE_PATH = Path('/tmp') / (
            f'wheeltec_nav2-{os.getuid()}.zombie-watch')
    return _NAV2_ZOMBIE_STATE_PATH


def _read_zombie_watch():
    try:
        raw = _nav2_zombie_state_path().read_text(encoding='ascii').strip()
        pid_text, at_text = raw.split(',', 1)
        return int(pid_text), float(at_text)
    except Exception:
        return None, 0.0


def _write_zombie_watch(pid):
    try:
        _nav2_zombie_state_path().write_text(
            f'{pid},{time.time()}', encoding='ascii')
    except OSError:
        pass


def _clear_zombie_watch():
    try:
        _nav2_zombie_state_path().unlink()
    except OSError:
        pass


def _wrapper_has_live_container(pid):
    """True if the launch wrapper has at least one live component_container child."""
    try:
        with open(f'/proc/{pid}/task/{pid}/children', 'r', encoding='ascii') as handle:
            children = handle.read().split()
    except OSError:
        return False
    for child in children:
        argv = _read_process_argv(Path('/proc') / child)
        if argv and os.path.basename(argv[0]).startswith('component_container'):
            return True
    return False


def _reclaim_zombie_launch_wrapper(lock_path):
    """2026-10-06: reclaim a launch wrapper that holds the singleton but owns no
    live navigation container.

    Real incident: containers were killed while `ros2 launch wheeltec_nav2 ...`
    survived.  The wrapper kept the flock and the bridge kept reusing its (empty)
    entry, so no fresh Nav2 could ever start until an operator killed the wrapper
    and removed the lock by hand.  Two consecutive container-free observations
    are required, so a launch that is merely still bringing its containers up is
    never touched.  Set STEMM_NAV2_ZOMBIE_RECLAIM=0 to disable.
    """
    if os.environ.get('STEMM_NAV2_ZOMBIE_RECLAIM', '1') == '0':
        return
    try:
        holder_text = lock_path.read_text(encoding='ascii').strip()
        holder = int(holder_text)
    except Exception:
        return
    if holder <= 0 or holder == os.getpid():
        return
    if not (str(lock_path).startswith('/tmp/') and lock_path.is_file()):
        return
    role = _classify_navigation_process(_read_process_argv(Path('/proc') / str(holder)))
    if role is None:
        # Not a navigation launch wrapper: leave it to the flock check.
        return
    if _wrapper_has_live_container(holder):
        _clear_zombie_watch()
        return
    watch_pid, watch_at = _read_zombie_watch()
    now = time.time()
    if watch_pid != holder or (now - watch_at) < 5.0:
        # First container-free sighting: record it and retry shortly.  A launch
        # that is still starting its containers will have them by the next pass.
        _write_zombie_watch(holder)
        raise RuntimeError(
            'Nav2 launch wrapper PID %d holds the singleton but owns no live '
            'component_container; treating it as a zombie and retrying in %.0fs'
            % (holder, 5.0))
    try:
        os.kill(holder, 9)
    except OSError as exc:
        raise RuntimeError(
            f'Cannot reap zombie Nav2 launch wrapper PID {holder}: {exc}') from exc
    deadline = now + 5.0
    while time.time() < deadline:
        if not Path(f'/proc/{holder}').exists():
            break
        time.sleep(0.1)
    try:
        lock_path.unlink()
    except OSError:
        pass
    _clear_zombie_watch()
    print(
        f'[nav2-guard] Reclaimed zombie Nav2 launch wrapper PID {holder} '
        f'(held the singleton with no live container); starting fresh.',
        flush=True)


def _claim_nav2_launch_singleton():
    """Claim the launch singleton and fail closed if cleanup is incomplete."""
    global _NAV2_SINGLETON_FD, _NAV2_SINGLETON_PATH
    if _NAV2_SINGLETON_FD is not None:
        return

    lock_path = _nav2_lock_path()
    flags = os.O_CREAT | os.O_RDWR | os.O_CLOEXEC
    flags |= getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(lock_path, flags, 0o600)
    except OSError as exc:
        raise RuntimeError(
            f'Cannot open Nav2 singleton lock {lock_path}: {exc}') from exc

    locked = False
    try:
        lock_stat = os.fstat(fd)
        if (not stat.S_ISREG(lock_stat.st_mode) or
                lock_stat.st_uid != os.getuid()):
            raise RuntimeError(
                f'Unsafe Nav2 singleton lock ownership/type: {lock_path}')

        # 2026-10-06: a wrapper that outlived its containers must not keep the
        # singleton (and the bridge's reuse path) forever.
        _reclaim_zombie_launch_wrapper(lock_path)

        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            locked = True
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                raise
            os.lseek(fd, 0, os.SEEK_SET)
            holder = os.read(fd, 64).decode('ascii', errors='replace').strip()
            holder = holder or 'unknown'
            raise RuntimeError(
                'A wheeltec_nav2 launch is already active '
                f'(singleton holder PID {holder}); refusing a second launch')

        os.ftruncate(fd, 0)
        os.write(fd, f'{os.getpid()}\n'.encode('ascii'))
        _clear_zombie_watch()

        stale = _find_stale_navigation_processes()
        if stale:
            details = ', '.join(f'PID {pid} ({role})' for pid, role in stale)
            raise RuntimeError(
                'Nav2 cleanup is not confirmed; found stale navigation '
                f'processes: {details}. Refusing to start because composable '
                'nodes could attach to an old Nav2 component container')
    except Exception:
        if locked:
            fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
        raise

    # Keep the descriptor alive in the ros2 launch process. O_CLOEXEC prevents
    # child nodes from inheriting it; process exit releases the lock naturally.
    _NAV2_SINGLETON_FD = fd
    _NAV2_SINGLETON_PATH = str(lock_path)


def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    _claim_nav2_launch_singleton()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_lasertracker = LaunchConfiguration('enable_lasertracker')
    enable_waypoint_cycle = LaunchConfiguration('enable_waypoint_cycle')

    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')

    # The LiDAR is not started here.  wheeltec-lidar.service owns the single
    # serial reader on /dev/wheeltec_lidar and stays up independently of
    # navigation, so this launch and everything else only *consume* /scan.
    # Two drivers on that port split the 500 kbaud byte stream and corrupt /scan.
        
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')
    cfg_params = load_yaml(os.path.join(get_package_share_directory('turn_on_wheeltec_robot'),'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")

    # 2026-09-16: navigation must use the map the mapping session actually produced.
    # The session writes ~/.local/share/stemm/maps/stemm_cartographer_map.* (the same
    # files the map service and the mini program read); this package's map/ directory
    # kept an old copy and was silently loaded instead, so navigation ran on a stale
    # map.  Prefer the session output and fall back to the packaged map if absent.
    nav_map_dir = os.path.join(wheeltec_nav_dir, 'map')
    session_map_dir = os.path.expanduser('~/.local/share/stemm/maps')
    session_map_yaml = os.path.join(session_map_dir, 'stemm_cartographer_map.yaml')
    map_dir = session_map_dir if os.path.exists(session_map_yaml) else nav_map_dir
    # 如果 install 下仍断裂，可用主机本地路径替代：
    # map_dir = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map'
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'stemm_cartographer_map.yaml'))

    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, f'param_{car_mode}.yaml'))
    print(os.path.join(param_dir, f'param_{car_mode}.yaml'))

    # AMCL estimates /amcl_pose with tf_broadcast=false.  amcl_tf_guard is the
    # sole map -> odom_combined publisher and enforces a fixed authorized anchor.
    enable_amcl_tf_guard = LaunchConfiguration('enable_amcl_tf_guard')
    # 2026-09-13 decouple: guard disabled by default; scheduled for removal.
    guard_default = 'false'   # 2026-09-29: reverted -- with the guard enabled this build never started localization (state=starting/failed, no map->base TF, no amcl node). The safe half of the fix is kept below: the limiter must be faster than the drivetrain.
    # Localization must start so AMCL can qualify the fixed anchor, while the
    # navigation lifecycle stays unconfigured until the interlock authorizes it.
    # If the guard is explicitly disabled, preserve normal Nav2 autostart.
    navigation_autostart = PythonExpression([
        "'", enable_amcl_tf_guard, "'.lower() != 'true'"
    ])
    enable_amcl_noise_scheduler = LaunchConfiguration('enable_amcl_noise_scheduler')
    # Sole publisher of map -> odom_combined (2026-09-13): rate limits AMCL
    # corrections, never holds or vetoes them.
    enable_amcl_odom_limiter = LaunchConfiguration('enable_amcl_odom_limiter')
    # 守卫与限幅器都是"map -> odom_combined 唯一发布者"的设计，同时启用会让该变换
    # 出现两个发布者（作者在参数文件里明确警告过：定位校正会变得不确定），故互斥。
    limiter_enabled = IfCondition(PythonExpression([
        "'", enable_amcl_odom_limiter, "'.lower() == 'true' and '",
        enable_amcl_tf_guard, "'.lower() != 'true'",
    ]))
    both_publishers_enabled = IfCondition(PythonExpression([
        "'", enable_amcl_odom_limiter, "'.lower() == 'true' and '",
        enable_amcl_tf_guard, "'.lower() == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'enable_lasertracker',
            default_value='false',
            description='Start the shared laser tracker with Nav2'),
        DeclareLaunchArgument(
            'enable_waypoint_cycle',
            default_value='false',
            description='Start the optional interactive waypoint-cycle console'),
        DeclareLaunchArgument(
            'enable_amcl_tf_guard',
            default_value=guard_default,
            description='Guard and exclusively publish map to odom_combined TF'),
        DeclareLaunchArgument(
            'enable_amcl_noise_scheduler',
            default_value='false',
            description='Dynamically tune AMCL motion and laser noise after validation'),
        DeclareLaunchArgument(
            'enable_amcl_odom_limiter',
            default_value='true',
            description='Rate limit and exclusively publish map to odom_combined TF'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
            condition=IfCondition(enable_waypoint_cycle),
        ),
        # LiDAR intentionally not started here: wheeltec-lidar.service owns it.
        Node(
            name='lasertracker',
            package='simple_follower_ros2',
            executable='lasertracker',
            output='screen',
            condition=IfCondition(enable_lasertracker),
        ),
        Node(
            name='nav2_guard_interlock',
            package='wheeltec_nav2',
            executable='nav2_guard_interlock',
            output='screen',
            parameters=[{
                'anomaly_topic': '/amcl_tf_guard/anomaly',
                'lifecycle_manager_service':
                    '/lifecycle_manager_navigation/manage_nodes',
                'raw_cmd_vel_topic': '/nav2_cmd_vel',
                'cmd_vel_topic': '/cmd_vel',
                'blocked_topic': '/nav2_guard_interlock/blocked',
                'manage_initial_startup': True,
                'guard_message_timeout_sec': 2.0,
                # The guard already requires six qualified samples over at least
                # 3 s; retain another 1 s interlock debounce plus the 0.5 s
                # post-activation cancellation hold before opening the gate.
                'healthy_stability_sec': 1.0,
                'post_pause_settle_sec': 0.25,
                'post_activation_cancel_sec': 0.5,
                'manager_retry_period_sec': 0.5,
                # STARTUP legitimately takes longer than PAUSE/RESUME on Jetson.
                # Every timeout is followed by per-node lifecycle verification;
                # an old response cannot unlock a newer guard epoch.
                'manager_startup_timeout_sec': 30.0,
                'manager_transition_timeout_sec': 10.0,
                'manager_state_verification_timeout_sec': 5.0,
                'cancel_retry_period_sec': 0.2,
                # Clear a previously forwarded Nav2 command at the safety edge,
                # then leave the shared final /cmd_vel bus silent. This avoids
                # freezing independent manual/follow publishers while blocked.
                'blocked_zero_burst_sec': 0.20,
                'zero_burst_period_ms': 20,
                'blocked_heartbeat_hz': 10.0,
                'control_period_ms': 20,
            }],
            condition=IfCondition(enable_amcl_tf_guard),
        ),
        LogInfo(
            condition=both_publishers_enabled,
            msg='enable_amcl_tf_guard and enable_amcl_odom_limiter are both set: the '
                'TF guard owns map -> odom_combined, so the odom rate limiter is NOT '
                'started (two publishers of the same transform make corrections '
                'nondeterministic).'),
        Node(
            name='amcl_odom_limiter',
            package='stemm_nav2_manager',
            executable='amcl_odom_limiter',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom_combined',
                'base_frame': 'base_footprint',
                'pose_topic': '/amcl_pose',
                'publish_rate': 30.0,
                # Correction budget: at most 0.25 m/s / 0.35 rad/s of
                # map -> odom_combined movement, so an AMCL mode jump
                # arrives as a short ramp instead of a step.
                'max_translation_rate': 0.60,
                'max_yaw_rate': 2.00,
                # 2026-09-28: the refresh below keeps the age in the 0.5-5 s band,
                # so a 2 s warning fired ~87 times per 15 min on a healthy system.
                # 8 s still flags a real stall.
                'warn_pose_age_sec': 8.0,
                # 2026-09-28: the limiter now refreshes a stale pose itself, so it no
                # longer depends on the cold-start path only.  Exposed here so future
                # tuning stays launch-only.
                'nomotion_request_age_sec': 5.0,
                'nomotion_retry_sec': 2.0,
                'nomotion_rearm_sec': 10.0,
                'nomotion_max_attempts': 100000,
                'status_topic': '/amcl_odom_limiter/status',
            }],
            condition=limiter_enabled,
        ),
        Node(
            name='amcl_tf_guard',
            package='wheeltec_nav2',
            executable='amcl_tf_guard',
            output='screen',
            parameters=[{
                'global_frame': 'map',
                'odom_frame': 'odom_combined',
                'base_frame': 'base_footprint',
                'mode_topic': '/amcl_tf_guard/mode',
                # Keep these aligned with the four_wheel_diff MPPI limits.
                'max_linear_speed': 0.5,
                'max_angular_speed': 2.0,
                'max_linear_acceleration': 1.5,
                'max_angular_acceleration': 1.5,
                # Conservative localization correction bounds for corridor aliases.
                'max_odom_correction': 0.30,
                'max_odom_yaw_correction': 0.35,
                # Every commit stays inside a small per-anchor envelope. A
                # scan-qualified rolling anchor follows only bounded corrections;
                # both a true sliding-window budget and a non-resetting session
                # budget prevent high- or low-frequency small-step ratcheting.
                'max_cumulative_map_odom_translation': 0.30,
                'max_cumulative_map_odom_yaw': 0.20,
                'recovery_cumulative_map_odom_translation': 0.30,
                'recovery_cumulative_map_odom_yaw': 0.20,
                'rolling_anchor_enabled': True,
                'rolling_anchor_translation_deadband': 0.12,
                'rolling_anchor_yaw_deadband': 0.08,
                'rolling_anchor_max_step_translation': 0.03,
                'rolling_anchor_max_step_yaw': 0.025,
                # A fresh, odom-continuous, scan-qualified candidate may catch
                # up several missed AMCL callback intervals. The 0.30/0.20 hard
                # envelope is unchanged: the bounded step is committed only if
                # it brings the candidate back inside that envelope.
                'rolling_anchor_catchup_max_gap_sec': 4.0,
                'rolling_anchor_catchup_max_step_scale': 4.0,
                'rolling_anchor_catchup_max_likelihood_drop': 0.03,
                'rolling_anchor_window_sec': 30.0,
                'rolling_anchor_window_translation_budget': 0.12,
                'rolling_anchor_window_yaw_budget': 0.10,
                # The base budget is a floor. Only source-monotonic, timing- and
                # kinematics-valid odom motion from the same 30 s window may add
                # bounded capacity. Window consumption is the directional net
                # anchor displacement; the session fuse below remains gross.
                'rolling_anchor_translation_budget_per_odom_meter': 0.15,
                'rolling_anchor_yaw_budget_per_odom_rad': 0.15,
                'rolling_anchor_window_translation_motion_cap': 0.18,
                'rolling_anchor_window_yaw_motion_cap': 0.10,
                'rolling_anchor_odom_translation_epsilon': 0.001,
                'rolling_anchor_odom_yaw_epsilon': 0.002,
                # Independent disaster fuse: deliberately much larger than the
                # old 0.30/0.20 fixed envelope so normal long-session correction
                # does not regress to the incident, while still preventing an
                # unlimited low-frequency ratchet.
                'rolling_anchor_session_translation_budget': 2.0,
                'rolling_anchor_session_yaw_budget': 1.5,
                # A small, continuous, scan-qualified cumulative overrun first
                # freezes the candidate for several genuinely new source stamps.
                # It never commits or rolls the anchor during this window and it
                # does not change the 0.30/0.20 publish/commit envelope. Weak
                # scan/odom evidence, a large correction, an exhausted session
                # fuse, or an excessive overrun remains an immediate HOLD.
                'cumulative_overrun_defer_enabled': True,
                'cumulative_overrun_defer_consecutive_samples': 3,
                'cumulative_overrun_defer_min_duration_sec': 0.50,
                'cumulative_overrun_defer_max_duration_sec': 1.50,
                'cumulative_overrun_defer_max_translation_excess': 0.30,
                'cumulative_overrun_defer_max_yaw_excess': 0.15,
                'cumulative_overrun_defer_max_correction_translation': 0.10,
                'cumulative_overrun_defer_max_correction_yaw': 0.08,
                'cumulative_overrun_defer_max_source_gap_sec': 0.75,
                'cumulative_overrun_defer_max_odom_receipt_age_sec': 0.25,
                'cumulative_overrun_defer_pose_odom_stamp_tolerance_sec': 0.01,
                # If raw AMCL leaves the unchanged 0.30 m / 0.20 rad normal
                # envelope, freeze it and validate last_trusted map->odom against
                # fresh scans projected by same-stamp odom. Raw distance,
                # covariance, relative score and local-neighbor optimum never
                # authorize or revoke override. Two fresh scans / 0.25 s keep
                # navigation healthy; a recoverable HOLD needs three / 0.50 s
                # while stationary. Finite scan-quality excursions require five
                # distinct bad scans; non-finite metrics, an off-map base, or
                # hard timing/TF/odom faults still assert HARD_HOLD immediately.
                'trusted_override_enabled': True,
                'trusted_override_active_consecutive_samples': 2,
                'trusted_override_active_min_duration_sec': 0.25,
                # Not a mode lifetime: each strictly newer scan input refreshes
                # this liveness deadline; cached/repeated/out-of-order stamps cannot.
                'trusted_override_active_evidence_timeout_sec': 10.00,
                'trusted_override_hold_consecutive_samples': 3,
                'trusted_override_hold_min_duration_sec': 0.50,
                'trusted_override_hold_stationary_duration_sec': 0.40,
                'trusted_override_verify_max_duration_sec': 5.00,
                'trusted_override_return_consecutive_samples': 3,
                'trusted_override_return_min_duration_sec': 0.50,
                'trusted_override_min_valid_beams': 25,
                'trusted_override_min_inlier_ratio': 0.55,
                'trusted_override_min_likelihood': 0.55,
                'trusted_override_max_trimmed_mean_distance': 0.20,
                'trusted_override_max_unknown_offmap_ratio': 0.20,
                'trusted_override_catastrophic_min_valid_beams': 10,
                'trusted_override_catastrophic_min_inlier_ratio': 0.35,
                'trusted_override_catastrophic_min_likelihood': 0.30,
                'trusted_override_catastrophic_max_trimmed_mean_distance': 0.35,
                'trusted_override_catastrophic_max_unknown_offmap_ratio': 0.40,
                'trusted_override_soft_failure_consecutive_samples': 5,
                # ACTIVE quality may pair AMCL with the nearest scan only inside
                # this bound. The scan is projected with odom at its own stamp;
                # bootstrap still requires an exact same-stamp scan.
                'active_quality_scan_tolerance_sec': 0.35,
                # No manual /initialpose is required or accepted. At startup the
                # guard supplies a non-zero-covariance coarse prior, scores live
                # scan against the map, verifies covariance, then fixes a six-sample
                # medoid anchor from the converged AMCL output.
                'bootstrap_consecutive_valid_samples': 1,
                'bootstrap_requires_initialpose': False,
                'auto_global_localization': False,
                'allow_manual_rebase': False,
                # The robot normally starts near the charging/start pose. This
                # coarse Gaussian prior is published automatically (not by the
                # operator); the final anchor is still the converged AMCL medoid.
                'auto_initial_pose_enabled': True,
                'auto_initial_pose_x': 0.0,
                'auto_initial_pose_y': 0.0,
                'auto_initial_pose_yaw': 0.0,
                'auto_initial_pose_variance_x': 0.01,
                'auto_initial_pose_variance_y': 0.01,
                'auto_initial_pose_variance_yaw': 0.0625,
                'bootstrap_min_epoch_age_sec': 5.0,
                'bootstrap_min_post_global_updates': 3,
                'bootstrap_good_samples': 6,
                'bootstrap_min_confirm_duration_sec': 3.0,
                'bootstrap_seed_max_translation': 0.12,
                'bootstrap_seed_max_yaw': 0.10,
                'bootstrap_cov_max_xy_eigenvalue': 0.04,
                'bootstrap_cov_max_yaw': 0.0225,
                'bootstrap_zero_cov_epsilon': 1.0e-10,
                'bootstrap_timeout_sec': 45.0,
                # Keep the quality gates unchanged, but allow enough AMCL
                # no-motion updates to use the complete 45 s acquisition budget
                # before entering the 15 s retry cooldown.
                'bootstrap_max_nomotion_requests': 90,
                'bootstrap_retry_cooldown_sec': 15.0,
                # Score more of the available scan without weakening any
                # validity or scan-map quality threshold.
                'score_max_beams': 180,
                'score_min_valid_beams': 35,
                'score_sigma_m': 0.20,
                'score_inlier_distance_m': 0.20,
                'score_min_inlier_ratio': 0.65,
                'score_min_likelihood': 0.55,
                'score_max_trimmed_mean_distance': 0.18,
                'score_max_unknown_offmap_ratio': 0.10,
                'score_neighbor_translation_m': 0.10,
                'score_neighbor_yaw_rad': 0.15,
                'score_max_neighbor_likelihood_gain': 0.04,
                # Request convergence updates more often without reducing the
                # six-sample or 3 s fixed-anchor qualification requirements.
                'nomotion_update_period_sec': 0.5,
                # An ACTIVE anomaly may force only a bounded number of AMCL
                # no-motion updates. Timing-integrity inhibits disable this path
                # entirely so repeated correlated corridor scans cannot drive a
                # static particle cloud toward an alias indefinitely.
                'recovery_nomotion_max_requests': 12,
                'recovery_nomotion_max_window_sec': 8.0,
                'recovery_consecutive_valid_samples': 5,
                'initialpose_rebase_consecutive_valid_samples': 5,
                'recovery_cluster_consecutive_samples': 5,
                # In-session far re-anchor is a one-shot safety recovery. It is
                # allowed only after the interlock independently confirms a fresh
                # BLOCKED heartbeat, 2 s of odom stillness, and a fixed-seed
                # scan/covariance-qualified cluster spanning at least 3 s. The
                # anomaly remains asserted through a second verification window.
                'recovery_cluster_translation_tolerance': 0.06,
                'recovery_cluster_yaw_tolerance': 0.05,
                'recovery_cluster_odom_translation_tolerance': 0.025,
                'recovery_cluster_odom_yaw_tolerance': 0.025,
                'recovery_cluster_min_duration_sec': 3.0,
                'recovery_stationary_duration_sec': 2.0,
                'interlock_blocked_topic': '/nav2_guard_interlock/blocked',
                'interlock_blocked_max_age_sec': 0.75,
                'auto_reanchor_enabled': True,
                'auto_reanchor_max_translation': 1.0,
                'auto_reanchor_max_yaw': 0.75,
                'auto_reanchor_min_likelihood_improvement': 0.08,
                'auto_reanchor_max_odom_receipt_age_sec': 0.25,
                'auto_reanchor_cooldown_sec': 10.0,
                'max_auto_reanchors_per_session': 1,
                'post_reanchor_consecutive_valid_samples': 5,
                'post_reanchor_min_confirm_duration_sec': 2.0,
                'max_amcl_pose_age_sec': 2.0,
                'max_future_pose_skew_sec': 0.25,
                'max_amcl_validation_gap_sec': 1.5,
                'amcl_silence_timeout_while_moving_sec': 2.0,
                'amcl_silence_motion_translation': 0.30,
                'amcl_silence_motion_yaw': 0.25,
                # Do not convert a long pre-motion AMCL age into an immediate
                # fault at the instant the odom motion threshold is crossed.
                'amcl_silence_motion_grace_sec': 1.0,
                'max_odom_observation_gap_sec': 2.0,
                'max_odom_tf_age_sec': 1.0,
                # A lone near-threshold sample (observed 0.507 s age and
                # ~1.01 s receipt/source gaps under startup load) does not
                # pause Nav2. Repeated timer observations of the same old source
                # stamp do not count as new evidence; a same-reason soft fault
                # must persist for 0.25 s. Materially stale input remains an
                # immediate fail-closed fault.
                'odom_timing_soft_fault_consecutive_samples': 3,
                'odom_timing_soft_fault_min_duration_sec': 0.25,
                'hard_odom_tf_age_sec': 2.0,
                'hard_odom_observation_gap_sec': 3.0,
                'max_odom_step_translation': 0.35,
                'max_odom_step_yaw': 0.35,
                'max_publish_timer_gap_sec': 2.5,
                'timer_recovery_consecutive_ticks': 10,
            }],
            condition=IfCondition(enable_amcl_tf_guard),
        ),
        Node(
            name='amcl_noise_scheduler',
            package='wheeltec_nav2',
            executable='amcl_noise_scheduler',
            output='screen',
            parameters=[{
                'amcl_node': '/amcl',
                'odom_topic': '/odom_combined',
                'scan_topic': '/scan',
                'max_linear_speed': 0.5,
                'max_angular_speed': 2.0,
            }],
            condition=IfCondition(enable_amcl_noise_scheduler),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'navigation_autostart': navigation_autostart,
                'enable_nav2_cmd_vel_gate': enable_amcl_tf_guard,
                'params_file': param_file}.items(),
        ),

    ])
