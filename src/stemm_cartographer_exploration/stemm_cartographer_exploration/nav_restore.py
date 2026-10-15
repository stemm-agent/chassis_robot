#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone map-mode navigation restore helper (P25).

A-20261014 —— 从"独立发起者"改为"桥的可验证兜底"。
  本助手保留"脱离会话生命周期"这一正确设计（那正是 P25/P29/P32 一路修出来的
  东西：在 runner 内等待会被桥的 ownership 判据挡住而死锁），但不再与桥的
  auto-navigation 并发发起同一条 launch。见 main() 里的"让桥先动手"段。

Spawned detached by session_runner after it writes the terminal state.  It must
NOT run inside the session runner: the MQTT bridge refuses `start_launch` while
it still considers the mapping session to own vehicle motion, and it only clears
that once the session runner (and the launch it owns) has exited.  Waiting for
the restore inside the runner therefore deadlocks -- measured 2026-10-13 17:45,
where all four in-runner attempts were refused and map-mode navigation stayed
down for the whole session.

This helper inherits nothing from that lifetime: it starts after the runner is
gone, retries on its own, and reports every outcome to the journal.
"""
import json
import os
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

NAVIGATION_COMMAND_URL = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_COMMAND_URL',
    'http://127.0.0.1:3000/api/command').strip()
NAVIGATION_LAUNCH_PACKAGE = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_PACKAGE', 'wheeltec_nav2').strip()
NAVIGATION_LAUNCH_FILE = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_FILE', 'wheeltec_nav2.launch.py').strip()
NAVIGATION_LAUNCH_MARKER = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_MARKER',
    NAVIGATION_LAUNCH_FILE).strip()


def _log(message, error=False):
    stream = sys.stderr if error else sys.stdout
    print('[nav-restore] %s' % message, file=stream, flush=True)


def _env_float(name, default):
    try:
        return max(0.0, float(os.getenv(name, str(default))))
    except (TypeError, ValueError):
        return float(default)


def _env_int(name, default):
    try:
        return max(1, int(os.getenv(name, str(default))))
    except (TypeError, ValueError):
        return int(default)


def navigation_launch_running():
    """True when a `wheeltec_nav2.launch.py` process is alive (same probe as the
    session runner uses, so both agree on what "running" means)."""
    marker = NAVIGATION_LAUNCH_MARKER.encode('utf-8', 'replace')
    if not marker:
        return False
    try:
        entries = list(os.scandir('/proc'))
    except OSError:
        return False
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % entry.name, 'rb') as handle:
                if marker in handle.read():
                    return True
        except OSError:
            continue
    return False


def main():
    if os.getenv('STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_ENABLED',
                 '1').strip().lower() in {'0', 'false', 'no', 'off'}:
        _log('restore is disabled; nothing to do.')
        return 0
    # The runner has just exited; give the bridge a moment to observe the
    # released session before the first request.  Retries cover the rest.
    settle = _env_float('STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_SETTLE_SEC', 8.0)
    attempts = _env_int('STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_ATTEMPTS', 6)
    timeout = _env_float('STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_TIMEOUT_SEC', 25.0)
    retry = _env_float('STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_RETRY_SEC', 12.0)
    # A-20261014: 桥先行的观察窗口（秒）。见下方长注释。
    bridge_first = _env_float(
        'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_BRIDGE_FIRST_SEC', 75.0)

    if navigation_launch_running():
        _log('map-mode navigation is already running; nothing to restore.')
        return 0
    if settle > 0.0:
        _log('waiting %.0fs for the session to be observed as released.' % settle)
        time.sleep(settle)

    # A-20261014 —— 让桥先动手。
    #
    # 本助手与主桥的 auto-navigation 钩子（_autonav_after_session_tick）都在
    # 会话结束后拉起同一条 wheeltec_nav2。实测 2026-10-14 09:36：桥 09:36:32
    # 发起、本助手 09:36:35 又发起一次，同一条指令被两个独立角色各发一次，
    # 而启动流程本身要跨 20~40 秒 —— 这正是"容器 load_node 双双超时、
    # /amcl 从未创建"那次失败的窗口。
    #
    # 桥的钩子带着五重门禁（终止原因白名单、播种拒绝检测、服务恢复前置、
    # 静置窗口、只消费一次），本助手一条都没有。所以正确分工是：
    #   * 桥是决策者，先动手；
    #   * 本助手只在**桥确实没能动手**时兜底 —— 也就是它最初被做出来的理由
    #     （桥在建图会话仍被认为占有车辆运动时拒绝 start_launch，17:45 实测）。
    # 于是这里先只观察、不发请求：桥一旦把栈拉起来，本助手直接退出。
    if bridge_first > 0.0:
        _log('giving the bridge %.0fs to bring map-mode navigation up first.'
             % bridge_first)
        deadline = time.monotonic() + bridge_first
        while time.monotonic() < deadline:
            if navigation_launch_running():
                _log('map-mode navigation restored by the bridge; '
                     'nothing to do.')
                return 0
            time.sleep(2.0)
        _log('the bridge did not bring map-mode navigation up within %.0fs; '
             'taking over.' % bridge_first)

    for attempt in range(1, attempts + 1):
        if navigation_launch_running():
            _log('map-mode navigation restored (attempt %d).' % attempt)
            return 0
        request_id = 'restore-nav-%d-%d' % (attempt, int(time.time() * 1000))
        payload = json.dumps({
            'action': 'start_launch',
            'package': NAVIGATION_LAUNCH_PACKAGE,
            'launchFile': NAVIGATION_LAUNCH_FILE,
            'requestId': request_id,
        }).encode('utf-8')
        _log('requesting map-mode navigation restore (%s/%s) attempt %d/%d.'
             % (NAVIGATION_LAUNCH_PACKAGE, NAVIGATION_LAUNCH_FILE,
                attempt, attempts))
        try:
            request = Request(
                NAVIGATION_COMMAND_URL,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urlopen(request, timeout=10.0) as response:
                _log('request accepted: HTTP %s'
                     % getattr(response, 'status', 'ok'))
        except (HTTPError, URLError, OSError, ValueError) as exc:
            _log('request failed: %s' % exc, error=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if navigation_launch_running():
                _log('map-mode navigation restored (attempt %d).' % attempt)
                return 0
            time.sleep(2.0)
        if attempt < attempts:
            _log('did not come up within %.0fs (attempt %d/%d); retrying in %.0fs.'
                 % (timeout, attempt, attempts, retry), error=True)
            time.sleep(retry)
    _log('map-mode navigation did not come up after %d attempts; it stays '
         'unavailable until started manually.' % attempts, error=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
