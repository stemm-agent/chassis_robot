# -*- coding: utf-8 -*-
"""A + 两个 P0 的回归测试（2026-10-14）。

失败背景（实测日志）：
  09:34:03  session_runner 在 ``ros2 launch`` 刚 fork、子进程尚未创建的窗口里，
            用 ``pgrep -P`` 判 "no child processes left" —— 实测误判：随后该次
            会话正常拉起 23 个子进程并完整跑完建图。
  09:36:26  建图 launch 所有子进程 finished cleanly 后仍卡在 ep_poll，等满 60s
            宽限被强杀（SIGINT），整套建图 nav2 栈在 1 秒内被拆掉。
  09:36:32  桥 auto-navigation 拉起导航栈（撞上强杀引发的重启风暴）
  09:36:35  nav-restore 助手又发起同一条 launch（双主控）
  09:37:15  两个容器 load_node 双双超时 → /amcl 从未创建 → 栈被判 failed
"""
import ast
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PKG = PACKAGE_ROOT / 'stemm_cartographer_exploration'
SESSION_RUNNER = PKG / 'session_runner.py'
NAV_RESTORE = PKG / 'nav_restore.py'
BRIDGE = Path('/opt/demo-control/jetson-agent/launch_mqtt_bridge.py')

HELPER_MARKER = 'stemm_cartographer_session_runner'
Q3 = chr(34) * 3


# --------------------------------------------------------------------------
# P0-(a) session_runner 启动窗口保护
# --------------------------------------------------------------------------
OBSERVE_CONST_RE = re.compile(
    r"INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC\s*=\s*max\(\s*[0-9.]+,\s*float\(\s*"
    r"os\.getenv\(\s*'STEMM_CARTOGRAPHER_ORPHAN_MIN_OBSERVE_SEC',\s*'([0-9.]+)'")


def _observe_const_default(source):
    m = OBSERVE_CONST_RE.search(source)
    assert m, '未找到 INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC 的默认值'
    return float(m.group(1))


def _load_orphan_predicate():
    """抽出 _inner_launch_orphaned，用最小宿主对象驱动。"""
    source = SESSION_RUNNER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    fn = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_inner_launch_orphaned':
            fn = node
            break
    assert fn is not None, '未找到 _inner_launch_orphaned'
    module = ast.Module(body=[fn], type_ignores=[])
    code = compile(module, str(SESSION_RUNNER), 'exec')
    # 该常量是模块级定义，单独 exec 函数体时不在命名空间里 —— 从源码解析出
    # 真实默认值注入，顺便让测试对默认值敏感。
    namespace = {
        'subprocess': subprocess,
        'time': time,
        'INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC': _observe_const_default(source),
    }
    exec(code, namespace)  # noqa: S102 - 只执行被测函数
    return namespace['_inner_launch_orphaned']


class _FakeChild:
    def __init__(self, pid=4242, alive=True):
        self.pid = pid
        self._alive = alive

    def poll(self):
        return None if self._alive else 0


class _OrphanHost:
    def __init__(self, child):
        self.child = child
        self.inner_launch_children_seen = False
        self.inner_launch_first_child_at = None
        self.inner_launch_childless_logged = False


def _probe_for(*, has_children):
    class _Result:
        def __init__(self, rc):
            self.returncode = rc
            self.stdout = ''

    def _run(cmd, **_kwargs):
        assert cmd[0] == 'pgrep', '判据必须用 pgrep 探子进程'
        return _Result(0 if has_children else 1)

    return _run


def test_orphan_predicate_requires_a_child_to_have_existed(monkeypatch):
    """★核心不变式：从未见过子进程时绝不判 orphan（启动窗口保护）。"""
    source = SESSION_RUNNER.read_text(encoding='utf-8')
    assert 'P0a-20261014' in source, '缺少 P0a 标记'
    assert 'INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC' in source, '缺少观察窗常量'
    default = _observe_const_default(source)
    assert 0.0 < default < 60.0, \
        '观察窗 %s 必须落在 (0, 60) 内（60 是 INNER_LAUNCH_EXIT_GRACE_SEC）' % default

    fn = _load_orphan_predicate()
    host = _OrphanHost(_FakeChild())
    monkeypatch.setattr(subprocess, 'run', _probe_for(has_children=False))
    assert fn(host) is False, \
        '★ 从未见到子进程就判 orphan —— 正是 09:34:03 那次误判'


def test_orphan_predicate_needs_the_observe_window_after_children_vanish(monkeypatch):
    """见到过子进程、但它们刚消失时，仍不得立刻判 orphan。"""
    fn = _load_orphan_predicate()
    host = _OrphanHost(_FakeChild())
    monkeypatch.setattr(subprocess, 'run', _probe_for(has_children=True))
    assert fn(host) is False, '有子进程时不得判 orphan'
    assert host.inner_launch_children_seen is True
    assert host.inner_launch_first_child_at is not None

    monkeypatch.setattr(subprocess, 'run', _probe_for(has_children=False))
    host.inner_launch_first_child_at = time.monotonic() - 1.0
    assert fn(host) is False, '刚过 1 秒就判 orphan —— 观察窗形同虚设'

    host.inner_launch_first_child_at = time.monotonic() - 3600.0
    assert fn(host) is True, '超过观察窗后应能判定 orphan（P35 语义不变）'


def test_orphan_predicate_still_returns_false_for_live_or_dead_child(monkeypatch):
    """回归：child 已退出或不存在时仍返回 False（不改变原语义）。"""
    fn = _load_orphan_predicate()
    monkeypatch.setattr(subprocess, 'run', _probe_for(has_children=False))
    assert fn(_OrphanHost(_FakeChild(alive=False))) is False
    assert fn(_OrphanHost(None)) is False


# --------------------------------------------------------------------------
# A  nav_restore 让桥先行
# --------------------------------------------------------------------------
def test_nav_restore_defers_to_the_bridge_first():
    """★核心不变式：助手必须先给桥一段先行窗口，只观察、不发请求。"""
    source = NAV_RESTORE.read_text(encoding='utf-8')
    assert 'A-20261014' in source, '缺少 A 标记'
    assert 'BRIDGE_FIRST_SEC' in source, '缺少先行窗口参数'
    assert 'giving the bridge' in source, '缺少让桥先行的日志'
    assert 'the bridge did not bring map-mode navigation up within' in source, \
        '缺少"桥没动手、我接管"的分支'

    idx_wait = source.index('giving the bridge')
    idx_first_request = source.index(
        "'requesting map-mode navigation restore (%s/%s) attempt %d/%d.'")
    assert idx_wait < idx_first_request, \
        '让桥先行的代码必须排在第一次发请求之前'

    m = re.search(
        r"STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_BRIDGE_FIRST_SEC',\s*([0-9.]+)",
        source)
    assert m, '先行窗口默认值未找到'
    assert float(m.group(1)) >= 60.0, \
        '先行窗口 %s 太短：桥要等建图 runner 退场（其自身宽限 60s）' % m.group(1)


def test_nav_restore_waits_for_the_bridge_and_exits_quietly():
    """功能性：桥已把栈拉起时，助手必须直接退出、一次请求都不发。"""
    with tempfile.TemporaryDirectory() as tmp:
        marker_script = Path(tmp) / 'wheeltec_nav2.launch.py'
        marker_script.write_text('import time\ntime.sleep(30)\n', encoding='utf-8')
        proc = subprocess.Popen([sys.executable, str(marker_script)],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        try:
            env = dict(os.environ)
            env['STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_SETTLE_SEC'] = '0'
            env['STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_BRIDGE_FIRST_SEC'] = '30'
            # 请求地址故意指向必然失败的端口：若助手真的发了请求就会打日志
            env['STEMM_CARTOGRAPHER_NAVIGATION_COMMAND_URL'] = 'http://127.0.0.1:1/nope'
            env['PYTHONPATH'] = str(PACKAGE_ROOT) + os.pathsep + env.get('PYTHONPATH', '')
            completed = subprocess.run(
                [sys.executable, '-m', 'stemm_cartographer_exploration.nav_restore'],
                capture_output=True, text=True, timeout=60, env=env)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        out = (completed.stdout or '') + (completed.stderr or '')
        assert completed.returncode == 0, '助手应正常退出，输出=%r' % out
        assert 'already running; nothing to restore' in out, \
            '桥已拉起时助手应识别并退出，实际输出=%r' % out
        assert 'requesting map-mode navigation restore' not in out, \
            '★ 桥已在跑，助手却仍发了请求（双主控）: %r' % out


# --------------------------------------------------------------------------
# P0-(b)  桥等建图会话退场
# --------------------------------------------------------------------------
def _bridge_source():
    if not BRIDGE.is_file():
        pytest.skip('桥源码不在预期位置: %s' % BRIDGE)
    return BRIDGE.read_text(encoding='utf-8')


def test_bridge_waits_for_the_mapping_runner_before_starting_navigation():
    """★核心不变式：桥在拉起导航前必须确认 session_runner 已退场。"""
    source = _bridge_source()
    assert 'P0b-20261014' in source, '缺少 P0b 标记'
    assert 'def _mapping_session_runner_alive()' in source, '缺少判据函数'
    assert 'AUTO_NAVIGATION_RUNNER_WAIT_SEC' in source, '缺少等待上限常量'

    fn_raw = source[source.index('def _mapping_session_runner_alive()'):
                    source.index('def _autonav_after_session_tick(')]
    assert '/proc' in fn_raw and 'cmdline' in fn_raw, \
        '判据应直接读 /proc/<pid>/cmdline，而不是文本匹配'
    assert 'scandir' in fn_raw, '判据应扫描 /proc'
    assert 'entry.name.isdigit()' in fn_raw, '判据必须只统计数字 pid 目录'
    # 文档字符串里会**提到** pgrep（说明为什么不用它），先剥掉文档串再断言。
    parts = fn_raw.split(Q3)
    assert len(parts) >= 3, '该函数应带文档字符串'
    fn_code = parts[0] + Q3.join(parts[2:])
    assert 'pgrep' not in fn_code, \
        '判据代码不得用 pgrep（进程名文本匹配会把诊断命令行也算进去）'

    tick = source[source.index('def _autonav_after_session_tick('):]
    idx_gate = tick.index('_mapping_session_runner_alive()')
    idx_fire = tick.index("'[INFO] Auto-navigation: starting %s/%s after %s")
    assert idx_gate < idx_fire, '门禁必须排在 fire 之前'

    # 超时放行分支：把三个 ASCII 片段分开断言，避免中文/引号形态差异。
    assert 'still alive after' in tick, \
        '★ 缺少超时放行的告警文案'
    assert 'navigation anyway' in tick, \
        '★ 缺少超时放行：万一 runner 卡死，导航将永远不被拉起'
    m = re.search(r"MQTT_AUTO_NAVIGATION_RUNNER_WAIT_SEC',\s*'([0-9.]+)'", source)
    assert m, '等待上限默认值未找到'
    assert float(m.group(1)) >= 60.0, \
        '等待上限 %s 必须大于建图 runner 自身的 60s 收尾宽限' % m.group(1)


def test_bridge_runner_probe_finds_a_live_marker_process():
    """功能性：判据在存在 marker 进程时 True，退出后 False。"""
    source = _bridge_source()
    start = source.index('def _mapping_session_runner_alive()')
    end = source.index('def _autonav_after_session_tick(')
    namespace = {'os': os}
    exec(compile(source[start:end], str(BRIDGE), 'exec'), namespace)  # noqa: S102
    probe = namespace['_mapping_session_runner_alive']

    assert isinstance(probe(), bool)

    with tempfile.TemporaryDirectory() as tmp:
        script = Path(tmp) / (HELPER_MARKER + '.py')
        script.write_text('import time\ntime.sleep(20)\n', encoding='utf-8')
        proc = subprocess.Popen([sys.executable, str(script)],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        try:
            time.sleep(1.0)
            assert probe() is True, '有 runner 进程存活时必须返回 True'
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        time.sleep(0.5)
        assert probe() is False, 'runner 退出后必须返回 False'
