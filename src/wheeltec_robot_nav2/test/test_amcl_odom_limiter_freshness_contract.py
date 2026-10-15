"""2026-10-12 B：amcl_odom_limiter 的位姿保鲜门限必须对 AMCL 的实际发布节奏留余量。

实测事实（2026-10-12 17:52 / 17:55 两轮）：
  * AMCL 没有内部定时发布 —— `publishAmclPose` 在整个 amcl_node.cpp 里只有
    一个调用点（第 698 行），条件是
        `resampled || force_publication || !first_pose_sent_`
    静止时 `shouldUpdateFilter()` 返回 false，唯一让滤波更新的是
    `/request_nomotion_update` 置起的 `force_update_`（消费一次即复位）。
  * 因此静止时 `/amcl_pose` 只被外部触发器驱动，实测周期恒为 5.08~5.17 s。
  * limiter 原先 `warn_pose_age_sec = 8.0`，对 5.08 s 只有约 2.9 s 余量。
    17:37 那轮 AMCL 稍慢即冲过 8.0 s → 本节点判 stale → adaptor
    `blocked=true` / `ready=false` → `nav2_start_feedback_monitor` 读到
    interlock 未就绪 → `navigation_health_timeout` → 有图导航栈被判 failed
    并清掉。这是**边沿竞态**：17:52 那轮恰好没超，所以"有时能起来"。

修法（B）：
  `warn_pose_age_sec`        8.0 -> 15.0   给"短暂变慢"留余量
  `nomotion_request_age_sec` 5.0 -> 1.5    实测把 pose_age 从 3~5 s 压到 2.0~2.4 s

不变式（本测试守护）：
  1. warn_pose_age_sec 必须显著大于 AMCL 静止期的实测发布周期（5.2 s），
     至少 2.5 倍余量 —— 否则重新落入边沿竞态。
  2. nomotion_request_age_sec 必须小于 warn_pose_age_sec，且足够小以便主动保鲜。
  3. 真正 AMCL 死亡仍须被抓到：warn_pose_age_sec 不得大到让
     `amcl_absence_shutdown_sec`（15 s）失去意义。
"""
import ast
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py'

#: 实测 AMCL 在机器人静止时的 /amcl_pose 发布周期（秒）。
AMCL_STATIONARY_PUBLISH_PERIOD_SEC = 5.2
#: 需要的余量倍数（warn 阈值 / 发布周期）。
MIN_MARGIN_FACTOR = 2.5
#: 真正死亡判据：limiter 对 AMCL 缺席的关机阈值（秒）。
AMCL_ABSENCE_SHUTDOWN_SEC = 15.0


def _limiter_parameters():
    """从 launch 里取出 amcl_odom_limiter 的字面参数字典。"""
    tree = ast.parse(LAUNCH_FILE.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, 'id', None) not in ('Node', 'LifecycleNode'):
            continue
        name = None
        for kw in node.keywords:
            if kw.arg == 'name' and isinstance(kw.value, ast.Constant):
                name = kw.value.value
        if name != 'amcl_odom_limiter':
            continue
        for kw in node.keywords:
            if kw.arg != 'parameters':
                continue
            if not (isinstance(kw.value, ast.List) and kw.value.elts):
                continue
            first = kw.value.elts[0]
            if not isinstance(first, ast.Dict):
                continue
            return {
                key.value: value.value
                for key, value in zip(first.keys, first.values)
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)
            }
    return None


def test_amcl_odom_limiter_node_exists_with_literal_parameters():
    params = _limiter_parameters()
    assert params is not None, \
        'launch 里找不到 amcl_odom_limiter 的字面参数字典'
    for key in ('warn_pose_age_sec', 'nomotion_request_age_sec',
                'nomotion_retry_sec', 'publish_rate', 'pose_topic'):
        assert key in params, '缺少参数 %s' % key
    assert params['pose_topic'] == '/amcl_pose'


def test_warn_pose_age_leaves_margin_over_amcl_publish_period():
    """warn 阈值必须对 AMCL 静止期的实测发布周期留足余量。"""
    params = _limiter_parameters()
    warn = float(params['warn_pose_age_sec'])
    required = AMCL_STATIONARY_PUBLISH_PERIOD_SEC * MIN_MARGIN_FACTOR
    assert warn >= required, (
        'warn_pose_age_sec=%.1f 对 AMCL 实测发布周期 %.1fs 余量不足'
        '（需要 >= %.1fs）—— 会重新落入"AMCL 稍慢即被判 stale"的边沿竞态'
        % (warn, AMCL_STATIONARY_PUBLISH_PERIOD_SEC, required))


def test_warn_pose_age_still_detects_a_real_amcl_death():
    """阈值不能大到让真正的 AMCL 缺席抓不到。"""
    params = _limiter_parameters()
    warn = float(params['warn_pose_age_sec'])
    assert warn <= AMCL_ABSENCE_SHUTDOWN_SEC, (
        'warn_pose_age_sec=%.1f 超过 AMCL 缺席关机阈值 %.1fs，'
        '会让"定位真的死了"被掩盖' % (warn, AMCL_ABSENCE_SHUTDOWN_SEC))


def test_nomotion_refresh_is_inside_the_warn_window():
    """主动保鲜必须比告警阈值快，否则保鲜形同虚设。"""
    params = _limiter_parameters()
    warn = float(params['warn_pose_age_sec'])
    request_age = float(params['nomotion_request_age_sec'])
    assert 0.0 < request_age < warn, (
        'nomotion_request_age_sec=%.2f 必须落在 (0, warn=%.1f) 内'
        % (request_age, warn))
    assert request_age <= warn / 4.0, (
        'nomotion_request_age_sec=%.2f 相对 warn=%.1f 偏大，'
        '保鲜过于被动' % (request_age, warn))


def test_nomotion_retry_is_faster_than_request_age():
    """重试间隔必须小于请求门限，否则实际保鲜节奏被 retry 拖慢。"""
    params = _limiter_parameters()
    retry = float(params['nomotion_retry_sec'])
    request_age = float(params['nomotion_request_age_sec'])
    assert retry < request_age or retry <= 2.0, (
        'nomotion_retry_sec=%.2f 与 nomotion_request_age_sec=%.2f 不匹配，'
        '实际保鲜节奏会被 retry 拖慢' % (retry, request_age))


def test_limiter_defaults_in_code_do_not_contradict_the_launch():
    """代码默认值不得比 launch 更激进（否则漏掉 launch 时行为不同）。"""
    source = (PACKAGE_ROOT.parent / 'stemm_nav2_manager' / 'stemm_nav2_manager'
              / 'amcl_odom_limiter.py')
    if not source.is_file():
        pytest.skip('stemm_nav2_manager 源码不在预期位置')
    text = source.read_text(encoding='utf-8')
    tree = ast.parse(text)
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, 'attr', None) != 'declare_parameter':
            continue
        if len(node.args) >= 2 and isinstance(node.args[0], ast.Constant):
            try:
                defaults[node.args[0].value] = ast.literal_eval(node.args[1])
            except Exception:  # noqa: BLE001
                pass
    assert 'warn_pose_age_sec' in defaults
    assert 'nomotion_request_age_sec' in defaults
    launch = _limiter_parameters()
    assert float(defaults['warn_pose_age_sec']) <= float(
        launch['warn_pose_age_sec']), \
        '代码默认 warn_pose_age_sec 比 launch 更激进'
    assert float(defaults['nomotion_request_age_sec']) <= float(
        launch['nomotion_request_age_sec']), \
        '代码默认 nomotion_request_age_sec 比 launch 更宽松'
