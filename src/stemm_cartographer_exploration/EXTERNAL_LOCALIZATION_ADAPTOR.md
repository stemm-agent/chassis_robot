# 外部依赖：本地化权威契约适配器（不在本仓库内）

`/home/hnh/stemm_localization_authority/contract_adaptor.py`
systemd 单元：`stemm-localization-authority.service`

它把"限幅器（`amcl_odom_limiter`）自己的健康"转换成语音/agent 侧判断
"导航能否启动"的三个权威信号：

| 话题 | 含义 |
|---|---|
| `/amcl_tf_guard/anomaly` | False = 权威健康 |
| `/nav2_guard_interlock/blocked` | False = 允许运动 |
| `/nav2_guard_interlock/ready` | True = 就绪 |
| `/amcl_tf_guard/mode` | 仅诊断 |

**该文件不在 wheeltec_ros2 仓库内，本仓库只能记录其改动历史。**

## 2026-10-12 N-8：启动宽限期（打破 interlock 死锁）

* 问题：`_health()` 在"从未收到限幅器状态"时直接判不健康，而
  `/amcl_odom_limiter/status` 只在**有图导航栈**里产生。adaptor 早于导航栈
  启动 → 立刻 latch `interlock.ready=false` → 导航栈启动被挡 → 限幅器永不运行
  → 永久 `limiter_status_missing`。实测 15:49:55 重启后 0.2s 即 latch，
  导航栈停在 `starting`（`safetyError='AMCL TF guard status is missing or stale'`）。
* 修法：新增参数 `startup_grace_sec`（默认 90s）。从未收到限幅器状态且在宽限期内
  时按健康发布（`anomaly=false / blocked=false / ready=true`），让导航栈能起来、
  限幅器得以开始上报；超时后回到原有保守行为。收到过任何一条状态后宽限期立即失效，
  严格判据（stale / no_amcl_pose / correction_lag）全部不变。
* 安全性：宽限期内 `nav2_start_feedback_monitor` 仍独立校验
  `map -> odom_combined -> base_footprint` TF 链与 Nav2 生命周期；
  且窗口有界，不掩盖长期缺失的限幅器。置 `startup_grace_sec=0` 可关闭。
* 回退：`git reset --hard rollback-before-adaptorgrace-<ts>` 只回退本仓库记录；
  适配器本体回退请用同目录的 `contract_adaptor.py.bak-N8-pre-<ts>`。

## 2026-10-12 N-8b：宽限期改为"持续到限幅器首次上报"

* 问题：N-8 的 90s 墙钟宽限有实测时序漏洞 —— adaptor 15:54:01 重启后
  15:55:31 到期，而建图会话通常跑 4~5 分钟，等它结束时宽限期早已过期，
  adaptor 回到 `blocked=true`，**建图后的 auto-nav 仍被 interlock 挡掉**。
  死锁的本质是"限幅器首次上报依赖导航栈先起来"，与时钟无关。
* 修法：宽限期持续到收到**第一条**限幅器状态为止，不按时间收回；一旦收到过
  任何状态即永久失效，之后一律走原有严格判据。`startup_grace_sec` 改为
  "提示阈值"：超过该秒数仅把日志升级为周期性 ERROR（暴露限幅器长期不来），
  不改变放行行为；置 0 即完全关闭宽限。
* 回退：`contract_adaptor.py.bak-N8b-pre-<ts>`
