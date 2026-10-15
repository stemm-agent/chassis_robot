# 系统层配置：systemd 单元、drop-in、运维脚本

本目录是**运行这台机器人所必需的宿主级配置**快照。这些文件**不在 ROS 工作区内**，
原先只存在于机器的 `/etc/systemd/system/` 与 `/usr/local/`，一旦机器损毁无法恢复。

> 采集时间：2026-10-15
> 采集来源：`hnh-MIX-NUC-II`（192.168.124.34）的当前生效状态
> 采集原则：**只收当前生效的单元**，已启用但失效、或历史废弃的单元一律不收

## 目录结构与落位

| 本目录 | 应放置到 | 说明 |
|---|---|---|
| `units/*.service`、`units/*.timer` | `/etc/systemd/system/` | 单元文件本体 |
| `dropins/<unit>.d/*.conf` | `/etc/systemd/system/<unit>.d/` | 单元覆写片段 |
| `../usr-local/sbin/*` | `/usr/local/sbin/` | 被 unit 引用的脚本 |
| `../usr-local/bin/*` | `/usr/local/bin/` | 被 unit 引用的脚本 |

## 恢复步骤

```bash
sudo cp units/* /etc/systemd/system/
sudo cp -r dropins/* /etc/systemd/system/
sudo cp ../usr-local/sbin/* /usr/local/sbin/ && sudo chmod +x /usr/local/sbin/*
sudo cp ../usr-local/bin/*  /usr/local/bin/  && sudo chmod +x /usr/local/bin/*
sudo systemctl daemon-reload
```

## 说明

- 单元中的路径、用户、环境变量均为**当时生效的真实值**，直接可用。
- `stemm-cartographer-session.service` 处于 disabled 状态是**设计如此**：
  建图会话由 MQTT 桥按需 `systemctl start` 拉起，不应开机自启。
- 本目录**不含**任何凭据、令牌或密码；单元中的地址仅本地回环。
- 与 ROS 工作区内 `src/stemm_cartographer_exploration/deploy/`、`tools/*/` 下的同名单元
  可能存在版本先后差异，**以本目录为准**（它是机器上实际生效的那一份）。
