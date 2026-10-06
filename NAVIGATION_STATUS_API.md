# 导航状态轮询 API — 小车端实现文档

> 目标：小程序发完 `POST /api/navigation/goal` 后，通过 `GET /api/navigation/status` 持续轮询 Nav2 的 action feedback 和最终 result。

---

## 架构

```
小程序 → GET 3000/api/navigation/status
         → demo-control (server/index.js) 代理转发
         → JETSON_AGENT_URL/api/navigation/status
         → robot_agent.py 返回缓存的 Nav2 action 状态
```

涉及修改两个文件：

| 文件 | 改动 |
|------|------|
| `jetson-agent/robot_agent.py` | 核心：追踪 Nav2 action feedback/result + 新增 GET 端点 |
| `server/index.js` | 新增代理路由 `GET /api/navigation/status` |

---

## 一、robot_agent.py 改动

### 1.1 在 `VelocityPublisher.__init__` 中新增导航状态字段

文件位置：`C:\Users\Lenovo\Desktop\backup2\demo-control\jetson-agent\robot_agent.py`

在 `VelocityPublisher.__init__` 方法里，现有 `self.lock = threading.Lock()` 这行附近，新增一组导航状态追踪字段：

```python
# 导航状态追踪（在 __init__ 中 self.lock 附近新增）
self.navigation_state = {
    'ok': True,
    'status': 'idle',          # idle | navigating | succeeded | failed | canceled
    'goal': None,               # { x, y, yaw }
    'feedback': None,           # 最新 feedback 快照
    'result': None,             # 最终 result 快照
    'message': '',
    'updatedAt': '',
}
self.navigation_lock = threading.Lock()
self._active_goal_handle = None
```

### 1.2 改造 `send_navigation_goal`：注册 feedback + result 回调

当前 `send_navigation_goal` 用 `send_goal_async` 后只判断 accepted/rejected 就返回了，**不跟踪后续状态**。

需要改造为：

```python
def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict:
    """发送导航目标点，同时注册 feedback 和 result 回调以追踪状态。"""

    # --- 前置检查（不变）---
    if self.navigation_client is None:
        return {
            'ok': False, 'deviceId': DEVICE_ID,
            'message': 'Navigation action client is not available',
        }
    if not self.navigation_client.wait_for_server(timeout_sec=2.0):
        return {
            'ok': False, 'deviceId': DEVICE_ID,
            'message': 'Navigation action server is not ready',
        }

    # --- 如果上一个 goal 还在进行中，先取消 ---
    with self.navigation_lock:
        if self.navigation_state['status'] == 'navigating':
            old_handle = self._active_goal_handle
            if old_handle is not None:
                old_handle.cancel_goal_async()
            self.navigation_state['status'] = 'idle'
            self.navigation_state['goal'] = None
            self.navigation_state['feedback'] = None
            self.navigation_state['result'] = None
            self.navigation_state['message'] = ''
            self._active_goal_handle = None

    # --- 构造 goal（不变）---
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = self.node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    z, w = _quaternion_from_yaw(yaw)
    goal.pose.pose.orientation.z = z
    goal.pose.pose.orientation.w = w

    # --- 使用 send_goal_async + feedback_callback ---
    future = self.navigation_client.send_goal_async(
        goal,
        feedback_callback=self._on_navigation_feedback,
    )

    deadline = time.time() + 3.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    if not future.done():
        with self.navigation_lock:
            self.navigation_state['status'] = 'idle'
            self.navigation_state['message'] = 'Navigation goal timed out waiting for server response'
        return {
            'ok': False, 'deviceId': DEVICE_ID,
            'frameId': 'map', 'x': x, 'y': y, 'yaw': yaw,
            'message': 'Navigation goal timed out waiting for server response',
        }

    try:
        goal_handle = future.result()
    except Exception as exc:
        with self.navigation_lock:
            self.navigation_state['status'] = 'idle'
            self.navigation_state['message'] = f'Navigation goal failed: {exc}'
        return {
            'ok': False, 'deviceId': DEVICE_ID,
            'frameId': 'map', 'x': x, 'y': y, 'yaw': yaw,
            'message': f'Navigation goal failed: {exc}',
        }

    if not goal_handle.accepted:
        with self.navigation_lock:
            self.navigation_state['status'] = 'idle'
            self.navigation_state['message'] = 'Navigation goal rejected'
        return {
            'ok': False, 'deviceId': DEVICE_ID,
            'frameId': 'map', 'x': x, 'y': y, 'yaw': yaw,
            'message': 'Navigation goal rejected',
        }

    # --- goal 被接受，注册 result 回调 ---
    with self.navigation_lock:
        self.navigation_state['status'] = 'navigating'
        self.navigation_state['goal'] = {'x': x, 'y': y, 'yaw': yaw}
        self.navigation_state['message'] = 'Navigation goal accepted'
        self.navigation_state['updatedAt'] = datetime.utcnow().isoformat() + 'Z'
        self._active_goal_handle = goal_handle

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self._on_navigation_result)

    return {
        'ok': True,
        'deviceId': DEVICE_ID,
        'frameId': 'map',
        'x': x, 'y': y, 'yaw': yaw,
        'message': 'Navigation goal accepted',
    }
```

### 1.3 新增 feedback 回调

```python
def _on_navigation_feedback(self, feedback_msg) -> None:
    """Nav2 action feedback 回调，每次规划器更新时触发。"""
    fb = feedback_msg.feedback
    with self.navigation_lock:
        self.navigation_state['feedback'] = {
            'distanceRemaining': float(fb.distance_remaining) if hasattr(fb, 'distance_remaining') else None,
            'navigationTime': float(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9) if hasattr(fb, 'navigation_time') else None,
            'numberOfRecoveries': int(fb.number_of_recoveries) if hasattr(fb, 'number_of_recoveries') else 0,
            'currentX': float(fb.current_pose.pose.position.x) if hasattr(fb, 'current_pose') else None,
            'currentY': float(fb.current_pose.pose.position.y) if hasattr(fb, 'current_pose') else None,
        }
        self.navigation_state['updatedAt'] = datetime.utcnow().isoformat() + 'Z'
```

### 1.4 新增 result 回调

```python
def _on_navigation_result(self, future) -> None:
    """Nav2 action result 回调，导航结束时触发。"""
    try:
        result = future.result()
    except Exception as exc:
        with self.navigation_lock:
            self.navigation_state['status'] = 'failed'
            self.navigation_state['result'] = None
            self.navigation_state['feedback'] = None
            self.navigation_state['message'] = f'Navigation failed: {exc}'
            self.navigation_state['updatedAt'] = datetime.utcnow().isoformat() + 'Z'
            self._active_goal_handle = None
        return

    status_code = result.status
    with self.navigation_lock:
        self.navigation_state['result'] = {
            'statusCode': int(status_code),
            'totalElapsed': float(result.result.total_elapsed_time.sec + result.result.total_elapsed_time.nanosec * 1e-9) if hasattr(result.result, 'total_elapsed_time') else None,
            'errorCode': int(result.result.error_code) if hasattr(result.result, 'error_code') else None,
            'errorMsg': str(result.result.error_msg) if hasattr(result.result, 'error_msg') and result.result.error_msg else '',
        }
        self.navigation_state['feedback'] = None
        self.navigation_state['updatedAt'] = datetime.utcnow().isoformat() + 'Z'

        if status_code == 0:   # SUCCEEDED
            self.navigation_state['status'] = 'succeeded'
            self.navigation_state['message'] = 'Navigation succeeded'
        elif status_code == 5: # CANCELED
            self.navigation_state['status'] = 'canceled'
            self.navigation_state['message'] = 'Navigation canceled'
        else:
            self.navigation_state['status'] = 'failed'
            self.navigation_state['message'] = (
                self.navigation_state['result'].get('errorMsg')
                or f'Navigation failed with status {status_code}'
            )

        self._active_goal_handle = None
```

### 1.5 新增 `get_navigation_status` 方法

```python
def get_navigation_status(self) -> dict:
    """返回当前导航状态的快照（线程安全）。"""
    with self.navigation_lock:
        return dict(self.navigation_state)
```

### 1.6 新增 `GET /api/navigation/status` 端点

在 `robot_agent.py` 文件末尾（约 `send_navigation_goal` 路由旁边）添加：

```python
@app.get('/api/navigation/status')
def get_navigation_status() -> dict:
    if publisher is None:
        return {
            'ok': False,
            'status': 'idle',
            'feedback': None,
            'result': None,
            'message': publisher_error or 'Publisher is not ready',
        }
    return publisher.get_navigation_status()
```

---

## 二、server/index.js 改动

### 2.1 新增辅助函数 `fetchNavigationStatus`

在 `fetchRobotPose()` 附近（约 633 行）新增：

```js
async function fetchNavigationStatus() {
  return getJetsonAgent('/api/navigation/status');
}
```

### 2.2 新增 `GET /api/navigation/status` 路由

在 `GET /api/robot/pose` 路由附近（约 1080 行），`/api/robot/pose` 块之后、`/api/map/latest.pgm` 之前插入：

```js
if (req.method === 'GET' && pathname === '/api/navigation/status') {
  try {
    const navStatus = await fetchNavigationStatus();
    sendJson(res, navStatus.ok ? 200 : 502, navStatus);
  } catch (error) {
    sendJson(res, 502, { ok: false, status: 'idle', feedback: null, result: null, message: error.message });
  }
  return;
}
```

**插入位置参考：**

```
... GET /api/robot/pose 块结束
         ← 插在这里
... GET /api/map/latest.pgm 块开始
```

---

## 三、API 契约

### GET /api/navigation/status

| 字段 | 类型 | 说明 |
|------|------|------|
| `ok` | bool | 请求是否成功 |
| `status` | string | `idle` / `navigating` / `succeeded` / `failed` / `canceled` |
| `goal` | object \| null | 当前目标点 `{x, y, yaw}`，idle时为null |
| `feedback` | object \| null | 最新反馈，navigating时有值 |
| `feedback.distanceRemaining` | float \| null | 剩余距离(米) |
| `feedback.navigationTime` | float \| null | 已导航秒数 |
| `feedback.numberOfRecoveries` | int | 规划恢复次数 |
| `feedback.currentX` | float \| null | 当前位姿X |
| `feedback.currentY` | float \| null | 当前位姿Y |
| `result` | object \| null | 最终结果，succeeded/failed时有值 |
| `result.statusCode` | int | Nav2 状态码：0=成功 |
| `result.totalElapsed` | float \| null | 总耗时(秒) |
| `result.errorCode` | int \| null | 错误码 |
| `result.errorMsg` | string | 错误描述 |
| `message` | string | 人类可读的状态描述 |
| `updatedAt` | string | ISO8601 时间戳 |

### 状态转换图

```
idle ──(发布goal被接受)──→ navigating
                              │
              ┌───────────────┼───────────────┐
              ↓               ↓               ↓
         succeeded        failed         canceled
              │               │               │
              └───────────────┴───────────────┘
                              ↓
                     idle (下次goal时自动重置)
```

---

## 四、部署步骤

在 Jetson（小车）上执行：

```bash
# 1. 同步代码到 Jetson（scp / rsync / git pull，按你现有方式）

# 2. 检查 robot_agent.py 语法
cd /opt/demo-control/jetson-agent
python3 -c "import py_compile; py_compile.compile('robot_agent.py', doraise=True)"

# 3. 检查 server/index.js 语法
cd /opt/demo-control/server
node --check index.js

# 4. 重启两个服务
sudo systemctl restart robot-agent
sudo systemctl restart demo-control

# 5. 验证
sleep 3
curl http://127.0.0.1:8001/api/navigation/status
curl http://127.0.0.1:3000/api/navigation/status
```

预期返回：
```json
{"ok": true, "status": "idle", "goal": null, "feedback": null, "result": null, "message": "", "updatedAt": ""}
```

---

## 五、端到端测试

```bash
# 1. 确保 Nav2 已启动（通过 demo-control 或手动）
curl -X POST http://127.0.0.1:3000/api/command \
  -H 'Content-Type: application/json' \
  -d '{"deviceId":"robot-001","action":"start_launch","package":"wheeltec_nav2","launchFile":"wheeltec_nav2.launch.py"}'

# 2. 发一个导航目标
curl -X POST http://127.0.0.1:3000/api/navigation/goal \
  -H 'Content-Type: application/json' \
  -d '{"x": 1.0, "y": 2.0, "yaw": 0.0}'

# 3. 轮询状态（重复执行看变化）
curl http://127.0.0.1:3000/api/navigation/status
```

---

## 六、注意事项

1. **线程安全**：`navigation_state` 的读写都用 `navigation_lock` 保护，避免 FastAPI 线程和 ROS2 callback 线程竞争。
2. **重复 goal**：发送新 goal 时如果当前状态是 `navigating`，会自动 cancel 旧 goal 再发新的。
3. **robot-agent 重启后**：状态重置为 `idle`，不影响 demo-control 转发。
4. **Nav2 未启动时**：`send_navigation_goal` 会返回 `Navigation action server is not ready`，不会 crash。
5. **py_compile 验证**：必须在 Jetson 上做（本地 Windows 没有 ROS2 依赖）。
