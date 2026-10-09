# 07 一次完整巡检

## 当前已确认的执行骨架

1. SaaS 把 `start_patrol` 放在 heartbeat response；参数只能包含预期 mapVersion、routeId 和 MissionPlan。
2. `platform_edge` 校验 allow-list、edge generation 和 command ID 去重，经 navigation supervisor Unix socket 调 `patrol.start_selected`。
3. supervisor/NavigationManager 校验选中 candidate、generation ≥10、地图/路线/mission hash 绑定和控制所有权。
4. manager 先启动唯一 SDK receiver（会做姿态准备），再启动整代 localization/Nav2 runtime；等待新的 runtime instance、固定地图定位 usable 和 costmap healthy。
5. manager 调 `/go2/patrol/start`；runtime 再检查 map/route identity、起点距离/朝向、fixed-map localization、Pose、costmap 和所有 Nav2 server。当前 candidate runtime 不启用 Unitree state 和独立 FAST-LIO stationary-health gate。
6. Smac 规划到下一个 checkpoint；MPPI 跟随，完整安全链授权 Unitree 行走。
7. 当前 Pose 投影到 Route，达到 checkpoint index 后取消 route goal，撤销 motion authorization。
8. runtime 要看到最终 `/cmd_vel` 和授权均为停，且稳定 `settleBeforeS`，才生成 stop receipt。⚠️ 在当前 candidate runtime 中 `require_robot_state=false`，Unitree 本体速度被按 0 处理；只有旧 `active` 分支才把实测本体速度纳入此证明。
9. runtime 根据录制 body yaw/相机 pan 与当前 body yaw，尽量用相机完成观察方向；超出 preferred pan 时用 Nav2 Spin 转机身，hard limit 内可 camera fallback。
10. `platform_edge` 从 navigation status 看到 `WAITING_PLATFORM`，调用本地 Site Console 云台 API；真实角度误差需 ≤2°。
11. 旧协议：上报 reached/stopped/pose_ready/announcing，等待播报/dwell 后写 `capture` control；新 evidence-transaction 协议则必须先获得并持久化平台 capture receipt，才能写 capture control。
12. runtime 执行 checkpoint `spin`（当前是 360° body spin capture 动作）或直接进入等待 verdict。
13. SaaS verdict `continue/retake/skip` 经 heartbeat/inbox 写原子 checkpoint control；timeout 也可继续，retake 增加 attempt。
14. checkpoint 完成后 runtime 清 local costmap、稳定等待，Smac 从当前 Pose 规划到下一个 checkpoint，绝不重放已完成 Route 前缀。
15. 最后一个目标成功后 runtime `COMPLETED`；stop/teardown 路径执行 `StopMove` 并释放 motion owner。

状态：🟡 步骤 1–10、12–15 的狗端链已确认。步骤 11 具体走旧协议还是新证据事务取决于部署环境变量与 artifact capability；release 默认关闭新协议，r14 实际值尚未从非密钥部署回执确认。保存的 r14 lethal-start mission 是 `local_operator`，所以它不证明 SaaS checkpoint 流已走通。SaaS 服务端怎样创建任务、判定图片和生成最终报告不在本仓库，保持 ❓。

## 四端责任

| 步骤 | SaaS | 云建图 | 机器狗背板 | Unitree 本体 |
|---|---|---|---|---|
| 任务/命令 | 创建/下发 mission 与 allow-listed command | 无 | 校验、去重、启动选中 route | 无 |
| 定位/规划 | 只看状态，不算 Pose/Path | 无实时职责 | FAST-LIO、VGICP、Nav2 | 提供 SDK 状态；不参与本系统 map 定位 |
| 运动 | 不发速度 | 无 | Planner/MPPI/safety/UDP bridge | SDK2 `Move` 执行 |
| checkpoint | 播报、capture、verdict/receipt | 无 | 真停确认、机身/云台对齐、协议协调 | 执行转身/停止 |
| 结果 | 外部实现未入库 | 无 | 上报状态、checkpoint/evidence 协议 | 无报告职责 |

## 明确不应混入主链的代码

- `modules/mission/gogoguard_mission/MissionCoordinator` 是一套通用纯状态机，当前没有被任何 runtime entrypoint 调用。
- 当前真实 checkpoint 状态机是 `CheckpointExecutor` + `PatrolRuntimeManager`，平台协调是 `CheckpointCoordinator`。
- 因此通用 mission 模块只能作为设计/测试资产标为 ⚪，不能用它解释现场状态转换。
