# 06 运动控制链路

## 已确认的最终运动链

```text
Nav2 MPPI controller / Spin behavior
→ /nav2/raw_cmd_vel
→ nav2_velocity_smoother
→ /nav2/smoothed_cmd_vel
→ nav2_collision_monitor（点云多边形碰撞判定）
→ /patrol_cmd
→ unitree_safe_cmd_node（runtime authorization + command freshness + 限幅）
→ /cmd_vel
→ cmd_vel_udp_sender
→ UDP localhost:5005，G2CM version 2 packet
→ go2_sdk2_udp_receiver
→ Unitree SDK2 SportClient.Move(vx, vy, vyaw)
→ 本体底层运动
```

状态：✅ launch、YAML、Python safety node、UDP sender 和 SDK receiver 完整源码交叉确认。

## 谁决定左转/右转

- 常规沿路：MPPI 根据全局 Path、当前 Pose、预测动力学和 costmap 选择速度样本，输出线速度与角速度。
- checkpoint 机身转向：Patrol runtime 触发 Nav2 Spin/对齐状态机，行为节点输出角速度。
- 最终桥接层不做路径判断；它只检查授权、时效、数值与上限，再忠实转换为 `Move`。

## 独立停止层

| 停止条件 | 负责层 | 行为 | 状态 |
|---|---|---|---|
| 前方多边形点数超过 8 | Nav2 Collision Monitor | 将输出速度置零/停止 | ✅ |
| runtime authorization false/stale | `unitree_safe_cmd_node` | `/cmd_vel` 输出零 | ✅ |
| 上游命令超时/非法/超限 | `unitree_safe_cmd_node` | 输出零并记录原因 | ✅ |
| UDP 超过默认 250 ms 未更新 | `go2_sdk2_udp_receiver` | `StopMove()` | ✅ |
| runtime/edge teardown | Manager/receiver | 直接 motion probe/SDK `StopMove()`，清理进程 | ✅ |

## 当前有效边界

- ✅ SDK receiver 硬上限：前进 0.9 m/s、横移 0.2 m/s、偏航 0.6 rad/s；超过范围的包不会扩大执行能力。
- ✅ Nav2 runtime 当前把障碍物停止职责交给 Collision Monitor；`unitree_safe_cmd_node` 的 `require_obstacle_gate=false`。
- ✅ fixed-map localization 也不由 safe node 二次 gating；`require_localization=false`，真正的运动授权来自 patrol runtime 发布的 runtime authorization。
- ✅ receiver 启动可先执行 `StandUp`、`BalanceStand` 和一次 `StopMove`，然后才 bind UDP 5005。此“姿态准备”不是巡检路线运动，但会改变机器人本体状态。
- ⚠️ 运动安全依赖多个超时层；后续需要把每层的真实超时和重启/抢占交互放入参数影响表，而不是笼统称为“有 watchdog”。
