# 16 正向与反向闭环验证

## 正向：Sensor → Motion

| 箭头 | 证据 | 状态 |
|---|---|---|
| MID360 → Livox ROS topics | edge entrypoint + driver args | ✅ |
| LiDAR/IMU → FAST-LIO odom/deskewed cloud | loaded YAML + `laserMapping.cpp` | ✅ |
| lidar cloud → base cloud | commissioned mount + calibrated transformer | ✅ |
| current cloud + fixed PCD → map→odom | candidate launch + patched continuous localizer | ✅ |
| map Pose → Route target | PatrolRuntime TF lookup/route projection | ✅ |
| target + global costmap → Smac Path | Nav2 action/config/runtime call | ✅ |
| Path + Pose + local costmap → MPPI Twist | controller config + topics | ✅ |
| Twist → smoother → Collision Monitor → auth bridge | launch/topic chain | ✅ |
| `/cmd_vel` → UDP → SDK receiver → `SportClient.Move` | sender/receiver完整代码 | ✅ |
| Go2 movement → next sensor observation | 现实闭环；firmware internals external | ✅ 边界确认 |

正向每个箭头均存在。❓ 外部 Unitree firmware 的具体步态实现不在仓库，但这不阻断应用边界闭环。

## 反向：为什么狗会向左转？

以下是**当前系统的诊断追溯顺序**，不是声称 r14 真的左转；r14 主要现场结果是 start-lethal、最终命令为零。

1. SDK receiver 当时收到的 UDP v2 packet 是什么？先查最终 `vx/vy/wz` 与 sequence/time；正 yaw 命令的具体实机方向应以 bridge 坐标合同/现场校验为准。
2. UDP sender 从 `/cmd_vel` 收到什么？若不一致，查序列化/receiver cap/watchdog。
3. `unitree_safe_cmd` 为什么放行或限幅？查同一时刻 runtime authorization、新鲜度、timeout 与 final trace。
4. Collision Monitor 是否改写了 smoother 命令？对比 `/nav2/smoothed_cmd_vel` 与 `/patrol_cmd`，查 stop polygon 点数。
5. MPPI 为什么产生该 `wz/vy`？查 active Path、local costmap、当前 Pose、critics 与 FollowPath feedback。
6. Smac 为什么把 Path 放在这一侧？查 start/goal、global costmap 各 layer、keepout、footprint。
7. 为什么目标是这个点？查 PatrolRuntime 当前 Route projection、checkpoint route index、rejoin/recovery state。
8. Controller 当时认为 Pose 是多少？查 `map→base_link`、localization status/fitness/inlier 与 TF timestamps。
9. Pose 为什么是这个值？拆成 VGICP `map→odom` 与 FAST-LIO `odom→base_link`，再追 MID360 点云、IMU、外参和时间同步。

## 用 r14 实例反向验证

```text
现场结果：狗没有开始走，任务保持 SEARCHING_PATH
← final /cmd_vel 一直为 0，runtime motionAuthorized=false
← 没有稳定 accepted FollowPath；Smac 多次返回 start lethal
← Smac 检查完整 padded footprint
← diagnostics 只检查中心 cell=92，所以错误显示 costmap healthy
← r14 Pose 相比 recorded start 约偏 14cm/5°
← 该 footprint 在静态 PGM 上覆盖 1 个黑格，recorded start 为 0
```

最后一箭头是🟡高概率静态解释，不是完整根因定案：生产 incident 没有保存 static/obstacle/inflation 分层 costmap，因此必须先补 layer dump 才能确定黑格属于哪一层贡献。

## 验证结论

- ✅ 正向链完整；
- ✅ 反向从最终命令可以回溯到 Path、Pose、传感器；
- ✅ r14“未运动”已闭合到 planner start footprint lethal 与 terminal/health 契约问题；
- ❓ TF 双 authority、costmap layer contribution、Unitree measured stop 仍是明确证据缺口，不用确定语气补齐。
