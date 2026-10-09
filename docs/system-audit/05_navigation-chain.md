# 05 导航链路

## 当前 generation-10 主链

```text
MissionPlan（mapVersion + routeId + checkpoint route index）
+ generation-10 candidate（PCD + PGM + keepout + runtime profile）
+ /localization/pose（map→base_link）
+ /navigation/cloud_obstacles（实时障碍）
→ patrol runtime readiness gate
→ SmacPlanner2D ComputePathToPose
  目标 = 下一个 checkpoint 的 execution-route 点；无 checkpoint 时为 route 终点
→ Nav2 Path
→ MPPI FollowPath
→ velocity smoother
→ collision monitor
→ final safety / Unitree motion chain
→ 新 Pose 与 local costmap 反馈
```

状态：✅ 入口、插件、目标选择、Path、控制、进度和恢复均由 supervisor/manager/launch/patrol runtime/YAML 完整源码确认。

## Route、Waypoint、Path、Checkpoint 的区别

| 名称 | 当前系统含义 | 谁生成/使用 | 是否实时变化 |
|---|---|---|---|
| recorded trajectory | GLIM 优化后的录制运动轨迹；pose 原始为 lidar | cloud exporter；workspace seed/checkpoint binding | 固定资产 |
| Route | 操作员审核后的 base_link 平面巡检参考线，217 个 waypoint | workspace/RouteManager；任务顺序、进度、目标索引和恢复参考 | 版本化固定 |
| execution route | Route 按 0.15 m 上限插值，当前 714 点 | candidate/platform export；checkpoint `routeProgressIndex` | 版本化固定 |
| Waypoint | Route 上带 x/y/yaw 的参考点 | RouteManager 与 runtime | 固定 |
| Checkpoint | 绑定到 execution route index 的巡检停靠任务，还含 body/camera 观察方向 | recording + GLIM timestamp binding + MissionPlan | 每 mission 固定 |
| Nav2 Path | Smac 从当前 Pose 到下一 checkpoint/终点算出的实际可行路径 | PlannerServer → MPPI | 每段/每次重规划变化 |

## 一个容易误读的实现细节

当前候选具有 `navigation-map`，所以开始/恢复会先调用 `ComputePathToPose`，而不是把整条 recorded Route 原样送给 MPPI。Route 的关键职责是目标序列、checkpoint 索引、进度、后缀和绕行重接参照；真正从当前位置到下一目标的几何 Path 由 Smac 算。只有没有 navigation map 的兼容分支或部分 recovery path 才会直接发送 Route 后缀。状态：✅。

## 当前 Planner / Controller / Costmap

- ✅ Planner：`nav2_smac_planner/SmacPlanner2D`，ID `GridBased`。
- ✅ Controller：Nav2 MPPI Omni，运行 15 Hz；选择一个 `FollowPath` controller。
- ✅ Global/Local costmap 都含 static、obstacle、inflation、keepout；local 为 8 m × 8 m、0.08 m resolution，global 静态图为当前 0.10 m/cell。
- ✅ 机器人几何使用 padded rectangle x `[−0.43, 0.50]`、y `±0.30`，不再是仅圆半径。
- ✅ live obstacle 来自 `/navigation/cloud_obstacles`；点云自体过滤在进入 costmap/Collision Monitor 前完成。
- ✅ 当前 candidate runtime 起步前要求 runtime binding、fixed-map localization、Pose、planner/controller/spin server 和 local costmap 健康；起点还要距 Route 开头 ≤1.5 m、yaw error ≤45°。
- ⚠️ `runtime_source=active` 的旧/兼容分支还要求 Unitree robot state 和独立 FAST-LIO stationary-health；当前 manager 启动的是 `runtime_source=candidate`，这两项 gate 被显式关闭。FAST-LIO 故障仍会经 localizer 输入间接导致定位不可用，但没有单独的 stationary drift readiness 证明。

## 偏航、偏离和纠偏分别由谁做

- 当前 Pose 持续由 localization 链发布。
- Route progress 是 runtime 用当前 map Pose 投影到最近 Route sample 后单调递增的 index；它不是控制误差本身。
- MPPI 用 Path 与当前 Pose 计算横向/航向误差，采样速度轨迹并输出纠偏 Twist；因此“应该左转还是右转”通常由 MPPI critic 综合结果决定。
- Smac 负责从起点到目标的全局可通行几何，不负责每周期转向。
- Collision Monitor 可以把已平滑命令停为零，但不重新选择绕行方向。

## 失败与恢复

- localization/FAST-LIO/TF/costmap 短暂 gap：立即撤销运动授权；在 grace 内可保留 action，超时后 cancel，稳定恢复并清 costmap 后规划/发送后缀。
- controller failure：结合 localization、costmap、route obstruction 和真实 pose/cmd 运动证据分类为定位、costmap、阻塞、控制/执行失败，再选择 retry MPPI、等 costmap 或 Smac replan。
- 相同 `PATH_OBSTRUCTED` 连续 8 次且 costmap 健康：进入 `PERSISTENT_PATH_OBSTRUCTION` fault。
- 当前 r14 起点 lethal 的失败形状没有进入上述 bounded terminal，保持 `SEARCHING_PATH`，已列为状态机 Bug。

## 当前现场事实

- ✅ 保存的 r14 现场 mission 是 `local_operator`、r5、4 checkpoints；定位为 `TRACKING`，但 Smac 报 `Starting point in lethal space!`，运动授权始终 false。因此不是“狗定位丢了以后乱走”，也不是一次已执行的 SaaS checkpoint mission。
- ✅ runtime 的 `costmapHealth` 只检查机器人中心所在的单个 cell；r14 的该 cell cost 为 92，低于代码默认 lethal threshold 253，所以显示 healthy。Smac 检查的是整个矩形 footprint，二者同时出现不矛盾。
- 🟡 将现场 Pose `(-0.121, -0.144, yaw 0.265)` 投影回 r5 `navigation-map.pgm` 后，矩形 footprint 覆盖的约 56 个 10 cm cell 中有 1 个黑格（中心约 `(-0.15, -0.45)`）；同一 footprint 在 allowed-area mask 中全部允许。录制 Route 起点的原始 Pose 则是 0 个黑格。这与“现场 Pose 比 Route 起点偏约 14 cm，footprint 边角碰到静态不可通格”高度一致。
- ❓ production 诊断配置当时 `record_costmap=false`，没有保存分层 costmap；因此不能完全排除 live obstacle/inflation 的叠加作用，也不能把上述高概率证据写成已确认根因。
