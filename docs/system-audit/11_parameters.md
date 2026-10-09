# 11 参数与影响范围

## 参数优先级

```text
NavigationProfileStore active.json（操作员参数，会先迁移/校验成 v6）
→ NavigationManager 展开为 ros2 launch arguments
→ active_map_patrol.launch.py 把可调整值覆盖到 Nav2/PatrolRuntime 节点
+ generation-10 runtime_profile.json 固定地图、初始化区、route sampling 和身体几何
+ go2_nav2_patrol.yaml 提供其他 Nav2 基值
+ launch 中 localizer/safety 参数
+ SDK receiver 中最终硬上限
```

✅ r14 诊断回执记录的操作员原文仍是 v4/revision 4 的激进组合，但 `validate_profile()` 只对当年确切发布过的 v4 tuple 进行定向迁移：实际 launch 收到的是 v6 安全包络，即 turn `0.4 rad/s`、accel/decel `0.9 m/s²`、MPPI batch `1000`，而非文件表面的 `0.6 / 2.0 / 2.5 / 700`。这是“保存值 ≠ 有效值”的实例。

## r14 当前已确认有效值（第一批）

| 参数 | 有效值 | 消费者 | 修改后主要影响 |
|---|---:|---|---|
| target cruise / max forward | 0.60 / 0.90 m/s | MPPI / smoother / SDK cap | 轨迹可行性、刹停距离、跟踪偏差；不是只改“快慢” |
| lateral / yaw max | 0.20 m/s / 0.40 rad/s | MPPI、smoother、UDP/SDK | Omni 横向绕行能力、转向时间、checkpoint Spin |
| accel / decel | 0.90 / 0.90 m/s² | velocity smoother | 指令斜率、刹停距离、MPPI 指令可达性 |
| MPPI frequency / horizon / samples / iterations | 15 Hz / 56 / 1000 / 1 | controller_server | 约 3.73 s 预测时域，840k rollout states/s；改动同时影响 CPU 和路径选择 |
| physical / padded footprint | x `[-0.33,0.40]`, y `±0.20` / x `[-0.43,0.50]`, y `±0.30` m | global/local costmap、MPPI CostCritic、Collision Monitor | 哪些起点/通道可行，安全肩宽 |
| inflation radius / scaling | 0.45 m / 3.0 | global/local costmap、MPPI | 障碍物周围软代价宽度与衰减；与 footprint 共同决定可行通道 |
| local costmap | 8×8 m, 0.08 m/cell | MPPI/Spin/runtime health | 局部绕行视野、网格粒度、CPU/内存 |
| static map | 1944×1696, 0.10 m/cell | Smac + both costmaps | 全局可通性、起点/Route footprint 冲突 |
| obstacle z/range | 0.05–1.45 m; global mark/clear 10/12 m; local 4/5 m | ObstacleLayer | 什么点成为障碍、多远开始标记/清除 |
| runtime localization status/dropout/recovery | 0.60 / 1.00 / 0.50 s | PatrolRuntime | 定位 gap 多快撤权/取消，恢复多久后重新导航 |
| replan interval / progress window | 0.75 / 5.0 s | PatrolRuntime recovery | 规划失败重试频率、执行失败分类窗口 |
| route obstruction | cost 65, 2 连续样本, 0.50 s | runtime Route 中心线检查 | 何时把故障分为路径堵塞并尝试绕行 |
| SDK command watchdog | 250 ms | Unitree receiver | 丢包/上游卡死后多快 `StopMove` |

## 固定地图定位：v2 sealed profile

来源关系：locked YAML 是 v1；完整 Docker build 应用 `go2-vgicp-orin-v2.patch`，镜像内才是 v2；candidate runtime 再注入地图文件、map version 和初始化中心/yaw。continuous localizer 启动时会逐项校验 sealed profile，关键值不符就直接拒绝启动。

| 参数组 | 当前 v2 有效值 | 影响 |
|---|---:|---|
| update / status | 0.4s / 0.1s | registration 2.5Hz；安全状态 10Hz，不代表 registration 10Hz |
| cloud DDS history | keep-last 1 | 忙时丢旧扫描而非事后重放；降低陈旧定位，增加对最新帧质量依赖 |
| odom-cloud skew / input age | ≤0.08s / ≤0.35s | 时间同步和调度新鲜度；v2 stale frame 不推进 LOST、不打断恢复候选簇 |
| scan z / range | `[-1.5,2.5]m` / 24m | source 几何范围、动态点与计算量 |
| tracking source/map voxel | 0.25 / 0.30m | 精度、点数、计算量 |
| coarse source/map voxel | 0.65 / 0.80m | 启动搜索速度与粗粒度可分辨性 |
| threads | 4 | Orin CPU 并行度与系统争用 |
| local target radius / refresh | 32m / 移动4m | 历史 PCD 裁剪计算量；覆盖 24m scan + 5m start/recovery + 3m coarse gate |
| startup XY | center 来自 map candidate；radius 5m，step 1m | 初始位置容忍度与候选数量 |
| startup yaw | center 来自 candidate；radius 60°，step 15° | 初始朝向容忍度与重复解风险 |
| accumulation | window 2.5s、min 2.0s/8 scans/500 points、leaf .18m、max 40 scans | 初始化 source 的几何丰富度与等待时间 |
| coarse/refine | corr 3m、screen 5 iter/inlier .05、refine 15 iter、top 8 | 搜索速度与漏解/假解概率 |
| ambiguity | best/second ≥1.12；差异至少 1m 或20° | 拒绝重复场景中同样好的候选 |
| tracking | corr1.2m、30 iter、alpha .35、每次 max .15m/2° | 连续收敛范围、平滑与响应速度 |
| quality | ≥250点、inlier≥.20、MSE≤.20、min eig≥1e-6、condition≤1e8 | 几何支持、拟合与可观测性 |
| correction jump | ≤.75m / 15° | 防止可信轨迹突然跳到另一位置 |
| LOST recovery | 4m / 45°，3次确认，彼此≤.30m/6° | 找回范围与误锁风险；另需状态机连续好样本恢复 TRACKING |

r5 candidate 注入的启动中心约 `(-0.2176,-0.0411)`、yaw center `0.17495rad`；这些是地图版本资产，不是通用 YAML 默认 `0,0,0`。

## FAST-LIO 当前加载值

实际入口加载 `third_party/locked_stack/src/go2_mapping_capture/config/go2_mid360_capture_fastlio.yaml`，随后加载 `config/robot/fastlio_base_output.yaml`。不要误用同仓库另一个 `FAST_LIO/config/go2_mid360s.yaml` 推断现场值。

| 参数 | 当前值 | 影响 |
|---|---:|---|
| MID360 input | lidar_type 4，10Hz，timestamp ns | 使用 PointCloud2 逐点绝对事件时间；错误 type 会走不同消息/时间语义 |
| lidar / IMU QoS depth | 2 / 400 | 点云最新优先，同时保留足够 IMU 传播历史 |
| time sync / offset | false / 0.0 | 依赖驱动/系统提供正确共同时间；不做内部时间补偿 |
| blind / detection range | 0.5m / 100m | 近距离剔除与地图检索范围；localizer 后续只取24m |
| point filter / iterations | 3 / 3 | 点云采样与每帧状态更新计算量 |
| surf/map voxel | 0.5 / 0.5m | FAST-LIO 局部几何密度与计算量 |
| noise | acc/gyr .1；bias random walk .0001/.0001 | ESKF 对 IMU 预测与点云校正的相对信任 |
| online extrinsic estimate | false | 使用固定 LiDAR→internal IMU 外参，不在现场在线漂移标定 |
| publish | scan true/bodyframe true；path/map/pcd false | 巡检只消费去畸变扫描与 odometry，避免增长型 map/path 输出 |
| base output | commissioned `base→imu` translation/quaternion + 两个 calibration SHA | 把 FAST-LIO 内部 frame 结果规范成 `odom→base_link`；改错会同时污染 Pose 和点云 |

## 启动与安全 gate 的有效开关

| gate/capability | candidate r5/r14 | 解释 |
|---|---|---|
| Unitree measured state readiness | false | `runtime_source=candidate` 分支关闭，不是“数据正常所以没显示” |
| independent FAST-LIO health readiness | false | 同上；localizer 仍间接需要 `/Odometry`，但没有独立 stationary drift gate |
| safe cmd `require_localization` | false | 最终桥不再重复定位 gate；PatrolRuntime authorization 承担撤权 |
| safe cmd `require_obstacle_gate` | false | 障碍责任在 Costmap/Collision Monitor；最终桥保留的旧 gate 能力未启用 |
| safe cmd runtime authorization | true，timeout 约0.5s | 当前最终非零命令的核心 fail-closed gate |
| interaction / platform heartbeat | 默认 false / false | 条件子系统；必须看实际 runtime env 才能确认 r14 是否启用 |

## 参数的历史负担

- v1–v5 profile 都在运行时迁移，其中 v1/v2 存在 Unitree 接收链实际无法执行的值。
- 代码注释记录 v4 的整组激进参数曾让 `VelocityDeadband` 对低速转角轨迹罚分过重，破坏路径可行性。
- 因此后续任何速度调整必须同时看 MPPI critics、smoother、receiver cap 和现场可行路径，不能只看一个 JSON 数值。

checkpoint 的 camera/spin/dwell、证据交易开关和平台 timeout 属于 mission/asset 数据与 runtime env 共同决定；不能给出一个全场地固定表。最终 `system-model/parameters.json` 会把来源与影响边一起结构化。
