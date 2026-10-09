# 15 系统体检发现

## 已登记、尚未修复

| 分类 | 发现 | 状态 | 证据 |
|---|---|---|---|
| 现场 Bug/状态机 | Smac 起点 lethal 后 r14 的 bounded terminal rule 未覆盖该失败形状，运行保持 `SEARCHING_PATH` 直到人工 stop | ✅ 现场现象；根因拆为两个待测问题 | `PROJECT_STATE.md` 2026-08-25 |
| 地图/坐标历史缺陷 | LiDAR pose 被当 base pose、圆形 footprint 与矩形实机不一致、PGM origin 与审核 lattice 不一致曾先后造成 preview/runtime 偏差 | ✅ 历史根因和修复回执 | `PROJECT_STATE.md` 2026-08-24 |
| 可复现性 | 历史 `-dirty` 发布不能只靠 commit 重建 | ✅ | 部署回执 |
| 文档时效 | 8 月 12 日技术指南没有覆盖所有 generation-10/r14 事实 | ✅ | 文档日期与最新状态对比 |
| 安全/状态证明 | 当前 candidate runtime 关闭 Unitree state 与独立 FAST-LIO health gate；checkpoint stop receipt 不含真实本体速度证明 | ✅ 静态配置与 r14 status 均确认；现场风险尚未量化 | PatrolRuntime source 分支、manager launch、r14 status |
| 主链重复 | 通用 `MissionCoordinator` 和 `EvidenceFrameBuffer` 存在但未被当前 runtime 调用，真实职责分别在 navigation/platform_edge | ✅ 非当前主链 | runtime imports/entrypoints |
| 导航健康定义 | readiness 只检查中心 cell <253，不证明整个 footprint 可行；r14 因此同时出现 healthy/92 和 Smac start lethal | ✅ 代码+现场回执确认 | `evaluate_costmap_health()`、Smac log、incident decisions |
| r14 起点空间证据 | 现场 footprint 覆盖 r5 静态 PGM 中 1 个不可通黑格，录制起点为 0；allowed mask 全允许 | 🟡 静态格高概率解释 lethal，无 layer dump 暂不定案 | r5 PGM/YAML、r14 Pose、footprint |
| 资产部署回执 | Mac upload state 仅 r3 verified/not activated，r14 robot 却已选 r5 | 🟡 回执链不完整 | platform upload JSON、r5 manifest、r14 selected candidate |
| 管理面访问控制 | Robot Site Console 明确绑定 `0.0.0.0:8080`，当前 `BaseHTTPRequestHandler` 对读写 API 没有认证/授权检查；接口含 patrol start/stop、localization reset、profile 修改、gimbal、录制与地图导入 | ✅ 代码事实；实际可达网络范围 ❓ | `deployment/container/edge-entrypoint`、`site_console/__main__.py`、`server.py` |
| 认证实现漂移 | 旧/旁路 `site_console_capture_adapter` 强制加载私密 bearer token 并发送 `Authorization`，但当前 Site Console server 不读取或验证该 header | ✅ 静态代码 | adapter 与 server 完整代码 |

## 29 项系统体检

### 1. 当前系统架构

✅ 五边界：SaaS 负责任务/心跳/媒体协议；地图云负责离线 GLIM；Mac 负责录制同步、地图审核与发布；Edge 负责实时 ROS/任务/安全；Unitree 负责执行 `Move/StopMove`。具体图由 `system-model/system.json` 的 `boundaries` 与 `startupTree` 生成。

### 2. 一次巡检完整执行链

✅ `start_patrol(expected map/route, MissionPlan) → supervisor → NavigationManager → candidate runtime → fixed localization/costmap/Nav2 readiness → Route 投影选择目标 → Smac Path → MPPI/safety/SDK → checkpoint settle/pose/capture/verdict → 下一点 → StopMove/receipt`。详见 `07_inspection-workflow.md`。

### 3. 当前定位链路

✅ `MID360 LiDAR+IMU → FAST-LIO odom→base → calibrated body cloud → continuous VGICP fixed PCD → map→odom → map→base Pose`。Unitree state 不参与几何 Pose。

### 4. 当前建图链路

✅ 狗端录 bag/checkpoint/哈希封存；Robot→Mac 可续传；Mac→云 GLIM；exporter 补齐 trajectory prefix；workspace 一次性做 LiDAR→base、2D slice、人工编辑和 allowed area；发布不可变 r5 candidate。

### 5. 当前重定位链路

✅ 启动短时累积 current cloud，与固定 PCD 32m local target 做 coarse seed + top-8 refine VGICP；有 ambiguity/fitness/inlier/condition/jump gate。LOST 围绕最后可信 Pose 搜 4m/45°，3 次一致确认后再经过正常 good-sample 状态机。

### 6. 当前导航链路

✅ Route 给任务进度和下一目标；global costmap 叠加 static/obstacle/inflation/keepout；SmacPlanner2D 每次计算当前 Pose 到目标的 Path。Route 不是直接重放命令。

### 7. 当前路径跟踪链路

✅ MPPI Omni 以 Pose、Smac Path、local costmap 为输入，15Hz 滚动预测；PatrolRuntime 监视 route 投影、action feedback、进展、定位和 costmap，必要时取消/重规划/恢复。

### 8. 当前运动控制链路

✅ `/nav2/raw_cmd_vel → smoother → /nav2/smoothed_cmd_vel → Collision Monitor → /patrol_cmd → unitree_safe_cmd授权/限幅/watchdog → /cmd_vel → UDP v2:5005 → SDK receiver → SportClient.Move`；250ms 超时和 teardown 走 StopMove。

### 9. SaaS / 云端 / 狗端通信链路

✅/🟡 platform_edge 默认 5s 出站 heartbeat，响应命令经过 allow-list/去重，巡检只允许 start/stop；checkpoint verdict/announcement 经 schema 进入 inbox。地图云是另一条 SSH/rsync 链。SaaS 服务端内部未知；optional interaction 默认关闭，r14 enable 未确认。

### 10. 当前地图体系

✅ bag、GLIM PLY、固定定位 PCD、浏览 map.json、静态 PGM、keepout、global/local costmap、Route、Path 均有不同创建者和消费者。详见 `09_maps-and-coordinate-systems.md`。

### 11. 当前坐标系体系

✅ `map→odom→base_link→lidar_link`。map 是长期固定坐标；odom 连续但漂移；base 是狗体导航基座；LiDAR 由 commissioned mount 固定连接。❓ odom→base 现场 authority 是否双发布仍待取证。

### 12. 当前实际使用算法

✅ FAST-LIO、small_gicp VGICP、SmacPlanner2D、MPPI Omni、Costmap layers、Collision Monitor；🟡 外部 GLIM 边界确认但内部算法配置未知。ICP/NDT/PCL GICP 只在旧节点/benchmark。

### 13. 每个算法存在的原因

- FAST-LIO：高频连续相对运动与扫描去畸变；必要。
- VGICP：把每次启动的 odom 锚到历史 3D map；必要，但依赖启动先验。
- GLIM：用整段录制减小实时里程计累计漂移；合理，版本治理不足。
- Smac：处理当前二维可通性而非机械重放 Route；必要。
- MPPI：利用 Go2 Omni 自由度做滚动跟踪/局部避障；有产品价值，需现场对照证明复杂度收益。

### 14. 当前关键参数

✅ r14 effective：forward target/max `.6/.9m/s`，lateral `.2m/s`，yaw `.4rad/s`，accel/decel `.9/.9m/s²`；MPPI 15Hz/56/1000/1；padded footprint x[-.43,.50], y±.30m；inflation .45m；VGICP 24m scan/32m target 与 sealed quality gate。详见 `11_parameters.md`。

### 15. 参数影响范围

✅ 速度不是只影响快慢，还联动 MPPI critic、smoother、刹停距离和 SDK cap；footprint/inflation 决定 start/通道可行性；VGICP voxel/range/seed 联动计算量、收敛域和歧义；timeout 联动 fail-closed 灵敏度与误撤权。

### 16. 当前 fallback 机制

✅ 定位：TRACKING→DEGRADED→LOST→锚点 recovery；导航：action cancel、重规划、route suffix/rejoin；checkpoint：相机 yaw 优先、Spin 补余量、对齐失败有 verdict/fallback；媒体：有界指数退避重连；命令：任何授权/输入超时归零。⚠️ 没有固定地图 registration 算法 fallback。

### 17. 当前异常恢复机制

✅ supervisor required child 退出会清理并交给 systemd 重启；NavigationManager 有 recover runtime；PatrolRuntime 对定位 drop、controller/path obstruction、progress timeout 分级恢复；StopMove 在多层 teardown 执行。⚠️ r14 证明 planner start-lethal 失败形状没有进入有界 terminal。

### 18. 当前历史遗留

✅ 旧 ICP route relocalizer、submap builder、CSV follower、patrol_control/controller_core、v1–v5 profile 迁移、同名 locked/current nav package、旧注释/manifest 均已登记，不在本阶段删除。

### 19. 当前重复实现

✅ 真重复/职责重叠候选：旧 waypoint follower vs Smac/MPPI；旧 route relocalizer vs continuous map localizer；MissionCoordinator vs PatrolRuntime checkpoint state；EvidenceFrameBuffer vs platform evidence transaction；可能双 TF broadcaster。完整/combined Docker、Route/Path、FAST-LIO/VGICP 则不是重复。

### 20. 当前疑似 Dead Code

🧹 `waypoint_follower_go2_2.py`、`patrol_control.py`、`controller_core.py`、未调用 MissionCoordinator/EvidenceFrameBuffer。⚪ benchmark/submap builder/route relocalizer 更准确是“可安装但非当前主链”，是否仍作测试工具需所有权确认后再删。

### 21. 当前最严重技术债务

1. 现场运行身份跨 source/image/env/map/SaaS/GLIM，未被一个 FieldRunManifest 原子绑定；
2. health/readiness 的“中心格/真实 footprint”“candidate/active”“命令零/实测本体停止”语义裂缝；
3. 外部 GLIM 与 SaaS 没有可查询的统一 deployment identity；
4. 同名/旧实现与 build patch 使“看到源码默认值”不等于实际镜像；
5. 管理 HTTP 面无服务端认证，依赖未知网络边界。

### 22. 当前最影响现场稳定性的 5 个问题

1. **P1** r14 start footprint lethal，静态黑格为高概率原因但缺 layer dump；
2. **P1** planner failure 不进入 bounded terminal，形成无穷 SEARCHING_PATH；
3. **P1** diagnostics 中心格健康不能证明 planner footprint 可行；
4. **P1** candidate readiness 不要求 Unitree measured state/独立 FAST-LIO health，checkpoint stop 证明弱；
5. **P2** saved profile、migrated effective profile、SDK cap 三层不直观，现场调参易归因错误。

### 23. 哪些属于 Bug

- ✅ planner start-lethal 没累计到 bounded persistent failure/终止逻辑；
- ✅ costmap health 使用中心 cell 与 Smac footprint 语义不一致（至少是诊断契约 Bug）；
- 🟡 Python shutdown error exit 可能只是清理噪声，尚不能定性；
- ✅ 注释/manifest generation/source selection 与实际构建不一致是维护性 Bug。

### 24. 哪些属于参数问题

- 🟡 r14 footprint 压黑格可能由 Pose 数厘米/几度偏差、static map cell、padding/inflation 或 obstacle range 组合触发；没有 layer dump 前不能归为单参数。
- ✅ v4 保存值经过定向迁移，不应直接按文件数值调试。
- 速度、deadband critic、smoother、SDK cap 需作为参数包整体评估，而非单旋钮。

### 25. 哪些属于算法问题

- ❓ 重复场景、大环境变化、超 seed window 是 VGICP 方法本身的已知边界，但当前没有现场统计证明它已成为主要失败源。
- ❓ MPPI 在本场景是否优于更简单 controller 需要 A/B field evidence；不能因复杂就判过度设计。
- r14 start-lethal 首先是输入地图/Pose/health/恢复契约问题，不是“Smac 搜索算法不会规划”。

### 26. 哪些属于架构问题

- ✅ 系统事实与有效配置分散在 source patch、profile migration、runtime env、candidate asset 和 receiver cap；
- ✅ readiness/safety responsibility 在 PatrolRuntime、Costmap、Collision Monitor、safe cmd、SDK 多层存在，但诊断模型没有统一安全证明；
- ✅ Site Console 把只读状态、文件传输和高权限控制放在一个无认证 HTTP 面；
- ✅ 外部 GLIM/SaaS、Mac、Edge 缺统一 deployment/run identity。

### 27. 哪些复杂度产品可能不需要

- 高概率可移除：旧 ICP route transformer、旧 CSV follower、旧 submap builder、未调用通用 mission/evidence abstractions；
- 可逐步收敛：v1–v5 profile 迁移（在现场资产升级后）、safe cmd 中已关闭的旧 obstacle/localization gates、locked/current 同名 package 可见性；
- 不能简单移除：FAST-LIO+VGICP 两层定位、Route+Path、global+local costmap、planner/controller/最终停车多层安全。

### 28. 推荐目标架构

保持当前基本算法分层，但收紧事实与所有权：

```text
Versioned Sensor/Calibration Boundary
→ FAST-LIO odom authority（唯一 TF broadcaster）
→ FixedMapLocalizer map authority（sealed profile + observable recovery）
→ MissionRuntime（Route/checkpoint 唯一状态所有者）
→ Nav2 Planner/Controller
→ Unified SafetyProof（footprint readiness + measured stop + auth freshness）
→ Unitree Motion Adapter

所有边界共同引用 immutable FieldRunManifest
```

管理面拆成 authenticated control API 与 read-only/large-artifact surface；GLIM/SaaS 暴露 deployment ID；effective config 在启动时生成一个规范化只读快照。

### 29. 推荐重构顺序

1. **先补证据，不改算法**：保存 costmap layer/footprint start check、TF authority、Unitree measured stop、effective config 和 FieldRunManifest；
2. 修 r14 bounded terminal 与 health/Smac footprint 定义一致性；
3. 给 Site Console 管理接口加显式认证/网络信任合同；
4. 统一 candidate/active readiness，明确每层 safety owner；
5. 建立 GLIM/SaaS/map/release receipt transaction；
6. 现场 A/B 基准后才评估 VGICP/MPPI 算法替换或简化；
7. 最后删除/归档旧 follower、ICP relocalizer、submap builder、未调用 abstractions，并收敛 profile migrations。

每一步都应先有回归/现场验收标准；本次任务只完成 READ/UNDERSTAND/DOCUMENT/EXPLAIN，没有执行上述重构。
