# 10 当前算法：作用、边界与代码证据

> 只把“被当前启动链加载并参与现场运行”的实现称为当前算法。仓库里可编译、可安装或可测试，不等于现场正在使用。

## 总表

| 环节 | 当前选择 | 状态 | 核心输出 | 不负责什么 |
|---|---|---|---|---|
| 实时相对运动估计 | FAST-LIO | ✅ | `odom→base_link`、`/Odometry`、去畸变点云 | 不知道历史地图绝对位置 |
| 离线建图 | 外部 GLIM pipeline | ✅ commit/config/plugins/run/output 已确认；源码树不在本仓库 | 优化轨迹、submap dump、三维地图 PLY | 不在巡检时实时运行 |
| 固定地图定位/重定位 | small_gicp `VGICP` | ✅ | `map→odom`、`/localization/pose`、质量状态 | 不规划路线、不发运动命令 |
| 全局规划 | Nav2 `SmacPlanner2D` | ✅ | 当前 Pose 到目标的 `Path` | 不直接控制 Unitree |
| 局部跟踪 | Nav2 MPPI，Omni 模型 | ✅ | `/nav2/raw_cmd_vel` | 不给出绝对 Pose |
| 障碍/禁区表达 | 自体点云过滤 + Costmap layers | ✅ | 障碍栅格、禁区和膨胀代价 | 不是一个单独“避障算法节点” |
| 最终碰撞停止 | Collision Monitor polygon stop | ✅ | `/patrol_cmd`，危险时零速 | 不替代全局绕行 |

当前没有证据表明固定地图定位存在运行中的 ICP、PCL GICP 或 NDT fallback。它们只出现在不被当前 launch 启动的旧节点或 benchmark 中。

---

## A. FAST-LIO：实时 LiDAR/IMU 里程计

### L1 产品表现 / L2 系统模块

狗迈步或转向后，系统连续知道“相对刚才移动了多少”，扫描也不会因狗在扫描期间运动而明显拉花。Livox 驱动提供点云与 IMU；FAST-LIO 融合二者并维护实时局部地图。它是连续运动估计底座，不是历史地图定位器。

### 1. 它解决什么现实问题？

MID-360 一帧点不是同一瞬间拍下，而是在一段时间逐点扫出。狗在扫描时也会移动；直接叠加会形变。系统还需要比固定地图配准更高频、更连续的短期运动估计。

### 2. 输入是什么？

- `/mapping/livox/lidar`：MID-360 原始 Livox 点云；
- `/mapping/livox/imu`：角速度与线加速度；
- LiDAR–IMU 外参与噪声参数。

### 3. 它大概怎么做？

IMU 像更新很快但会漂移的“惯性草稿”；LiDAR 像稍慢但能看到墙地几何的“校对”。FAST-LIO 用 IMU 预测扫描期间姿态并去畸变，再用当前点与增量局部几何的误差修正状态。代码是 ESKF 式传播/更新和增量地图，不是每帧去比历史 PCD。

### 4. 输出是什么？

- `/Odometry`：`odom` 中 `base_link` 的 6DoF Pose/运动；
- TF `odom→base_link`；
- `/navigation/cloud_lidar`：导航链使用的点云；
- FAST-LIO 内部增量局部地图。

这里的 X/Y/Z/Roll/Pitch/Yaw 是相对 `odom` 的姿态，不是最终 `map` 绝对坐标。

### 5. 为什么能工作？

相邻时刻仍有稳定几何、时间同步和安装外参正确时，IMU 的快速动态与 LiDAR 的几何约束互补。

### 6. 何时失败？

空旷/单墙/长走廊等约束退化；大量人车动态点；IMU 振动、饱和、错时；雷达遮挡、有效点减少；外参错误；长程累计漂移。FAST-LIO 不会自己把漂移挂回历史地图。

### 7. 为什么当前产品用它？

高频连续运动估计和去畸变对行走控制确有必要。代价是标定、同步、健康诊断与漂移管理更复杂。

### 8. 代码证据（L4）

- `services/edge/edge-entrypoint.sh` 启动 Livox、mount TF、cloud transformer、FAST-LIO；
- `third_party/locked_stack/src/FAST_LIO/src/laserMapping.cpp`，`publish_odometry()`；
- `third_party/locked_stack/src/go2_map_manager/src/calibrated_cloud_transformer.cpp`；
- 下游 localizer 订阅 `/Odometry` 与 `/navigation/cloud_body`。

---

## B. VGICP：固定三维地图定位与重定位

### L1 产品表现 / L2 系统模块

重新开始任务时，狗不只知道“开机后走了多少”，还知道自己在历史巡检地图中的位置。`continuous_map_localizer` 把当前点云与固定 PCD 局部块做 6DoF 配准，输出 `map→odom`；最终 `map→base_link = map→odom × odom→base_link`。

### 1. 它解决什么现实问题？

FAST-LIO 的 `odom` 原点每次可不同且会漂移。定位器要把今天看到的墙、地面和建筑与建图时保存的三维环境对上，把相对轨迹挂回稳定 `map`。

### 2. 输入是什么？

- 当前 `/navigation/cloud_body`；启动累积至少 2 秒、8 帧、500 点；
- 固定 PCD 在预测位置附近裁出的 target；
- `/Odometry`；
- 启动区域/期望 yaw，或上一次可信 Pose；
- 搜索、质量门限和恢复参数。

r5 启动中心约 `(-0.2176,-0.0411)`、半径 `5m`，期望 yaw 约 `10°`、容差 `60°`。它不是全地图任意姿态的无先验搜索。

### 3. 它大概怎么做？

把历史地图看作一团透明点，当前扫描是另一团较小透明点。算法平移/旋转当前点，使两团点的局部表面分布最一致。启动时按 `1m` XY、`15°` yaw 产生候选，粗配准保留前 8 个再精配准；跟踪时以上次结果为初值，只接受连续小修正。VGICP 利用点邻域分布/协方差表达表面方向，不是识别“同名物体”。

### 4. 输出是什么？

- 6DoF `map→odom`；
- `/localization/pose` 组合出的 `map→base_link`；
- `/localization/status`：fitness、inlier ratio、condition、confidence、新鲜度和 TRACKING/DEGRADED/LOST。

### 5. 为什么能工作？

不要求每个点都不变，只需仍有足够稳定、独特的几何形成明显更优对齐。因此垃圾桶消失只损失部分对应；墙、道路边缘和建筑仍在时仍可成功。当前要求至少 250 点、inlier ≥0.20、fitness MSE ≤0.20，并检查可观性和候选歧义。

### 6. 何时失败？失败怎样处理？

初始位置超出窗口；重复围墙/路灯出现多个同样好的解；大部分稳定结构改变/被遮挡；只有平地或单墙；FAST-LIO/时间同步先错；PCD 坐标或外参错；输入超过 `0.35s`；修正超过 `0.75m/15°`。

2 个坏样本进入 DEGRADED，累计 5 个进入 LOST。恢复围绕上次可信 Pose 做 `4m/45°` 搜索，需要 3 个结果在 `0.3m/6°` 内一致，再经连续好结果回 TRACKING；恢复时暂停 TF。重复场景的错误好匹配仍可能发生，歧义与连续性检查只能降低、不能消除风险。

### 7. 为什么当前产品用它？

已有高质量 3D 地图，户外道路有三维结构，VGICP 能利用它们校正 FAST-LIO。它比只用 2D 定位复杂，但能利用非平面几何；当前没有证据证明普通 ICP/GICP 更稳。

### 8. 代码证据（L4）

- `third_party/locked_stack/src/go2_map_manager/src/continuous_map_localizer.cpp`，`ContinuousMapLocalizer`，强制 `registration.type == "VGICP"`；
- `modules/localization/ros/patches/go2-vgicp-orin-v2.patch`；
- `third_party/locked_stack/src/go2_map_manager/config/continuous_map_localizer.yaml`；
- `third_party/locked_stack/src/go2_site_ops/go2_site_ops/localization_quality.py`；
- `modules/navigation/ros/go2_nav2_runtime/launch/active_map_patrol.launch.py`。

基础 YAML 仍是 v1；完整镜像构建会应用 v2 patch。因此静态 YAML 值不等于镜像最终值。

---

## C. GLIM：离线三维地图与全局轨迹优化

### L1 产品表现 / L2 系统模块

建图录制上传后，Mac 得到整体连贯的三维地图与优化轨迹。狗端录制/封存；Mac 经受限 SSH/rsync 调云端 GLIM pipeline，再由本仓库 exporter/workspace 转为产品资产。

### 1. 它解决什么现实问题？

实时里程计会积累漂移；绕圈回到旧位置时轨迹两端可能不闭合。离线处理能用整段数据全局优化，产生更一致的历史地图。

### 2. 输入是什么？

封存 rosbag（LiDAR、IMU、`/Odometry`）、录制 manifest/时间戳/哈希及外部 pipeline 配置。

### 3. 它大概怎么做？

当前 CPU profile 的真实过程已由有效配置和运行日志确认：`libodometry_estimation_cpu.so` 使用 GICP 连续估计；`libsub_mapping.so` 的 registration error factor 使用 VGICP；`libglobal_mapping.so` 启用优化、GICP between registration 和 VGICP registration error，并以 overlap/distance 产生隐式重叠约束。仍不能超出配置/日志去推断外部源码未展示的内部细节。

### 4. 输出是什么？

优化 trajectory、三维 PLY/GLIM dump/build metadata；之后派生固定 PCD、浏览 map.json、静态 PGM、route 和 checkpoint。

### 5. 为什么能工作？

✅ 当前有效配置确认它利用 IMU、相邻关系、submap registration error 和隐式重叠约束优化整段；稳定结构被重复观察时，约束会把累计误差分摊到轨迹图。具体数值见 `18_external-glim-evidence.md`。

### 6. 何时失败？

录制缺包/错时、重叠不足、动态环境、退化轨迹、外参错、云端版本/配置漂移。exporter 的时间戳前缀补齐和 RMS/max/join 门限只能保护转换，不能修复错误 GLIM 优化。

### 7. 为什么当前产品用它？

离线处理不受 Orin 实时预算限制，适合生成长期定位地图。最大治理风险是外部实现未与产品仓库形成完整可复现证据链。

### 8. 代码证据（L4）

- `modules/mapping/gogoguard_mapping/capture_manager.py`；
- `modules/map_factory/gogoguard_map_factory/robot_client.py` 与 cloud transfer/command；
- `third_party/locked_stack/src/go2_site_ops/go2_site_ops/map_store.py`；
- `modules/route/gogoguard_route/workspace.py`；
- ✅ GLIM commit `aafbbff…`、glim_ros2 commit `4dccdaa…`、配置 hash `e5dfdd…0325` 与 CPU 插件已由 session 快照确认；
- ❓ 外部源码树、worker image digest 和依赖 lock 未进入当前产品仓库。

---

## D. SmacPlanner2D：从当前位置到下一目标的全局 Path

### L1 产品表现 / L2 系统模块

系统在允许区域内寻找从当前位置到下一目标的可行路径，而非机械重放录制脚印。PatrolRuntime 由 Route 进度选择目标，Nav2 `ComputePathToPose` 调用 Smac 输出 Path。

### 1. 它解决什么现实问题？

从 A 到 B 不能只画直线；墙、禁区和障碍使一些格子不可走。规划器要在二维代价网格中搜索连通低代价路线。

### 2. 输入是什么？

当前 `map` Pose、目标 Pose、global costmap（static + obstacle + inflation + keepout）、footprint 与 planner 参数。

### 3. 它大概怎么做？

把地图看作有不同通行代价的方格纸：墙/禁区不可走，靠近障碍更贵。搜索从起点到目标的低代价连通路径。完整 footprint 会影响起点/路径是否合法，只看中心格不够。

### 4. 输出是什么？

`nav_msgs/Path`，交给 MPPI。失败返回 planner result；当前 PatrolRuntime 约每 `0.75s` 重试/恢复。

### 5. 为什么能工作？

起点、目标 footprint 不在 lethal/unknown，costmap 连通，分辨率与 inflation 能表达实路宽度。

### 6. 何时失败？

起点中心可通行但 footprint 压到 lethal；目标在墙/禁区/孤岛；膨胀封死走廊；Pose 偏移几十厘米；静态图或 keepout 与现场不一致。

r14 反复报 `Starting point in lethal space!`。离线复算表明现场 Pose footprint 覆盖 1 个静态黑格，而录制起点为 0；静态图/定位偏移是高概率原因，但无 layer dump，不能排除实时 obstacle/inflation。

### 7. 为什么当前产品用它？

Route 不能处理实时障碍和起步偏移，全局 planner 有必要。当前首要缺陷是 footprint 健康检查和失败终止逻辑，而非已证明 Smac 本身不合适。

### 8. 代码证据（L4）

- `modules/navigation/ros/go2_nav2_runtime/config/go2_nav2_patrol.yaml`；
- `modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/nav2_profile_contract.py`；
- `modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/patrol_runtime_manager.py`；
- candidate nav2 PGM/keepout；r14 incident 日志、status 与本审计 footprint 复算。

---

## E. MPPI：跟踪 Path 并持续纠偏

### L1 产品表现 / L2 系统模块

狗移动中不断根据最新 Pose、Path 和近处障碍调整前进、横移和转向。Nav2 Controller Server 使用 `MPPIController` + Omni，15Hz 输出 `/nav2/raw_cmd_vel`，后面仍有 smoother、collision monitor、授权与安全桥。

### 1. 它解决什么现实问题？

真实运动有误差，Path 会弯曲/贴障碍。控制器需权衡贴合路径、朝向、碰撞和运动限制，持续选择下一速度。

### 2. 输入是什么？

当前 Pose、Smac Path、local costmap、Omni 模型、速度/加速度上限、采样数、预测步数和 critics。

### 3. 它大概怎么做？

控制器快速“试跑”许多未来速度序列，为偏离、障碍、转向等代价打分，再从较好样本更新当前命令；执行一点后重新观察、滚动预测。

### 4. 输出是什么？

15Hz `Twist`。r14 有效值：forward target/max `0.6/0.9m/s`、lateral `0.2m/s`、yaw `0.4rad/s`、56 steps、`dt=1/15s`、batch 1000、iteration 1，窗口约 `3.73s`。

### 5. 为什么能工作？

Pose/costmap 新鲜、Path 可行、狗短时间内符合 Omni 模型、速度约束符合实机能力。

### 6. 何时失败？

局部空间 lethal、Pose 跳变、costmap 延迟/过紧、低速死区和实机不匹配、计算超 15Hz 截止，或下游授权/限幅使“控制器想走但狗不走”。

### 7. 为什么当前产品用它？

Go2 可前进、横移、转向，Omni MPPI 能结合障碍统一纠偏。复杂度是否过量应由现场可行率、CPU 截止和简单控制器对照实验决定。

### 8. 代码证据（L4）

- `modules/navigation/ros/go2_nav2_runtime/config/go2_nav2_patrol.yaml`；
- `modules/navigation/gogoguard_navigation/profiles.py`；
- `modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/nav2_profile_contract.py`；
- r14 incident `parameters.json`/diagnostics；
- 下游 velocity smoother → collision monitor → safe cmd → UDP sender。

---

## F. Costmap、点云过滤与 Collision Monitor

### L1 产品表现 / L2 系统模块

狗可绕开规划层障碍；若近身安全区出现足够多点，即使上游非零命令也强制停车。`ObstacleCloudFilter` 去自体点；global/local costmap 叠加障碍、膨胀、禁区；Collision Monitor 在出口做 polygon stop。

### 1. 它解决什么现实问题？

三维点云不能直接等于“能否通行”：会看到狗身、地面、远处点。系统需投影为导航二维代价，并保留独立近身停车层。

### 2. 输入是什么？

body-frame 点云、静态 PGM、keepout、footprint、高度/障碍/膨胀参数、smoother 速度。

### 3. 它大概怎么做？

先去自体和无效点，把有效高度点投到栅格；障碍格 lethal，周围按距离增加代价。Collision Monitor 画随狗移动的多边形，点数超过门限就输出零速。

### 4. 输出是什么？

global/local costmap，以及危险时归零的 `/patrol_cmd`。

### 5. 为什么能工作？

点云 frame/过滤正确、二维投影适合地面巡检、footprint 符合实物。当前 padding 后 footprint 约 `x[-0.43,0.50], y±0.30m`，inflation `0.45m`。

### 6. 何时失败？

自体/地面幽灵点；低矮或悬空障碍超出高度；玻璃雨雾漏检；中心格健康却 footprint lethal；unknown 与 occupied 在 PGM 表达混淆；点密度变化使停车点数门限失衡。

### 7. 为什么当前产品用它？

绕行与近身停车职责不同，分层合理。当前认知裂缝是 diagnostics 只检查中心单格，Smac 检查完整 footprint。

### 8. 代码证据（L4）

- `modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/obstacle_cloud_filter.py`；
- `modules/navigation/ros/go2_nav2_runtime/config/go2_nav2_patrol.yaml`；
- `modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/nav2_profile_contract.py`；
- `modules/navigation/ros/go2_nav2_runtime/launch/active_map_patrol.launch.py`；
- Humble Collision Monitor `max_points=8` 语义为超过门限触发 stop。

---

## G. 存在但不属于当前主链的算法

| 实现 | 做什么 | 状态 | 证据结论 |
|---|---|---|---|
| `route_relocalizer.cpp` PCL ICP | 当前 odom cloud 与固定 map 做全局 XY/yaw ICP，再把 CSV route 变换到当前 odom | ⚪ / 🧹 | 可安装但当前 launch 不启动；与 current `map→odom` 方案不同 |
| `localization_registration_benchmark.cpp` | 离线比较 NDT、PCL GICP、small_gicp VGICP | ⚪ 测试 | benchmark 出现不等于运行启用 |
| `submap_builder.cpp` | `/cloud_registered` + `/Odometry` 累积/保存 PCD | ⚪ | 可安装不启动；当前狗端录 bag、云端 GLIM |
| `waypoint_follower_go2_2.py` | CSV 最近/前视点 + 简单 P/yaw，直接发 `/patrol_cmd` | 🧹 | 当前 Smac + MPPI；launch 不启动 |
| `patrol_control.py` / `controller_core.py` | 旧直线/转角或跟踪辅助逻辑 | 🧹 | 不在当前动态 launch 调用链 |

## 仍需补证

- ❓ GLIM 外部源码 archive、worker image digest 与依赖 lock；commit、环境、插件和有效参数已确认；
- ❓ 现场 TF graph 是否同时存在两个 `odom→base_link` broadcaster；
- ❓ r14 start-lethal 中 static、obstacle、inflation 各层实际贡献；
- ❓ 各场地重复结构、大启动误差和大规模变化下的 VGICP 真实成功边界。
