# 03 定位链路

## 已确认主链

```text
MID-360 点云 / IMU
→ Livox ROS driver
  → /mapping/livox/lidar  (sensor_msgs/PointCloud2, lidar_link)
  → /mapping/livox/imu    (sensor_msgs/Imu)
→ FAST-LIO
  → IMU 传播 + 点云去畸变 + 当前扫描到局部增量地图匹配
  → /Odometry             (nav_msgs/Odometry, odom → base_link)
  → TF odom → base_link
  → /navigation/cloud_lidar（去畸变单帧，lidar_link）
→ calibrated_cloud_transformer
  → /navigation/cloud_body（同一单帧变到 base_link）
→ continuous_map_localizer
  → 当前单帧/短时累积与固定 GLIM PCD 局部块做 small_gicp VGICP
  → TF map → odom
  → /localization/pose     (PoseWithCovarianceStamped, map → base_link)
  → /localization/status、/usable、/confidence
→ Nav2 使用 map frame 当前位姿
```

状态：✅ 节点、Topic、Message、frame、算法和输出已由 entrypoint、FAST-LIO 完整源码、点云转换器、补丁后的 localizer 源和 launch 交叉确认。

## 六自由度 Pose 的来源

| 分量 | 直接来源 | 固定地图如何介入 | 状态 |
|---|---|---|---|
| x/y/z | FAST-LIO 的 LiDAR-IMU ESKF 在 `odom` 中积分/扫描匹配；再左乘 localizer 求得的 `map→odom` | VGICP 输出当前 `map→base` 候选，并换算/平滑 `map→odom` | ✅ |
| roll/pitch | IMU 重力方向和角速度传播为强约束，LiDAR 平面残差共同修正；再由固定地图校正整体姿态 | 同上 | ✅ |
| yaw | IMU 角速度传播 + LiDAR 几何匹配；绝对地图朝向由 VGICP 锚定 | 同上 | ✅ |

Unitree 本体状态不参与几何定位。当前链路使用 Unitree 电量/运动执行等状态，但没有把腿式里程计或本体姿态融合进这套 `map` Pose。状态：✅。

## FAST-LIO 在本系统里解决什么

- 输入：MID-360 每帧点云、IMU、LiDAR↔IMU↔base 外参。
- 过程：初始化重力与陀螺零偏；用 IMU 在帧间传播状态并给一帧内各点去畸变；把当前扫描和持续增长的局部点云地图做迭代平面匹配，再修正 ESKF。
- 输出：平滑连续但只相对起点成立的 `odom→base_link`，以及去畸变单帧。
- 它不会回答“在历史巡检地图哪里”；那由下一层固定地图 localizer 完成。

## 固定地图重定位/连续校正

- 历史数据：候选 bundle 中固定 GLIM 地图 PCD，加载后体素降采样，运行时围绕预测位置取半径 32 m 的局部 target。
- 当前数据：`/navigation/cloud_body` 的去畸变单帧；启动时累积至少 2 s、8 帧和 500 点形成 source。
- 初始搜索：在候选声明的中心/半径/朝向容差内产生 XY/Yaw seed，先粗筛，再对至多 8 个相互区分的候选做精配准并检测歧义。
- 注册实现：small_gicp `RegistrationPCL`，`registration_type=VGICP`。比较的是当前 source 点与历史 target 邻域的空间分布，不是比较图像、GPS 或 waypoint。
- 输出：一个 6DoF `map→base_link` 候选；结合同时刻 `odom→base_link` 得到 `map→odom`。
- 运行更新：2.5 Hz；可信时按 alpha 0.35 平滑，每次最多接受约 0.15 m / 2° 更新。

## 接受、歧义和恢复

- ✅ 基本门限：source 至少 250 点、inlier ratio ≥ 0.20、fitness MSE ≤ 0.20、最小 Hessian 特征值 ≥ 1e-6、condition ≤ 1e8、数据年龄 ≤ 0.35 s。
- ✅ 已跟踪/恢复候选相对可信锚点的单次 correction jump 必须 ≤ 0.75 m / 15°。
- ✅ 业务 confidence 由 inlier 45%、fitness 40%、condition 15% 组合；它不是库原生概率。
- ✅ 初次搜索要排除接近的竞争解；若最佳/次佳不够可分，拒绝初始化。
- ✅ 状态机：初始化后仍需 2 个 good sample 才 `TRACKING`；跟踪连续 2 个 bad 进 `DEGRADED`，再 5 个 bad 进 `LOST`；恢复要求 3 个相互一致候选，之后还需 2 个 good 才重新 `TRACKING`。
- ✅ 恢复期间保留最后可信锚点，但暂停发布 `map→odom`，避免把未经确认的跳变送给导航。

## 当前未闭合问题

- ⚠️ FAST-LIO 与 localizer 都可能发布 `odom→base_link`；按 launch 配置 localizer 的候选 runtime 参数为 true，而 FAST-LIO 源码也始终广播。两者内容预期来自同一 `/Odometry`，但多发布者是否必要、运行时是否造成 TF authority 警告仍需现场日志确认。
- ❓ 实际环境变化的经验失效边界没有统计数据；代码只能给出门限和机制，不能凭行业常识伪造现场成功率。

## 待补证据

- 把全部参数来源、最终有效值和 launch override 写入 `11_parameters.md`。
- 用真实 PCD/route/candidate manifest 验证地图 frame 和 seed zone。
- 将环境变化、重复结构、初值偏差做成交互式 guide 场景，同时明确区分代码事实与原理推演。
