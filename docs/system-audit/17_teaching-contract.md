# 17 第二版教学契约

## 结论

✅ 第二版 Guide 不再按仓库模块或算法名组织主线。主线采用“现实问题 → 直觉 → 技术名 → 当前算法 → 实现证据 → 失败条件”，并执行 Zero Undefined Prerequisites。

## 为什么第一版不足

第一版的事实和证据链可以保留，但首次阅读过早出现 FAST-LIO、VGICP、odom、TF、base_link、MPPI。读者能看到正确名词，却必须同时猜测这些词的现实含义，产生概念债务。继续增加 Tooltip 不能修复错误的概念顺序。

## 第二版约束

1. 每一步最多引入 1–2 个新技术概念；`tools/validate_system_guide.py` 自动检查。
2. 定位主课分十步：Pose → IMU → LiDAR/PointCloud → Registration → Drift → FAST-LIO → 历史地图 → PCD/VGICP → map/odom → base_link/TF。
3. 导航主课从 Route/Checkpoint 开始，再引入 Costmap/Path、Smac、MPPI、Twist、Footprint 和 Unitree 最终执行。
4. 每个图先说明现实问题和它在当前链路的位置。
5. 失败条件只能在成功机制之后出现。
6. L1/L2 默认可读；L3/L4 按需展开，代码不能代替解释。
7. 页面所有事实来自 `system-model/*.json`；动画只表达直觉，不作为运行证据。

## 验收问题

- 不知道 FAST-LIO/VGICP/TF 的读者能否解释机器人为什么知道位置？
- 读者能否解释偏离 Path 后为什么会产生纠偏速度？
- 读者能否区分当前 ContinuousMapLocalizer 和未启动的 PCL ICP route_relocalizer？

当前自动校验只能验证课程结构、术语预算和事实路径，不能替代真实目标读者的理解测试。
