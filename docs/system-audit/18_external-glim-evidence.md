# 18 外部 GLIM 实现与运行证据

## 结论

✅ GLIM 的当前职责、实际输入输出、精确 commit、有效配置哈希、加载插件和一次 production-like map job 已由本地保存的云端快照确认。

❓ GLIM 与 glim_ros2 的完整源码树、云 worker 镜像 digest 和依赖 lock 不在当前产品仓库，不能宣称单靠本仓库可重建 worker。

## 真实运行位置与入口

- cloud host receipt：Ubuntu 24.04 x86_64，ROS Jazzy；
- adapter：`deployment/cloud/gogoguard-map-job`；
- 外部入口：`/opt/go2/gogoguard_dog_go2_real_code/deploy/cloud_mapping_worker/run_pipeline.sh`；
- 配置：`/opt/go2/glim/jazzy-config`；
- 作业身份：`/opt/go2/jobs/map-<12 hex>`，以 `go2mapping` 用户运行；
- 流程：prepare-session → run-session → create-build → produce-review-artifacts → 本仓库 exporter。

## 版本和配置

来自 `go2_config_manifest.json`：

- GLIM commit：`aafbbff547bef21e48234ed3f810729552c2aa80`；
- glim_ros2 commit：`4dccdaa4a1b27a3ab324981336a8fdb4ea67f1b4`；
- effective config file-set hash：`e5dfdd0a62f22da7c07f1f1b22d2f7d1e65a47574118b011ccee7fd63dd50325`；
- compute profile：CPU；
- LiDAR Topic：`/mapping/livox/lidar` PointCloud2；
- IMU Topic：`/mapping/livox/imu` Imu；
- MID360 内部标定 ID 与 hash 已封存。

## 实际加载的插件与算法

运行日志确认加载：

- `libodometry_estimation_cpu.so`；
- `libsub_mapping.so`；
- `libglobal_mapping.so`。

有效 CPU 配置确认：

- odometry registration：GICP，最多 8 iterations；
- sub mapping registration error factor：VGICP；
- global between registration：GICP；
- global registration error factor：VGICP；
- global optimization 开启，implicit loop 最大距离 100m、最低 overlap 0.2。

这说明“GLIM 和 VGICP 的关系”不是一个模糊判断：当前云端 GLIM pipeline 内多个阶段确实使用 GICP/VGICP；巡检时的 ContinuousMapLocalizer 又独立使用 small_gicp VGICP。两者职责与运行位置不同。

## 实际作业证据

`map-dc7036e35379`：

- duration 207.066s；
- 2085 optimized poses；
- 21 submaps；
- trajectory distance 90.897m；
- maximum pose step 0.123m；
- status accepted；
- 输出 GLIM dump、trajectory、map.ply、glim-build.json 和 overview。

日志还记录两个相邻 submap overlap 偏低（约 0.205、0.162），global mapping 插入 between factor 防止孤立；CPU config 缺三个较新 voxel 参数时使用了日志显示的默认值。这些 warning 不是失败，但属于有效配置治理证据。

## 与 FAST-LIO、巡检 VGICP 和地图的关系

- FAST-LIO：狗端实时连续相对运动；
- GLIM：云端离线整段轨迹/子图/全局地图优化；
- GLIM PLY/dump：建图评审和产品化源；
- localization PCD：从建图产物经一次性 frame 转换、裁剪/审核产生；
- ContinuousMapLocalizer VGICP：巡检时把当前扫描定位到 PCD，GLIM 此时不运行。

## 推荐治理

map job receipt 应再绑定 worker image digest、GLIM/glim_ros2 source archive URI+hash、系统依赖 lock。精确 commit/config 解决“当时跑了哪套逻辑”的大部分问题，但仍不能替代可获取的安装产物。
