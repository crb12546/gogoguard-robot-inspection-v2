# 00 系统资产清单

## 审计快照

| 项目 | 状态 | 当前事实 | 证据 |
|---|---|---|---|
| 活动产品仓库 | ✅ | `gogoguard_robot_inspection_v2_field` | 外层 `AGENTS.md`、仓库 `AGENTS.md` |
| Git 跟踪文件 | ✅ | 691 个 | `git ls-files`，2026-08-25 本地审计 |
| 生成索引可见文件 | ✅ | Git 基线为 691 个；`rg --files --hidden -g '!.git'` 与索引一致。普通 `rg --files` 少 8 个隐藏文件 | Git index 与本地文件清单 |
| 模块 manifest | ✅ | 16 个 | `architecture/modules/*.json` |
| ROS package | ✅ | 源码树有 10 个 `package.xml`、9 个唯一 package name（`go2_nav2_runtime` 有 locked/module-owned 两份）；完整镜像实际选择 module-owned Nav2，不构建 `go2_mapping_capture` | `package.xml`、两个 Dockerfile |
| 忽略运行资产 | ✅ | `runtime-data` 约 4.6 GB，`workstation-data` 约 11 GB，`release-cache` 约 74 GB | `du -sh`；`.gitignore` |
| 子模块 | ✅ | 无 Git submodule entry | `git submodule status`、索引 mode 检查 |
| Git LFS | ✅ | 当前无 LFS 跟踪对象 | `git lfs ls-files` |

## 顶层职责（待代码验证）

| 路径 | 初步职责 | 当前状态 |
|---|---|---|
| `architecture/modules` | 机器可读模块所有权、入口、依赖和状态 | 🟡 16 份 manifest 已读；部分 status/入口滞后于 gen10 主链，不可单独作为 runtime 证据 |
| `modules` | 核心产品责任模块 | ✅ contracts/calibration/device/capture/transfer/route/navigation/evidence/interaction 等边界已盘点 |
| `services` | 云建图、实时交互、平台边缘运行适配 | ✅ map_factory、interaction_edge、platform_edge 入口已确认 |
| `apps` | Site Console 与 Mac 工作站组合 | ✅ robot HTTP surface 与 Mac workflow 入口已确认 |
| `deployment` | cloud/container/robot/workstation 启动与发布 | ✅ 入口、发布身份、安装/激活分离已读 |
| `config` | 无密钥产品/现场配置 | ✅ 仓库默认已读；机器人密钥/实际 runtime env 未读且不应进入审计文档 |
| `third_party/locked_stack` | 冻结的 ROS/硬件能力和完整镜像构建来源，不等同于另一套当前应用 | ✅ 完整 Dockerfile 复制 locked FAST-LIO、map manager、UDP sender 等，再用 V2-owned 源/补丁覆盖特定组件；combined 镜像继承冻结 native 层 |
| `tests` | 单元与纵向组合回归 | 🟡 |
| `runtime-data`、`workstation-data`、`release-cache` | 被忽略的现场证据、地图/录制和发布缓存 | ✅ 路径/规模已确认；资产目录待只读盘点 |

## 当前镜像的源码选择

| 运行能力 | 构建来源 | 状态 |
|---|---|---|
| FAST-LIO | `third_party/locked_stack/src/FAST_LIO` + `config/robot/fastlio_base_output.yaml` | ✅ 完整 Dockerfile；combined 继承冻结镜像 native 层 |
| 连续固定地图定位 | locked `go2_map_manager` + `modules/localization/patches/go2-vgicp-orin-v2.patch` | ✅ 完整 Dockerfile；combined 继承冻结镜像 native 层 |
| Nav2 runtime 包 | `modules/navigation/ros/go2_nav2_runtime` | ✅ 两个 Dockerfile 都明确复制当前模块；combined 会重装此 Python 包 |
| SDK2 UDP receiver / VUI | receiver 源和 CMake 由 `modules/device_io/ros/...` 覆盖 locked 包；UDP sender 保留 locked 源 | ✅ Dockerfile 与 CMake |
| Edge Python 能力 | `modules/*/python`、`services/*/python`、`apps/*/python` | ✅ 两个 Dockerfile 均复制；entrypoint 决定实际启动集合 |

## ROS package 与当前选择

| package | 镜像中的来源/职责 | 当前主链 |
|---|---|---|
| `livox_ros_driver2` | 完整镜像构建时生成/编译；MID360S 驱动 | ✅ 常驻 |
| `fast_lio` | locked FAST-LIO + 当前 base-output YAML | ✅ 常驻 |
| `unitree_go` / `unitree_api` | Unitree ROS message/API | ✅ 设备边界 |
| `go2_site_ops` | map store、质量模型、mount TF/工具 | ✅ 其中 mount/runtime 工具被用 |
| `go2_map_manager` | cloud transformer、patched continuous localizer；另安装旧工具 | ✅ transformer/localizer；其余待参见 legacy |
| `go2_cmd_vel_bridge` | 当前 owned SDK receiver/VUI + locked UDP sender | ✅ 运动链 |
| `go2_nav2_interfaces` | `StartPatrol` service contract | ✅ |
| `go2_fastlio_patrol` | 安装 safe cmd；同包中 route recorder/老 waypoint follower 不在 launch | ✅/⚪ 混合 |
| `go2_nav2_runtime` | module-owned gen10 launch/runtime/YAML；locked 同名包不入镜像 | ✅ |
| `go2_mapping_capture` | locked 录制/标定 launch | ⚪ 未被当前 Dockerfile 构建；当前 CaptureManager 直接管理 rosbag |

## 当前确认启动的 ROS 节点组

- 常驻感知：Livox driver、mount TF publisher、`calibrated_cloud_transformer`、FAST-LIO、battery observer、navigation observer、incident recorder。
- 动态 gen10：obstacle filter、performance monitor、trace recorder、continuous localizer、2 map servers、filter info server、planner/controller/behavior server、velocity smoother、collision monitor、lifecycle manager、patrol runtime、safe cmd、UDP sender，外加独立 SDK receiver。
- ⚪ `submap_builder`、`route_relocalizer`、`localization_registration_benchmark`、`route_recorder`、`waypoint_follower_go2_2`、locked `go2_nav2_runtime`、`go2_mapping_capture` launch 都不在当前启动树。

## 忽略但属于现场事实的资产

- ✅ `runtime-data`：约 6,240 个文件条目，含 20 个 map job、18 个 recording、404 个 incident 目录。
- ✅ `workstation-data`：约 3,894 个文件条目，含 20 个平台资产 revision 目录。
- ✅ `release-cache`：约 599 个文件条目、60 个发布缓存目录。
- ✅ 20 个 map job 中 19 个 complete、1 个 failed；当前选中 `map-0d2b63d9da78` 对应作业完整且有固定地图、导航图和 route revision。
- ⚠️ 这些内容未被 Git 跟踪，后续只能通过 manifest/hash/回执建立版本身份；审计不读取密钥文件内容。

Topic/Service/Action 的主链结构化全表将放入 `system-model`，以便 HTML 直接消费。
