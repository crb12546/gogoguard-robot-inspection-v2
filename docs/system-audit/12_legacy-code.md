# 12 历史遗留与技术债务

当前阶段不删除任何内容。

| 模块/资产 | 当前实现 | 当前使用 | 历史/重复候选 | 风险 | 证据 | 建议 |
|---|---|---|---|---|---|---|
| 两个 Dockerfile | 完整从 locked source 重建 native ROS 栈 + 基于冻结 V6 镜像的增量 combined | ✅ 当前现场 r14 使用 combined；两者职责不同 | 外观像重复，不是同一发布成本 | 修改 native 源后若仍构建 combined，变更不会进入镜像 | Dockerfile、combined Dockerfile、发布回执 | 保留并在发布工具中明确 native/pure-Python 变更边界 |
| `third_party/locked_stack` 中 navigation 与 `modules/navigation/ros` | frozen 原包 + 当前 module-owned runtime 包 | ✅ 运行选择 module-owned nav package | 同名包仍易误读 | 工程师可能修改 locked 同名文件却看不到效果 | 两个 Dockerfile COPY/colcon 选择 | 在生成索引中显示 selected source |
| `odom→base_link` TF 发布 | FAST-LIO 广播；localizer 也可按 launch 参数广播收到的同一 odometry | 🟡 两者在候选 runtime 同时启用的可能性已由代码确认，实际 TF authority 影响待日志确认 | 双发布者 | 警告、时间竞争或未来实现分叉 | FAST-LIO `publish_odometry()`、localizer `odom_callback()`、launch 参数 | 暂不改；核对运行日志和必要性 |
| safe cmd 中点云/定位 gate | 节点保留完整 gate 代码，但当前 launch 两项均 false | ⚪ 当前不承担障碍/定位 gate；runtime auth 仍使用 | 旧安全责任保留在通用节点 | 只读代码易误判为双重安全；配置切换可能改变 stop owner | launch + `unitree_safe_cmd_node.py` | 在架构和参数表明确“能力存在≠当前启用” |
| `active` / `candidate` runtime readiness | 同一 PatrolRuntime 依据 source 切换 Unitree state 与 FAST-LIO health gate | ✅ 当前 manager 为 candidate，两个 gate 均关闭 | active 兼容分支拥有更强 startup/true-stop 证据 | 状态名“true stop”容易让人误以为当前也验证了本体实测速度 | manager launch args、PatrolRuntime `require_*`、r14 status `fastlioHealth.required=false` | 在现场策略中决定是否统一 gate；当前先记录不改 |
| `modules/mission/MissionCoordinator` | 通用纯 mission 状态机 | ⚪ 无 runtime import/call | 与 `CheckpointExecutor`/PatrolRuntime 状态职责重叠 | 读错状态机、未来双实现漂移 | 调用搜索 + 当前 launch | 明确 design/reference 或移除；梳理期不删 |
| `modules/inspection/EvidenceFrameBuffer` | 可持久化图片 evidence buffer | ⚪ 无 runtime import/call | 新 evidence transaction 已在 platform_edge 另实现 receipt/outbox | 让人误以为当前 checkpoint 图片由狗端 buffer 上传 | 调用搜索、entrypoint | 明确未来用途或删除；当前不用于解释巡检 |
| `RobotClient.deploy_map` 注释 | 当前实际准备 generation-10 candidate | ✅ 代码路径使用 gen10 RouteManager | 注释仍写“generation-5 candidate” | 维护者判断部署边界时受误导 | `robot_client.py` 注释 vs `CANDIDATE_GENERATION=10` | 后续文档/注释整理阶段修正 |
| Navigation profile v1–v5 迁移 | 运行时统一规范化为 v6 | ✅ r14 保存 v4，实际使用定向迁移后参数 | 五代 schema 和对特定发布 tuple 的特判 | 操作员看到的文件值与实际 launch 值不同；难以解释/回放 | `profiles.py`、r14 incident `parameters.json`、diagnostics normalized profile | 保留兼容期间要显示 effective profile；后续清点是否仍需老 schema |
| 地图平台 receipt | Mac 保存上传回执 | 🟡 当前文件仅到 r3 verified；robot 现场为 r5 | r4/r5 bundle 存在但上传/激活本地回执缺口 | 无法只靠当前本地 upload state 还原 r5 上线交易 | platform upload JSON、r4/r5 manifests、r14 selected candidate | 将 prepare/upload/verify/activate/robot-select receipt 绑成一条不可变更链 |
| `route_relocalizer.cpp` | 当前 continuous VGICP 发布 `map→odom` | ⚪ 可安装但未被当前 launch 启动 | 旧实现用 PCL ICP 全局网格搜当前 cloud→map，再把 route CSV 变换到 odom | 工程师搜索到 ICP 后会误判为当前重定位；两套坐标责任完全不同 | 完整源文件、CMake install、当前 `active_map_patrol.launch.py` | 标为 legacy/reference；确认无外部脚本依赖后再决定归档 |
| `localization_registration_benchmark.cpp` | 当前运行只允许 small_gicp VGICP | ⚪ 离线 benchmark | 同文件还运行 PCL NDT、PCL GICP | “仓库出现算法名”易被误读为 production fallback | benchmark main、continuous localizer 强制类型、launch | 保留测试用途，但命名/README 明确非 runtime |
| `submap_builder.cpp` | 当前建图录 rosbag，云端 GLIM 输出完整地图 | ⚪ 可安装但当前不启动 | 从 `/cloud_registered` + `/Odometry` 累积并每 10m 保存 PCD | 形成第二套“建图”叙事，默认写 `$HOME/maps` 也不在当前资产治理中 | 完整源文件、CMake、edge/supervisor 启动树 | 标记 prototype/legacy；不要纳入当前地图解释 |
| `waypoint_follower_go2_2.py` | Route 定目标，Smac 规划，MPPI 跟踪 | 🧹 旧 follower 未启动 | 直接用 `/Odometry`、CSV 最近/前视点与简单 P/yaw 规则发 `/patrol_cmd` | 若误启动会与 Nav2 争夺运动控制；无法使用当前固定 map/costmap 契约 | 完整文件、旧 entry points、当前 launch 无引用 | 后续移入明确 legacy 包或删除入口；当前不删 |
| `patrol_control.py` / `controller_core.py` | 当前 MPPI + PatrolRuntime | 🧹 历史控制辅助 | 直线/转角状态与旧 route 跟踪计算 | 与当前 controller 名称相似，代码搜索容易混线 | 完整文件与当前 import/launch 图 | 与旧 follower 一并治理 |
| `go2_mapping_capture` ROS package | 当前 `CaptureManager` 直接管理 rosbag | ⚪ 包存在但 full image 未构建 | architecture/data-capture manifest 仍列该包 | 资产清单与真实 Docker selection 漂移 | `package.xml`、Dockerfile colcon selection、manifest | 修生成索引的 source-selection 规则；梳理期不删 |
| shutdown 日志中的 Python node error exit | supervisor teardown 主动停 ROS 子进程 | 🟡 更像 rclpy context 已失效后的退出噪声 | 多个节点共享类似退出形态 | incident 会把正常清理误呈现为异常，掩盖真实 first failure | r14 teardown 日志与节点 shutdown 路径 | 后续单独复现实验；在确认前不改退出码 |
| 地图/导航 generation 文档与注释 | 当前 generation 10、field state r14 | 🟡 部分生成索引仍见 gen9/旧职责描述 | 多轮演进遗留文字 | 文档看似权威但落后于代码与现场 receipt | PROJECT_STATE、repository index、源码常量、注释 | 事实来源带 generation/build 时间并自动验漂移 |

## 暂不定性为“重复”的复杂度

- 完整 Dockerfile 与 combined Dockerfile：职责不同；真正风险是修改源选择错误，而非文件本身重复。
- Route 与 Path：Route 是任务/进度/目标/恢复基准，Path 是当次 Smac 规划结果；两者都需要。
- global/local costmap 与 Collision Monitor：一个用于规划/跟踪，一个用于最终近身停车；不是简单重复。
- FAST-LIO 与 VGICP：一个给连续相对运动，一个把它挂到历史 map；不能二选一替代。

仍需继续清点旧 launch/config、永远不可达分支、hardcode/magic number 与测试覆盖盲区。以上条目都只记录，不在本阶段删除。
