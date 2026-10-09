# 01 真实运行入口

## 机器狗开机主树

```text
systemd gogoguard-edge.service
└─ /opt/gogoguard/bin/run-edge
   └─ docker run --network host --ipc host --privileged $GOGOGUARD_IMAGE
      └─ deployment/container/edge-entrypoint
         ├─ MediaMTX
         ├─ [optional] interaction edge
         ├─ Livox ROS driver
         │  ├─ /mapping/livox/lidar
         │  └─ /mapping/livox/imu
         ├─ static mount TF publisher
         ├─ calibrated_cloud_transformer
         ├─ FAST-LIO
         ├─ Unitree battery observer
         ├─ navigation observer
         ├─ incident recorder
         ├─ navigation supervisor（Unix socket；按任务动态拉起 Nav2 runtime）
         ├─ [optional] platform heartbeat / pose stream
         └─ site console
```

状态：✅ 已由 systemd unit、`run-edge`、entrypoint 和各进程入口完整交叉确认。

### systemd 与容器边界

- ✅ unit 要求 Docker、网络和时间同步；启动前验证 `/etc/gogoguard/runtime.env` 并最多等时钟 60 秒。
- ✅ 主容器使用 host network、host IPC、privileged，挂载 `/var/lib/gogoguard`、只读 runtime 配置和可选 CA。
- ✅ entrypoint 对必需子进程执行 `wait -n`；任一必需进程退出会清理整组进程，使 systemd 重启容器。
- ✅ Nav2/固定地图定位不是常驻启动：常驻 supervisor 接到 prepare/start 请求后才创建一代 runtime。

## 动态导航 runtime 启动树

```text
navigation supervisor RPC
└─ NavigationManager.start_runtime()
   ├─ 启动 go2_sdk2_udp_receiver（先执行可选 StandUp + BalanceStand + StopMove）
   └─ ros2 launch go2_nav2_runtime active_map_patrol.launch.py
      ├─ obstacle filter / perf monitor / trace recorder
      ├─ continuous fixed-map localizer
      ├─ static map + keepout servers
      ├─ Nav2 planner / controller / behavior / lifecycle
      ├─ velocity smoother
      ├─ collision monitor
      ├─ patrol runtime manager
      ├─ unitree_safe_cmd_node
      └─ cmd_vel_udp_sender
```

状态：✅ 启动关系已由 `supervisor.py`、`manager.py`、launch 和 YAML 确认。⚠️ runtime 启动本身会让独立 SDK receiver 执行姿态准备；这早于定位可用和巡检授权，属于必须在交互指南明确展示的行为边界。

## Mac 工作站入口

- ✅ 主入口：`make workstation` → `deployment/workstation/run-native` → `python3 -m gogoguard_field_workstation`。
- ✅ 默认监听 `0.0.0.0:8080`，读取 `config/workstation.json`，资产根为 `workstation-data`，并托管静态前端。
- ✅ native 入口可从 macOS Keychain 读平台 device token，且直接使用 Mac 有线网/本机 SSH。Compose 是可移植替代，映射 8080 和 cloud SSH key/known_hosts，不使用 macOS `UseKeychain`。

## 云端与 SaaS 入口边界

- ✅ 云端入口：Mac `MapFactoryManager` 经 workstation 适配器提交 map job，cloud `gogoguard-map-job` 调用已安装的 GLIM pipeline，再用本仓库 exporter 生成 PLY/trajectory/map JSON/SVG/build receipt。它不拥有机器狗实时 ROS。外部 GLIM 实现不在当前仓库，只能作为已部署黑盒核对输入/产物/回执。
- ✅ SaaS 狗端入口：optional `platform_edge` 定时外发 heartbeat，处理返回的 allow-listed `start_patrol/stop_patrol/start_live/stop_live`、mission/verdict/announcement 消息；巡检命令经 Site Console/Navigation supervisor Unix 接口进入动态 runtime，媒体命令经 interaction Unix socket。
- ✅ checkpoint 平台协调只对 `decisionMode=platform` 拥有决策权；`local_operator` mission 明确不受它干预。
- ❓ SaaS 服务端任务创建、调度、持久化和报告代码不在仓库；本次只能证明协议边界和已保存的 robot/Mac 回执。
