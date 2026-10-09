# 04 建图链路

## 已确认的生产链

```text
人工遥控机器狗走完整巡检区域
→ 狗端 CaptureManager
  → ros2 bag record：/mapping/livox/lidar、/mapping/livox/imu、/Odometry
  → 5 Hz snapshots.jsonl（工作站需要的传感器/位姿快照）
  → 操作员标记 checkpoint：同刻快照 + Z1Pro JPEG + 云台角
→ stop 后封存 RecordingBundle（每文件 bytes + SHA-256）
→ Mac 通过狗端只读 artifact API 做可恢复同步
→ Mac 通过 SSH/rsync 上传云端 immutable map job
→ 阿里云固定 gogoguard-map-job
  → 外部已安装 GLIM pipeline：prepare → optimize/submap → build → review artifacts
  → exporter 封装 PLY、优化轨迹、评审 JSON/SVG 和 build receipt
→ Mac 下载并复验完整产物、轨迹时间序列和 recording-start coverage
→ 操作员编辑 navigation workspace
  → recorded Route（转换到 base_link）
  → allowed-area polygon
  → 高度切片自动障碍 + 人工 block/clear
→ 生成 generation-10 candidate
  → map.pcd（固定地图定位）
  → route.json / runtime_profile.json
  → navigation-map.pgm/yaml（Nav2 StaticLayer）
  → allowed-area-mask.pgm/yaml（KeepoutFilter）
  → checkpoints / portable platform bundle
→ 以 mapVersion + routeId + 各文件 hash 发布给平台和机器狗
```

状态：✅ 入口、传输、输入、产物、复验、编辑和候选生成均由完整代码与当前资产交叉确认。现存云 session 快照还确认了 GLIM/glim_ros2 精确 commit、有效配置哈希、CPU 插件、GICP/VGICP 配置、运行日志和实际 submap/trajectory 产物。❓ 外部源码树与 worker 镜像 digest 未保存在本仓库，不能假装已逐行审完外部实现。

## 狗端 RecordingBundle

- ✅ robot mode 用 `ros2 bag record` 录制配置中的三条 Topic；当前配置为 LiDAR、IMU、`/Odometry`。
- ✅ 另以 0.2 s 周期写 `samples/snapshots.jsonl`。
- ✅ checkpoint 只能在活动录制中标记；保存 JPEG 内容身份、当前 snapshot index、raw pose、云台 pan/tilt/roll、note、spin。
- ✅ stop 先结束采样和 rosbag，随后遍历所有文件生成 `gogoguard.recording_bundle.v1`；中断不会伪装为 sealed。
- ✅ sealed 是导出硬边界；Mac/狗的交换层逐文件校验安全路径、长度和 hash。

## GLIM 产物的真实含义

- `map.ply`：GLIM 导出的全量 3D 点云，候选构建以它为权威。
- `trajectory-poses.json`：保留优化后的每个 timestamp、位置和 quaternion，用于 checkpoint 时间绑定和姿态；不是狗端实时定位输入。
- `map.json`：浏览器/工作站友好的评审模型，点数最多抽样 50,000，同时携带压缩的 trajectory 和 routeCoverage。
- `overview.svg`：人看的静态预览。
- `glim-build.json`：云 build 回执。
- 若 GLIM 首个可优化 Pose 晚于录制起点超过 0.5 s，exporter 用最多前 20 s 的 timestamp-matched 样本拟合一个受限 SE(2) 变换并补回缺失前缀；残差或接缝不合格就拒绝发布。

## 从 3D 图到可导航候选

- ✅ `map.ply` 全量点以 x/y/z/intensity 原样转为 binary `map.pcd`，不由 2D 编辑器裁切；它供 VGICP 固定地图定位。
- ✅ GLIM trajectory 历史 pose frame 默认为 `lidar_link`。工作区在 route 边界利用 commissioned mount calibration 转成 `base_link`，以后编辑/发布都只接受 base pose。
- ✅ 2D static map 取 PCD 中 `obstacleMinZ..obstacleMaxZ` 的高度切片，将重复/邻近支持的格作为自动障碍；recorded route 的实机矩形 footprint 可清除录制时自体/地面噪声，人工 clear/block 再覆盖。
- ✅ 生成器在输出 PGM 前后都以 padded rectangle `[front 0.50, rear -0.43, y ±0.30]` 校验 route footprint；PGM origin 对齐同一网格 lattice。
- ✅ 允许区域另生成 keepout mask。2D ground surface 是 review-only 2.5D 预览，`navigationAuthority=false`，不会改变 Nav2 通行权。

## 当前选中地图的实物快照

| 项目 | 当前值 | 证据状态 |
|---|---:|---|
| map job | `map-0d2b63d9da78` | ✅ job/candidate/r14 回执 |
| recording | `20260823T131600Z-fcc0bbbe` | ✅ job.json |
| 云 worker | `cloud-glim` | ✅ job.json |
| GLIM optimized poses | 2,085 | ✅ trajectory contract |
| 补回 prefix poses | 13；RMS 0.04797 m；max 0.11805 m；join 0.15057 m | ✅ routeCoverage |
| PLY→PCD 点数 | 406,778 | ✅ candidate metadata |
| `map.json` 评审抽样点 | 50,000 | ✅ artifact |
| workspace | revision 5，recorded route，ready | ✅ workspace/candidate |
| route | 217 workspace points；714 execution points；94.903 m | ✅ candidate/manifest |
| static map | 1,944 × 1,696，0.10 m/cell，28,737 occupied | ✅ navigation-map metadata |
| 人工 surface edits | blocked 0；clear 5,041 | ✅ workspace r5 |
| checkpoint | 4 | ✅ platform asset manifest |

## 未确认

- ✅ 当前现场快照绑定 GLIM `aafbbff…`、glim_ros2 `4dccdaa…` 与配置 file-set hash `e5dfdd…0325`；详见 `18_external-glim-evidence.md`。
- ❓ 云 worker image digest、依赖 lock 和可获取的外部源码 archive 尚未进入 map job receipt。
- ❓ 20 个历史 map job 中每个作业对应的云 pipeline 版本并非都有原子版本回执。
