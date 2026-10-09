# 09 地图与坐标系

## 当前系统到底有几种“地图”

| 名称 | 它是什么 | 谁创建/何时 | 存储/寿命 | 谁使用 | 当前状态 |
|---|---|---|---|---|---|
| 原始 LiDAR 点云 | MID-360 每次扫描的测距点 | Livox 10 Hz | ROS 流；录制时进入 rosbag | FAST-LIO、离线 GLIM | ✅ |
| 去畸变单帧 | FAST-LIO 用 IMU 把扫描期间运动消掉后的点 | FAST-LIO 实时 | `/navigation/cloud_lidar`；瞬时 | 坐标变换器 | ✅ |
| body 单帧 | 同一帧从 `lidar_link` 转到 `base_link` | calibrated transformer | `/navigation/cloud_body`；瞬时 | 固定图 localizer、obstacle filter | ✅ |
| FAST-LIO 局部地图 | odom frame 内增量 ikd-tree | FAST-LIO 实时 | 内存；随启动重建 | 当前 scan matching/里程计 | ✅ |
| GLIM 内部 submap/graph | 离线优化过程的子图/因子图产物 | 云 pipeline | 云 job work/dump | GLIM build/editor | 🟡 外部 pipeline 内部未审计 |
| `map.ply` | GLIM 全量 3D 点图（含 intensity） | cloud review/export | map job immutable | candidate builder/editor | ✅ |
| `map.json` | 最多 50k 点的浏览器评审图 + 压缩 trajectory | cloud exporter | map job immutable | Mac UI/route seed；不是 localizer 的点源 | ✅ |
| `map.pcd` | 从全量 PLY 确定性转换的 fixed 3D map | RouteManager publish | candidate/robot 长期版本化 | VGICP localizer | ✅ |
| 2.5D ground preview | 下包络测量/插值/未知地面格 | workspace preview | 工作站响应/审阅 | 只给人看；无导航权 | ⚪ 非 runtime authority |
| static 2D map PGM/YAML | allowed polygon 内的 reviewed height-slice occupancy | candidate builder | candidate/robot 版本化 | Nav2 StaticLayer | ✅ |
| keepout PGM/YAML | polygon 外为禁止区的 raster | candidate builder | candidate/robot 版本化 | Nav2 KeepoutFilter | ✅ |
| Global costmap | static + live obstacle + inflation + keepout 合成 | Nav2 runtime | 内存，动态更新 | Smac/MPPI | ✅ |
| Local costmap | 8m×8m rolling-style local planning window，叠加同类 layers | Nav2 runtime | 内存实时更新 | MPPI、runtime health/obstruction | ✅ |
| Route | 版本化的巡检参考线/任务索引 | workspace/RouteManager | route.json | runtime/checkpoint/progress | ✅ |
| Path | 当前 Pose 到下一目标的规划线 | Smac runtime | ROS Action 结果；会重算 | MPPI | ✅ |

## PCD、3D Map、2D Map、Occupancy Grid、Costmap

- PCD 是文件格式；当前 `map.pcd` 恰好承载固定 3D localization map。
- “3D Map”是语义类别；GLIM 的 PLY、候选 PCD 和 FAST-LIO 内存局部图都属 3D 点图，但来源、坐标和寿命不同。
- “2D Map”在当前通常指 `navigation-map.pgm/yaml`，是落在 XY 网格上的静态占据值。
- `nav_msgs/OccupancyGrid` 是运行时 ROS message 表达；map_server 会把 PGM/YAML发布成这种消息，costmap 自己也以 OccupancyGrid 对外发布。
- Costmap 不是另一张磁盘地图；它是运行时把静态图、实时障碍、膨胀和禁区层合成的“规划代价”。

## 坐标树与现实意义

```text
map（历史 GLIM 地图，长期固定）
└─ map → odom：fixed-map VGICP localizer 估计并平滑
   odom（本次开机 FAST-LIO 的局部连续世界）
   └─ odom → base_link：FAST-LIO LiDAR+IMU ESKF
      base_link（机器狗机身导航参考）
      ├─ base_link → lidar_link：commissioned 固定外参
      └─ base_link / IMU mount：FAST-LIO base-output 校准
```

- `map`：回答“我在历史巡检场地哪里”，允许 localizer 修正 `map→odom`。
- `odom`：回答“从本次启动开始我连续移动了多少”，短时平滑但会漂移，不保证和历史地图永久对齐。
- `base_link`：机器狗身体位姿，Route、footprint、Nav2 控制均以它为实体参考。
- `lidar_link`：MID-360 光心/传感器坐标；点和 GLIM 原始 trajectory pose 在这个 frame。
- `lidar_imu`：FAST-LIO 内部/legacy 输出涉及的 IMU/LiDAR frame；当前 Nav2 点云主链不消费 legacy `/cloud_registered_body`。

## 当前 r5 的关键事实

- GLIM 历史 artifact 未显式写 pose frame，当前兼容规则按 `lidar_link` 解释；workspace 明确记录 `sourceTrajectoryPoseFrame=lidar_link`、`routePoseFrame=base_link`。
- current fixed PCD 406,778 点，hash `4d703190…7dc2`；浏览器 map.json 只有 50,000 抽样点，二者不能互换。
- 2D static map 1,944×1,696、0.10m，PGM origin 已 lattice aligned，route 的最终矩形 footprint raster conflict 为 0。

## 仍需验证

- ❓ 实际 runtime TF tree 是否出现两个 `odom→base_link` authority；静态代码显示 FAST-LIO 和 localizer 均可能发布。
- ❓ r14 lethal start 的 exact costmap layer/cell；2D 文件静态验收通过不代表运行时 inflation/live layer 后起点一定 free。
