# GoGoGuard × 机器狗“到点核查”狗端回复

> 日期：2026-08-10
>
> 回复范围：《GOGOGUARD 下一阶段联合开发》第四节
>
> 口径：只报告已经验证、已有代码和明确不支持的事实，不把“硬件理论上能做”写成产品已支持。

## 先给结论

实时通话不会重做。现有 `interaction` 模块已经合并到当前发版分支，保留了 LiveKit 实时视频、麦克风、Go2 扬声器、半双工/打断、重连、权限校验和数据通道。它没有运动接口，不能绕过导航控制狗。

下一段不是再造一套大系统，而是用三个薄模块把已有能力串起来：

```text
navigation：只负责走、停、继续，并出具“真停了”的回执
mission：   只负责到点 → 停稳 → 核查 → 等待/离线继续的状态编排
inspection：只负责转相机、接收证据帧、校验和断网缓存，不判断是否合格
interaction：继续提供已有实时音视频和 LiveKit 数据通道
平台：       定义查什么、做判定、存证据、生成报告
```

当前已完成离线合同、状态机、证据缓存和能力声明基础；已有实时通话的 `start_live/stop_live` 心跳适配器。到点核查的平台适配、导航暂停/恢复接线、云台真机静态验收和位姿数据通道发布尚未完成。

## 第四节逐条回复

| # | 狗端当前真实答案 | 对平台的口径 |
|---:|---|---|
| 1 | 当前路线不是 CSV，是与地图版本绑定的 `go2.route.v1` JSON，包含有序 x/y/yaw 路点。运行时有单调的 `routeProgressIndex`，可知道已走到路线的哪一段。 | 检查点应绑定 `mapVersion + routeId + routeProgressIndex`，不能只传“第 N 个点”。路线重发布后 index 可能变化。 |
| 2 | 现在只有内部路线进度，还没有对平台的“已到达 checkpointId”业务事件。新 `mission` 合同已把这个事件定义为路线进度跨过配置 index 时产生。 | 平台最终不应再用 6 m 半径猜到点；接线前这项能力必须声明为 `false`。 |
| 3 | 现有对外“停止巡检”会退出导航运行时并释放遥控权，不是业务暂停。定位恢复虽会从未完路线后缀继续，但也不是平台暂停 API。新合同要求回执同时匹配 mission/checkpoint/pause/map/route，运动授权为 false，线速度 ≤ 0.02 m/s、角速度 ≤ 0.03 rad/s，且连续稳定 ≥ 0.5 s。 | 当前 `pauseResume=false`。只有导航适配器真正实现上述回执后才能改为 true；“收到暂停请求”不算真停。 |
| 4 | 现有导航会上报状态、第一原因、可恢复情况和自动恢复次数，但没有将“24 个检查点完成 9 个”聚合成业务任务结果。 | 新 `MissionStatus` 会区分 traveling/pausing/inspecting/waiting/resuming/completed/interrupted/failed，但尚未接入实时运行时。 |
| 5 | Z1Pro 硬件和官方 GCU 协议支持程控。官方范围为 pitch -110°∼+120°、yaw ±140°，最高控制速度 150°/s；支持 TCP 2332 和 0x10 角度控制。协议编解码已进入 `device_io`，但尚未在这台狗静止状态下完成“转动—到位—重复”验收。 | 当前声明 `gimbal.supported=false, hardwareSupported=true`。官方的 ±0.01° 是增稳精度，不是已证明的重复定位精度；重复精度和到位时间要真机实测后填。 |
| 6 | 0x10 模式是混合参考系：yaw/pan 是云台相对狗身，roll/pitch 是欧拉角。我们现在没有每帧向 GCU 提供可验证的载体 INS，因此不声称世界绝对角。 | 平台方向定义应明确写 `panBodyDeg` 和 `tiltEulerDeg`；如果需要对世界固定方向，平台或狗端适配层必须结合停车时的狗身 yaw 做补偿。 |
| 7 | Go2 本身能原地转向，但不把它作为摄像头方向的默认降级方案。转狗会改变底盘姿态、占用空间，还需重新校验停稳和障碍。 | 先验收云台；云台不可用时退化为当前方向取一帧，不自动转狗。 |
| 8 | 现有实时链路已真机验证 Z1Pro 1920×1080 视频，平台可直接从已有 `z1pro-camera` 视频轨抓帧。Z1Pro 官方还支持 0x20 快门和最高 2688×1520 JPEG，但狗端尚未接通照片文件取回，也未实测是否影响实时流。 | 第一版就复用已有实时视频抓帧，不为静态照片另造一条重链路。 |
| 9 | 导航 PCD 是 binary PCD v0.7，字段为 float32 `x y z intensity`，单位米，坐标系为该版本的 `map`，地图在发布时固定且重力对齐，z 轴向上。当前合同禁止事后再找一个平面强行拉平。 | **不能保证所有地面的数值都恰好 z=0**；平台应直接使用 PCD 和同版本位姿，不做第二次旋转或缩放。 |
| 10 | 已有不可变地图号 `map-xxxxxxxxxxxx`。路线 ID 包含地图工作区修订号，运行状态携带 `mapVersion` / `routeId` / `manifestHash`。 | 平台点位、任务和证据必须与三者绑定；版本不符时拒绝静默沿用。 |
| 11 | 开机后由 FAST-LIO 提供连续里程，VGICP 把当前扫描与固定地图对齐。外层还检查初始区、与最后可信锚点的跳变、候选一致性和时效；恢复时要连续 3 帧互相一致才改写锚点。对外有 `usable` / `confidence` / `reason` / `age`。 | `confidence` 是工程质量分，**不是经过统计标定的“正确率百分比”**。平台第一版应先用 `usable + reason + age`决定是否核查，不要自己猜一个百分比门槛。 |
| 12 | 重录会产生新的不可变地图版本，不覆盖旧地图。当前没有检查点自动迁移算法。 | 平台应把旧点位标为待迁移/待作废，由人在新图确认，不默默套用旧坐标。 |
| 13 | 狗端现已有有界、校验和、原子落盘的证据帧缓存基础；默认规划上限为 2000 帧 / 4 GiB，支持帧 ID 幂等、SHA-256、FIFO 淘汰、上传确认后删除和篡改检测。但还没有与平台到点协议接线和真机压测。 | 当前声明 `offlineBuffer.supported=false, implementationPresent=true`。接线并验收后再改 true，不用“代码存在”替代“真实可用”。 |
| 14 | 现有导航、定位和运动都在狗端，已下载路线不因 Mac/外网断开而天然停止；`interaction` 也没有运动权。到点核查的新目标是：任务预先授权离线继续时，只有证据已持久化才恢复路线，联网后补传和延迟判定。 | 普通路线可本地继续；到点的离线继续策略已进合同，但尚未部署。 |
| 15 | 定位源 `/localization/pose` 约 10 Hz；现有导航公开状态文件约 5 Hz。LiveKit 会话已经校验 `canPublishData` 并返回 `dataConnected`，所以数据通道能力存在；但位姿序列化、topic 和 10 Hz publisher 还没接上。 | 当前 `poseStream.supported=false, livekitDataChannelAvailable=true, posePublisherIntegrated=false`。平台给出 topic/字段约定后，直接复用已有会话，不再起一条长连接。 |
| 16 | 当前没有经验收的自动返航/充电坞闭环，也没有可用于产品排程的续航压测数据。 | `lowBatteryReturn.supported=false`；平台先按人工充电和明确中断上报设计。 |
| 17 | 狗端已新增版本化能力档案和 `GET /api/v1/capabilities`，会同时告诉平台“硬件支持”、“代码存在”和“产品已验收”，不把三者混为一个 true。新的实时通话心跳也已携带该档案。 | API 和心跳适配已在本次分支，尚未进入机器狗已部署镜像。 |

## Z1Pro 依据和待验收项

- 官方产品页：<https://www.allxianfei.com/en/z-1pro-black-light-full-color-night-vision-pod.html>
- 官方 User Manual V1.4：<https://download.s21i.co99.net/22859939/0/4/ABUIABA9GAAgwYPLxAYojOXylAQ.pdf?f=Z-1Pro+User+Manual-XF%28A5%29V1.4.pdf&v=1754448321>
- 官方 GCU Private Protocol V2.0.6：<https://download.s21i.co99.net/22859939/0/4/ABUIABA9GAAglc7tuAYo6OLsjwc.pdf?f=GCU+Private+Protocol-XF%28A5%29V2.0.6.pdf&v=1729849109>

上真机前还需要一次狗趴着的静态验收：控制端按官方建议 30∼50 Hz 通信，只转云台不发狗身运动；测左/中/右和上/中/下的到位回读、到位时间、10 次重复误差、边界限位、断线恢复，再验证原生快门是否影响实时视频。

## 平台下一步可以依赖什么

平台可以基于上述真实状态起草《到点核查接口约定》，但暂时不要把下列能力当成已上线：`waypointEvent`、`pauseResume`、`gimbal`、`offlineBuffer`、`poseStream`。这五项必须在狗端适配、部署和联调完成后，分别从 false 改为 true。

两条硬保证保持不变：

1. 位姿、路线、检查点和证据帧必须绑定同一 `mapVersion` / `routeId`。
2. 证据帧使用对齐后的绝对时间戳，位姿带源时间，不用上传到达时间冒充拍摄时间。
