# 08 云端与 SaaS 链路

## 必须分开的两条“云”

### A. 地图生产云

`Mac workstation → SSH/rsync → 阿里云 /opt/go2/jobs/map-* → 固定 GLIM adapter → immutable artifacts → rsync 回 Mac`

- ✅ 输入只能是 sealed RecordingBundle。
- ✅ 云作业 ID immutable；若五个完整 output 已存在，重试复用而不重跑 GLIM。
- ✅ worker 以非 root `go2mapping` 跑 mapping pipeline；adapter 校验路径、配置和输入。
- ✅ 云不运行狗端实时 ROS，不输出实时 Pose，不控制机器狗。
- 🟡 内部 GLIM pipeline 是云端已安装外部能力，不在活动产品仓库；本仓库能确认输入/阶段/输出合同，不能确认内部每一个 factor/optimizer 参数。

### B. 业务 SaaS

`SaaS heartbeat endpoint ↔ platform_edge ↔ status files / Unix supervisor / interaction socket`

- ✅ 机器狗每 5 s 默认上报 build identity、boot/edge/runtime identity、电量、map Pose、最终 Twist、patrol/checkpoint 和 interaction 状态。
- ✅ 心跳响应可带最多 32 条 command；command ledger 持久去重。
- ✅ patrol allow-list 只有 `start_patrol` 和 `stop_patrol`；平台没有速度、任意 goal 或 ROS 参数接口。
- ✅ `start_patrol` 被翻译成 supervisor `patrol.start_selected`，并带 expected mapVersion、routeId、MissionPlan。
- ✅ `checkpoint_verdict` / `announcement_completed` 经过 schema 校验写本地 JSONL inbox，再由 coordinator 应用。
- ✅ 结果单独 POST command result；checkpoint events/verdict ack 有各自 API。
- ✅ SaaS 不直接调用 Unitree；唯一运动所有权仍在 navigation manager/runtime。

## 资产发布链

Mac 为一个 mapVersion/routeId 生成 portable bundle：PCD、Route、execution route、workspace、2D map、checkpoint 和参考 JPEG，并为每文件记录 bytes/SHA-256。平台资产上传代码和 revision receipt 待继续审计；当前 r5 本地 manifest 已确认 4 checkpoints 和所有文件 identity。

## 可选实时交互链

状态：✅ 代码/启动条件确认；❓ r14 实际环境是否启用。

`GOGOGUARD_INTERACTION_ENABLED=1` 时，edge entrypoint 才启动 interaction service；发布模板默认 `0`，因此它是条件链，不应画成所有巡检必经链：

`SaaS start_live command → platform_edge → 本地 Unix socket → InteractionEdgeService → LiveKit room ↔ 云端 agent → Z1Pro/BOYA/Go2 speaker`

- `StartLiveCommand` 只允许 credential-free `wss`（显式开发开关才允许 `ws`），解析 JWT claims 并检查 expiry/not-before、`sub=robot:<id>`、roomJoin、publish/subscribe/data grants。这里解析 claims 做命令约束，但签名的真正验证仍由 LiveKit connect 完成。
- Z1Pro RTSP 以 1920×1080/30fps/H.264 上行；BOYA `S24_3LE` 双声道 48kHz 转 mono s16；agent `agent-voice` 经 120ms prebuffer 和最多 3000ms 有界 jitter buffer送至 Go2 speaker。
- 当前 echo policy 是 half-duplex：agent 播音时 microphone mute，播音结束恢复；wake phrase gate 为“小玖小玖”，30s idle 休眠。
- DataChannel 只接受 `agent:` participant 的冻结 schema：playback control、wake transcript、checkpoint verdict、announcement completed。Pose 上行是 latest-only，有正在发布的消息时丢旧样本而不排队。
- `motionCommandsPermitted` 固定为 false，persona 也明确实时对话没有底盘运动权限；实际底盘命令仍只能经过 navigation/runtime 安全链。
- 任何 native media 子任务异常会触发有限指数退避重连，media health 标为 DEGRADED；token refresh 只能保持同 room/robot identity。

## 当前未知边界

- ❓ SaaS 服务端内部任务表、图片 AI/人工判定、报告生成和版本部署不在当前仓库。
- ❓ 实际 r14 是否开启 `GOGOGUARD_CHECKPOINT_EVIDENCE_TXN_ENABLED=1`；模板/发布默认是 0。
- ❓ SaaS、云 GLIM installation、Mac、狗端 release 是否存在一个可查询的统一 deployment snapshot ID；现有证据显示各自版本独立。

## 管理面与信任边界

- ✅ platform_edge 出站心跳可带 `X-Device-Token`；LiveKit connect command 还会校验 JWT identity、room grant、有效期与发布/订阅权限。这里存在应用层身份校验。
- ✅ 与之不同，狗端 Site Console 启动在 `0.0.0.0:8080`，当前 `server.py` 没有认证/授权检查，而写接口能启动/停止巡检、reset localization、修改 profile、控制云台、导入地图。它不是只读监控页。
- ❓ 现场网络是否用 VLAN、防火墙、反向代理或 VPN 把 8080 限制在可信运维网；仓库和 r14 证据没有证明。
- ⚪ `site_console_capture_adapter.py` 自己要求 bearer token，但当前 console server 不验证这个 header；该 adapter 也不在当前狗端启动主链，不能把它当成 console 已认证的证据。
