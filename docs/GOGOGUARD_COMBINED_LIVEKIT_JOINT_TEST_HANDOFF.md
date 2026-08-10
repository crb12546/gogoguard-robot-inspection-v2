# GoGoGuard 联合联调交付说明（机器狗 V6 巡检 + LiveKit 实时对话）

> 交付日期：2026-08-10
>
> 机器人：`LLYJ0001`
>
> 状态：狗端代码与 Linux/ARM64 发行包已就绪，等待平台方与现场联合验收。

## 一、这次交付了什么

这次不是重做巡检，而是在今天已经完成实地行走验收的 V6 巡检基线上，合入前两天已经用真实 BOYA 麦克风、Z1Pro 相机、Go2 扬声器和 LiveKit 验证过的实时对话能力，再补齐平台心跳控制桥。

最终分成三个清晰的模块：

1. `navigation` 继续独立负责定位、路线跟随、避障和运动控制。
2. `interaction` 独立负责 LiveKit 音频、视频、播放中断、断线重连和小玖对话状态。
3. `platform_edge` 每 5 秒向 GoGoGuard 发送心跳，只转发实时会话白名单指令，不拥有 ROS、`cmd_vel` 或任何运动控制入口。

巡检 V6 冻结提交是 `922b301`，实时对话合入提交是 `6e842d1`。从冻结点到本交付，`navigation`、`localization`、`route`、Nav2 和 Unitree 运动实现没有变更；新增的只是交互配置、平台适配器和后续巡检点的离线合同基础。未完成的“到点核查”没有接入当前巡检运行时。

## 二、平台对接契约

### 1. 狗端主动心跳

狗端默认每 5 秒调用：

```http
POST https://gogoguard.cn/api/v1/robot/heartbeat
Content-Type: application/json
```

心跳中包含：

- `robotId`：`LLYJ0001`
- `motion`：当前 map 坐标系位姿和最终执行速度
- `patrol`：巡检状态、地图/路线标识和进度
- `interaction`：LiveKit 期望状态、实际状态、发布/订阅状态、数据通道、错误码和观测时间
- `capabilities`：狗端当前真实支持的能力，未完成的能力会明确返回 `false`

当前狗端还没有可信的电量 SOC 数据源，因此没有伪造 `battery`。请平台在本次联调中将该字段视为可选。

### 2. 平台下发的会话指令

启动 LiveKit 会话：

```json
{
  "id": "platform-command-id",
  "action": "start_live",
  "params": {
    "url": "wss://gogoguard.cn",
    "room": "patrol-test-001",
    "token": "<short-lived LiveKit JWT>",
    "publishVideo": true,
    "publishAudio": true,
    "subscribeAudio": true,
    "publishData": true
  }
}
```

停止 LiveKit 会话：

```json
{
  "id": "another-platform-command-id",
  "action": "stop_live"
}
```

平台 ASR 向狗端同步已完成识别的文本：

```json
{
  "id": "platform-event-id",
  "action": "wake_transcript",
  "text": "小玖小玖"
}
```

狗端只允许上面三种 action。其他指令，包括任何移动、启停巡检或运动参数指令，都会被拒绝。

### 3. LiveKit 轨道与语音状态

| 方向 | 轨道名 | 内容 |
|---|---|---|
| 狗 -> 平台 | `robot-microphone` | BOYA mini 2，48 kHz 单声道 PCM |
| 狗 -> 平台 | `z1pro-camera` | Z1Pro，1920×1080，请求 30 FPS / 2.5 Mbit/s |
| 平台 -> 狗 | `agent-voice` | 48 kHz 单声道语音，支持有序打断 |

LiveKit 连接状态与对话唤醒状态分离：房间可以保持 `live`，但对话处于 `sleeping`。平台的 ASR/agent 第一版仍负责主门禁：未听到“小玖小玖”时不得将普通语音送进对话模型；唤醒回复“我在”；30 秒无有效对话或听到明确结束语后回到休眠。

### 4. 幂等、刷新与安全

- 同一个 `id` 只执行一次。
- token 刷新必须使用新的命令 `id`；同房间的新 token 仅在内存中替换，不会主动重连健康会话。
- token、LiveKit URL 和 room 都不写入指令去重文件；token 和 URL 也不出现在公开状态 API 中。当前 room 名作为会话身份观测字段会回报给平台。
- 默认强制 HTTPS/WSS 和 TLS 校验，平台指令不能降级传输安全。
- 狗端支持通过 `GOGOGUARD_DEVICE_TOKEN` 带独立 Bearer 凭证。当前 P1.5 测试端点只根据 `robotId` 识别设备，可用于联调，但不等于生产认证。

## 三、平台方联调前需要准备

1. 确认 `POST /api/v1/robot/heartbeat` 对 `LLYJ0001` 可用，并将 `battery` 字段视为本次可选。
2. 准备可用的 `wss://gogoguard.cn` LiveKit 路由和短期 JWT；token 建议 60 分钟有效，剩余不足 5 分钟时用新命令 ID 刷新。
3. 确认平台 agent 订阅 `robot-microphone` 和 `z1pro-camera`，并发布唯一的 `agent-voice`。
4. 确认 ASR/agent 唤醒门禁已启用，休眠状态的普通语音不进模型。
5. 联调时保留每次下发的命令 ID、平台 participant SID、轨道状态、重连原因和首音频延迟，不要在日志中打印 JWT。

## 四、明天的联合验收顺序

不直接带着狗跑。先让狗贴地趴着，完成静态零运动联调，再做巡检并发。

1. 安装 ARM64 发行包，先不启动服务，核对镜像与文件 SHA-256。
2. 安装器会保留机器人现有的 root-only `runtime.env`和 Unitree 设备密钥，只补充缺失的发行默认项。保留已验收的 V6 地图、路线和导航配置，执行 `sudo /opt/gogoguard/bin/configure-combined-joint-test` 启用 `interaction` 与 `platform_edge`；这个命令不会启动服务。
3. 狗保持趴地，平台下发 `start_live`，核对心跳中 `state=live`、音频/视频轨道和 `dataConnected=true`。
4. 分别验证：双向语音、当前视觉、“小玖小玖”唤醒、休眠期普通语音拒绝、语音打断、token 刷新、主动断网重连和 `stop_live`。
5. 静态联调全部通过后，启动今天已验收的 V6 巡检，同时保持对话 10 分钟。
6. 记录 CPU、内存、温度、网络、相机帧率、音频延迟、LiveKit 重连和巡检控制节拍，然后下发 `stop_live`并停止巡检。

## 五、通过标准

- 静态阶段零运动命令，实时对话不影响狗的姿态。
- `interaction.state=live`，麦克风/相机发布成功，`agent-voice` 订阅成功，`dataConnected=true`。
- 小玖人设、唤醒/休眠、当前视觉拒绝、打断和 token 刷新符合契约。
- 非故意断网测试期间不发生无故重连；断网测试后能恢复且不有序重放旧音频。
- V6 巡检能完成已验收路线，不出现由对话模块引起的定位、Nav2、MPPI、避障或 Unitree 控制参数变化。
- 10 分钟并发期间无进程异常退出，无持续资源恶化，心跳持续在线。

## 六、可观测性

现场可以通过机器狗 Edge Console 的以下只读 API 查看状态；也可以经现场已建立的 Mac -> 狗 SSH 转发访问。Mac 本地工作台不会伪造或镜像这些狗端实时状态：

- `/api/v1/interaction`：LiveKit 会话、轨道和对话状态
- `/api/v1/platform`：平台心跳成功时间、错误码和命令计数
- `/api/v1/capabilities`：当前真实支持/未支持的检查能力

## 七、自测和发行回执

- 本地全量自测：`143 passed`
- Python 编译、前端烟雾、容器契约、JSON/脚本语法、生成文档一致性和 `git diff --check`：全部通过
- 完整本地 HTTP 心跳 -> Unix socket -> 真实交互状态机模拟：通过
- 命令去重、新 ID token 刷新、错误机器人拒绝、运动指令拒绝、敏感信息不落盘：通过
- 镜像内平台心跳 -> Unix socket -> 交互状态机：`7/7 passed`；镜像内会话/设备 IO/交互边界：`19/19 passed`
- 发行安装模拟：现有设备密钥与定制值保留、新默认项补全、交互显式启用、密钥缺失失败关闭、服务不自动启动：通过
- 既有真设备可行性回执：185.83 秒、18 轮对话、9,000 麦克风帧、3,495 个 1080p 视频帧、9,212 个平台音频帧、18 次有序打断、扬声器丢帧 0；首音频 P50 0.712 秒 / P95 1.248 秒，无重连，峰值温度 57.75 ℃，运动命令 0。
- ARM64 镜像：`gogoguard-robot-inspection:v2-edge-20260810-combined-live-r1`
- 镜像 ID / 架构 / 大小：`sha256:6027a7d17ff961da1772a308b074bb4b24568871c126d291b0f9b47dbd18f11c` / `linux/arm64` / `1,388,559,726 bytes`
- V6 底座一致性：已验收 V6-r4 的 32 个 rootfs layer 是合并镜像 44 个 layer 的完整前缀，只追加 12 个应用/交互 layer；无 `apt` 和 ROS/Nav2 重装。
- 离线发行包：`release-cache/v2-edge-20260810-combined-live-r1/` （镜像归档 `1,388,612,096 bytes`）
- 镜像归档 SHA-256：`9e25dca2cbee5784cd81ccb60b0e65155231055094fc5545d7ef272d4fbce174`
- 源码提交：`TBD_AFTER_COMMIT`

## 八、本次不做虚假承诺的部分

1. P1.5 测试端点的 `robotId` 只能识别设备，不是生产认证；上线前必须启用独立设备凭证或其他等价机制。
2. `wss://gogoguard.cn` 还需在狗的真实 4G 网络上生成 TLS/SNI 回执，不把 Mac 或旧 IP 备用路由当成生产证据。
3. Z1Pro 原生拍照文件取回、云台静态标定、LiveKit 位姿 publisher 和完整“到点核查”仍是后续模块，没有混入这次已验收的巡检运行时。

---

请平台方按第三节准备联调环境，并回复以下三项：

1. `LLYJ0001` 心跳端点是否已就绪；
2. `wss://gogoguard.cn` 和平台 agent 是否已就绪；
3. 明天可联调的时间段。
