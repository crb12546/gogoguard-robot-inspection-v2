# 到点闭环修复待办

更新时间：2026-08-23

状态：**狗端开发与离线验证已完成；尚未部署、尚未注入机器狗令牌、尚未进行静态或真实巡检验收。**

主责模块：`platform_edge`

不属于本次范围：定位、VGICP、Nav2、MPPI、避障、路线、速度、加速度、
安全圈、Z1Pro 角度算法、LiveKit 建连算法。

## 2026-08-23 开发交接

- 到点终态现在先于 mission event 补发处理；401/403 使用低频退避，不再阻断
  `announcement_completed` 或 verdict 应用。
- 证据事务 v1 只有在双方显式启用
  `GOGOGUARD_CHECKPOINT_EVIDENCE_TXN_ENABLED=1` 时生效，release 默认仍为 `0`。
- `capture_ready` 只接受顶层 `captureResponse` 包裹。成功回执持久化后才允许生成
  `capture` 控制；`captured` 事件只携带平台 `captureId`，不再合成 `evidenceId`。
- `captureRequestId` 已按双方补充约定钉死为紧凑 UTF-8 JSON 数组的 SHA-256；
  黄金值测试覆盖 attempt 数字类型和空字符串槽位。固定 v1 artifact 不改字节，
  `manifestSha256` 仍为
  `58230fa7782fb5c4921a5ba730eb359f06b56b57eff57494732caac341281eb0`。
- 增加了持久任务占用与机器可读冲突码、四层运行身份、逐 transcript 唤醒回执、
  心跳唤醒状态，以及从 JPEG SOF 读取的真实宽高/字节数/摘要。
- 离线结果：253 项单元/集成测试通过，Python 编译、知识索引、release 脚本语法和
  `git diff --check` 通过。容器契约检查仍被工作区已有的 combined Dockerfile
  冻结基线缺失阻断；本次没有覆盖该现有修改。

## 一句话说明

平台的播报结束和判定消息确实已经送到狗端，狗端也回复了 `done`；但这个
`done` 目前只表示“消息通过校验并写进了收件箱”，不表示“巡检状态机已经执行”。

负责把收件箱消息变成导航 `capture` / `continue` 控制的到点协调器，在读取消息
之前强制执行一串 `POST /api/v1/robot/mission/event`。只要其中一次 HTTP 调用失败，
协调器就停在旧阶段，不再消费已经到达的终态消息。导航状态机只能等自己的固定
超时，因而每个点多停约 38 秒。

## 现场证据

平台原始说明位于 Mac 桌面：

`/Users/mac/Desktop/到点闭环-平台侧时序证据.md`

对应任务：`mission-20260812-221035-RG-狗02`，2026-08-12 22:10，两个巡检点。

| 证据 | 结论 |
|---|---|
| 播报结束在约 2 秒内发出，现场确实听到声音 | LiveKit 房间和 agent 音频有效 |
| `announcement_completed` 与 `checkpoint_verdict` 四条命令均收到 `done` | 平台命令格式已通过狗端校验并写入收件箱 |
| 狗端随后仍分别保持 `WAITING_PLATFORM` 和 `WAITING_VERDICT` | ACK 没有转化为状态机控制 |
| 两个点的等待时长高度一致 | 实际走的是狗端固定兜底超时，不是随机网络延迟 |
| 平台未看到狗端 mission event | 到点协调器的事件流程没有完整建立 |

### 2026-08-17 完整狗端复现

证据目录：
`runtime-data/analysis/platform-checkpoint-20260817-232116/`

任务：`mission-20260817-231747-RG-狗02`

地图：`map-62d8cec9a1dc`

路线：`route-62d8cec9a1dc-workspace-r5`

| UTC 时间 | 狗端证据 | 结论 |
|---|---|---|
| 15:18:19.409 | `cp_01` 真停车完成，距目标约 0.10 m | 到点和定位正常 |
| 15:18:19.414 | 观察角分配为 `camera_only`，镜头目标 `+62.976 deg` | 角度计算正常，不是资产要求镜头不动 |
| 随后每 0.2 s | `checkpoint-state.json` 仍为 `stage=new, lastError=RuntimeError` | 同步 mission-event 在改变 stage 前抛错，并无退避重试 |
| 整个任务 | 无 15:18 的 `gimbal.move_converged` 或 `gimbal.move_failed`，最后反馈约 0 deg | 协调器没有调用本地云台端点 |
| 15:18:21--22 | 平台播报完成 | 平台心跳兼容路径可以绕过狗端 event 流播报 |
| 15:18:26.524 | `announcement_completed` 已校验并写入 inbox | 传输正常，但协调器没有消费 |
| 任务全程 | 没有对应 `checkpoint_verdict` | `waiting_verdict` event 同样 401，平台不知道狗正在等判定 |
| 15:18:46.555 | 命令账本记录 `stop_patrol` | 平台记录证实该命令为 `by=admin` 手动点击，不是平台或狗端兜底 |
| 15:18:47.825 | 导航进入 `STOP_REQUESTED`，点位失败原因 `STOPPED` | 狗正确执行平台停止 |

平台日志已将该 `RuntimeError` 精确为 HTTP 401，响应体是
`{"detail":"设备令牌无效"}`。08-17 当天从 11:06:19 UTC 起共有 18,541
次 mission-event 请求，100% 为相同 401；只有 08-11 历史上成功过 8 次。
平台的校验顺序是 eventId -> phase -> 设备鉴权，因此 401 同时证明
报文格式和 phase 合法，失败仅属于令牌不匹配。

狗端取证另外排除了原待办中的“本地相机前置异常”：云台 HTTP
端点无论成功或失败都会写入 gimbal journal，本次没有任何对应
记录，所以协调器在云台调用前已经退出。

## 已确认与尚待取证的边界

### 已确认的代码缺陷

1. 心跳命令 ACK 和状态机应用是两个独立步骤。
2. `announcement_completed` 与 `checkpoint_verdict` 都先写入
   `/var/lib/gogoguard/platform/checkpoint-inbox.jsonl`。
3. `CheckpointCoordinator` 在消费收件箱之前同步发送 mission event。
4. mission event 任一步抛异常，当前 tick 立即退出；不会生成导航控制文件。
5. 导航层在约 25 秒和 `verdictTimeoutSec + 2 秒`后自动推进，所以现场表现为
   “消息早已收到，但狗仍把时间等满”。
6. `expiresAt` 只做“过期丢弃”判断，不存在“等到 expiresAt 再执行”的代码。

### 已确认的鉴权不一致

- `/robot/heartbeat` 当前只按 `robotId` 识别，不检查令牌。
- `/robot/mission/event` 优先检查 `X-Device-Token`；只要请求带了错误
  令牌就直接 401，不回落到 `robotId`。
- 之前提供的新令牌只注入了 Mac 工作台进程，用于 r5 资产上传；
  机器狗主容器从根用户专有的 `/etc/gogoguard/runtime.env` 取值，
  且部署脚本会主动保留已有值，因此镜像更新没有替换旧令牌。
- 平台将改为令牌不匹配时回落到 `robotId`，并细分 401 诊断。这能恢复
  联调，但狗端仍必须配置正确令牌，不应把回落当成长期身份方案。

## 对平台四个问题的正式口径

1. **ACK 是否等于状态机已消费？**
   不是。当前 `done / command accepted by robot service` 只证明消息被校验、落入
   收件箱并记入命令账本，不能证明 `capture` 或 `continue` 已写给导航状态机。

2. **`announcementId` 应该怎么关联？**
   正常主流程中，狗端应从 `announcing` HTTP 响应取得平台生成的
   `announcementId`。但平台的心跳兼容路径会根据 `WAITING_PLATFORM` 直接开始播报；
   如果狗端 HTTP 流程失败，狗端就没有机会取得这个 ID。修复后应先按
   `missionId + checkpointId + attempt` 确认当前点位，再用 `announcementId` 去重；
   不应因为本地暂时没有预存该 ID 而丢弃同一当前点位的合法终态。

3. **`expiresAt` 是什么语义？**
   只表示判定的最晚有效时间。到达时已经过期就丢弃；未过期就立即执行。平台不需
   改短，也不要改字段名。

4. **收到终态后能否立即推进？**
   必须立即推进。播报终态进入后进入短暂稳定等待并触发拍摄；有效的
   `continue/skip/retake` 进入后立即生成对应导航控制。固定超时只保留为最后兜底。

## 开发方案

### P0：解除错误的前置依赖

- [ ] 通过机器狗本地安全配置注入正确 `GOGOGUARD_DEVICE_TOKEN`；不得
      写入 Git、镜像、release archive 或日志。
- [x] 在 `CheckpointCoordinator` 每次 tick 开始时先读取并验证收件箱消息。
- [x] 对当前 `missionId + checkpointId + attempt` 的播报终态，允许在没有预存
      `announcementId` 时采用收到的 ID，并立刻推进到稳定等待。
- [x] 对当前地图、路线、任务、点位、attempt 且未过期的 verdict，先生成
      `continue/skip/retake` 控制，再处理不影响动作安全的业务事件上报。
- [x] 将 mission event 改成幂等、可重试、**不会阻塞终态消费**的通知流程。
      平台已经声明 event 不要求有序，因此补发不会破坏协议。
- [x] 为 mission event 增加有上限的指数退避；401 记录为鉴权故障后降到
      低频重试，不得再每 0.2 秒请求一次。
- [x] 保留既有 `eventId` 幂等规则、迟到 verdict 拒绝和导航最终超时兜底。
- [x] DataChannel、心跳、HTTP 响应三路收到同一终态时只产生一次业务动作。

### P1：把“收到”和“执行”说清楚

- [x] mission fallback 的命令回执改为明确的“validated and queued”，不再使用容易
      被理解成状态机已执行的笼统文案。
- [x] 在狗端只读状态中记录：最近入队消息、最近匹配结果、最近应用的控制、拒绝原因、
      当前 coordinator stage、最近失败的 mission-event phase 和安全化错误码。
- [x] `checkpoint-state.json` 不只保存 `RuntimeError` 类型；至少能区分 HTTP 状态、
      连接失败、本地相机前置失败和相关字段不匹配，同时不得记录令牌。
- [x] 平台心跳应能暴露协调器是否健康，避免下次只能从“等满了”反推内部失败。

### P2：单独确认稳定等待时间

当前资产中两个点的 `dwellSec` 都是 3 秒；平台说明的自然语言又写了“播报结束后
稳定 1 秒拍照”。这不是本次 38 秒问题的根因，但存在口径差异。

- [ ] 下次开发前与产品确认平台任务的有效稳定时间到底是 1 秒还是资产中的 3 秒。
- [ ] 未确认前不要偷偷把导航兜底或平台超时调短来掩盖问题。

## 必须增加的自动化测试

- [x] mission event 第一次就失败，但收件箱已有合法 `announcement_completed`：
      一个 coordinator tick 内进入稳定等待，在配置的 `dwellSec` 后产生
      `capture`，不能等待 20/25 秒。
- [x] mission event 失败，但已有合法、未过期 `checkpoint_verdict=continue`：
      一个 tick 内产生 `continue`，不能等待 verdict timeout。
- [x] 没有预存 `announcementId`，但任务、点位、attempt 全匹配：接受终态并记住 ID。
- [x] 错误任务、错误点位、错误 attempt 的播报终态不得推进。
- [x] 错误地图、错误路线或已经过期的 verdict 必须丢弃。
- [x] DataChannel 和心跳重复送达同一消息，只生成一个确定性的控制 ID。
- [x] HTTP 响应直接携带 verdict 时立即应用。
- [x] mission event 恢复后按相同 `eventId` 补发，不重复触发平台动作。
- [x] mission event 持续返回 401 时按预期退避，且本地云台、播报终态和
      verdict 应用仍正常。
- [x] 命令 ACK 文案明确区分 `queued` 与 `applied`。
- [x] 现有本地人工点位流程、LiveKit、导航和平台命令自动化回归测试全部保持通过。

## 已完成的狗端取证

2026-08-17 已在不再启动导航的情况下只读复制以下文件：

1. `/var/lib/gogoguard/platform/checkpoint-state.json`
2. `/var/lib/gogoguard/platform/checkpoint-inbox.jsonl`
3. `/var/lib/gogoguard/platform/checkpoint-control.json`
4. `/var/lib/gogoguard/platform/command-ledger.json`
5. `/var/lib/gogoguard/runtime-logs/platform-heartbeat.log`
6. 23:17 新任务对应的导航 runtime trace 和 navigation status

除旧版本未保存的精确 HTTP 状态码和失败 event phase 外，狗端证据
已齐全。新版必须在状态中保存这两项安全化诊断，不得依赖下次再猜。

## 开发和部署顺序

1. 读取本文件、`PROJECT_STATE.md` 和 `architecture/modules/platform_edge.json`。
2. 使用已归档的 `runtime-data/analysis/platform-checkpoint-20260817-232116/`
   做回放和对照，不提交真实日志。
3. 修改主责模块 `services/platform_edge`；没有证据不得改 Nav2、MPPI 或定位参数。
4. 先用 fake poster 离线模拟 mission event 全部失败，同时向收件箱注入终态。
5. 跑全量测试、编译、知识索引和容器契约检查。
6. 构建 ARM64 镜像，在机器狗趴下且运动栈停止时静态部署。
7. 不让狗运动，先做冻结协议建议的“单点位空跑”：人工切换状态/注入消息，确认
   `capture` 与 `continue` 在 1 秒内出现。
8. 最后才进行一趟真实两点巡检并同时保留狗端和平台时序。

## 现场验收标准

- 播报真正结束到狗端进入拍摄动作：不超过 1 秒加明确配置的 `dwellSec`。
- 有效 verdict 到达到生成导航控制：不超过 1 秒。
- 正常流程不得命中 20/25 秒播报兜底或 `verdictTimeoutSec + 2 秒`导航兜底。
- 两条传输路径重复消息只执行一次。
- `expiresAt` 之前到达立即执行，之后到达明确记录为 expired 并忽略。
- 平台能区分消息已排队、状态机已应用和被拒绝的原因。
- 两个巡检点均播报、抽帧、判定、继续，路线最终 `COMPLETED`。
- 整个验证不引入新的导航参数变化。

## 完成定义

只有同时满足以下条件才能关闭本待办：

1. 离线故障注入测试证明 event 失败不再阻断终态。
2. 静态单点位空跑通过。
3. 真实两点巡检不再出现固定 23～25 秒和 10～17 秒空等。
4. 平台侧能看到正常 mission event 或明确的补发/失败状态。
5. 狗端原始时序、平台时序和最终路线完成状态能相互对上。
