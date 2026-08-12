# 到点闭环修复待办

更新时间：2026-08-12

状态：**根因分析完成，尚未修改运行代码，等待机器狗下次上线后开发和部署。**

主责模块：`platform_edge`

不属于本次范围：定位、VGICP、Nav2、MPPI、避障、路线、速度、加速度、
安全圈、Z1Pro 角度算法、LiveKit 建连算法。

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

### 尚未确认的第一触发错误

机器狗已经下线，当前无法读取 22:10 那趟的狗端原始文件。因此还不能把最早一次
mission event 失败精确写成 HTTP 400、401、超时或本地前置异常。这个细节不改变
上面的结构性缺陷，但下次上线后应先取证再覆盖运行环境。

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

- [ ] 在 `CheckpointCoordinator` 每次 tick 开始时先读取并验证收件箱消息。
- [ ] 对当前 `missionId + checkpointId + attempt` 的播报终态，允许在没有预存
      `announcementId` 时采用收到的 ID，并立刻推进到稳定等待。
- [ ] 对当前地图、路线、任务、点位、attempt 且未过期的 verdict，先生成
      `continue/skip/retake` 控制，再处理不影响动作安全的业务事件上报。
- [ ] 将 mission event 改成幂等、可重试、**不会阻塞终态消费**的通知流程。
      平台已经声明 event 不要求有序，因此补发不会破坏协议。
- [ ] 保留既有 `eventId` 幂等规则、迟到 verdict 拒绝和导航最终超时兜底。
- [ ] DataChannel、心跳、HTTP 响应三路收到同一终态时只产生一次业务动作。

### P1：把“收到”和“执行”说清楚

- [ ] mission fallback 的命令回执改为明确的“validated and queued”，不再使用容易
      被理解成状态机已执行的笼统文案。
- [ ] 在狗端只读状态中记录：最近入队消息、最近匹配结果、最近应用的控制、拒绝原因、
      当前 coordinator stage、最近失败的 mission-event phase 和安全化错误码。
- [ ] `checkpoint-state.json` 不只保存 `RuntimeError` 类型；至少能区分 HTTP 状态、
      连接失败、本地相机前置失败和相关字段不匹配，同时不得记录令牌。
- [ ] 平台心跳应能暴露协调器是否健康，避免下次只能从“等满了”反推内部失败。

### P2：单独确认稳定等待时间

当前资产中两个点的 `dwellSec` 都是 3 秒；平台说明的自然语言又写了“播报结束后
稳定 1 秒拍照”。这不是本次 38 秒问题的根因，但存在口径差异。

- [ ] 下次开发前与产品确认平台任务的有效稳定时间到底是 1 秒还是资产中的 3 秒。
- [ ] 未确认前不要偷偷把导航兜底或平台超时调短来掩盖问题。

## 必须增加的自动化测试

- [ ] mission event 第一次就失败，但收件箱已有合法 `announcement_completed`：
      一个 coordinator tick 内进入稳定等待，在配置的 `dwellSec` 后产生
      `capture`，不能等待 20/25 秒。
- [ ] mission event 失败，但已有合法、未过期 `checkpoint_verdict=continue`：
      一个 tick 内产生 `continue`，不能等待 verdict timeout。
- [ ] 没有预存 `announcementId`，但任务、点位、attempt 全匹配：接受终态并记住 ID。
- [ ] 错误任务、错误点位、错误 attempt 的播报终态不得推进。
- [ ] 错误地图、错误路线或已经过期的 verdict 必须丢弃。
- [ ] DataChannel 和心跳重复送达同一消息，只生成一个确定性的控制 ID。
- [ ] HTTP 响应直接携带 verdict 时立即应用。
- [ ] mission event 恢复后按相同 `eventId` 补发，不重复触发平台动作。
- [ ] 命令 ACK 文案明确区分 `queued` 与 `applied`。
- [ ] 现有本地人工点位流程、LiveKit、导航和平台命令回归测试全部保持通过。

## 机器狗下次上线后的取证清单

在安装新版本之前，先只读复制以下文件；不要启动定位或 Nav2：

1. `/var/lib/gogoguard/platform/checkpoint-state.json`
2. `/var/lib/gogoguard/platform/checkpoint-inbox.jsonl`
3. `/var/lib/gogoguard/platform/checkpoint-control.json`
4. `/var/lib/gogoguard/platform/command-ledger.json`
5. `/var/lib/gogoguard/runtime-logs/platform-heartbeat.log`
6. 22:10 那趟对应的最新导航 runtime trace 和 navigation status

取证目标：确定最早失败的是哪个 mission-event phase、HTTP 返回状态是什么、
协调器当时的 stage/announcementId 是什么，以及收件箱四条消息的完整相关字段。

## 开发和部署顺序

1. 读取本文件、`PROJECT_STATE.md` 和 `architecture/modules/platform_edge.json`。
2. 先拉上述旧版本证据，复制到忽略的 `runtime-data/analysis/`，不提交真实日志。
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
