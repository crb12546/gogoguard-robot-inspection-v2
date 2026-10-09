# 14 Git 与部署可复现性

## 2026-08-25 本地快照

| 项目 | 状态 | 事实 |
|---|---|---|
| 当前分支 | ✅ | `agent/livekit-wss-r14` |
| 当前 HEAD | ✅ | `ce507afd9022e515bbd8ec376213c66cd5cdd7c6` |
| `main` / `origin/main` | ✅ | 与 HEAD 相同，ahead/behind 为 0/0 |
| 工作树 | ✅ | 无 tracked/untracked 非忽略修改（审计文档创建前） |
| submodule | ✅ | 无 |
| Git LFS | ✅ | 无 |
| ignored local assets | ✅ | 约 10,734 个文件条目：runtime 6240、workstation 3894、release 599、`.DS_Store` 1 |

## 初步结论

“是否能仅凭 Git commit 准确复现任一次现场测试”：**不能一概而论**。

- ✅ r14 有镜像 tag、Mac manifest ID、robot config ID、release archive SHA-256、候选 map/route revision、PGM hash 和静态启动回执。
- ✅ generation-10 r12 记录 clean `main` commit `3184de...` 和 image digest。
- ⚠️ 多个历史现场 release 明确标为 `<commit>-dirty`，Git commit 不包含当时未提交差异。
- ⚠️ 地图、录制、候选 workspace、日志和发布包都在忽略目录；复现还依赖资产 hash、配置/身份和容器 digest，不只依赖源码。
- ⚠️ 外部 SaaS 与已部署云脚本有独立版本/哈希，不属于同一 Git commit 原子快照。
- ⚠️ 当前 Mac 上传回执 `platform-uploads/map-0d2b63d9da78.json` 只记录 r3 `verified` 且“尚未激活”；但 r14 robot 快照确认当时已选 r5，且 r4/r5 bundle 在 Mac 存在。这表明当前本地 receipt 不能单独证明 r5 的完整 upload→verify→activate 链。

## 发布缓存审计

| 项目 | 结果 | 判断 |
|---|---:|---|
| release cache 目录 | 60 | ✅ 每份均保存 image tar、release archive 及 SHA 文件 |
| 带 source commit + image digest + build ID 的 `release.env` | 12 | ✅ 新式发布身份较完整 |
| 上述 12 份中 clean source | 2（r12、r13） | ✅ 可从 Git + digest 交叉重建 |
| 上述 12 份中 `-dirty` | 10 | ⚠️ commit 只说明基线，不包含本地差异；现有 image tar 可回放二进制，但不能从 Git 精确重建源码 |
| 旧式只记录 image tag 的 release | 48 | ⚠️ 能用缓存 tar/sha 回放镜像，但源码归因不足 |
| Git tag | 0 | ⚠️ 现场 release 没有 Git tag 层索引 |

`prepare-robot-release` 当前已经记录 source version/commit、image digest/build ID、archive SHA；`install-release` 会验证 archive 和 Docker load，但会保留已 commissioned 的 `runtime.env`，只填补缺失默认项，并且安装后不自动激活。这是合理的安全边界，同时也意味着 release archive **不包含最终有效运行配置的完整快照**。

## 一次现场测试真正需要的身份

仅有 commit 不够。能够重放和解释一次测试的最小 `FieldRunManifest` 应至少冻结：

```text
runId / startedAt / operator / siteId / robotId
sourceCommit + sourceTreeDirtyPatchHash
containerImageDigest + buildId + releaseArchiveSha256
effectiveRuntimeConfigHash（脱敏后的规范化配置）
navigationProfileSavedHash + navigationProfileEffectiveHash
mapVersion + routeId + assetRevision + 每个关键资产 SHA-256
calibrationId / sensorId / robotConfigId
SaaS deployment id + GLIM pipeline id（如参与本次任务）
incident/trace/log bundle IDs
```

不要把密钥值写入 manifest；只写 secret slot/commissioning revision 或不可逆摘要。对 `runtime.env` 先过滤 secret 字段、排序规范化，再计算 hash。

## 简单可执行方案

1. **发布前必须 clean**：现场正式包拒绝 `-dirty`；紧急试验若必须 dirty，自动保存 `git diff --binary` 到 release evidence 并计算 hash，随后尽快形成 commit。
2. **构建只认 digest**：保留人类可读 tag，但安装/回执以 image digest、archive SHA 和 build ID 为准。
3. **地图也当 release**：每个 asset revision 不可变，prepare/upload/verify/activate/robot-select 都追加同一个 transaction ID 的 receipt。
4. **测试开始自动拍快照**：Site Console/SaaS 的“开始任务”先生成脱敏 `FieldRunManifest`，任务与 incident 都引用 runId。
5. **外部能力显式版本化**：GLIM pipeline 与 SaaS 至少暴露 deployment ID/config hash；否则保留 `unknown`，不能伪装成可复现。
6. **保留产物但设期限**：Git 存源码与小型 schema/manifest；对象存储保存 image、地图、bag、incident。manifest 永久，重资产按现场/合规周期归档。

## 长期工程信息分工

| 载体 | 只负责什么 | 不应该承载什么 |
|---|---|---|
| Git | 代码、默认配置、schema、构建/部署脚本发生了什么变化 | 大型 bag/地图、现场密钥、未说明的 dirty 状态 |
| `PROJECT_STATE.md` | 当前现场事实、最新验收/失败、下一步和关键 receipt 链 | 教科书式长期算法说明、整份日志复制 |
| `docs/system-audit` | 为什么如此设计、当前架构、证据与遗留判断 | 每次运行的原始 telemetry |
| `system-model` | 可被工具验证和网页消费的结构化当前事实 | 没有证据的行业常识 |
| Interactive Guide | 快速建立系统直觉、按 L1→L4 跳到证据 | 成为另一份手工维护的事实副本；它应从 model 加载 |
| 代码注释 | 局部代码无法自然表达的约束理由、失败模式、兼容窗口 | 逐行翻译代码或长期系统架构 |

结论：当前对“某些新式 clean release”可以较好复现；对任意一次历史现场测试不能保证。先实施 manifest/receipt 链，不需要引入复杂版本管理平台。
