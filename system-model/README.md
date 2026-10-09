# GoGoGuard structured system model

这里保存从代码、配置、入口、现场回执和审计笔记中提取的结构化事实。交互式 `system-guide` 只消费这里标记了状态和证据的事实；不得在网页里自行补齐未知链路。

结构化来源分成四份，避免教学表达与系统事实相互污染：

- `system.json`：当前边界、启动树、数据/控制流、模块、接口、地图、坐标、算法状态、现场发现和未确认项；
- `teaching.json`：零前置知识课程、术语依赖、L1–L4 教学内容和五个核心算法的完整解释契约；
- `diagnostics.json`：十类现场现象的反向排查树，以及参数到现场行为的影响模型；
- `repository-health.json`：Git 快照、legacy 删除判断、系统体检和长期工程规则。

维护规则：

1. 先在 `docs/system-audit` 记录结论与证据；
2. 只有完成状态判断后才更新 `system.json`；
3. guide 使用构建出的 `assets/model-snapshot.js`，不在 HTML 手写另一套架构；
4. 修改任一 model 后运行 `tools/build_system_guide_model.py`；
5. 再运行 JSON、术语预算、算法深度、证据路径和 guide smoke test。

公开源码中只分发模型和审计结论。模型可能引用忽略目录中的历史原始回执；这些引用用于追溯，不代表下载源码后能取得对应日志、地图或录制。`tools/validate_system_guide.py` 会报告未分发的外部回执数量，只验证仓库内证据和教学页合同；现场事实仍以 `PROJECT_STATE.md` 的日期与回执边界为准。
