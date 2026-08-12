# 文档导航

这个目录只保留五类文档。新开发者不要按文件名猜当前状态，请按下面顺序阅读。

1. [系统架构与开发者交接总文档](INSPECTION_SYSTEM_TECHNICAL_GUIDE.md)：产品目标、三端部署、算法、全部模块、数据合同、运行流程、开发和部署规则。
2. [当前项目状态](../PROJECT_STATE.md)：最新代码、已部署版本、真实现场证据、已知问题和下一步。它是动态事实，不是架构教材。
3. [生成的仓库索引](generated/repository-index.md)：每个模块的代码目录、入口、依赖、输入输出和来源。由模块清单生成，不手工编辑。
4. 专题文档：
   - [现场工作台操作](FIELD_WORKSTATION_GUIDE.md)
   - [导航编排](navigation-orchestration.md)
   - [实时音视频与小玖](realtime-interaction.md)
   - [当前到点闭环修复待办](CHECKPOINT_CLOSURE_REPAIR_TODO.md)：下一次机器狗上线后的取证、开发、测试和验收步骤。
5. `evidence/`：早期不可变构建/部署证据；`archive/`：已经完成使命、只用于追溯的一次性交付文档。

## 文档维护规则

- 架构和模块边界变化：更新总文档和对应的 `architecture/modules/*.json`，再运行 `make knowledge`。
- 最新部署、现场结果或已知缺陷变化：只更新 `PROJECT_STATE.md`。
- 操作按钮和现场步骤变化：更新 `FIELD_WORKSTATION_GUIDE.md`。
- 一次性对外回复或阶段交付完成后：移入 `archive/YYYY-MM-主题/`，不要继续作为当前说明使用。
- 录音、地图、日志、镜像和密钥永远不进入 `docs/` 或 Git。
