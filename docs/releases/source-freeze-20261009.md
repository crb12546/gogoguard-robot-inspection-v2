# GoGoGuard V2 源码定版与交接（2026-10-09）

本次固定的是当前源码和交接资料，版本标识为 `v2-source-20261009`。
接手者应使用这个 tag 对应的 commit；后续修改产生新版本，不移动定版 tag。
这是现场调试系统的源码快照，完整巡检产品验收仍有待办。

## 包含什么

- 当前机器人、Mac 工作站、云端适配和平台对接源码、配置模板、合同、测试与锁定的第三方能力源。
- `PROJECT_STATE.md` 中截至 2026-09-29 的现场记录，以及本次源码定版记录。
- `docs/system-audit/` 的系统审计笔记、`system-model/` 的结构化模型、`system-guide/` 的交互教学页和两个构建/验证工具。
- 根目录和文档导航中的接手入口。

审计与模型保留其取证时点。例如 Git 与部署审计的 2026-08-25 快照不是本次 tag 的 Git 状态。最新现场事实只认 `PROJECT_STATE.md`。

录制、地图、运行日志、事故原始包、设备凭据、SSH 密钥和 Docker 镜像归档不属于本次公开源码；它们位于忽略目录。模型中的历史原始回执路径仅供已有资产的维护者追溯。

## 如何取得和查看

```bash
git clone --branch v2-source-20261009 https://github.com/crb12546/gogoguard-robot-inspection-v2.git
cd gogoguard-robot-inspection-v2
```

先读 [项目状态](../../PROJECT_STATE.md)、[架构与开发者交接](../INSPECTION_SYSTEM_TECHNICAL_GUIDE.md)和[模块索引](../generated/repository-index.md)。
教学页无需启动服务，直接用浏览器打开 `system-guide/index.html`。

公开可查看不等于增加了开源许可。第三方能力遵守各自已有许可；本次没有替项目新增授权条款。

## 本次验证

在 macOS、Python 3.9.6 下验证：

- `make test`：284 项测试通过。
- `make compile`、`make ui-smoke`、`make container-validate`、`make knowledge-check`：通过。
- `python3 tools/validate_system_guide.py`：教学页、模型和仓库证据合同验证。
- 凭据检查：当前源码及当前分支可达历史的高置信度密钥/token 模式扫描未发现匹配；检查到的字面 device token 是测试夹具。

这些是离线源码/合同检查。容器检查没有构建 ARM64 镜像，也不证明机器狗接受了完整现场任务。

## 机器狗版本和已知边界

按照既有现场回执，机器人最后记录的镜像为
`gogoguard-robot-inspection:v2-edge-20260824-raster-r14`，当时构建源标识含 `-dirty`。
本次源码 tag 不补造旧镜像的 clean 构建身份，不代表新镜像已安装。

V6 是已获操作员接受的移动基线；r14 的静态部署和起点 lethal / 持续 `SEARCHING_PATH` 现场问题保留在项目状态中。完整巡检点闭环、导航恢复及运动验收依然需要按已有待办逐项完成。

本次没有连接机器人、连接云 worker、修改运行参数、构建/安装镜像或启动运动。
