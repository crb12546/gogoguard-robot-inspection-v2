# 19 Legacy 删除判断证据

## 结论

当前可以确认多套实现不在主启动链，但还不能直接删除。`system-model/repository-health.json` 为每项记录原问题、替代者、启动状态、Git 证据、删除风险和验证建议。

## Git 历史边界

`route_relocalizer.cpp`、registration benchmark、`submap_builder.cpp`、旧 waypoint follower、`patrol_control.py` 和 `controller_core.py` 都只在 `04d55da`（2026-08-06，`feat: add three-end field navigation workflow`）出现一次；该提交也同时导入当前 ContinuousMapLocalizer 和 Nav2 runtime。

因此：

- ✅ Git 能确认它们以一次大批量 capability snapshot 进入当前仓库；
- ❓ Git 不能证明哪一套在时间上先运行，也不能恢复作者的原始选型实验；
- 当前“legacy”判断依据是启动树、Topic/TF 责任、配置和实际 current runtime，而不是臆造的提交历史。

`MissionCoordinator` 与 `EvidenceFrameBuffer` 在 `090635c` 创建；MissionCoordinator 在 `f4dd9ac` 扩展。当前仅 setup/architecture/tests 引用，实际 checkpoint runtime/platform transaction 不 import。

## 关键删除风险

- `route_relocalizer`：主链不启动，但需检查产品仓库外运维脚本是否手工 `ros2 run`；
- benchmark：删除会失去现成 NDT/GICP/VGICP 对照工具，更适合先移到明确的 benchmark 区；
- submap builder：可能被当临时取图工具使用，应核对现场 SOP；
- waypoint follower：仍有 console entry point，误启动会竞争 `/patrol_cmd`，删除风险和保留风险都高于普通 dead code；
- MissionCoordinator/EvidenceFrameBuffer：先决定是目标架构还是未完成接入，再同步 architecture index、setup 和 tests；
- safe cmd 未启用 gates：节点当前在跑，但两个能力参数为 false。应区分 capability 与 active policy，不能简单把整节点标成 legacy。

## 通用删除门

1. 当前 systemd/container/launch 不引用；
2. 已部署机器人、Mac、云端脚本和手工 SOP 不引用；
3. Topic、TF、Action 和文件产物没有消费者；
4. current replacement 覆盖原来必要的现场能力；
5. 镜像构建、容器契约、回放和受控现场 dry-run 通过；
6. 先删除入口/安装，再观察一个发布周期，最后删除源码。
