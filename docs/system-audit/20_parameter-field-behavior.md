# 20 参数到现场行为

## 结论

✅ 第二版将 14 组关键参数建模为“控制含义、当前有效值、默认/保存值、增减方向、改善、副作用、来源和证据”。网页从 `system-model/diagnostics.json` 生成参数卡。

## 必须区分三层值

1. stored：操作员或 incident 保存的 profile；
2. normalized：`profiles.py` 对 v1–v5 兼容迁移后的 generation-10 profile；
3. effective：launch 覆盖到 Nav2/localizer/safe cmd/SDK 的实际参数。

r14 的保存 profile 是 v4 aggressive tuple：turn 0.6、accel 2.0、decel 2.5、detour 0.6。由于现场证明这组 tuple 让低速路径不可行，`validate_profile()` 只对完全匹配的已发布 tuple 定向迁回 0.4/0.9/0.9。基础 Nav2 YAML 的 MPPI batch 是 700，但当前 effective profile/launch 为 1000。只读 YAML 或 incident 文件都会得到错误结论。

## 影响跨模块的参数

- `targetCruiseMps=0.60`：MPPI vx_max 和 velocity smoother 前进上限；
- `maxForwardMps=0.90`：safe cmd 与 UDP sender 硬上限，不是正常巡航目标；
- footprint padding 0.10m：同时影响 global/local costmap、Smac、MPPI 和 Collision Monitor；
- inflation 0.45m：在 0.30m padded half-width 外提供 0.15m 软肩；
- localizer 24/32/4m：控制当前 scan、历史 target 和 refresh，源于 v2 image build patch；
- VGICP points/inlier/MSE/jump：决定匹配能否进入状态机；
- recovery 3 次、0.30m/6° 一致：防止 LOST 后单次偶然好解；
- progress timeout 5s 与 replan interval 0.75s：影响恢复节奏，但不能修复无界 SEARCHING_PATH 架构缺口；
- SDK watchdog 250ms：最终有效 UDP 包消失后的 StopMove 延迟。

## 使用规则

参数建议不是现场改值授权。每次变更必须绑定一个可观察问题、预期改善、可接受副作用、回放/受控场验证和回滚 profile。无法说明参数控制对象时，不允许以“试试看”方式改 production profile。
