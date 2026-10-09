# 13 未确认项

1. 🟡 r14 起点 lethal：已证实现场 footprint 覆盖 r5 静态 PGM 中的 1 个黑格，allowed-area 则全允许；静态/未知格是高概率主因。但当时未保存分层 costmap，尚无法排除 live obstacle/inflation 叠加。
2. ❓ FAST-LIO 与 localizer 双发 `odom→base_link` 在实际 r14 日志中的 authority/时间表现，以及 localizer 再广播是否仍必要。
3. ❓ SaaS 服务端内部实现不在本仓库；只能确认接口合同和 robot/Mac 适配。
4. ❓ `PROJECT_STATE.md` 顶部更新时间为 2026-08-24，但包含/引用更晚 8 月 25 现场段落；文档元数据需标记偏差。
5. ❓ 当前 20 个历史 map job/18 个 recording 与每次现场测试、源码 commit、容器 digest 的一一对应是否完整。
6. ❓ Mac `platform-uploads/map-0d2b63d9da78.json` 仅记录 r3 已验证未激活，而 r14 robot 现场快照确认已选 r5；r4/r5 实体 bundle 也存在。r5 究竟经过哪一次平台上传/激活交易，本地当前 receipt 不完整。
7. ❓ GLIM 外部源码 archive、worker image digest 与依赖 lock；准确 commit、CPU 插件/GICP-VGICP 配置、环境、运行日志和产物已由本地云 session 快照确认。
8. ❓ 现场网络是否用 VLAN、防火墙、VPN 或反向代理限制 Site Console `0.0.0.0:8080`；代码本身没有服务端认证。
9. ❓ r14 实际是否开启 realtime interaction 与 checkpoint evidence transaction；模板/默认值不足以证明现场有效环境。
