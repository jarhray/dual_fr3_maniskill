# MPM 基线：2026-09-12 测量，2026-09-14 归档

本目录保存原始 JSON 的逐字节副本，来源和 SHA-256 见 [manifest.json](manifest.json)。
历史记录内的绝对路径只表示当时的输入来源，不是运行依赖。大体积粒子 `.npz`、图片和调试日志仍在原工作区的
`artifacts/cuda_graph_20260912/`、`artifacts/contact_acceleration_20260912/`；本目录不包含它们。

## 环境与范围

RTX 3090、Python 3.10.12、ManiSkill2 0.5.3、SAPIEN 2.2.2、配套 Warp 0.3.1。
MTC 配置为 10507 粒子，控制/刚体/MPM 频率 50/500/5000 Hz，轴向 6 次、接触 2 次迭代。
每个控制步推进 20 ms 仿真。无交互窗口，不包含 ROS 发布和完整 MTC 规划/执行；渲染另行计时。
结果仅代表这些短区间；材料没有按真实线缆标定。

| 区间 | 优化前 ms/步 | 优化后 ms/步 | 加速 | 优化后 RTF |
| --- | ---: | ---: | ---: | ---: |
| 预热 1 步、测 3 步的初始区间 | 187.48 | 89.26 | 2.10× | 0.224 |
| 预热 2 步、测 8 步，包含托板/线槽接触 | 328.00 | 261.76 | 1.25× | 0.076 |

短区间见 [before_graph_short.json](before_graph_short.json)、[after_graph_short.json](after_graph_short.json)；
接触区间见 [contact_before.json](contact_before.json)、[contact_after.json](contact_after.json)。
短测之后仅补充了 Graph 签名中的诊断缓冲区指针，内核数学未变；接触区间后测对应整理前最终实现。
分段计时有同步扰动，不能把其组件占比当成纯设备时间。

[contact_counters.json](contact_counters.json) 的独立计数轮记录了 121831 次最近点查询和 365493 次射线查询，
绝大多数来自线槽。计数轮使用普通约束提交，其耗时不作为 Graph 加速比。
[comparison.json](comparison.json) 保留消融实验汇总：4 次迭代更快，但因数值差异过大没有采用。

## 验证与失败结果

[execution_checks.json](execution_checks.json)：245 个边界用例、40 组相同输入约束链对照通过。
包含双缓冲、移动姿态、扩容、分辨率变化、重置、拒绝超预算分配、Graph 回退及释放检查。
相同输入的位置、速度、APIC 差异为零，累计接触冲量最大差约为 4.55e-13。
[usb_checks.json](usb_checks.json) 保存 USB 场景运动结果，测得最大穿透为零；当次脚本还通过了渲染和重置断言。
当时 28 项针对配置、几何与计时的 CPU 检查通过；这不是本仓库所有测试的总数。

独立运动对照为 20 步运动＋5 步重置后步进，原始结果见 [motion_comparison.json](motion_comparison.json)：

| 相对普通 6 次的比较 | 运动末态最大坐标差 mm | 全程最大坐标差 mm |
| --- | ---: | ---: |
| 普通 6 次自身重复 | 0.123322 | 0.232518 |
| 加速 6 次 | 0.100255 | 0.174522 |
| 加速 4 次 | 16.284704 | 16.284704 |

**严格的全程 0.16 mm 检查未通过，`acceleration_accepted` 保留 `false`。**
加速 6 次的运动末态符合该预算，但不能据此声称任意轨迹全程满足；普通实现自身重复也超过全程限值。
4 次迭代明显不符合预算，`four_iterations_accepted` 为 `false`，默认保留 6 次。
位置坐标差、三维点距离、RMS、速度差和穿透是不同指标；原报告分别记录，穿透阈值始终是独立的 0.1 mm。

这批结果验证局部物理与执行路径，不代表完整 MTC 已通过，也不证明 MPM 已达到性能上限。
后续后端的比较应重新记录同场景、同运动及同几何下的性能与物理指标；不能直接套用旧加速比。
