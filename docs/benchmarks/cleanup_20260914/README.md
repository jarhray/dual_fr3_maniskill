# 2026-09-14 整理检查记录

本轮整理 README、安装/性能文档、测试参考源码、结果归档和 Git 忽略规则。
未继续优化物理、未加入细杆后端或求解器选择参数，未提交或推送 Git。
基准提交、7 个生产物理模块的 SHA-256 与结果摘要见 [summary.json](summary.json)。
这 7 个模块与 2026-09-12 最后一次 MPM 性能测量完全相同，默认 YAML 也保持原值。

## 本轮结果

| 检查 | 结果 |
| --- | --- |
| 三个 ROS 包构建 | 通过，见 [build.log](build.log) |
| 跨包 pytest | 283 通过、2 失败，见 [pytest.log](pytest.log) |
| 默认仓库参考的 24 组接触对照 | 通过，六类状态最大差均为零，见 [contact_equivalence.json](contact_equivalence.json) |
| 保守缓存/扫掠边界回归 | 通过，见 [contact_acceleration.json](contact_acceleration.json) |
| 移动后的 NumPy 参考与 GPU 导向 | 16 组对照和边界行为通过，见 [guide_equivalence.json](guide_equivalence.json)、[cuda.log](cuda.log) |
| 参考源码、历史 JSON 的归档哈希 | 全部与原文件一致 |
| 文档本地文件链接、测试脚本语法、Git 空白检查 | 通过 |
| 完整 MTC 规划与执行 | 本轮未运行 |

原始日志/JSON 的来源与哈希见 [manifest.json](manifest.json)。本轮没有重跑性能，也没有用新测量覆盖旧结果。

## 两个 MTC 测试失败项

失败位于本轮没有修改的 MTC 测试/运行代码中，保留原始断言和日志，不以修改物理或规划阈值处理：

1. `test_fr3_cartesian_regression.py::test_fr3_continuous_line_passes_without_disabling_jump_or_tcp_checks`
   在预期拒绝的低跳变阈值分支中，测试写死 `Achieved: 0.901639`，本机返回 `Achieved: 0.906977`。
   两者均为未达完整笛卡尔距离，但字符串断言失败后，后面的默认阈值接受分支没有执行，不能记为已验证。
2. `test_mtc_prototype.py::test_create_motion_planners_configures_move_group_ompl`
   测试期望默认 `jump_threshold=2.0`，当前规划器配置为 `2.5`。

这两个问题与本轮参考文件搬移、文档整理无直接代码关系；后续应单独核对 MTC 默认值和跨环境回归断言。
不能将 283 项通过写成“全部回归通过”。历史 MPM 全程 0.16 mm 对照未通过的记录也继续保留，见
[2026-09-12 基线](../mpm_20260912/README.md)。

## 用户检查与备份范围

- `dual_fr3_maniskill`：保留先前兼容性/性能改动，新增文档和测试参考；删除了 20 个已跟踪的 Python 字节码缓存，新增 `.gitignore`。
- `dual_fr3_moveit_config`：本轮补充 README 交接链接；已有 `scripts/check_usb_cable.py` 的无效重置验证修正需要一起检查。
- `dual_fr3_trunking_mtc`：本轮只补充 README 交接链接。

关键 JSON 已放入仓库，完整粒子快照与调试产物仍留在原工作区 `artifacts/`。
仓库根级别的工作区目录不是 Git 仓库，需分别检查各 `src` 子仓库。
后续 agent 从[线缆后端交接说明](../../cable_backends.md)开始阅读。
