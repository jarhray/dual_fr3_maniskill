# 冻结的测试参考实现

这些文件仅供数值对照，生产代码不得导入。2026-09-14 从既有实验文件归档，文件内容未改动，
原路径与 SHA-256 见 [manifest.json](manifest.json)。不因新实现变化而同步修改参考计算或放宽阈值。

| 文件 | 参考阶段 | 用途 |
| --- | --- | --- |
| [contacts_before_clearance.py](contacts_before_clearance.py) | 已有精确查询缓存，尚未增加保守距离跳过、持久复用和后续轮次跳过 | 接触加速内核及相同输入对照 |
| [mpm_before_clearance.py](mpm_before_clearance.py) | 接触加速前的 MPM 调度与缓存生命周期 | 历史接触运动对照 |
| [guide_numpy.py](guide_numpy.py) | 导向 GPU 化前的 NumPy 实现 | 导向几何与运动对照 |

这是针对特定模块的参考，不是整个历史依赖环境的完整快照。
`mpm_before_clearance.py` 仍通过相对导入使用当前包内的模型、纤维等模块；测试显式替换它的接触类。
加载接触/MPM 参考时使用 `dual_fr3_maniskill.cable` 下的独立模块名，以保留相对导入和独立 Warp 内核。

四个接触对照脚本默认使用这里的参考文件，仍允许用 `--reference` 和 `--reference-mpm` 指定其他来源。
命令见[测试入口](../README.md)。本目录不安装为生产 Python 包。
