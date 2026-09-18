# 测试入口

所有命令从工作区根目录执行，先加载 ROS 和 `install/setup.bash`。
`test_*.py` 是 pytest 检查；`check_*.py` 是显式运行的物理/数值脚本，不会自动启动真实机器人。
GPU 检查期间应停止其他仿真。新报告目录须尚不存在。

插入重构的源码基线、几何/参数等价性、服务与完整场景结果见
[本轮验证记录](../docs/insertion_refactor_validation.md)，调参后选测路径见
[插入参数](../docs/insertion_parameters.md#验证入口)。涉及真实 rclcpp/MTC 的测试需可用的本机 DDS 环境；
受限沙箱中的网络/日志权限错误或原生崩溃应单独记录，不修改断言来规避。

## 离线回归

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test
```

覆盖配置、几何、轨迹、环境兼容、Warp 安装工具、启动和 MTC 调度；不运行 CUDA 物理步进。
ROS/MTC 依赖必须先安装。不要把历史通过数量当作当前运行结果。

## 当前 MPM 实现

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_contacts_cuda.py
.venv/bin/python src/dual_fr3_maniskill/test/check_guide_cuda.py \
  --output /tmp/guide-equivalence.json
.venv/bin/python src/dual_fr3_maniskill/test/check_execution_cuda.py \
  --output /tmp/trunking-execution-check
bash src/dual_fr3_moveit_config/scripts/validate_usb_cable.sh \
  --steps 25 --render-output /tmp/usb-cable.png
```

分别检查有限半径接触、导向 GPU/NumPy 对照、Graph/网格边界与生命周期，以及 USB 运动/渲染/重置。
`check_execution_cuda.py` 包括相同输入对照、双缓冲、真实姿态变化、扩容、分辨率变化、拒绝超预算分配和 Graph 回退。

## 性能与迭代数实验

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/profile_trunking_cable.py \
  --steps 8 --warmup-steps 2 --render-frames 1 --contact-details \
  --output /tmp/trunking-profile
.venv/bin/python src/dual_fr3_maniskill/test/check_execution_motion_cuda.py \
  --steps 20 --position-limit-mm 0.16 --output /tmp/trunking-motion
```

第一项保持机械臂目标不动，分别测量普通执行、同步分段计时和可选碰撞计数；应以未插桩轮的耗时评价速度。
第二项比较普通 6 次迭代两轮、加速 6 次和加速 4 次，包含小幅双臂运动及重置。
历史记录中，4 次迭代误差过大；6 次的严格全程 0.16 mm 对照也曾未通过，普通实现自身重复也越过该限值。
脚本会保留结果并非零退出，不应为得到绿色结果删除对照或放宽阈值。
详见 [MPM 性能与验证](../docs/mpm_performance.md) 和[历史结果](../docs/debugging_summary.md#mpm-baseline)。

## 历史实现对照

以下命令默认使用[仓库内参考源码](reference/README.md)，无需原工作区的 `artifacts/`：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_contact_equivalence_cuda.py \
  --report /tmp/contact-equivalence.json
.venv/bin/python src/dual_fr3_maniskill/test/check_contact_acceleration_cuda.py \
  --report /tmp/contact-acceleration.json
.venv/bin/python src/dual_fr3_maniskill/test/check_contacts_shadow_cuda.py \
  --output /tmp/contact-shadow
.venv/bin/python src/dual_fr3_maniskill/test/check_trunking_contacts_cuda.py \
  --output /tmp/contact-motion
.venv/bin/python src/dual_fr3_maniskill/test/check_trunking_guide_cuda.py \
  --output /tmp/guide-motion
```

前两项检查内核边界用例，第三项逐次比较实际运动中的碰撞输入；后两项比较独立运行轨迹。
独立 MPM 运行存在单精度原子归约差异，其结果与相同输入内核对照分别评价。
这些脚本不执行完整 MTC；完整 ROS/MTC 检查见[环境说明](../docs/setup.md)。

## Rope-Actor 与后端选择

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test/test_cable_backends.py
.venv/bin/python src/dual_fr3_maniskill/scripts/check_cable.py \
  --cable-solver rope_actor --steps 300 --output /tmp/rope_actor_check.json
bash src/dual_fr3_maniskill/scripts/validate_integration.sh \
  --mode mtc --cable-solver rope_actor
```

第一项运行真实 CPU PhysX 约束、质量、滑动、重置和失败清理测试，不创建渲染器。
其中孔口回归包含非居中孔的合法偏心、碰壁/脱孔拒绝，以及真实研究手指 CAD 中
无导向关节胶囊的重力支撑和有/无摩擦滑动对照。
分段检查覆盖 60 段与 MTC 的 151 段；穿透诊断检查包括物理对象名、段编号、
局部形状偏移/旋转到世界坐标的转换，以及清除接触或重置后的诊断清理。
第二项在实际 MTC 场景中检查运动、接触和重置，并输出耗时，可切换 `--cable-solver mpm`。
第三项在独立 ROS 域执行完整 MTC；`--cable-config` 可用于配置对照。

右手指接触的 CPU 最小实验见 [后端说明](../docs/cable_backends.md#不依赖渲染器的右手指接触最小实验)。
`scripts/check_rope_finger_contact.py` 不创建 Vulkan 渲染器，覆盖短链、移动真实 CAD 代理、
固定/交替/自适应步长和逐子步诊断；它仍不代替完整 MTC。
`test_cable_backends.py` 检查容差与步长解耦、默认兼容及有界诊断缓存；
`test_rope_finger_contact.py` 验证无接触运行正确失败并保存记录、拒绝覆盖已有报告。
连续旋转代理回归覆盖 0.2/1 ms 步长、位姿/速度目标、无效边界拒绝和对象清理。
旧 `SActor` 与单链接运动学代理的独立对照见
[旋转代理故障](../docs/debugging_summary.md#proxy-rotation)。
