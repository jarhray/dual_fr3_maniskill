# 双 FR3 ManiSkill 仿真

`dual_fr3_maniskill` 使用 ManiSkill2 / SAPIEN 2 执行双 FR3 仿真，线缆可选择 MPM 或 Rope-Actor 建模。ROS 桥接接收机械臂和夹爪动作，发布实测关节状态与仿真时钟，供 MoveIt、MTC 和 RViz 使用。

## 当前状态（2026-09-16）

支持 `cable_solver:=mpm`（默认）和 `cable_solver:=rope_actor`。
本次接触夹持验收聚焦 **Rope-Actor + USB-only**；MPM 延后完善。默认后端保持不变，使用下面显式指定 `rope_actor` 的命令。
旧固定夹持版本的 Rope-Actor 简化线槽 / 2 mm 线缆曾由用户确认完成完整 MTC 走线，配置保存在
[trunking_cable_simplified_2mm.yaml](config/trunking_cable_simplified_2mm.yaml)。
该配置使用简化线槽做碰撞，ManiSkill / RViz 显示原始线槽模型。
当前默认配置为上述 `trunking_cable_simplified_2mm.yaml`（简化碰撞线槽 / 2 mm 线缆），线缆由 USB 直接连接右孔，不再绕回左孔（`guide.routing: usb_to_right`）。新流程采用动态 USB 接触夹持、定位释放和滑移监测，
实际验证及范围见[本次验证记录](docs/usb_grasp_validation.md)。历史 3 mm 实验见
[调试总结](docs/debugging_summary.md)。Rope-Actor 仍为实验后端，材料与摩擦尚未标定。
参数、模型差异和验证方法见[线缆建模方式](docs/cable_backends.md)。
`simulation_backend:=maniskill` 选择机器人仿真环境。

MTC 当前默认 1.5 m 长、2 mm 直径、1 mm MPM 采样，MPM 5000 Hz、轴向迭代 6 次、接触迭代 2 次。
CUDA Graph 与 GPU 网格检查默认启用；右手导向和碰撞投影也在 GPU 执行。
材料、采样、迭代和穿透限制的默认值见 [trunking_cable_simplified_2mm.yaml](config/trunking_cable_simplified_2mm.yaml)。

RTX 3090 上的独立短测约为 89 ms/控制步，包含托板、线槽接触的区间约为 262 ms/控制步；
每步推进 20 ms 仿真，仍未达到实时。4 次轴向迭代的运动末态偏差约 16.3 mm，因此保留 6 次。
严格的全程 0.16 mm 对照曾未通过，不能把本版本描述为任意轨迹误差均小于 0.16 mm。
这些数据来自旧 2 mm MPM 配置；测量范围与通过/失败结果见[历史基线总结](docs/debugging_summary.md#mpm-baseline)。

## 安装与构建

项目使用 Python 3.10、ManiSkill2 0.5.3、SAPIEN 2.2.2 和配套的 CUDA Warp。刚体场景需要 Vulkan；MPM 线缆还需要 NVIDIA CUDA GPU；Rope-Actor 物理由 CPU 执行。安装脚本会编译 Warp，并从环境变量、PATH 和常见 CUDA 安装路径查找工具链；可用 `MANISKILL_CUDA_PATH` 显式指定。

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh
colcon build --symlink-install --packages-up-to dual_fr3_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export MANISKILL_PYTHON="$PWD/.venv/bin/python"
```

脚本将依赖安装到工作区 `.venv`，源码和构建依赖放在 `.deps`。CUDA 路径不同、旧环境迁移或 Python 版本问题，见[环境安装与验证](docs/setup.md)。不需要激活虚拟环境；每个新终端先加载 ROS 和工作区，再设置 `MANISKILL_PYTHON`。

## 使用方法

完整 MoveIt/RViz 入口位于 `dual_fr3_moveit_config`，每次选择一个场景运行。

### 普通机器人

```bash
ros2 launch dual_fr3_moveit_config maniskill.launch.py maniskill_scene:=robot
```

在 RViz 中选择 `left_fr3_arm`、`right_fr3_arm` 或 `dual_fr3_arms`，使用 **Plan & Execute** 规划并执行。

### 独立 USB 线缆演示

```bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py cable_solver:=rope_actor
```

启动时创建世界临时定位的动态 USB 和可选线缆；夹爪可以接触闭合、解除定位并重新张开。`load_cable:=false` 只创建 USB，不初始化线缆后端。模型、参数和重置方法见 [USB 线缆说明](docs/usb_cable.md)。

### MTC 准备与走线

先构建任务包：

```bash
colcon build --symlink-install --packages-up-to dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=true execute:=true
```

USB-only 快速抓持调试：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=false \
  execute:=true preparation_interactive:=false
```

省略 `cable_solver` 仍使用 MPM；其本阶段验收延后。

此入口选择 `trunking_cable` 场景：在机械臂接近前按关键点准备目标定位 USB/线缆，再张开接近，再接触闭合、解除定位并验证稳定；验证成功才更新规划附着体、下降和走线。默认需要终端确认；自动仿真可加 `preparation_interactive:=false`。

MTC 当前默认原始线槽 / 2 mm 线缆，两侧孔中心均沿各自 TCP 局部 +Z 偏移 2.4 mm；左右孔均检查初始直线覆盖。
复用已通过的简化线槽 / 2 mm 基线时，在上面的命令中添加：

```text
cable_config:="$PWD/src/dual_fr3_maniskill/config/trunking_cable_simplified_2mm.yaml"
```

此配置保持 2 mm 线径，使用新版 304 面简化碰撞线槽，显示采用当前选择的原始 CAD
（`scene.trunking_visual_mesh: original`）；模型尺寸变化见
[线槽说明](../dual_fr3_moveit_config/docs/trunking_mesh.md)。

日常 Rope-Actor 默认使用固定 500 Hz 快速模式。需要更细的接触时间分辨率时，
在相同启动命令中指定 `cable_config:="$PWD/src/dual_fr3_maniskill/config/trunking_cable_simplified_2mm_precise.yaml"`。
两套参数、速度与验证范围见[性能记录](docs/rope_performance.md)。

左端连接 USB；Rope-Actor 右孔使用真实网格接触与摩擦，MPM 使用理想滑孔。
`execute:=false` 只规划；`load_cable:=false` 不创建线缆，但继续原双臂准备、下降和后续 MTC 运动，仅作为 USB 搬运调试；`maniskill_cable:=false` 关闭整个 USB/线缆场景。详见 [MTC 线缆接口](docs/mtc_cable.md)。

## 常用设置

末端与手指作用力默认发布到 `/maniskill/forces`，可用的局部力/力矩另提供
`WrenchStamped` 话题；支持 JSONL 记录。数据来源、缺失分量、基座配置与查看命令见
[末端与手指作用力采集](docs/forces.md)。

| 设置 | 用途 |
| --- | --- |
| `maniskill_python:=/绝对路径/.venv/bin/python` | 覆盖物理桥接解释器 |
| `maniskill_viewer:=false use_rviz:=false` | 关闭两个窗口；仍需 Vulkan |
| `cable_solver:=mpm` / `cable_solver:=rope_actor` | 选择线缆模型 |
| `load_cable:=true` / `load_cable:=false` | 正常 USB+线缆 / USB-only 抓持调试 |
| `cable_config:=/绝对路径/cable.yaml` | 覆盖线缆材料、尺寸与布局 |
| `maniskill_config:=/绝对路径/simulation.yaml` | 覆盖桥接频率、容差等参数 |

`maniskill_config` 由 MoveIt 的 ManiSkill 入口提供。机器人场景默认物理 / 控制 / 发布频率为 500 / 100 / 50 Hz，线缆场景为 500 / 50 / 25 Hz。

本包的 `sim.launch.py` 只启动物理桥接，需要调用者提供 `robot_description` 和 `robot_description_semantic`；通常直接使用上面的完整入口。

## 检查与文档

基础依赖和刚体检查：

```bash
.venv/bin/python -m pip check
.venv/bin/python src/dual_fr3_maniskill/scripts/check_simulation.py
```

- [环境安装与验证](docs/setup.md)：安装细节、离线测试、GPU 和 ROS 集成检查。
- [ROS 接口与仿真模型](docs/interfaces.md)：动作、状态反馈、时间和模型边界。
- [USB 线缆说明](docs/usb_cable.md)：独立演示、材料与接触参数。
- [MTC 线缆接口](docs/mtc_cable.md)：延迟生成、夹持限制和场景更新。
- [MPM 性能与验证](docs/mpm_performance.md)：优化机制、专项检查及已知误差边界。
- [线缆建模方式](docs/cable_backends.md)：后端选择、参数、约束和测试。
- [测试入口](test/README.md)：离线、CUDA、历史实现对照的范围和命令。
- [调试总结](docs/debugging_summary.md)：历史问题、修复结论、运行摘要和清理范围。

两种线缆模型均为实验模型，材料尚未按真实电缆标定。柔性线缆不参与 MoveIt 避障。
左端为真实接触夹持，定位释放后靠实际接触保留 USB；MPM 右孔为理想滑孔，Rope-Actor 右孔由实际手指网格碰撞和摩擦导向。

## Git 同步与本地产物

工作区根目录不是 Git 仓库。分别检查 `src/dual_fr3_maniskill`、
`src/dual_fr3_moveit_config` 与 `src/dual_fr3_trunking_mtc` 的源码、配置、测试和文档改动。

历史实验结论已合并到[调试总结](docs/debugging_summary.md)，录制配置与关键指标保存在
[精简运行记录](docs/debugging_runs.json)。原始调试目录、截图、逐子步轨迹及一次性实验脚本已清理。
正式诊断工具、`test/data/` 中的冻结接触样本和 `test/reference/` 中的参考实现继续随仓库保存。

新调试输出放在已忽略的 `artifacts/` 或 `/tmp`；需要录制时显式设置 `cable_trace_dir`。
`.venv`、`.deps`、`build`、`install`、`log` 和 Python 缓存由本地重新生成，不应上传。
重新安装环境请使用本仓库安装脚本，不复制另一台电脑编译出的 Warp 二进制。
