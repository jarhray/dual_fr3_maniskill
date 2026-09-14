# 双 FR3 ManiSkill 仿真

`dual_fr3_maniskill` 使用 ManiSkill2 / SAPIEN 2 执行双 FR3 刚体仿真，并提供可选的 MPM 线缆场景。ROS 桥接接收机械臂和夹爪动作，发布实测关节状态与仿真时钟，供 MoveIt、MTC 和 RViz 使用。

## 当前状态（2026-09-14）

MPM 实现已整理为后续对照基线，本阶段暂停继续优化。当前只有 MPM 线缆后端；
细杆模型及两种求解器的选择参数尚未实现，接入位置和兼容要求见[线缆后端交接说明](docs/cable_backends.md)。
`simulation_backend:=maniskill` 选择的是机器人仿真环境，不是 MPM/细杆求解器。

MTC 默认保持 1.5 m 长、2 mm 直径、1 mm 采样，MPM 5000 Hz、轴向迭代 6 次、接触迭代 2 次。
CUDA Graph 与 GPU 网格检查默认启用；右手导向和碰撞投影也在 GPU 执行。
材料、采样、迭代和穿透限制的默认值见 [trunking_cable.yaml](config/trunking_cable.yaml)。

RTX 3090 上的独立短测约为 89 ms/控制步，包含托板、线槽接触的区间约为 262 ms/控制步；
每步推进 20 ms 仿真，仍未达到实时。4 次轴向迭代的运动末态偏差约 16.3 mm，因此保留 6 次。
严格的全程 0.16 mm 对照曾未通过，不能把本版本描述为任意轨迹误差均小于 0.16 mm。
测量范围、原始通过/失败结果见[已归档基线](docs/benchmarks/mpm_20260912/README.md)。

## 安装与构建

项目使用 Python 3.10、ManiSkill2 0.5.3、SAPIEN 2.2.2 和配套的 CUDA Warp。刚体场景需要 Vulkan；线缆场景还需要 NVIDIA CUDA GPU。安装脚本会编译 Warp，并从环境变量、PATH 和常见 CUDA 安装路径查找工具链；可用 `MANISKILL_CUDA_PATH` 显式指定。

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
ros2 launch dual_fr3_moveit_config usb_cable.launch.py
```

启动时将 USB 固定在左 TCP，并生成默认 1.5 m 长、3.5 mm 直径的线缆。移动左臂可观察线缆响应；左夹爪固定，动作请求会被拒绝。模型、参数和重置方法见 [USB 线缆说明](docs/usb_cable.md)。

### MTC 准备与走线

先构建任务包：

```bash
colcon build --symlink-install --packages-up-to dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill execute:=true
```

此入口选择 `trunking_cable` 场景：完整任务预检通过后执行准备动作，两次夹爪闭合成功才生成 USB 和线缆，随后下降并走线。默认需要终端确认；自动仿真可加 `preparation_interactive:=false`。

MTC 场景默认线径为 2 mm，以适配研究手指孔；左端固定，右 TCP 提供允许轴向滑动的理想孔约束。`execute:=false` 只规划、不生成线缆，`maniskill_cable:=false` 关闭线缆。详见 [MTC 线缆接口](docs/mtc_cable.md)。

## 常用设置

| 设置 | 用途 |
| --- | --- |
| `maniskill_python:=/绝对路径/.venv/bin/python` | 覆盖物理桥接解释器 |
| `maniskill_viewer:=false use_rviz:=false` | 关闭两个窗口；仍需 Vulkan |
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
- [线缆后端交接说明](docs/cable_backends.md)：现有代码分工、耦合时序及细杆接入要求。
- [测试入口](test/README.md)：离线、CUDA、历史实现对照的范围和命令。

线缆采用 MPM、轴向纤维和刚体接触的混合实验模型，材料尚未按真实电缆标定。柔性线缆不参与 MoveIt 避障，固定夹持与滑孔也不等同于真实摩擦夹持。

## 检查与备份

本轮构建、CUDA 对照以及跨包回归中的两个 MTC 失败项，见[整理检查记录](docs/benchmarks/cleanup_20260914/README.md)。

工作区根目录不是 Git 仓库。检查 `src/dual_fr3_maniskill` 的源码、配置、测试和文档，
并分别检查 `src/dual_fr3_moveit_config` 与 `src/dual_fr3_trunking_mtc` 的改动。
本轮相关的跨包修正还包括 `dual_fr3_moveit_config/scripts/check_usb_cable.py` 中的重置验证。

必要的历史参考源码位于 `test/reference/`，关键 JSON 结果位于 `docs/benchmarks/`，可随仓库备份。
完整调试日志与粒子快照仍在原工作区 `artifacts/`，不参与运行或构建；如需保留所有原始状态，应另行归档该目录。
`.venv`、`.deps`、`build`、`install`、`log` 和 Python 缓存由本地重新生成，不应作为跨电脑运行环境上传。
重新安装环境请使用本仓库安装脚本，不复制另一台电脑编译出的 Warp 二进制。
