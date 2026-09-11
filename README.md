# 双 FR3 ManiSkill 仿真

`dual_fr3_maniskill` 使用 ManiSkill2 / SAPIEN 2 执行双 FR3 刚体仿真，并提供可选的 MPM 线缆场景。ROS 桥接接收机械臂和夹爪动作，发布实测关节状态与仿真时钟，供 MoveIt、MTC 和 RViz 使用。

## 安装与构建

项目使用 Python 3.10、ManiSkill2 0.5.3、SAPIEN 2.2.2 和配套的 CUDA Warp。刚体场景需要 Vulkan；线缆场景还需要 NVIDIA CUDA GPU。当前安装脚本会编译 Warp，因此安装时也需要 CUDA 工具链，默认路径为 `/usr/local/cuda-11.8`。

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

线缆采用 MPM、轴向纤维和刚体接触的混合实验模型，材料尚未按真实电缆标定。柔性线缆不参与 MoveIt 避障，固定夹持与滑孔也不等同于真实摩擦夹持。
