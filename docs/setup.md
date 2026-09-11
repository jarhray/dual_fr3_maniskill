# 环境安装与验证

本页是三个双 FR3 包共用的 ManiSkill 环境说明。场景启动命令见 [README](../README.md)。所有命令从工作区根目录执行。

## 环境要求

- ROS 2 Humble 与系统 Python 3.10，已安装工作区需要的 MoveIt / Franka 依赖。
- 可用的 Vulkan 渲染环境。当前 SAPIEN 即使关闭交互窗口也会创建渲染器。
- 线缆仿真需要 NVIDIA CUDA GPU；安装脚本需要 CUDA 11.x 编译工具链，默认使用 `/usr/local/cuda-11.8`。
- 安装时需联网下载 Python 依赖和固定版本源码。

刚体使用 CPU 求解；线缆的 MPM、纤维和接触计算使用 CUDA。`maniskill_viewer:=false` 仅关闭窗口，不取消 Vulkan 要求。

## 安装 Python 与 Warp 依赖

```bash
source /opt/ros/humble/setup.bash
bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh
```

CUDA 位于其他目录时，在运行脚本前设置实际路径：

```bash
export MANISKILL_CUDA_PATH=/你的/CUDA工具链路径
```

[安装脚本](../scripts/setup_maniskill2.sh)创建隔离的 `.venv`，将 ManiSkill2 0.5.3 源码放入 `.deps/ManiSkill-0.5.3`，安装 [requirements-maniskill2.txt](../requirements-maniskill2.txt)，并编译该版本自带的 Warp。SAPIEN 固定为 2.2.2；导入名为 `mani_skill2`、`sapien.core`。配套 Warp 不能直接用通用 `warp-lang` 替换。

若已有虚拟环境不符合脚本的隔离检查，会先保存为 `.venv.maniskill3-backup-时间戳`。依赖快照写入 `.deps/maniskill2-installed.txt`；脚本不修改系统 Python、显卡驱动或全局 CUDA 链接。

## 构建 ROS 包

构建包含 MTC 的完整使用环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export MANISKILL_PYTHON="$PWD/.venv/bin/python"
```

只使用机器人或 USB 独立场景时，将目标包改为 `dual_fr3_moveit_config`。相关依赖已构建、只更新这三个包时可用 `--packages-select dual_fr3_maniskill dual_fr3_moveit_config dual_fr3_trunking_mtc`。

ROS 节点使用系统 Python，物理桥接单独使用 `.venv/bin/python`。无需激活虚拟环境，但必须先加载 ROS 与工作区，使该解释器可导入 `rclpy` 和项目包。

若曾用其他 Python 版本配置 CMake，在构建时增加 `--cmake-clean-cache`。检查安装日志中的解释器和模块安装目录，避免 Conda Python 与 Humble 的 Python 3.10 混用。

未设置 `MANISKILL_PYTHON` 时，启动文件默认查找**当前目录**的 `.venv/bin/python`。从其他目录启动应显式指定 `maniskill_python:=/工作区绝对路径/.venv/bin/python`。

## 验证层次

按依赖检查、离线测试、物理仿真和 ROS 集成的顺序验证。各层检查的范围不同，结果以实际运行输出为准。

### 依赖与离线测试

```bash
.venv/bin/python -m pip check
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test
```

离线测试覆盖配置、几何、启动场景、调度和 MTC 逻辑，不调用 GPU 物理。个别测试仍依赖 ROS、MTC 绑定及已安装的机器人模型。失败应按当前源码定位，不沿用旧文档中的通过数量。

### 刚体物理与渲染

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_simulation.py
.venv/bin/python src/dual_fr3_maniskill/scripts/check_simulation.py \
  --render-output /tmp/dual_fr3_maniskill_scene.png --fixture-closeup
```

检查实际关节运动、TCP 与 URDF 正运动学的一致性；第二条保存安装板与线槽近景，需要 Vulkan。

### ROS 动作与完整 MTC

```bash
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode actions
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode mtc
```

脚本固定使用 `ROS_DOMAIN_ID=87`、`ROS_LOCALHOST_ONLY=1`，自行启动并清理仿真。该域内应无另一套机器人环境。加 `--viewer` 打开 RViz 和 ManiSkill，`--timeout` 调整 MTC 检查时限。

`actions` 检查普通机器人场景中的双臂并发、夹爪、取消、非法目标、容差和 TF；`mtc` 使用当前默认任务及默认线缆开关执行，因此需要 CUDA。日志分别保存为 `/tmp/dual_fr3_maniskill_actions.log` 和 `/tmp/dual_fr3_maniskill_mtc.log`。

### CUDA 与线缆

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_mpm.py
PYTHONPATH=src/dual_fr3_maniskill .venv/bin/python \
  src/dual_fr3_maniskill/test/check_contacts_cuda.py
bash src/dual_fr3_moveit_config/scripts/validate_usb_cable.sh \
  --steps 25 --render-output /tmp/usb_cable_demo.png
bash src/dual_fr3_moveit_config/scripts/validate_usb_cable_ros.sh --viewer
```

`check_mpm.py` 使用官方 `Hang-v0` 参考场景检查 CUDA MPM，不验证本项目电缆材料。后面三项分别检查接触内核、USB 线缆物理和 MoveIt/ROS 集成，细节见 [USB 线缆说明](usb_cable.md)。首次运行可能需要编译 CUDA 内核。

## 常见环境问题

| 现象 | 检查方向 |
| --- | --- |
| 无法导入 `rclpy` 或项目包 | 先加载 ROS 和工作区，核对 Python 3.10 与 `MANISKILL_PYTHON` |
| 找不到 `nvcc` | 设置 `MANISKILL_CUDA_PATH`，确认工具链目录下有 `bin/nvcc` |
| Warp 缺少共享库 | 重新运行安装脚本，检查 `.deps/ManiSkill-0.5.3/warp_maniskill/warp/bin/warp.so` |
| `cudaErrorNoDevice` | 当前进程未发现可用 CUDA 设备，线缆物理检查无法运行 |
| 关闭窗口后仍无法初始化 | 检查 Vulkan，关闭 viewer 不会移除渲染器依赖 |
| 状态源冲突 | 关闭同一 ROS 域中的其他仿真；观察终端与启动终端使用相同的域 |
