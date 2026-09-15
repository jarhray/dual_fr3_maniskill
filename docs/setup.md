# 环境安装与验证

本页是三个双 FR3 包共用的 ManiSkill 环境说明。场景启动命令见 [README](../README.md)。所有命令从工作区根目录执行。

## 环境要求

- ROS 2 Humble 与系统 Python 3.10，已安装工作区需要的 MoveIt / Franka 依赖。
- 可用的 Vulkan 渲染环境。当前 SAPIEN 即使关闭交互窗口也会创建渲染器。
- MPM 线缆仿真需要 NVIDIA CUDA GPU；编译 Warp 需要 CUDA 工具链。本机已使用 CUDA 12.8 完成编译，其他版本需以实际验证结果为准。
- 安装时需联网下载 Python 依赖和固定版本源码。

刚体和 Rope-Actor 线缆使用 CPU PhysX 求解；MPM、纤维和对应接触计算使用 CUDA。
依赖安装仍沿用下面的 ManiSkill2 环境；选择 Rope-Actor 时不初始化 MPM/CUDA 求解器。`maniskill_viewer:=false` 仅关闭窗口，不取消 Vulkan 要求。

## 安装 Python 与 Warp 依赖

```bash
source /opt/ros/humble/setup.bash
bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh
```

安装脚本依次使用 `MANISKILL_CUDA_PATH`、`CUDA_HOME`、`CUDA_PATH`、PATH 中的 `nvcc`，
再检查 `/usr/local/cuda` 和 `/usr/local/cuda-11.8`。设置了显式路径但不可用时会报错。
CUDA 位于其他目录时，在运行脚本前设置实际路径：

```bash
export MANISKILL_CUDA_PATH=/你的/CUDA工具链路径
```

[安装脚本](../scripts/setup_maniskill2.sh)创建隔离的 `.venv`，将 ManiSkill2 0.5.3 源码放入 `.deps/ManiSkill-0.5.3`，安装 [requirements-maniskill2.txt](../requirements-maniskill2.txt)，并编译该版本自带的 Warp。SAPIEN 固定为 2.2.2；导入名为 `mani_skill2`、`sapien.core`。配套 Warp 不能直接用通用 `warp-lang` 替换。

若已有虚拟环境不符合脚本的隔离检查，会先保存为 `.venv.maniskill3-backup-时间戳`。依赖快照写入 `.deps/maniskill2-installed.txt`；脚本不修改系统 Python、显卡驱动或全局 CUDA 链接。

仅通过 pip 安装 ManiSkill2 时，可能只有 Warp 源码而没有 `warp.so`。已加载 ROS 和工作区后，
可单独补齐当前虚拟环境的 Warp，既适用于 pip 安装，也适用于源码可编辑安装：

```bash
.venv/bin/python -m dual_fr3_maniskill.warp_setup
# 切换 CUDA 工具链或修复旧的二进制时，显式重新编译：
.venv/bin/python -m dual_fr3_maniskill.warp_setup --cuda-path /实际/CUDA路径 --force
```

构建工具会将旧 Warp 的 Linux 驱动加载名称修正为 `libcuda.so.1`，兼容仅安装运行时库、
没有 64 位 `libcuda.so` 开发链接的电脑；同时移除 NVRTC 输入中未使用的主机端内核启动包装，
避免 CUDA 12.8 将它们按设备端动态启动解析而报编译错误。实际内核仍由 Warp 的 `cuLaunchKernel` 执行。
修正仅作用于当前环境中的 Warp 源码和编译产物。
构建前会校验 ManiSkill2 0.5.3 版本及 Warp 是否来自同一安装目录。补丁和编译先在临时目录完成，
新库通过加载检查后才替换当前文件；编译或加载失败保留原有安装，替换过程异常时回滚已替换的文件。
重复执行还会检查已有库是否可加载。此检查不代替 GPU 物理验证；切换电脑或 CUDA 工具链时仍应使用 `--force` 重编译。
MTC 线缆场景会在启动时检查 Warp/CUDA 初始化，依赖失败时不会等到机械臂准备完成后才报错。

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

### MPM 性能与数值对照

当前 MPM 配置、性能基线、Graph 与碰撞缓存说明及专项检查已整理到
[MPM 性能与验证](mpm_performance.md)。整理状态及后续细杆接入边界见
[线缆后端交接说明](cable_backends.md)。

## 常见环境问题

| 现象 | 检查方向 |
| --- | --- |
| 无法导入 `rclpy` 或项目包 | 先加载 ROS 和工作区，核对 Python 3.10 与 `MANISKILL_PYTHON` |
| 找不到 `nvcc` | 设置 `MANISKILL_CUDA_PATH`，确认工具链目录下有 `bin/nvcc` |
| Warp 缺少共享库 | 运行 `.venv/bin/python -m dual_fr3_maniskill.warp_setup`，检查报错路径对应的 `warp.so` |
| `Could not open libcuda.so` | 用上述构建工具加 `--force` 重编译，应用 `libcuda.so.1` 兼容修正 |
| `cudaErrorNoDevice` | 当前进程未发现可用 CUDA 设备，线缆物理检查无法运行 |
| `vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent` | 检查 NVIDIA Vulkan ICD 是否被 SAPIEN 的驱动搜索遗漏，见下文 |
| 关闭窗口后仍无法初始化 | 检查 Vulkan，关闭 viewer 不会移除渲染器依赖 |
| 状态源冲突 | 关闭同一 ROS 域中的其他仿真；观察终端与启动终端使用相同的域 |

SAPIEN 2.2.2 在未设置 `VK_ICD_FILENAMES` 时只扫描 `/usr/share/vulkan/icd.d`，
可能漏掉 `/etc/vulkan/icd.d/nvidia_icd.json`。这时即使 `nvidia-smi` 和
`vulkaninfo --summary` 能看到 NVIDIA GPU，SAPIEN 仍可能报 `ErrorExtensionNotPresent`。
本项目通过 `sapien_compat.py` 导入 SAPIEN，并恢复导入前的 `VK_ICD_FILENAMES`，
让系统 Vulkan loader 使用自己的驱动搜索规则。兼容处理不指定显卡品牌、GPU 编号或
驱动文件路径，适用于 ROS 启动及本项目的 Python 仿真检查脚本。
用户设置的 `VK_ICD_FILENAMES`（含多文件列表）、`VK_DRIVER_FILES`、
`VK_ADD_DRIVER_FILES` 和 XDG 搜索路径均保留；ROS launch 中设置的环境变量也会正常传给桥接进程。
驱动搜索与变量优先级见 [Vulkan loader 官方说明](https://github.com/KhronosGroup/Vulkan-Loader/blob/main/docs/LoaderDriverInterface.md#driver-discovery)。

此处理解决的是驱动发现差异；驱动本身仍需可用，并支持 SAPIEN 所需的 Vulkan 扩展。
线缆的 CUDA 要求不变。其他 Python 程序若复用本项目的物理模块，应先导入本项目模块，
再导入 ManiSkill/SAPIEN，以便在首次导入时保留原始环境。

单独诊断原始 SAPIEN 或使用尚未更新的代码时，可在同一终端指定驱动文件后重试：

```bash
# 使用本机实际存在的 NVIDIA ICD 路径。
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
.venv/bin/python -c 'import sapien.core as s; r = s.SapienRenderer(offscreen_only=True); print("Vulkan OK")'
```

若已设置 `VK_DRIVER_FILES`，需同时核对它：Vulkan loader 会优先使用该变量。
无需为这个路径问题重装 CUDA 或升级 SAPIEN。
