# USB + MPM 线缆场景

在当前双 FR3、工作台、底板和线槽场景中，用左夹爪固定 USB 头，通过 RViz 的 **Plan & Execute** 移动左臂，观察 ManiSkill2 MPM 线缆。无需启动原来的 MTC 或 Gazebo；此场景不改变原有 `simulation_backend` 的默认值。

线缆实现已合并到 `dual_fr3_maniskill/cable/`，场景扩展位于
`dual_fr3_maniskill/scenes/usb_cable.py`。原 `dual_fr3_usb_cable_demo` ROS 包已移除。
完整启动编排在 `dual_fr3_moveit_config`，物理仿真包不反向依赖 MoveIt 配置包。
原 `/usb_cable_demo/*` 话题、服务和 USB link 名保持不变，便于继续使用已有 RViz 配置和检查工具。

## 启动

本工作区已经构建后可直接执行：

```bash
bash src/dual_fr3_moveit_config/scripts/run_usb_cable_demo.sh
```

快捷脚本默认使用隔离的 `ROS_DOMAIN_ID=88`（若终端已设置此变量，则沿用终端值），启动 RViz 和 ManiSkill。使用另一个终端观察话题或调用 reset 时，也需先 `export ROS_DOMAIN_ID=88`。关闭任一仿真窗口或在启动终端按 Ctrl+C 会结束 demo。

在工作区根目录执行（现有 `.venv` 已安装 ManiSkill2 0.5.3、SAPIEN 2.2.2 和 CUDA Warp fork）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_fr3_maniskill dual_fr3_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_INSTALL_DIR=local/lib/python3.10/dist-packages
source install/setup.bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py maniskill_python:="$PWD/.venv/bin/python"
```

上面的 `usb_cable.launch.py` 是便利入口，复用统一入口：

```bash
ros2 launch dual_fr3_moveit_config maniskill.launch.py maniskill_scene:=usb_cable
```

`maniskill_scene:=robot` 为原普通双臂场景。也可通过 `demo.launch.py`
传入 `simulation_backend:=maniskill maniskill_scene:=usb_cable`。
三种入口都只启动一个物理桥接，仿真、MoveIt、TF、RViz 使用同一份最终 URDF/SRDF。
`cable_config:=...` 控制 USB 和线缆参数，`maniskill_config:=...` 覆盖 ROS 桥接参数。
线缆场景默认使用 `config/simulation_usb_cable.yaml`（控制 50 Hz、物理 500 Hz、发布 25 Hz），
普通场景沿用 `config/simulation.yaml`（控制 100 Hz、物理 500 Hz、发布 50 Hz）。
模型构造不导入 ManiSkill/SAPIEN/Warp；只有运行线缆仿真时才加载 CUDA 相关实现。

此次整理保留了固定 USB 和禁止左夹爪动作的实验语义，尚未把线缆场景接入 MTC 任务入口。
动态线缆也没有加入 MoveIt 的规划避障模型。

需要可用的 NVIDIA CUDA GPU 和 Vulkan 显示环境。启动时会编译自定义固定点、网格边界和纤维内核。先关闭其他发布 `/joint_states`、`/clock` 的机器人仿真。明确指定 Python 3.10 可避免当前终端的 Conda Python 影响构建目录。

RViz 默认选择 `left_fr3_arm`。拖动末端交互标记，点击 **Plan & Execute**，从小幅运动开始。拖动标记只修改规划目标，执行后的实际关节反馈才驱动 RViz 机器人。橙色线来自 MPM 粒子反馈。默认速度和加速度比例均为 0.05；MPM 可能明显慢于实时，请按仿真时间观察。

左夹爪每个手指关节初始位置为 0.0037 m，总开口为 7.4 mm，比 USB 的 7.6 mm 抓取宽度略收紧 0.2 mm。USB 原点通过固定 URDF 关节与 TCP 原点重合。此处采用固定安装，不验证摩擦夹持或滑移，左夹爪动作请求会被拒绝。右臂、右夹爪沿用现有执行接口。

## 几何与参数

`dual_fr3_maniskill/config/usb_cable.yaml` 集中配置 USB、线缆和网格参数；修改后重新启动，也可用 `cable_config:=/绝对路径/配置.yaml`。

- 使用 `temp/USB1.stl` 的副本 `meshes/USB1.stl`，网格已是米单位，缩放为 1。沿 X/Y/Z 的尺寸约为 7.6 × 37.9 × 14.4 mm。质量仍为配置值，惯性用新网格的包围盒近似。
- 保留 USB 本地坐标：插头朝 +Y，抓取中心 `(0, 0, 0)` m，接线中心 `(0, -0.02, 0)` m，线从尾部向本地 −Y 延伸。初始左腕姿态令 USB +Y 对齐世界 +Y；移动后方向随夹爪旋转。
- 当前配置长度为 1.5 m、直径 3.5 mm，密度 1200 kg/m³，总质量约 17.3 g。改为 `length: 3.0` 时质量约 34.6 g。圆截面每层 7 个材料点；体积和质量按实际圆柱计算。
- USB 尾部前 7 mm 的线缆材料点固定在 USB 坐标系，固定点反作用与 MPM 接触力反馈到 SAPIEN 机器人。USB 质量暂设 15 g。
- 默认 `initial_layout: table_spiral`：线从 USB 尾部向本地 −Y 出线，随后弯向工作台，盘放的线圈从桌面上方约 8 cm 处下落。初态盘放在两个机器人底座的 +X 侧，避开原初态与右底座的重叠。盘放仅是初态，运动过程中没有固定在线圈路径上。重置会根据当前 USB 位姿重新生成所配置长度的初态，并检查是否与场景重叠。
- `initial_layout: straight` 可切换为水平展开后的坠落压力测试，计算域和运动幅度会明显增大。复用现有场景碰撞几何，包括线槽的非凸三角网格；不额外添加地板。

这是 **ManiSkill2 体积 MPM + 轴向 XPBD 纤维约束 + 独立刚体接触约束的混合实验模型**，没有使用刚体链。MPM 使用原始粒子与网格传递、弹性应力内核；适配层修正移动网格的边界留白，并施加 USB 固定端条件与反作用力。刚体接触由本包 `contacts.py` 处理，替代原来的网格速度接触。

单纯使用粗网格体积 MPM 时，这根细线在测试中出现固定端附近材料点脱离。因此每个截面的 7 个材料点分别通过纵向纤维约束连接相邻截面，补充轴向刚度，保持连续性。此处不是直接调用 ManiSkill2 现成的“线缆 API”，也不应将结果当成纯 MPM 的验证结论。当前配置为 `young_modulus: 300000` Pa、`poisson_ratio: 0.3`、`axial_young_modulus: 30000000` Pa、`axial_iterations: 6`；这些均为演示参数，尚未标定真实 USB 电缆的弯曲刚度、最小弯曲半径、滞后或导体结构。

默认网格 6 mm，比 3.5 mm 线径粗，适合先验证执行、固定点与定性响应；体积形变和弯曲精度有限。刚体接触检查独立于此网格。网格按粒子包围盒分配双缓冲，超过 1600 万格时可自动降低精度（最大 15 mm），并在日志与 diagnostics 中显示。更精细的实验需减小 `grid_spacing`、提高 `frequency` 并评估显存；关闭 `adaptive_resolution` 可保持固定精度，超预算时会明确报错。刚度提高时也须减小 MPM 时间步。

MoveIt 对机器人、USB 和静态场景规划碰撞检查；动态线缆仅在仿真中参与物理接触，未作为 MoveIt 的避障物体，规划不会主动避开线缆。本次接触修复针对线缆与刚体，自接触仍依赖粗网格 MPM，没有显式线段自碰撞约束，不能据此判断打结、紧密缠绕或实际穿线间隙。

## 刚体穿模修复

原实现存在三个问题：网格节点的网格查询半径只有约 0.58 mm，容易漏掉薄壁和深穿透；XPBD 拉伸约束修改位置后没有再检查接触；初态末端与右底座重叠。此外，Warp 0.3.1 在多个等距离三角形附近的内外判定会误把线槽空腔当成实体。单纯增大接触距离不能解决这些问题。

现在每轮纤维约束后均执行截面球体接触，半径按 1.75 mm 物理半径和相邻截面间距计算，保守覆盖截面之间的线段。默认截面间距下额外包络约 0.21 mm，接触位置会略偏保守。保守推进检查从上次安全位置到预测位置的路径，防止一步跨过薄壁。网格最近点查询覆盖实体内部，内外判定采用三条斜向射线投票。输入碰撞网格要求封闭且朝向一致，保留线槽的真实非凸空腔。

接触同时修正材料点速度与 APIC 局部速度场，使用无弹性法向响应和库仑摩擦，并将反作用冲量传回 SAPIEN。SAPIEN 每步更新机械臂后再做一次相对运动检查。旋转运动使用相邻刚体位姿间的相对运动弦线近似，适用于本 demo 的小时间步和慢速运动；它不是任意大幅瞬时旋转的精确 CCD。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `cable.contact_margin` | 0.00002 m | 接触后保留约 0.02 mm 间隙，减小边界抖动；增大时更容易出现悬空感。 |
| `cable.contact_iterations` | 2 | 每轮纤维约束后的碰撞遍历次数；增大有助于多表面夹角接触，但会变慢。 |
| `cable.penetration_tolerance` | 0.0001 m | 穿透监测的暂停阈值，不是接触厚度；调大只会放宽报警。 |
| `mpm.marker_stride` | 1 | RViz 保留所有截面；跳点可能把绕过墙角的曲线画成穿墙弦线。 |

固定端只有与 USB、左夹爪安装部件的接触被排除；固定端撞到线槽等其他物体仍会报错。改变机器人位姿后重置，若新的初态路径穿过物体，reset 会拒绝该状态。

## 反馈、重置与验证

无需 GPU 的模型和启动回归测试（先 source ROS 和工作区，使用项目 venv）：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test/test_backend_launch.py
```

这些检查验证线缆几何、机器人模型、单一桥接、配置传递和模型构造不加载 GPU 库；
实际物理和 ROS 执行检查使用下面的命令，需要可用的 CUDA/Vulkan 环境。

```bash
ros2 topic echo /usb_cable_demo/diagnostics
ros2 service call /usb_cable_demo/reset std_srvs/srv/Trigger '{}'
bash src/dual_fr3_moveit_config/scripts/validate_usb_cable.sh --steps 25 --render-output /tmp/usb_cable_demo.png
bash src/dual_fr3_moveit_config/scripts/validate_usb_cable_ros.sh --viewer
PYTHONPATH=src/dual_fr3_maniskill .venv/bin/python src/dual_fr3_maniskill/test/check_contacts_cuda.py
```

重置保留机器人位姿，动作执行期间拒绝重置。`/usb_cable_demo/markers` 发布世界坐标系下的实际线缆中心线；两侧可视化均按实际截面中心和 3.5 mm 线径绘制。`diagnostics` 提供固定点误差、中心线长度、相邻截面最大间距、网格精度、固定点反作用力、粒子包围盒、运行速度，以及 `max_rigid_penetration_m` 和 `contacted_bodies`。穿透监测在每个控制步结束时沿线段采样，超过阈值会指出碰撞物体。若发现数值失稳、轴向断开或超出网格预算，demo 同样中止当前动作、暂停物理并发布 ERROR；可用 reset 恢复。

验证脚本使用隔离 ROS domain 88。物理测试检查重力、机械臂运动、固定点误差、材料连续性、穿透和重置；可通过 `--report-output /tmp/report.json` 保存结果。CUDA 专项测试覆盖跨越 1 mm 薄壁、深穿透、切向滑动、移动碰撞体、截面间穿透监测和实际线槽网格的空腔判定。ROS 测试启动完整场景，通过 MoveIt 的规划服务和 ExecuteTrajectory action 实际移动左臂，检查 USB/线缆/末端 TF 一致，并关闭它自己启动的进程。

## 本机验证记录

以下物理运行记录来自合并前的验证。此次包合并已通过构建和 49 项离线回归检查；
当前执行环境的 Warp 初始化返回 `cudaErrorNoDevice`，未重新运行 GPU 物理及 ROS 执行检查。

`USB1.stl` 当前配置已通过构建、16 项几何/配置测试、0.5 秒物理验证和 RViz + ManiSkill 的 Plan & Execute 集成验证。实测左夹爪总开口为 0.0074 m，USB 抓取原点与 TCP 的 TF 位置误差为 0，接线点与 RViz TF 的误差约 1.9e-7 m；穿透采样为 0 m，重置正常。安装近景、物理报告及 ROS 报告分别保存于 `temp/usb1_grip_closeup.png`、`temp/usb1_demo_report.json`、`temp/usb1_demo_ros_report.json`。

2026-09-10，RTX 3050 Laptop 4 GB，接触算法基线测试使用旧 USB 网格、1.5 m 线长：3 秒物理测试完成 15,000 个 MPM 子步、6,013 个材料点；每个控制步的线段穿透采样最大值为 0 m。最终中心线约 1.4941 m，相邻截面最大间距约 1.778 mm，固定点误差约 2.2e-8 m，USB 移动约 6 mm。报告与图像保存在工作区 `temp/usb_cable_contacts_report.json`、`temp/usb_cable_contacts_fixed.png`。

该次运行约用 147 秒墙钟时间，加入独立接触检查后比旧实现慢；打开两个 GUI 后还会变慢。这里的“0 m”是本次测试采样测得的结果，不代表任意轨迹、自接触或无限时长下的无穿透证明。

16 项几何/配置测试和 7 项 CUDA 接触专项检查通过。RViz 与 ManiSkill 同时启动的 ROS 集成测试通过：左臂 joint1 规划执行增加 0.04 rad，USB 固定点与 TF 误差约 2.4e-7 m，末端实测位置与 TF 误差约 0.023 mm；接触诊断记录到 `plate`、`trunking`，穿透采样为 0 m。固定夹爪拒绝打开和 reset 也通过，报告保存于 `temp/usb_cable_contacts_ros_report.json`。
