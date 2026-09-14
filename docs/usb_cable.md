# USB + MPM 线缆场景

独立 USB 场景在启动时把插头固定到左 TCP，通过 RViz 的 **Plan & Execute** 移动左臂，观察线缆响应。场景、材料与接触实现位于本包 `scenes/` 和 `cable/`，完整 MoveIt 启动入口位于 `dual_fr3_moveit_config`。

原 `dual_fr3_usb_cable_demo` 包已合并到本包，反馈接口仍保留 `/usb_cable_demo/*` 命名。

## 启动

先按[环境安装与验证](setup.md)准备依赖并构建工作区，然后可直接执行：

```bash
bash src/dual_fr3_moveit_config/scripts/run_usb_cable_demo.sh
```

快捷脚本默认使用 `ROS_DOMAIN_ID=88`（若终端已设置则沿用该值），启动 RViz 和 ManiSkill。观察话题或调用 reset 的终端需加载 ROS 和工作区，并使用相同的 `ROS_DOMAIN_ID` 与 `ROS_LOCALHOST_ONLY=1`。关闭任一仿真窗口或按 Ctrl+C 会结束演示。

也可在已加载 ROS 和工作区的终端直接启动，此方式沿用当前 ROS 域：

```bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py \
  maniskill_python:="$PWD/.venv/bin/python"
```

上面的 `usb_cable.launch.py` 是便利入口，复用统一入口：

```bash
ros2 launch dual_fr3_moveit_config maniskill.launch.py maniskill_scene:=usb_cable
```

`maniskill_scene:=robot` 为普通双臂场景。也可通过 `demo.launch.py`
传入 `simulation_backend:=maniskill maniskill_scene:=usb_cable`。
三种入口都只启动一个物理桥接，仿真、MoveIt、TF、RViz 使用同一份最终 URDF/SRDF。
`cable_config:=...` 控制 USB 和线缆参数，`maniskill_config:=...` 覆盖 ROS 桥接参数。
线缆场景默认使用 `config/simulation_usb_cable.yaml`（控制 50 Hz、物理 500 Hz、发布 25 Hz），
普通场景沿用 `config/simulation.yaml`（控制 100 Hz、物理 500 Hz、发布 50 Hz）。
模型构造不导入 ManiSkill/SAPIEN/Warp；只有运行线缆仿真时才加载 CUDA 相关实现。

本页的独立 USB 场景采用固定安装并禁止左夹爪动作。MTC 使用另一个 `trunking_cable` 场景，在准备夹持后生成线缆，见 [MTC 线缆接口](mtc_cable.md)。两种场景的柔性线缆均不参与 MoveIt 避障。

当前只实现 MPM 线缆。执行优化与验证见 [MPM 性能说明](mpm_performance.md)；
后续细杆模型及参数选择的接入边界见[后端交接说明](cable_backends.md)，该选择功能尚未实现。

需要可用的 NVIDIA CUDA GPU 和 Vulkan 显示环境。启动时会编译自定义固定点、网格边界和纤维内核。先关闭其他发布 `/joint_states`、`/clock` 的机器人仿真。

RViz 默认选择 `left_fr3_arm`。拖动末端交互标记，点击 **Plan & Execute**，从小幅运动开始。拖动标记只修改规划目标，执行后的实际关节反馈才驱动 RViz 机器人。橙色线来自 MPM 粒子反馈。默认速度和加速度比例均为 0.05；MPM 可能明显慢于实时，请按仿真时间观察。

左夹爪每个手指关节初始位置为 0.0037 m，总开口为 7.4 mm，比 USB 的 7.6 mm 抓取宽度略收紧 0.2 mm。USB 原点通过固定 URDF 关节与 TCP 原点重合。此处采用固定安装，不验证摩擦夹持或滑移，左夹爪动作请求会被拒绝。右臂、右夹爪沿用现有执行接口。

## 几何与参数

`dual_fr3_maniskill/config/usb_cable.yaml` 集中配置 USB、线缆和网格参数；修改后重新启动，也可用 `cable_config:=/绝对路径/配置.yaml`。

- 使用包内 `meshes/USB1.stl`，网格已是米单位，缩放为 1。沿 X/Y/Z 的尺寸约为 7.6 × 37.9 × 14.4 mm。质量仍为配置值，惯性用新网格的包围盒近似。
- 保留 USB 本地坐标：插头朝 +Y，抓取中心 `(0, 0, 0)` m，接线中心 `(0, -0.02, 0)` m，线从尾部向本地 −Y 延伸。初始左腕姿态令 USB +Y 对齐世界 +Y；移动后方向随夹爪旋转。
- 当前配置长度为 1.5 m、直径 3.5 mm，密度 1200 kg/m³，总质量约 17.3 g。改为 `length: 3.0` 时质量约 34.6 g。圆截面每层 7 个材料点；体积和质量按实际圆柱计算。
- USB 尾部前 7 mm 的线缆材料点固定在 USB 坐标系，固定点反作用与 MPM 接触力反馈到 SAPIEN 机器人。USB 质量暂设 15 g。
- 默认 `initial_layout: table_spiral`：线从 USB 尾部向本地 −Y 出线，随后弯向工作台，盘放的线圈从桌面上方约 8 cm 处下落。初态盘放在两个机器人底座的 +X 侧，避开右底座。盘放仅是初态，运动过程中没有固定在线圈路径上。重置会根据当前 USB 位姿重新生成所配置长度的初态，并检查是否与场景重叠。
- `initial_layout: straight` 可切换为水平展开后的坠落压力测试，计算域和运动幅度会明显增大。复用现有场景碰撞几何，包括线槽的非凸三角网格；不额外添加地板。

这是 **ManiSkill2 体积 MPM + 轴向 XPBD 纤维约束 + 独立刚体接触约束的混合实验模型**，没有使用刚体链。MPM 使用原始粒子与网格传递、弹性应力内核；适配层修正移动网格的边界留白，并施加 USB 固定端条件与反作用力。刚体接触由 `cable/contacts.py` 处理。

单纯使用粗网格体积 MPM 时，这根细线在测试中出现固定端附近材料点脱离。因此每个截面的 7 个材料点分别通过纵向纤维约束连接相邻截面，补充轴向刚度，保持连续性。此处不是直接调用 ManiSkill2 现成的“线缆 API”，也不应将结果当成纯 MPM 的验证结论。当前配置为 `young_modulus: 300000` Pa、`poisson_ratio: 0.3`、`axial_young_modulus: 30000000` Pa、`axial_iterations: 6`；这些均为演示参数，尚未标定真实 USB 电缆的弯曲刚度、最小弯曲半径、滞后或导体结构。

默认网格 6 mm，比 3.5 mm 线径粗，适合先验证执行、固定点与定性响应；体积形变和弯曲精度有限。刚体接触检查独立于此网格。网格按粒子包围盒分配双缓冲，超过 1600 万格时可自动降低精度（最大 15 mm），并在日志与 diagnostics 中显示。更精细的实验需减小 `grid_spacing`、提高 `frequency` 并评估显存；关闭 `adaptive_resolution` 可保持固定精度，超预算时会明确报错。刚度提高时也须减小 MPM 时间步。

MoveIt 对机器人、USB 和静态场景规划碰撞检查；动态线缆仅在仿真中参与物理接触，未作为 MoveIt 的避障物体，规划不会主动避开线缆。独立接触约束针对线缆与刚体，自接触仍依赖粗网格 MPM，没有显式线段自碰撞约束，不能据此判断打结、紧密缠绕或实际穿线间隙。

## 刚体接触

每轮纤维约束后均执行截面球体接触，半径按 1.75 mm 物理半径和相邻截面间距计算，保守覆盖截面之间的线段。默认截面间距下额外包络约 0.21 mm，接触位置会略偏保守。保守推进检查从上次安全位置到预测位置的路径，防止一步跨过薄壁。网格最近点查询覆盖实体内部，内外判定采用三条斜向射线投票。输入碰撞网格要求封闭且朝向一致，保留线槽的真实非凸空腔。

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

这些检查分别覆盖模型、接触内核和独立 USB 场景，不代表完整 MTC 走线已经通过验收。结果以当前机器实际运行输出为准。
