# 线缆建模方式

使用单一启动/ROS 参数 `cable_solver` 选择线缆模型：

| 参数值 | 实现 | 计算资源 |
| --- | --- | --- |
| `mpm`（默认） | 原有 MPM + 轴向纤维 + GPU 接触和理想滑孔 | CUDA + Vulkan |
| `rope_actor` | 参考 `temp/Rope-Actor/create_actors.py` 的胶囊刚体、三轴转动关节链 | CPU PhysX + Vulkan |

**当前配置：新版 304 面简化线槽 / 2 mm 线缆 / 101 段。**
碰撞使用新版网格，显示保留当前 `scene.trunking_visual_mesh: original` 选择。
已通过准备姿态短测及真实接触夹持、释放支撑、张爪下落和清理重置回归。
日常配置使用固定 500 Hz，精细配置 `_precise.yaml` 使用自适应步长。快速配置已额外通过
双臂下降 5 cm；完整 MTC 走线尚未重新验证，参数及性能见[性能记录](rope_performance.md)。

历史简化线槽配置曾由用户确认完整走线通过（录制 `run_20260915T113808Z_3scs5omg`）；
该结论不能直接用于本次新网格与 101 段配置。Rope-Actor 仍为实验后端，材料和摩擦尚未标定。

### 历史验证

从早期接触、分段与容差试验，到代理旋转、PGS 步长调度、USB 根部和凸胶囊接触修正，
结论已合并到[调试总结](debugging_summary.md)。旧实验目录及一次性配置已清理；
正式配置、诊断工具和冻结接触回归样本保留。

`simulation_backend:=maniskill` 选择机器人仿真环境；`maniskill_scene` 选择场景。
`cable_solver` 在启动时选择，未知值直接报错，不会自动回退。运行中更换模型需要重启场景。

## MTC 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor execute:=true \
  preparation_interactive:=false maniskill_python:="$PWD/.venv/bin/python"
```

切换回原有模型只需改为 `cable_solver:=mpm`。独立 USB 演示同样支持：

```bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py cable_solver:=rope_actor
```

选择参数贯通 MTC → MoveIt demo/ManiSkill launch → 物理桥接 → Simulation → `cable/backends.py` 工厂。
物理桥接也接受 ROS 参数 `cable_solver`；本包 `sim.launch.py` 支持同名参数。
两个后端共用 `cable_config` 的几何、USB、摩擦、初态和穿透容限；仿真只校验所选后端的专用段。
MoveIt/MTC 几何读取不要求 `mpm` 段，也不初始化 CUDA。已有 YAML 和默认 MPM 参数保持兼容。

### 完整 MTC 的失败记录

排查实际下降失败时保留当前物理参数，只添加 `cable_trace_dir`：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor \
  cable_trace_dir:="$PWD/artifacts/cable_trace"
```

桥接打印 `Cable trace enabled: .../run_...`。按原来的交互提示执行准备和下降；
失败暂停时会打印 `Cable failure trace saved: .../failure_001.json`。
每次启动创建新目录，重置线缆也保留已保存的失败文件。

目录包含：

- `metadata.json`：实际线缆/桥接配置、机器人描述及 Python 源码校验值。
- `controls.jsonl`：每个控制周期的实际目标关节位置、测得的位置/速度和双 TCP 位姿；包括生成线缆前的准备动作与等待。
- `initial_001.json`：线缆开始步进前的状态和场景刚体位姿；重置后编号递增。
- `substeps.jsonl`：每个线缆物理子步的时间、速度、动能、接触点/冲量、代理误差和连接误差。
- `failure_001.json`：原始异常及穿透定位、失败现场的几何复查、最后 64 个子步状态；schema 2 额外保存 `last_control` 中的控制输入与该周期起始状态，避免小步长使尾部不足一个控制周期。
- `summary.json`：正常退出时保存最后一轮汇总和末尾状态；强制终止进程可能没有此文件，已写出的失败文件独立保留。

默认不记录；非空目录仅支持 Rope-Actor 线缆场景。记录不调整时间步、容差、速度上限或施力，
但会增加实际运行耗时，交互等待产生的仿真时长可能随之变化。
平常子步不额外执行昂贵的几何审计，穿透字段为 `null`；超速和失败现场单独审计。
`pre_solve_separation_m` 仍表示 PhysX 求解前距离。汇总穿透峰值仅覆盖记录器实际审计的样本。
schema 2 在没有审计样本时将汇总穿透写为 `null`；旧版的零次审计 / 零峰值不能解释为没有穿透。
记录中的刚体状态不包含 PhysX 内部缓存，不能直接当作精确重启快照。
启动时输出目录不可写会报错；运行中的写入异常会单独记录日志，保留原有物理保护的报错。

## Rope-Actor 实现

保留参考方法的胶囊形状、关节弯曲/扭转自由度和阻尼。参考代码用两个辅助 link 加三个转动关节表示球形关节；
这里使用 PhysX 原生 D6 关节，锁定三向平移，保留三个转动自由度。
这样无需辅助刚体，也不受 SAPIEN 2 单个 articulation 最多 64 个 link 的限制。
MTC 默认的 `trunking_cable_simplified_2mm.yaml` 使用 101 个胶囊：首段 7 mm，其余 100 段各 14.93 mm。
本次分段调整的测量及验证范围见[性能记录](rope_performance.md)。
段长指胶囊轴线上两关节之间的距离；包含球形端帽后的胶囊总长还要加上线径。
MTC 的碰撞几何使用 `collision_geometry: convex_capsule`，通过凸体/三角网格接触路径
处理孔口和槽边。旧的解析胶囊在已捕获姿态中漏掉最深接触：实际重叠约 0.160 mm，
原生报告仅约 0.002 mm。凸体对照恢复到约 0.159 mm。
每个凸体有 194 个顶点，在单位半径下烘焙后按物理半径缩放；直接烘焙毫米尺寸会丢失顶点。
凸体内接于理想胶囊，2 mm 线径下表面最大内缩界为 0.028 mm。
视觉、质量/惯量及几何验收仍使用原始理想胶囊；快速模式穿透容差为 0.5 mm，
精细模式为 0.1 mm。修改验收阈值不会修改碰撞形状。
缺失该配置的旧 YAML 与独立 USB 演示仍使用解析 `capsule`。
独立 USB 演示和未配置 `links` 的旧配置仍默认 60 段，最大支持 256 段。
两个配置启用 PhysX PCM（持续接触流形）。MTC 场景改用 PGS，独立 USB 场景保留 TGS。
TGS 在 MTC 细链手指接触、小步长条件下的速度失稳可由录制状态复现，因此该场景改用 PGS，
同时保留原来的关节阻尼。未设置 `solver_type` 的旧 YAML
仍使用 TGS；旧实验的结果应结合当时的配置和代码版本阅读。真实手指网格的独立胶囊测试中，
旧接触模式产生了异常冲量；PCM 模式下贴壁支撑和摩擦滑动通过回归。
PCM 会复用并更新接触点，说明见 [PhysX 接触文档](https://nvidiagameworks.github.io/PhysX/4.1/documentation/physxguide/Manual/AdvancedCollisionDetection.html#persistent-contact-manifold-pcm)。

在 ManiSkill 创建引擎之前初始化配置中的 PhysX 长度/速度容差；坐标、速度和质量仍使用 SI 单位。
USB 配置使用 `0.001/0.02`，MTC 配置保留 `0.1/0.2`，分别依据场景验证，不能将一组参数的通过结果泛化到另一场景。
SAPIEN 2 在同一进程共享 PhysX 内部对象，
第一次 Engine 初始化决定容差，因此应通过独立桥接进程启动模型，并在切换后端时重启。
后续 Engine 包装对象输出“constructor arguments ignored”是此共享机制的提示。

首段长为 `cable.pin_length`，其余段均分剩余长度。胶囊覆盖整个段及端部；初态按实际段长重采样，
弯曲布局不会因弦长缩短而预加载关节。总质量等于 `density * pi * radius² * length`。
MTC 使用 `root_joint: spherical`：首段起点连接 USB 出线口，首段可以弯转，
沿用线段关节的角限位、角弹簧和阻尼；7 mm 仅是首段离散长度，不是刚性胶套。
USB 与左夹爪仍为固定连接，线缆与槽壁仍有真实碰撞。
独立 USB 演示和缺失 `root_joint` 的旧配置保留 `fixed`，即首段整体锁定到 USB。
`attachment_error_m` 测量各自约束点的位置误差；柔性模式不把首段中心的正常转动误判成脱离。
配置 `cable.linear_density`（kg/m）时，`density` 由线密度与截面积计算，质量等于
`linear_density * length`；序列化后的 `density` 是计算结果，重新载入时仍以线密度为准。
旧的纯 `density` 配置继续支持。

用户实测 2 m / 45 g 包含两个接头，接头单独质量未知。目前保留已建模 USB 的 15 g 估值，
剩余 30 g 按 2 m 分配，使用等效线密度 `0.015 kg/m`。1.5 m 的线段为 22.5 g，
加 USB 为 37.5 g；未单独建模的远端接头计入等效线段。这不是实测的线身 / 接头质量拆分。
`inertia_floor` 对旋转惯量作数值正则化，避免毫米级细杆的病态惯量以及 SAPIEN 2 的极小显式惯量替换行为；
它会影响转动响应，不能将此模型当成经过标定的电缆材料。

`rope_actor` 配置段提供：

| 设置 | 含义 |
| --- | --- |
| `links` | 胶囊段数，6–256；每段长度（包括首段）均须大于线径 |
| `root_joint` | `spherical`：出线点固定、首段可弯转；`fixed`：首段整体固定。MTC 用前者，旧配置默认后者 |
| `collision_geometry` | `convex_capsule`：内接凸胶囊，MTC 默认；`capsule`：原生解析胶囊，旧配置默认 |
| `engine_tolerance_length` / `engine_tolerance_speed` | PhysX 内部长度/速度尺度，m、m/s；不是穿透容限，不缩放场景 |
| `frequency` | PhysX 步进频率，必须为桥接 `sim_freq` 的整数倍；adaptive_timestep=true 时是允许继续细分的基础频率，否则使用固定步长；ROS 时钟按累计步长推进 |
| `adaptive_timestep` | 是否按接触运动距离细分步长；缺省 true，日常简化配置显式 false，精细配置 true |
| `max_contact_travel` | 仅 adaptive_timestep=true 时使用的运动距离上限，默认 0.00005 m；必须为正且不超过半径的一半，独立于穿透验收容差 |
| `joint_stiffness` / `joint_damping` | 关节角弹簧和阻尼，SI 单位 |
| `twist_limit_deg` / `bend_limit_deg` | 扭转、弯曲关节角限位 |
| `inertia_floor` | 转动惯量下限，kg m²，须大于 `1e-8` |
| `solver_type` | `pgs` 或 `tgs`；MTC 选 PGS，独立 USB 与缺失该字段的旧配置保留 TGS |
| `solver_iterations` / `solver_velocity_iterations` | PhysX 求解迭代次数，均为 1–255；日常简化配置为 200/10，精细配置为 40/10 |
| `contact_offset` | 用于胶囊、手指网格代理和线缆专用静态三角网格副本的 PhysX 接触提前检测距离，MTC 为 0.0001 m / 形状；两形状相加。独立于 MPM `cable.contact_margin`、步长预算和穿透容差；生产配置的静止偏移为零 |
| `constraint_tolerance` | 控制步结束时线段连接和 USB 固定端的位置误差上限，m；不限制孔内偏心量 |
| `max_stretch_ratio` | 可选的整条中心线累计伸长比例上限，日常配置为 0.01（1.5 m 线长对应 15 mm）；缺失或 null 不检查。只读报警，不校正位置、不增大刚度；单个关节均未超差时也可能触发 |
| `max_speed` | 胶囊中心线端点速度异常上限，m/s；包含平移和横向旋转的矢量合成 |

MPM 的网格、采样和材料参数不影响 Rope-Actor。两种离散方式的速度和接触误差应分别测量。
Rope-Actor 的关节/惯量参数不是 MPM 杨氏模量、泊松比的直接换算。

## 夹持、接触与接口

两种模型均使用：完整 MTC 预检 → 准备 → 两次夹爪闭合成功 → `/maniskill/cable/spawn` → 下降 → 走线。
`execute:=false` 不生成线缆；`maniskill_cable:=false` 关闭线缆。生成失败会阻止后续执行。

左端固定到 USB；Rope-Actor 的右孔完全由孔壁碰撞和库仑摩擦导向，不创建径向或转角锁定关节。

`guide_measurements` 及子步日志中的 `guide` 提供孔口材料弧长、相对 TCP X 轴的滑动速度、
USB 至孔口线长超过两点直线距离的余量，以及已报告的法向接触载荷。
`guide_arc_length_m` 连续变化表示材料在穿孔；线长余量不是张力测量。
PhysX 4 的接触报告仅提供法向冲量，因此 `guide_friction_axial_force_N` 为 `null`，
不能把它解释成没有摩擦。运动学代理对机器人的延迟反作用反馈也仅包含这些可读取的法向冲量；
线缆侧仍由 PhysX 求解摩擦。当前模型尚不是完整标定的力学模型。
录制回放与三处边缘接触回归见[滑动与边缘接触验证](debugging_summary.md#contacts)。
右 TCP 每个物理步只跟踪穿孔材料位置，不将线缆拉回孔轴，也不强制整段胶囊对齐。
控制步结束时，`MeshAperture` 对真实手指碰撞几何作 TCP 横截面，计入线缆半径后检查是否仍在孔内。
观察器适用于当前相对 TCP 呈星形的研究手指孔口，使用实际手指位姿更新截面；它不施力。
`guide_radial_error_m` 仅报告偏轴距离，孔内合法的偏心不再按固定 1 mm 阈值报错。
Rope-Actor 用原始手指三角网格的运动学代理保留实际孔洞，代理只与线缆接触；反作用冲量传回对应手指 link。
代理使用当前手指位姿及速度预测下一物理子步，并通过 PhysX kinematic target 运动，
使碰撞求解器能读到边界速度。反作用冲量先缓存，在下一子步按该步实际 `dt` 转为力；
这是一个子步延迟的分区耦合，步长变化不会额外改变冲量。
SAPIEN 2 的 Python API 未暴露 kinematic target，因此构建本包时编译小型 `_rope_physx` 扩展。
手指代理使用单根链接的运动学 articulation，而非普通运动学 `SActor`。
本地 SAPIEN 2.2.2 中，普通 `SActor` 的预步进会在旋转后进入陀螺力矩路径，破坏运动学目标；
连续旋转的两步最小实验可复现位置归零和无效四元数。新的代理避开该路径，
仍通过 PhysX kinematic target 提供碰撞边界的线速度和角速度。
每步求解后验证代理姿态与预测目标，异常即停止，避免继续使用失效的碰撞几何。
完整 MTC 记录、修正前后对照及验证范围见
[旋转代理故障](debugging_summary.md#proxy-rotation)。
默认从工作区 `.venv` 或 `VIRTUAL_ENV` 查找 SAPIEN 头文件；其它路径可通过
`--cmake-args -DDUAL_FR3_SAPIEN_INCLUDE_DIR=/path/to/site-packages/sapien/include` 指定。
扩展缺失时带手指接触的 Rope-Actor 场景会明确拒绝启动。
机器人原有碰撞体继续服务机器人自身碰撞。线槽、托板与其它刚体由 PhysX 胶囊接触求解。
原生 CCD 在本场景会让连接点出现分离，因此线缆关闭 CCD，按每段端点速度、
机器人接触表面速度估计及重力提前细分物理步。整条链始终使用不大于段长 1/10 的预测运动预算。
对每段计算胶囊包围盒到刚体包围盒的距离 `gap`，预算取
`min(0.1 * segment_length, max_contact_travel + 0.25 * max(gap - 2 * max_contact_travel, 0))`。
间隙不大于两倍接触预算时，严格使用 `max_contact_travel`；靠近障碍物的过程连续收紧，
避免旧的远 / 近开关导致步长骤降。包围盒会保守地包含非凸孔洞，实际碰撞仍使用原始 CAD。

每个物理子步都重新计算位移预算。控制周期内只按二分法细化待执行的步长；
周期边界才能放大，并且相对于上一子步最多放大两倍。整数子步计数使周期恰好结束，
不补一个极小的末尾剩余步。该方法不把整个控制周期的步长冻结，也不设一个超过当前预算的强制最小步长。
记录中的 `suggested_timestep_s` 和 `predicted_motion_budget_fraction` 可逐步验证预算，后者应不大于 1；
`predicted_contact_travel_m` 是上述近距离段的预测行程，没有近距离段时为 null。
这是速度模型的预测，不是对本步接触冲量或 PD 驱动加速度的严格位移保证。

### 步长与验收容差解耦

`cable.penetration_tolerance` 只用于布局、穿透和孔口验收，不再参与 Rope-Actor 的时间步计算。
两个生产 YAML 显式设置 `rope_actor.max_contact_travel: 0.00005`。距离预算保持 0.05 mm，
当前调度算法和质量配置已更新，不会与历史子步序列完全相同。
旧 YAML 缺少该字段时也使用固定 0.05 mm，**不会根据其中的穿透容差推导**。
历史 256 段 / 0.15 mm 的正常入口结果使用 0.075 mm 位移上限；重放这些历史结果时，
需要在独立配置副本中显式加入 `max_contact_travel: 0.000075`。归档中的固定基线实验则对应 0.00005。
不能把缺少新字段的旧 YAML 在当前代码下的结果直接当作原来的正常入口重放。
诊断新增 `max_contact_travel_m`、`penetration_tolerance_m` 和 `last_rigid_timestep_s`，便于核对实际参数。

MPM 的 `guide.half_length` 定义理想直孔半长。Rope-Actor 的有限孔道由手指 CAD 接触决定；
`guide.half_length` 仍用于共同穿孔初态的直线段生成。两者的接触边界不可视为完全相同。

生成和重置先检查完整胶囊段的初始几何，拒绝相交布局。重置保留双臂位姿；验证失败时保留旧线缆状态。
运行时在控制步结束检查原生接触对的实际几何、连续性和约束误差；不能把 PhysX 求解前的接触距离直接作为残余穿透。
右手指附近的完整胶囊还会独立进行三维穿透检查，即使最后一个物理子步没有报告接触也不会跳过。
资源清理由统一的 `close()` 完成，失败生成会恢复修改的碰撞过滤并清理临时对象。

### 原始线槽 / 3 mm 线缆的接触距离修正

录制 `run_20260915T133352Z_9sszncyq` 在生成线缆后 1.10 s 的双臂下降末段失败：
第 3 段在槽边穿透 0.148675 mm，超过 0.1 mm 容差，末尾 64 子步该接触对的法向冲量为零。
胶囊的 `contact_offset` 已为 0.1 mm，但原始线槽仍继承场景的 1 mm 设置。
在独立场景中固定相同的末态、仅改变线槽的接触距离，可得到以下对照：

| 线槽接触距离 | 原生接触报告的最小 separation |
| --- | --- |
| 1 mm（原设置） | +0.192521 mm，未报告槽边重叠 |
| 0.1 mm（修正） | −0.150150 mm，检出槽边重叠 |

这项对照定位了本次姿态下的接触距离敏感性，尚未确定 PhysX 内部具体分支。
早期探针已将线槽设为 0.1 mm，不能用它与原运行的差异推断接触缓存失效。

现在为含非凸三角网格的静态场景物体创建线缆专用碰撞副本。副本复用相同网格、
局部位姿、材料和静止偏移，接触距离使用 `rope_actor.contact_offset`，只与线缆碰撞。
原物体继续用于机器人接触，保留原有接触距离和过滤设置；箱体等基础形状维持原有路径。
副本跟随静态物体的显式位姿修改，并在关闭或生成失败时清理。
原生子步日志中的名称为 `rope_fixture_trunking`，几何报错与 `contacted_bodies` 使用原名 `trunking`。

冻结末态已加入 `test/data/rope_trunking_3mm_20260915.json`，回归检查接触深度、网格一致性、
机器人碰撞隔离、位姿同步和资源清理。另从录制初态重建场景并执行录制控制输入，
修正后运行 1.6 s、80 个控制边界检查通过，审计最大穿透为 0.013916 mm。
超出已录输入后保持最后目标。冷启动未恢复准备阶段和 PhysX 内部缓存，原代码的冷启动基线也未失败，
因此该结果仅覆盖重建场景；完整 MTC 下降及后续走线仍需重新运行验证。

保留 `/usb_cable_demo/markers`、`/usb_cable_demo/diagnostics`、`/usb_cable_demo/reset`、USB TF 和 MoveIt USB 附着对象。
诊断 `solver` 给出实际模型；`guide_material_coordinate` 仍表示可带小数的截面索引，Rope-Actor 另报 `guide_arc_length_m`。
`guide_model=mesh_contact_only` 表示孔壁接触导向；`guide_inside_aperture` 是最近一次几何检查结果，
`guide_plane_wall_distance_m` 是穿孔中心线交点到 CAD 截面边界的距离，不是三维最小间隙。
公共 `display.marker_stride` 控制 RViz 显示，旧 MPM `marker_stride` 仍受支持。
诊断还报告 `max_endpoint_speed_m_s`、包含平移和转动的 `kinetic_energy_J`、
`kinetic_energy_before_step_J`、`last_contact_impulse_Ns` 和 `last_contact_pair`。
超速异常包含这些物理量及实际子步长，不能仅凭“超速”断言机器人指令过快。

穿透错误报告 `object`、`segment_index`（从 0 开始，对应 `rope_actor_N`）、段长，以及
`world` 坐标系中的 `centerline` 和 `obstacle_surface`，坐标单位为米。
前者是测得最大穿透的中心线采样点，后者是该点最近的障碍物表面点；
这是求解后几何检查的位置，不是 PhysX 求解前的冲量接触点。
初始化和重置时拒绝相交布局的错误也包含这些定位信息。
同一数据作为 `max_penetration_contact` 发布到诊断 JSON；没有检测到穿透时为 `null`。
网格的 `collision_triangle_index` 指实际碰撞网格中的三角形编号，未必对应原始 STL 面序号。
`min_segment_length_m` / `max_segment_length_m` 报告本次实例的实际分段长度。

柔性线缆本身不参与 MoveIt 刚性碰撞规划，接触在物理执行中检查。两个后端均未验证打结/自碰撞；
左端仍采用固定约束，MPM 右孔仍为理想滑孔；Rope-Actor 右孔摩擦系数尚未标定。
原有 MPM 数值和性能记录见[归档基线](debugging_summary.md#mpm-baseline)。

## 检查

```bash
# 不依赖渲染器的真实 PhysX 关节、质量、滑动、重置和失败清理检查
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test/test_cable_backends.py

# 同一记录姿态上的运动、接触、重置和耗时报告；可改 --cable-solver mpm
.venv/bin/python src/dual_fr3_maniskill/scripts/check_cable.py \
  --cable-solver rope_actor --steps 100 --output /tmp/rope_actor_check.json

# 单独复现从准备位下降 5 cm；失败也会保存诊断并非零退出
.venv/bin/python src/dual_fr3_maniskill/scripts/check_cable.py \
  --cable-solver rope_actor --steps 100 --descent 0.05 --output /tmp/rope_actor_descent.json
```

局部运动检查不能替代完整 MTC。完整任务可使用 `scripts/validate_integration.sh --mode mtc --cable-solver rope_actor`。

## 不依赖渲染器的右手指接触最小实验

`scripts/check_rope_finger_contact.py` 只使用 CPU PhysX，不创建 ROS 节点或 Vulkan 渲染器。
默认保留 13 个胶囊，普通段长取 1.5 m / 256 段配置的 5.855 mm，首段仍为 7 mm。
线缆穿过真实研究手指 CAD，左端固定；右孔先停留 20 ms，沿初始 TCP 的 +Z 平滑移动 1 mm / 100 ms，
再保持 20 ms。默认固定时间步 0.1 ms，摩擦、惯量、关节参数及代理接触实现沿用加载的配置。

`--coupling delayed` 使用动态手指源，通过固定关节连接到运动学 TCP 载体，并复用生产代理的
速度预测和一子步延迟反作用。`--coupling prescribed` 将源手指也设为运动学物体，显式忽略反作用，
仅用于隔离对照。两者均用规定运动的载体替代整台机器人的动力学，因此该实验不是完整失败状态重放。
没有线槽，也没有保留整条线缆的张力边界；短链通过不能证明完整 MTC 稳定。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_rope_finger_contact.py \
  --output /tmp/finger_fixed.json
.venv/bin/python src/dual_fr3_maniskill/scripts/check_rope_finger_contact.py \
  --timestep alternating --dt .0001 --alternate-dt .000357143 \
  --output /tmp/finger_alternating.json
.venv/bin/python src/dual_fr3_maniskill/scripts/check_rope_finger_contact.py \
  --timestep adaptive --dt .001 --max-contact-travel .00005 \
  --output /tmp/finger_adaptive.json
```

`--penetration-tolerance` 与 `--max-contact-travel` 可以分别改变，均经生产参数校验。
`--motion-axis X Y Z` 指初始 TCP 坐标系中的方向，`--distance 0` 可作静止对照；
`--reference-links` 控制普通段长，`--links` 控制保留的短链段数。
`--tail-force Fx Fy Fz` 可在末端施加世界坐标系中的规定力（N），以研究截断长线后的边界载荷影响；
默认零力。该输入不是对完整 MTC 张力的实测重建。

每次输出新的 JSON 汇总与同名 JSONL 子步流，已有结果不会覆盖。所有物理子步均记录速度、
动能增量、原生接触点/冲量、关节间隙和代理/真实手指位姿差；JSON 额外保存最后 64 个子步的刚体状态。
这些状态用于诊断，不包含 PhysX 内部接触缓存，不能视作精确可恢复的引擎快照。
原生 `pre_solve_separation_m` 是求解前接触距离，不能当作残余穿透。
昂贵的求解后几何检查默认每 20 个子步运行一次，也在第一步、最后一步以及超速/穿孔跟踪失败时运行；
未审计的子步 `max_penetration_m` 为 `null`，不会发布陈旧的零值。`--audit-every 1` 可检查每个子步。
几何审计或约束检查失败会非零退出；全程没有实际孔壁接触也会非零退出，防止空测通过。

历史对照及 818 个子步完全一致的容差解耦验证结论见
[步长解耦与右手指短链实验](debugging_summary.md#decoupling)。
