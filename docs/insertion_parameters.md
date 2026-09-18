# 插入调参位置与有效值

以运行代码和所选配置的数值为准。本页对应 `mtc_prototype.launch.py` 的 ManiSkill
`trunking_cable` 场景，默认配置为 `trunking_cable_simplified_2mm.yaml`。
按用户要求，MTC 入口默认 ManiSkill/Rope-Actor/带线缆，并只在释放世界固定后等待一次确认；物理参数、控制阈值和历史实验配置不改。当前 YAML 的路径比例 3.0 为用户已有修改。

## 先确定修改位置

| 要修改什么 | 修改位置 | 首先验证 |
| --- | --- | --- |
| 独立 ROS skill、只规划/执行 | MTC `insertion_task/cli.py` / `launch/insertion_skill.launch.py` | `test_insertion_skill.py`，已有夹持场景内独立调用 |
| 终末动作顺序、失败上报、收尾开关 | MTC `insertion_task/pipeline.py:TerminalInsertion` | `test_terminal_insertion.py`，再完整 USB-only 场景 |
| 接近/退出规划、规划重试、TCP 路程限制 | MTC `insertion_task/planning.py`（插入前整段），`insertion_task/motion.py`（插入后），`mtc/path_length.py` | 编排及路径测试，再看每段规划日志 |
| MoveIt 插座、USB 解除附着、临时 ACM | MTC `insertion_task/planning_scene.py` | 原碰撞许可保留、世界 USB 唯一、退出/失败后 ACM 恢复 |
| 孔尺寸、CAD 关键点、实测抓姿到目标位姿 | `usb/geometry.py` | 顶点/变换对照、几何报告和原生 PhysX 探针 |
| 阻力反馈、停滞/过载、成功保持 | `usb/insertion.py:InsertionLimits/InsertionPolicy` | 策略测试、局部插入及真实孔内障碍测试 |
| 服务拒绝、控制权、heartbeat、IK 指令保护 | `usb/bridge.py:handle_* / before_tick` | `test_insertion_services.py`、`test_insertion.py` |
| 插座实体、支撑反力、保持约束、清理 | `usb/scene.py:SocketInsertion` | 原生探针、固定后开爪、reset |
| 线缆/手指接触、物理子步 | `scenes/usb_cable.py`、`cable/rope_actor.py` | 对应后端/接触/步长测试，再带线缆场景 |

路径未标包名时均属于 `dual_fr3_maniskill/` Python 目录。服务名与反馈活动状态集中在
`usb/insertion.py`，不引入 ROS 依赖；几何模块导入不创建 ROS 节点、渲染器或物理场景。
MoveIt 保留 STL 三角网格，PhysX 保留 37 个凸体，二者只共享孔壁调整计算。

## 覆盖关系（不能只看一个默认值）

1. `scenes.resolve_cable_config()` 决定 YAML：显式 `cable_config` 优先；`trunking_cable`
   缺省选简化 2 mm 配置，`usb_cable` 缺省选 `usb_cable.yaml`。不同文件**不互相合并**。
   `cable.model.load_config()` 校验并补缺省；`load_geometry_config()` 使用 `solver=None`，不初始化后端。
2. YAML 中的 `insertion` 字段覆盖 `InsertionLimits` 或消费函数的 `.get()` 缺省；
   没有该段的配置使用代码缺省。`insertion.enabled` 是例外：
   `cable/trunking_bridge.py` 用 ROS `insertion_enabled` **覆盖**它。
3. 桥接 ROS 参数在 `ros.launch.create_bridge_node()` 按以下顺序提供：
   `maniskill_config`（若空则用场景的 simulation YAML）→ 可选 camera YAML → 模型参数 → launch 显式 options。
   后面的同名值覆盖前面的值；未给的参数才使用 `ros/bridge.py` 的声明默认值。
   `cable_config` 是普通场景 YAML 路径，不是 ROS 参数 YAML；其中的 `insertion.speed_m_s`
   等字段没有自动成为可动态修改的 ROS 参数，改文件后需重新启动相应消费者。
4. MTC launch 与 CLI 缺省 `simulation_backend=maniskill`，`cable_solver=rope_actor`（launch）、`load_cable=true`、`execute=true`。
   `insertion_enabled` 在 ManiSkill 接触夹持模式缺省 true，其他后端或 `maniskill_cable=false` 缺省 false，显式参数优先。
   MoveIt 的 ManiSkill launch 仍缺省启用插入；`create_bridge_node()`、桥接声明和 YAML enabled 仍为 false，保留入口差异。
   `dual_fr3_maniskill/sim.launch.py` 不转发该开关，不能替代推荐入口。
5. MTC 入口将 `rope_actor` 传给下层；其余独立物理 launch/ROS 声明保留 `mpm` 缺省。
   带线缆插入当前要求 `rope_actor`；`load_cable=false` 跳过线缆后端。
6. **`insertion.max_path_length_ratio` 当前推荐 YAML 是 3.0，代码缺省为 1.5。**
   3.0 是用户已有数值，保留不改；历史实验中的 2.0 保留在原记录。
   precise/其他无 insertion 段的配置仍使用 1.5，不统一不同配置。
7. `run_insertion=true` 决定运输结束后自动调用终末 skill；设 false 可留给独立入口，
   此时 `insertion_enabled=true` 仍保留物理插座和整段规划检查。

`simulation_usb_cable.yaml` 实际给出 `control_freq=50`、`sim_freq=500`、`publish_freq=25` Hz；
robot 场景 `simulation.yaml` 和 `ManiSkillBridge` 缺省分别是 `100/500/50` Hz。
启动后可用 `ros2 param dump /dual_fr3_maniskill` 核对 ROS 参数；`insertion/status` 的 `limits`
给出当前策略有效值，其余场景几何与物理参数核对 `cable_config` 指向的文件。

## 场景几何与坐标

下表字段位于 `insertion` 段，除特别说明外推荐 YAML 与代码缺省相同。长度均为 m，四元数为 wxyz。

| 参数 / 实际值 | 坐标与作用模块 | 修改影响、关联与验证 |
| --- | --- | --- |
| `visual_mesh` / `collision_mesh` = `meshes/usb_base_visual.stl` / `meshes/usb_base_collision.stl` | 包 share 路径；`SocketInsertion.create` / MoveIt 场景 | 无缩放；改 CAD 必须复查孔壁选择边界、凸体数量和几何报告 |
| `xy_frame=left_fr3_link0`, `xy_m=[.37,-.07]` | 先将 `[x,y,0]` 变换到 world，使用 world XY | `resolve_pose`；关联 height 和 world 机器人基座位姿，核对 `socket_world_pose` |
| `height_frame=trunking`, `height_m=.010` | 将该 frame 的 `[0,0,height]` 变换到 world，仅取 world Z | 指 CAD 底面原点，不是槽壁顶面；修改后复查手指/安装件碰撞 |
| `quaternion_wxyz=[1,0,0,0]` | **world** 中的插座朝向，不与 XY/height frame 旋转再次相乘 | 改变插入轴和力坐标轴；检查变换及接近/退出路径 |
| `hole_center_m=[0,.0175,.0686]` | 插座 CAD 坐标，孔口 X=0 | 同时影响碰撞孔、目标、测量、力矩参考点；检查移动前后孔壁与目标角点 |
| `clearance_yz_m=[.0004,.0004]` | 插座 Y/Z 每侧间隙；`adjust_hole_vertices` | 孔截面 = 金属段 `[.00445,.012] + 2*clearance`；不是 contact offset。两侧网格/原生凸体均需验证 |
| `preinsert_m=.008`, `target_depth_m=.010`, `end_clearance_m=.001` | 插座 -X 深入为正，前置目标 depth=-preinsert | `tcp_goal` 与 policy；必须保持 target=`DEPTH(.011)-end_clearance`，同时核对 max_advance |

USB `TIP=[0,.0179,0]`、`SHOULDER=.0059`、金属截面、`DEPTH=.011` 和 `USB_IN_SOCKET`
由 `usb/geometry.py` 保存，单位 m；USB +Y 对应插座 -X，USB +X 对应插座 +Y。
`matrix`/`tcp_goal`/`measure` 统一变换。目标使用**实测** TCP—USB 相对姿态，运输后再次读取，不能用初始抓姿替代。

## 线缆、接触和夹爪物理

| 参数 / 推荐配置实际值 | 单位 / 作用位置 | 影响、关联与验证 |
| --- | --- | --- |
| `usb.mass=.015`, `usb.friction=.5` | kg / 无量纲；USB 刚体及接触材料 | 影响惯性与滑移，不能与 cable 摩擦去重；接触夹持、释放后稳定和载荷测试 |
| `usb.contact_offset=.0002`, `usb.finger_contact_offset=.0001` | m；USB / 真实左手指接触壳 | 预接触距离，rest offset 保持 0；关联几何间隙，复跑真实夹持 |
| `insertion.contact_offset_m=.0001`, `rest_offset_m=0` | m；插座形状 | USB+插座检测壳总厚度 .0003 m，小于每侧 .0004 m 几何间隙；原生探针 |
| 插座材料 `.5/.5/0`、质量 `.1`、主惯量 `[.0001]*3` | 静/动摩擦及恢复系数、kg、kg·m²；`SocketInsertion.create` 代码固定值 | 世界支撑反力测量依赖基座禁重力及准静态，不用改质量来模拟夹紧；力/力矩探针 |
| `usb.grip_center=[0,0,0]`, `tcp_grip_offset=[0,0,.012]` | 前者 USB 坐标，后者 TCP 坐标，m | 初始夹持位置；`cable.threading.task_usb_mount`，关联 CAD 手指孔和实际抓姿；初始几何检查 |
| `cable.length=1.5`, `diameter=.002`, `linear_density=.015` | m、m、kg/m；`cable.model`/后端 | 体密度由线密度和直径派生；影响重量、孔间隙；后端质量/几何测试 |
| `cable.friction=.1`, `rope_actor.contact_offset=.0001` | 无量纲 / m；线缆及其代理 | 不影响 USB .5 摩擦；关联当前 `penetration_tolerance=.0008`（其他预设保留 .0001），运行孔沿/移动手指接触回归 |
| `grasp.contact_min_force_N=.10`, `contact_hold_s=.10` | 每根手指 N / 仿真 s；`usb/grasp.py` | 接触确认，不能代替插入成功；与 stable/verification/slip/drop 参数一起检查 |
| `release_width_m={left:.08,right:.03}` | 总开度 m；MTC `open_gripper` | 使用已有 open profile；与下行实际双指检查配套，验证槽壁不会阻挡右指 |
| `release_min_half_width_m={left:.035,right:.014}` | **每指**开度 m；bridge `require_open` | 两根手指都必须满足；与目标总开度区别，服务测试+真实开爪 |

夹持力来自 MTC `config/gripper_profiles.yaml`（如 cable_tip 10 N、cable_body 12 N）及阶段覆盖，
终末开爪复用 open profile；桥接 `gripper_force=40 N` 是其自身缺省，不能视为所有动作的实际夹持力。
调参分别查看 MTC `execution/gripper.py` 请求和 `/maniskill/forces` 实测载荷。

## 数值求解与有意不同的场景配置

| 参数 | 日常简化 2 mm YAML | precise YAML | 修改影响 / 验证 |
| --- | --- | --- | --- |
| `cable.friction` | .1 | .5 | 真实滑动响应不同；接触测试和带线缆场景 |
| `cable.pin_length` | .005 m | .007 m | 影响根段和剩余段长；初始布局/连接误差 |
| `rope_actor.links` | 151 | 101 | 离散段数、计算量和碰撞形状；后端质量/长度测试 |
| `rope_actor.frequency` | 1000 Hz | 1000 Hz | 基本物理子步；必须整除 sim_freq |
| `rope_actor.adaptive_timestep` | false | true | 后者按 `max_contact_travel=.00005 m` 细分；步长/接触回归 |
| `solver_iterations / solver_velocity_iterations` | 200 / 10 | 40 / 10 | 位置/速度求解，勿统一；约束误差/滑移/性能对照 |
| `constraint_tolerance` | .002 m | .001 m | 段关节/USB 连接误差保护，不是几何穿透量 |
| `max_stretch_ratio` | .01 | 未设置（不开启此项） | 总长度伸长保护；保留各场景验收标准 |
| `insertion` 段 | 显式列出 | 无该段 | precise 开启插入时走代码缺省，例如路程比例 1.5 |

这两份配置不是继承关系。其他 `trunking_cable.yaml`、`usb_cable.yaml` 也各自保留。
`cable.model` 的通用 Rope-Actor 缺省还包括 TGS、fixed 根关节、60 段、40/10 次迭代；
推荐 YAML 明确覆盖为 PGS、spherical 根关节等，不能拿通用缺省替代运行配置。

插入启用后，`scenes/usb_cable.py` 给场景启用 PCM，并将位置/速度迭代提升到至少 200/10。
`insertion.rigid_frequency_hz=1000` 要求为 `sim_freq` 的整数倍，实际基础子步取
线缆 frequency 与插入 frequency 中更细的一项；自适应线缆还可能进一步细分。
USB-only 仍使用插入刚体子步，但不创建线缆。这不是把所有场景的 500 Hz ROS 参数改成 1000 Hz。

## 运动、反馈、成功与异常保护

以下均为 `insertion` 字段，除路程比例外，表中数值同时来自推荐 YAML 和 `InsertionLimits` / 消费函数缺省。
反馈 wrench 为**作用在 USB 上**的力，插座轴向，力矩参考孔中心；+X 阻力抵抗 -X 插入。

| 参数 / 实际值 | 单位 / 作用模块 | 影响、关联与验证 |
| --- | --- | --- |
| `speed_m_s=.001`, `acceleration_m_s2=.002` | m/s、m/s²；policy | 连续推进及速度斜率；检查力变化下的速度曲线 |
| `resistance_gain_m_N_s=.0004`, `filter_tau_s=.04` | m/(N·s)、仿真 s；policy | `max(0,speed-gain*filtered_force)`；真实障碍测试检验减速/停滞 |
| `max_advance_m=.022` | m，沿插入轴的指令累计路程；policy | ≥preinsert+target，独立于实际深度；超程测试 |
| `axial_limit_N=5`, `lateral_limit_N=2`, `torque_limit_Nm=.04` | N、N、N·m；policy + 场景采样 | 控制周期均值和子步峰值保护，滤波不能掩盖尖峰；载荷探针/过载测试 |
| `lateral_tolerance_m=.00025`, `angle_tolerance_rad=.025` | 孔口 YZ 距离 m / 完整 SO(3) 角 rad | 对齐、推进和固定均使用；变换测试及姿态越限测试 |
| `depth_tolerance_m=.00015`, `hold_s=.3` | m / 仿真 s；policy | 实测深度连续满足后才成功；深度超调仍失败，保持测试 |
| `retention_speed_m_s=.001`, `retention_angular_rad_s=.03` | USB 相对基座 m/s、rad/s | 成功保持与约束建立前均检查；局部插入+开爪位移 |
| `blocked_s=1.5`, `progress_epsilon_m=.0001` | 仿真 s、实际深度 m | 无进展停止；与速度/滤波关联，真实孔内障碍检查 |
| `timeout_s=45`, `wall_watchdog_s=10` | 仿真 s / 墙钟 s | policy 总时限 / bridge heartbeat 租约；冻结仿真时间仍能失联停止 |
| `joint_speed_rad_s=.10`, `tracking_limit_m=.001` | rad/s、world TCP 距离 m；bridge | 每关节 IK 指令变化 / 实际跟踪偏差；tracking 同时用于初始深度对齐检查 |
| `retreat_m=.05`, `approach_lift_m=0` | 本侧 TCP -Z 退出 / world +Z 抬高，m；MTC | 右臂先退出回位，左臂固定开爪后退出回位；逐段轨迹和碰撞检查 |
| `planning_attempts=10`, `planning_timeout_s=15` | 次 / s；MTC motion | 重建 CurrentState 后重试规划，执行仅一次；timeout 赋给 MoveTo，withdraw 不单独设置此项 |
| `max_path_length_ratio=3.0`（代码缺省 1.5） | 无量纲；MTC motion/path_length | 每段 TCP 路程/实际起终点直线距离，独立于原 MTC 的 anchor 同名比率；查看接受/拒绝日志 |
| `retain_after_success / release_after_retention / return_after_release=true` | 布尔；scene + MTC | 分别控制固定、左爪释放、左臂回位；右臂先行释放回位不受这三项控制，开关编排测试 |

仍为代码固定值的边界：MTC 普通终末运动笛卡尔步长 .001 m、速度/加速度比例 .08/.08，
退出为 .001 m 和 .05/.05，跳变阈值均 2.0（插入前 `insertion_task/planning.py`，插入后 `insertion_task/motion.py`）；没有复用主 MTC 的 .2 比例。
bridge `require_ready` 的关节位置误差 .01 rad、速度 .02 rad/s；基座准静态条件为
.0001 m/s 和 .001 rad/s（`SocketInsertion.sample`）。IK eps=1e-6、最多 100 次迭代。
MTC 每 .1 墙钟秒 heartbeat，外层看门狗 `max(120,30*timeout_s)`；45 s 配置对应 1350 墙钟秒。
这些值不在 YAML 中，本轮不提升为新接口，也不改变数值。

## 显示、日志与性能

`maniskill_viewer` / `use_rviz` 仅控制窗口；完整 MTC launch 当前均默认 true，可显式关闭。
`publish_freq=25 Hz`、`realtime_factor=1` 来源为 simulation USB YAML；`force_enabled=true`、
`force_record_path=""`、`force_usb_base_names=""` 控制输出，插入状态流独立于 force_enabled。
`cable_trace_dir=""` 缺省不记录线缆诊断；调高记录量会改变墙钟耗时，应同时观察 heartbeat。
`display.marker_stride` 缺省 1；MPM 的 `cuda_graph` / `gpu_grid_check` 和 `mpm.frequency=5000 Hz`
只在启用 MPM 后端时生效，不是刚体插入控制参数。验证物理变化时保持记录/显示设置一致。

## 验证入口

从工作区根目录先加载 ROS 和 `install/setup.bash`，新增 Python 文件后重新 colcon build 两个修改包。

```bash
ROS_LOG_DIR=/tmp/insertion_tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test/test_insertion.py \
  src/dual_fr3_maniskill/test/test_insertion_services.py \
  src/dual_fr3_trunking_mtc/test/test_terminal_insertion.py
.venv/bin/python src/dual_fr3_maniskill/scripts/check_insertion_geometry.py --output /tmp/geometry_new.json
.venv/bin/python src/dual_fr3_maniskill/test/check_insertion_physx.py --output /tmp/physx_new.json
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_insertion_simulation.py --output /tmp/local_new.jsonl
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_insertion_simulation.py \
  --obstacle-depth .006 --output /tmp/obstacle_new.jsonl
```

后两项需要仿真运行环境，只从孔前测试初态开始，不包含完整 MTC 回位。
完整场景命令见 [插入说明](usb_insertion.md#启动与观测)，应分别检查插入、固定、开爪和回位标志，
再检查重复 start 和 reset 后世界/附着物清除。当前重构记录见 [本轮验证](insertion_refactor_validation.md)。

默认启动和独立入口调整的本轮结果见 [工作流验证记录](insertion_workflow_validation.md)。

包含全部线缆、夹持、控制、相机和 launch 参数的逐项说明见 [参数索引](parameters.md)，声明值与文件见 [默认值来源](parameter_defaults.md)。
