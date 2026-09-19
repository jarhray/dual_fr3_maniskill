# 参数索引

本表覆盖场景 YAML、ROS/launch、代码补缺省及构建工具参数；不把内部局部变量当成可调接口。
推荐入口是 MTC `mtc_prototype.launch.py`，默认 `maniskill / rope_actor / load_cable=true / execute=true`。
先读 [架构](../ARCHITECTURE.md) 和 [覆盖关系、坐标及插入约束](insertion_parameters.md)。
默认简化 2 mm 配置的 `cable.penetration_tolerance=0.0008`、`insertion.max_path_length_ratio=3.0` 均保留；
其他预设及代码补缺省不随之统一。
每项的**当前值、代码缺省和声明文件**逐项列在 [默认值与来源](parameter_defaults.md)。同名项保留各入口的差异，不能把代码缺省当成 YAML 覆盖后的值。下面各表明确消费文件、单位、作用及关联；验证命令相对于工作区根目录。改 YAML 后重启消费者；这些文件字段不是动态 ROS 参数接口。

## 启动、场景和数据来源

**消费代码：** [dual_fr3_maniskill/ros/launch.py](../dual_fr3_maniskill/ros/launch.py)、[dual_fr3_maniskill/ros/bridge.py](../dual_fr3_maniskill/ros/bridge.py)、[dual_fr3_maniskill/scenes/__init__.py](../dual_fr3_maniskill/scenes/__init__.py)、[dual_fr3_maniskill/cable/trunking_bridge.py](../dual_fr3_maniskill/cable/trunking_bridge.py)。

**验证：** `test_maniskill_scene_launch.py`、`test_cable_backends.py`；启动后 `ros2 param dump /dual_fr3_maniskill`。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `maniskill_scene` | 枚举 | robot / usb_cable / trunking_cable 选择唯一物理桥接及默认配置。 |
| `maniskill_config` | 文件路径 | 替代所选场景的 simulation YAML；机器人 100/500/50 Hz，线缆 50/500/25 Hz 不合并。 |
| `maniskill_python` | 解释器路径 | 物理节点 Python；优先 MANISKILL_PYTHON，其次工作目录 .venv，需具备 ROS 与 SAPIEN 依赖。 |
| `maniskill_viewer` | 布尔 | launch 对 viewer 的覆盖，只影响渲染窗口及墙钟性能。 |
| `viewer` | 布尔 | 桥接最终窗口开关；由 maniskill_viewer 覆盖节点 false 缺省。 |
| `robot_description` | URDF XML | 最终机器人/夹具模型，由上层生成；决定关节、碰撞和世界放置。 |
| `robot_description_semantic` | SRDF XML | 规划组和姿态定义，必须与同一 URDF 配套。 |
| `cable_config` | 文件路径 | 普通场景 YAML；显式文件优先，否则由 scenes 选默认文件；物理与 MoveIt 必须一致。 |
| `cable_solver` | 枚举 | mpm 或 rope_actor；MTC 默认 rope_actor，独立物理入口仍为 mpm，仅验证并初始化选中后端。 |
| `load_cable` | 布尔 | false 跳过柔性线缆后端，保留 USB、夹持和双臂运输；不是完整带线缆验证。 |
| `insertion_enabled` | 布尔 | ROS 值覆盖 insertion.enabled；trunking 场景创建插座与反馈流程。 |
| `leader_orientation_direction` | forward/reverse | 任务初始 USB 朝向，与 MTC leader 方向一致，覆盖 usb.orientation_direction。 |
| `usb_preparation_poses` | JSON，world / m、wxyz | 延迟 spawn 前的准备 TCP 位姿，由 MTC 提供；不是执行中瞬移目标。 |
| `cable_trace_dir` | 目录路径 | Rope-Actor 子步诊断和失败快照；空字符串关闭，非 rope_actor 场景拒绝。 |
| `scene.trunking_mesh` | original/simplified | 选择线槽碰撞 CAD，影响几何与接触；不能仅为性能随意替换。 |
| `scene.trunking_visual_mesh` | original/simplified | 独立选择显示 CAD，不改变碰撞网格。 |

## 机器人控制与反馈发布

**消费代码：** [dual_fr3_maniskill/ros/bridge.py](../dual_fr3_maniskill/ros/bridge.py)、[dual_fr3_maniskill/robot/simulation.py](../dual_fr3_maniskill/robot/simulation.py)、[dual_fr3_maniskill/robot/trajectory.py](../dual_fr3_maniskill/robot/trajectory.py)。

**验证：** `test_trajectory.py`、`test_usb_grasp_services.py`；真实 ROS 轨迹检查误差和取消保持。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `control_freq` | Hz | ROS 控制更新频率；影响轨迹采样，与 sim_freq 的子步整除关系配套。 |
| `sim_freq` | Hz | 场景基本步频；线缆和插入可进一步子步，不等于最终最小 dt。 |
| `publish_freq` | Hz | joint_states、时钟及状态发布频率；影响状态新鲜度和性能。 |
| `realtime_factor` | 仿真秒/墙钟秒 | 墙钟节拍目标；不改变仿真时间滤波和成功保持时间。 |
| `path_position_tolerance` | rad，关节 | 轨迹运行时位置误差上限，超限中止；与执行速度、控制频率关联。 |
| `goal_position_tolerance` | rad，关节 | 到点位置误差验收；不同于 MoveIt TCP 位置误差。 |
| `goal_velocity_tolerance` | rad/s | 到点静止判据；与 goal_time_tolerance 一起判断完成。 |
| `goal_time_tolerance` | 仿真 s | 到轨迹终点后的允许延迟；请求中的有效容差可覆盖默认值，按仿真 now 检查。 |
| `gripper_speed` | m/s，每指 | 桥接默认指关节速度；总开口变化速度为两倍，action 请求可覆盖。 |
| `gripper_force` | N | 桥接默认驱动力上限；grasp profile 的 force 请求优先，不等于实测接触力。 |
| `gripper_goal_tolerance` | m，每指 | 夹爪目标位置验收容差；与 action 的 grasp epsilon 区分。 |
| `gripper_stall_timeout` | 仿真 s | 夹爪无运动持续多久视为停滞，关联接触/目标判定。 |
| `gripper_timeout` | 仿真 s | 夹爪动作最长执行时间，超时停止并报告。 |

## USB 实体和初始夹持

**消费代码：** [dual_fr3_maniskill/cable/model.py](../dual_fr3_maniskill/cable/model.py)、[dual_fr3_maniskill/cable/threading.py](../dual_fr3_maniskill/cable/threading.py)、[dual_fr3_maniskill/scenes/usb_cable.py](../dual_fr3_maniskill/scenes/usb_cable.py)。

**验证：** `test_initial_grasp_geometry.py`、`test_usb_only.py`；`check_usb_grasp_simulation.py` 验证解除世界支撑后保持。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `usb.mesh_scale` | 比例 | USB CAD 缩放；插入要求 1，改变它会影响质量惯量及孔匹配。 |
| `usb.attachment` | m，USB CAD | 线缆根部连接点，与 pin_length、USB 姿态共同决定初态。 |
| `usb.grip_center` | m，USB CAD | USB 被夹持中心，参与 TCP→USB 初始安装变换。 |
| `usb.tcp_grip_offset` | m，TCP | 夹持中心相对 TCP 的偏移；与 grip_center 分属两个坐标系。 |
| `usb.mass` | kg | USB 刚体质量，影响重力、惯量和抓持稳定。 |
| `usb.friction` | 无量纲 | USB 接触材料摩擦，不与 cable.friction 合并。 |
| `usb.open_finger_position` | m，每指 | 生成/准备阶段开爪位置，不是插入后的 release_width_m 总开口。 |
| `usb.finger_position` | m，每指 | 左爪准备目标，MTC 使用 2 倍作为总开口，受实体接触影响。 |
| `usb.contact_offset` | m | USB 接触检测壳厚度；与插座 offset、几何间隙共同影响提前接触。 |
| `usb.finger_contact_offset` | m | 真实手指碰撞壳厚度；影响法向接触及闭爪判定。 |
| `usb.orientation_direction` | forward/reverse | 代码接受的初始朝向；trunking bridge 由 ROS leader_orientation_direction 覆盖。 |

## 接触夹持状态判据

**消费代码：** [dual_fr3_maniskill/usb/grasp.py](../dual_fr3_maniskill/usb/grasp.py)。

**验证：** `test_usb_grasp.py`、`check_usb_grasp_simulation.py`；分别检查接触、释放支撑、稳定、滑移和掉落。所有时间为仿真时间。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `grasp.contact_min_force_N` | N，每指 | 两指均超过此法向载荷才计入接触保持；不是夹爪命令力。 |
| `grasp.contact_hold_s` | s | 连续双指接触确认时长，与 contact_min_force_N 配套。 |
| `grasp.close_timeout_s` | s | 闭爪后等接触确认的最长时限。 |
| `grasp.verification_min_s` | s | 解除世界支撑后至少观察多久，防止刚释放即报告稳定。 |
| `grasp.stable_hold_s` | s | 稳定抓姿连续保持时长，决定 release_verify 完成。 |
| `grasp.verification_timeout_s` | s | 释放后的稳定验证总时限，必须大于 verification_min_s、stable_hold_s。 |
| `grasp.slip_translation_m` | m，TCP→USB 相对姿态 | 相对平移漂移超限进入滑移计时，与 slip_hold_s 关联。 |
| `grasp.slip_rotation_rad` | rad，相对 SO(3) | 相对旋转漂移阈值，与平移一起判断滑移。 |
| `grasp.slip_hold_s` | s | 滑移阈值连续超限多久才失败，抑制单步噪声。 |
| `grasp.drop_translation_m` | m，相对姿态 | 掉落平移阈值，必须大于 slip_translation_m。 |
| `grasp.drop_rotation_rad` | rad，相对 SO(3) | 掉落旋转阈值，必须大于 slip_rotation_rad。 |
| `grasp.drop_hold_s` | s | 掉落条件保持时长；与滑移判据分别计时。 |
| `grasp.contact_loss_hold_s` | s | 丢失双指接触的容忍时间，超时报告接触失效。 |

## 线缆几何与材料

**消费代码：** [dual_fr3_maniskill/cable/model.py](../dual_fr3_maniskill/cable/model.py)、[dual_fr3_maniskill/cable/mpm_cable.py](../dual_fr3_maniskill/cable/mpm_cable.py)、[dual_fr3_maniskill/cable/rope_actor.py](../dual_fr3_maniskill/cable/rope_actor.py)、[dual_fr3_maniskill/cable/contacts.py](../dual_fr3_maniskill/cable/contacts.py)。

**验证：** `test_cable_model.py`、`test_cable_backends.py`、接触/质量/长度测试；再运行对应后端场景。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `cable.initial_layout` | straight/table_spiral | 初始线缆布局；影响初始接触，不能用于运行中重置形状。 |
| `cable.length` | m，中心线 | 线缆总长；与 pin_length、粒子间距或 links 共同决定离散长度。 |
| `cable.diameter` | m | 物理截面直径；影响质量密度、弯曲刚度、孔隙和碰撞半径。 |
| `cable.particle_spacing` | m | MPM 轴向体积采样间隔；减小增加粒子与计算量。 |
| `cable.pin_length` | m | USB 端固定材料长度 / Rope 根段长；必须小于总长，Rope 段长须大于直径。 |
| `cable.linear_density` | kg/m | 优先质量输入；加载时 density=linear_density/(π(diameter/2)²)。 |
| `cable.density` | kg/m³ | 无 linear_density 时的体密度输入；有线密度时被派生值覆盖，不应同时独立调。 |
| `cable.young_modulus` | Pa | MPM 弹性模量，影响变形；Rope 不以此替代 joint_stiffness。 |
| `cable.axial_young_modulus` | Pa | MPM 轴向伸长修正刚度；与 axial_iterations、时间步关联。 |
| `cable.axial_iterations` | 次/子步 | MPM 轴向约束迭代次数，影响伸长残差和性能。 |
| `cable.poisson_ratio` | 无量纲 | MPM 泊松比，影响体积/横向弹性响应。 |
| `cable.yield_stress` | Pa | MPM 塑性屈服阈值；当前大数值近似抑制屈服，不静默统一。 |
| `cable.friction` | 无量纲 | 线缆接触摩擦；不同预设刻意保留 0.1/0.5 等差异。 |
| `cable.contact_margin` | m | MPM 接触投影保护间隙，需小于半径；不同于 PhysX contact_offset。 |
| `cable.contact_iterations` | 次/子步 | 接触位置修正迭代，影响残留穿透和耗时。 |
| `cable.penetration_tolerance` | m | 穿透诊断/保护阈值，必须小于半径；当前简化 2 mm 为 .0008，其他预设 .0001。 |

## 穿线与夹爪孔几何

**消费代码：** [dual_fr3_maniskill/cable/threading.py](../dual_fr3_maniskill/cable/threading.py)、[dual_fr3_maniskill/cable/guide.py](../dual_fr3_maniskill/cable/guide.py)、[dual_fr3_maniskill/cable/aperture.py](../dual_fr3_maniskill/cable/aperture.py)。

**验证：** `test_threading.py`、`test_initial_grasp_geometry.py`、手指孔接触回归。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `guide.routing` | both_guides/usb_to_right | 初态经过双导孔或从 USB 到右孔；决定初始布线路径。 |
| `guide.center_offset` | m，手指导孔局部系 | 孔中心偏移，影响初始中心线与孔边距离。 |
| `guide.half_length` | m，孔轴向 | 孔有效长度的一半，用于初始化和穿孔几何检查。 |
| `guide.initial_margin` | m | 孔外初始化延伸距离，避免线缆端点恰在孔边。 |
| `guide.left_loop_clearance` | m | 左侧初始线圈离夹爪的余量，改变初始布局而非运行时控制。 |
| `guide.radial_tolerance` | m | 径向穿孔几何容差，必须小于线半径。 |
| `guide.axis_tolerance_deg` | degree | 线段与孔轴方向误差验收，小于 90°，不是插入 angle_tolerance_rad。 |

## MPM 数值与性能

**消费代码：** [dual_fr3_maniskill/cable/model.py](../dual_fr3_maniskill/cable/model.py)、[dual_fr3_maniskill/cable/mpm_cable.py](../dual_fr3_maniskill/cable/mpm_cable.py)、[dual_fr3_maniskill/cable/timestep.py](../dual_fr3_maniskill/cable/timestep.py)。

**验证：** `test_cable_model.py`、CUDA 接触等价检查和 MPM 场景；保持同一模型、显示与记录设置比较。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `mpm.frequency` | Hz | MPM 子步频率，与 sim_freq 整除关系及刚体子步配套。 |
| `mpm.grid_spacing` | m | 网格单元边长，越细越耗显存；与粒子间距、稳定性关联。 |
| `mpm.max_grid_cells` | 格点数 | 网格内存预算，超出时粗化或拒绝，不允许静默截断线缆。 |
| `mpm.adaptive_resolution` | 布尔 | 允许在预算内逐步增加网格间距，会改变数值分辨率。 |
| `mpm.max_grid_spacing` | m | 自适应粗化允许的上限，达到后仍超预算则报错。 |
| `mpm.grid_padding` | 格 | 包围盒外网格留白，关联运动空间与显存。 |
| `mpm.cuda_graph` | 布尔 | CUDA Graph 执行开关，主要影响调度性能。 |
| `mpm.gpu_grid_check` | 布尔 | 在 GPU 检查粒子是否超网格，关联诊断成本。 |
| `mpm.marker_stride` | 整数 | 旧显示抽样字段；仅 MPM 且缺 display.marker_stride 时作为补缺省。 |
| `display.marker_stride` | 整数 | 可视化线缆抽样步幅，不改变物理离散；显式字段优先，Rope 缺省 1。 |

## Rope-Actor 数值与异常保护

**消费代码：** [dual_fr3_maniskill/cable/rope_actor.py](../dual_fr3_maniskill/cable/rope_actor.py)、[dual_fr3_maniskill/cable/timestep.py](../dual_fr3_maniskill/cable/timestep.py)、[dual_fr3_maniskill/cable/rope_diagnostics.py](../dual_fr3_maniskill/cable/rope_diagnostics.py)、[dual_fr3_maniskill/cable/model.py](../dual_fr3_maniskill/cable/model.py)。

**验证：** `test_cable_backends.py`、`test_rope_finger_contact.py`、子步/CUDA 执行检查；已知历史 CAD 深度失败独立记录。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `rope_actor.links` | 段数 | 链条离散数，6–256；关联直径、pin_length、长度和运行成本。 |
| `rope_actor.root_joint` | fixed/spherical | USB 根连接旋转自由度；改变它是物理行为修改。 |
| `rope_actor.collision_geometry` | capsule/convex_capsule | 原生胶囊或凸近似胶囊；接触深度和孔沿响应可能不同。 |
| `rope_actor.solver_type` | pgs/tgs | PhysX 约束求解器类型；保留各预设选择。 |
| `rope_actor.engine_tolerance_length` | m | PhysX 数值尺度，影响内部接触/约束阈值；不是允许几何穿透量。 |
| `rope_actor.engine_tolerance_speed` | m/s | PhysX 速度尺度，与长度尺度一起设置求解上下文。 |
| `rope_actor.frequency` | Hz | 链条基本子步频率；与 sim_freq、insertion.rigid_frequency_hz 配合。 |
| `rope_actor.adaptive_timestep` | 布尔 | 按接触位移上限进一步细分子步；推荐简化配置 false，precise true。 |
| `rope_actor.max_contact_travel` | m/子步 | 自适应步长允许的接触点移动量；不得大于直径四分之一。 |
| `rope_actor.contact_offset` | m | 线缆形状接触壳；与手指孔几何及 USB offset 分开设置。 |
| `rope_actor.inertia_floor` | kg·m² | 细段主惯量下限，须严格大于 SAPIEN 1e-8；影响旋转动态。 |
| `rope_actor.tail_weight_mass` | kg | 自由尾端额外配重；0 关闭。缺省 0，MTC 默认的 simplified_2mm YAML 为 0.005。配重与末段为同一刚体，不改变 cable.linear_density。 |
| `rope_actor.tail_weight_radius` | m | 配重套外半径；缺省 0.004（外径 8 mm）。启用时不得小于线缆半径；参与显示、碰撞、惯量与几何检查。 |
| `rope_actor.twist_limit_deg` | degree | 球关节扭转限位，小于 180°。 |
| `rope_actor.bend_limit_deg` | degree | 球关节摆动限位，影响可弯曲程度。 |
| `rope_actor.joint_stiffness` | N·m/rad | 关节角驱动刚度；0 不额外弹性拉直。 |
| `rope_actor.joint_damping` | N·m·s/rad | 关节角速度阻尼，抑制相邻段相对运动。 |
| `rope_actor.linear_damping` | 1/s | 刚体线速度阻尼，影响自由运动衰减。 |
| `rope_actor.angular_damping` | 1/s | 刚体角速度阻尼，独立于关节阻尼。 |
| `rope_actor.solver_iterations` | 次 | PhysX 位置迭代，范围 1–255；推荐 200，其他预设 40。 |
| `rope_actor.solver_velocity_iterations` | 次 | PhysX 速度迭代，范围 1–255；插入启用时场景至少 10。 |
| `rope_actor.constraint_tolerance` | m | 关节/USB 连接残差保护阈值，不是接触穿透阈值。 |
| `rope_actor.max_stretch_ratio` | 比例 | 可选全线伸长保护；不设置则不启用，保留每场景差异。 |
| `rope_actor.max_speed` | m/s | 线缆过快运动保护，超限报告失败/诊断。 |

## 插座几何和坐标

**消费代码：** [dual_fr3_maniskill/usb/geometry.py](../dual_fr3_maniskill/usb/geometry.py)、[dual_fr3_maniskill/usb/scene.py](../dual_fr3_maniskill/usb/scene.py)、[dual_fr3_maniskill/cable/model.py](../dual_fr3_maniskill/cable/model.py)。

**验证：** `check_insertion_geometry.py`、`check_insertion_physx.py`；MoveIt 与 PhysX 顶点/变换等价检查。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `insertion.enabled` | 布尔 | YAML 场景开关；trunking ROS 桥接总用 insertion_enabled 覆盖。 |
| `insertion.visual_mesh` | 包 share 相对路径 | 显示插座 CAD；不决定孔壁接触。 |
| `insertion.collision_mesh` | 包 share 相对路径 | 插座碰撞 CAD；MoveIt 三角网格与 PhysX 凸体共享孔壁调整。 |
| `insertion.xy_frame` | TF frame | 将该坐标系的 [x,y,0] 变换后仅取 world XY。 |
| `insertion.xy_m` | m，xy_frame | 插座 CAD 原点的平面定位，关联机器人基座及孔位。 |
| `insertion.height_frame` | TF frame | 用该 frame 的 [0,0,height] 变换后仅取 world Z。 |
| `insertion.height_m` | m，height_frame | CAD 底面高度，不是孔中心高度或槽壁顶部。 |
| `insertion.quaternion_wxyz` | 单位四元数，world | 插座绝对旋转，不再乘 xy_frame/height_frame 旋转。 |
| `insertion.hole_center_m` | m，插座 CAD | 孔口中心，X 必须为 0；影响目标、测量和力矩参考点。 |
| `insertion.clearance_yz_m` | m，插座 Y/Z，每侧 | 孔尺寸=USB 金属截面+2×间隙；与接触壳厚度分开验证。 |
| `insertion.contact_offset_m` | m | 插座接触壳；与 USB 壳之和必须结合几何间隙检查。 |
| `insertion.rest_offset_m` | m | 实体静止接触偏移，当前刚体插入只允许 0。 |

## 插入运动、反馈、成功判据和清理

**消费代码：** [dual_fr3_maniskill/usb/insertion.py](../dual_fr3_maniskill/usb/insertion.py)、[dual_fr3_maniskill/usb/bridge.py](../dual_fr3_maniskill/usb/bridge.py)、[dual_fr3_maniskill/usb/scene.py](../dual_fr3_maniskill/usb/scene.py)、[../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/planning.py](../../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/planning.py)、[../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/motion.py](../../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/motion.py)、[../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/pipeline.py](../../dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/insertion_task/pipeline.py)。

**验证：** `test_insertion.py`、`test_insertion_services.py`、MTC `test_terminal_insertion.py`，再局部插入/障碍探针和完整 ROS skill。仿真时间与墙钟超时分开验证。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `insertion.preinsert_m` | m，插座轴 | 孔前距离，目标 depth=-preinsert；与总推进上限配套。 |
| `insertion.end_clearance_m` | m | 金属端到 CAD 后壁余量；target_depth 必须等于 .011 减此值。 |
| `insertion.target_depth_m` | m，沿插座 -X 为正 | 目标实测插入深度；关联末端余量、成功容差和过冲保护。 |
| `insertion.max_advance_m` | m | 指令推进累计路程上限，至少覆盖 preinsert+target_depth。 |
| `insertion.speed_m_s` | m/s | 无阻力时推进速度，不控制 MoveIt 接近速度比例。 |
| `insertion.acceleration_m_s2` | m/s² | 反馈速度变化率限幅，影响冲击和到位速度。 |
| `insertion.resistance_gain_m_N_s` | m/(N·s) | 阻力减速增益：max(0,speed-gain×滤波阻力)。 |
| `insertion.filter_tau_s` | 仿真 s | 轴向阻力一阶滤波时间常数，不屏蔽原始/子步峰值保护。 |
| `insertion.axial_limit_N` | N，插座 +X | 抵抗 -X 插入的原始轴向载荷上限。 |
| `insertion.lateral_limit_N` | N，插座 YZ | 侧向接触合力上限，与孔隙和对齐误差关联。 |
| `insertion.torque_limit_Nm` | N·m，孔中心 | 接触力矩上限，不能直接当作 TCP 力矩阈值。 |
| `insertion.lateral_tolerance_m` | m，孔口 YZ | 对齐及成功判据的横向偏差上限。 |
| `insertion.angle_tolerance_rad` | rad，完整 SO(3) | USB 相对期望插座姿态的旋转误差上限。 |
| `insertion.depth_tolerance_m` | m | 到位深度容差，同时保护超调，不能用放宽它掩盖孔底碰撞。 |
| `insertion.hold_s` | 仿真 s | 全部成功条件连续保持多久；确认成功后才允许固定约束。 |
| `insertion.retention_speed_m_s` | m/s，相对插座 | 成功/固定前 USB 相对线速度上限。 |
| `insertion.retention_angular_rad_s` | rad/s，相对插座 | 成功/固定前 USB 相对角速度上限。 |
| `insertion.blocked_s` | 仿真 s | 未达到目标且无实际深度进展的最长时长。 |
| `insertion.progress_epsilon_m` | m，实测深度 | 重置停滞计时所需的最小进展，关联 speed、blocked_s。 |
| `insertion.timeout_s` | 仿真 s | 反馈阶段总时限；暂停仿真不代替 heartbeat 保护。 |
| `insertion.wall_watchdog_s` | 墙钟 s | 显式 heartbeat 控制租约期限；status 查询不续租。 |
| `insertion.joint_speed_rad_s` | rad/s，每关节 | 差分 IK 指令变化上限，关联 control_freq。 |
| `insertion.tracking_limit_m` | m，world TCP / 孔前深度 | IK 跟踪偏差及 start 深度偏差保护。 |
| `insertion.retain_after_success` | 布尔 | 成功后创建保持约束；false 停在未固定成功状态。 |
| `insertion.release_after_retention` | 布尔 | 固定确认后开左爪；不影响先行右爪释放。 |
| `insertion.return_after_release` | 布尔 | 左爪释放后退出回位；不影响先行右臂回位。 |
| `insertion.retreat_m` | m，本侧 TCP -Z | 右臂开爪后、左臂固定开爪后的退出距离。 |
| `insertion.approach_lift_m` | m，world +Z | 左臂孔前接近的可选抬高，0 省略；属于整段 MoveIt 预检。 |
| `insertion.planning_attempts` | 次 | 插入接近/回位规划预算；独立于主任务 planning_attempts。 |
| `insertion.planning_timeout_s` | s | 终末 MoveTo 的规划超时；withdraw 不额外设置此项。 |
| `insertion.max_path_length_ratio` | 无量纲 | TCP 路程/起终点直线距上限；推荐 YAML 3.0，代码回退 1.5。 |
| `insertion.rigid_frequency_hz` | Hz | 插入刚体最小子步频率，须为 sim_freq 整数倍；基础 dt 取与线缆更细者。 |
| `insertion.release_width_m.left` | m，总开口 | 固定后左爪打开目标，复用 MTC open profile。 |
| `insertion.release_width_m.right` | m，总开口 | 插入前右爪打开目标，与孔/线槽碰撞配套。 |
| `insertion.release_min_half_width_m.left` | m，每指 | 物理服务确认左侧两根手指都已打开的门限。 |
| `insertion.release_min_half_width_m.right` | m，每指 | 物理服务确认右侧两根手指都已打开的门限。 |

## 力输出和记录

**消费代码：** [dual_fr3_maniskill/sensing/force_output.py](../dual_fr3_maniskill/sensing/force_output.py)、[dual_fr3_maniskill/sensing/forces.py](../dual_fr3_maniskill/sensing/forces.py)。

**验证：** `test_forces.py`、`test_insertion_services.py`；检查单位、参考坐标和 JSONL 非覆盖写入。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `force_enabled` | 布尔 | 力话题/记录启用；插入反馈状态自身仍独立工作。 |
| `force_record_path` | 新 JSONL 路径 | 空值关闭记录；已有文件拒绝覆盖，性能比较时保持一致。 |
| `force_usb_base_names` | 逗号分隔实体名 | 指定已加载 USB 基座的反力输出对象；不创建新物理实体。 |

## RGB-D 相机及感知启动

**消费代码：** [dual_fr3_maniskill/sensing/camera.py](../dual_fr3_maniskill/sensing/camera.py)、[dual_fr3_maniskill/ros/launch.py](../dual_fr3_maniskill/ros/launch.py)。

**验证：** `test_perception_camera.py`、启动测试；再核对 CameraInfo、深度与 TF。感知算法参数见感知包参数索引。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `perception_enabled` | 布尔 | 启用独立相机和感知节点，要求 ManiSkill 线缆场景且 load_cable=true。 |
| `perception_python` | 解释器路径 | SAM2 环境，优先 PERCEPTION_PYTHON，否则 .venv-perception。 |
| `perception_config` | ROS YAML 路径 | 感知节点参数源，空值使用包内 perception.yaml。 |
| `perception_checkpoint` | 模型路径 | 覆盖 checkpoint，缺省可来自 SAM2_CHECKPOINT；不影响物理模型。 |
| `perception_camera_config` | ROS YAML 路径 | 覆盖场景相机预设，trunking 与 usb_cable 保留各自视角。 |
| `perception_preview` | 布尔/空 | 非空覆盖感知 preview，空值保留 YAML。 |
| `perception_device` | 设备名/空 | 非空覆盖 SAM2 推理设备，空值保留 YAML。 |
| `perception_save_dir` | 目录/空 | 非空覆盖感知保存目录，空值保留 YAML。 |
| `camera_width` | pixel | RGB-D 图像宽度，与高度及显存、推理耗时关联。 |
| `camera_height` | pixel | RGB-D 图像高度，修改后需核对内参。 |
| `camera_fovy` | rad | 垂直视场角，决定投影内参及可见范围。 |
| `camera_near` | m，相机光轴 | 近平面，近于此值不可见。 |
| `camera_far` | m，相机光轴 | 远平面，与感知 max_depth 分开设置。 |
| `camera_eye` | m，world | 相机位置；改动需重查遮挡和 TF。 |
| `camera_target` | m，world | 相机瞄准点，和 eye/up 共同决定朝向。 |
| `camera_up` | 方向向量，world | 相机上方向；内部转换 OpenGL 与 ROS optical 轴约定。 |
| `camera_frequency` | Hz | 图像发布频率，不改变物理子步。 |
| `camera_frame` | TF frame 名 | ROS 光学坐标系名称，须与 CameraInfo/深度一致。 |

## 开发环境构建工具

**消费代码：** [dual_fr3_maniskill/engine/warp_setup.py](../dual_fr3_maniskill/engine/warp_setup.py)。

**验证：** `test_warp_setup.py`；不改变运行时物理参数。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `cuda_path` | CUDA toolkit 路径 | Warp 构建工具显式工具链，省略时从环境/PATH 检测。 |
| `force` | 布尔 CLI 开关 | 即使已有 warp.so 也重建；与 force_enabled 力输出无关。 |


## 仍在代码中的固定设置

以下不是 ROS/YAML 参数。本轮只记录现值，不把它们变成新接口，也不统一不同场景的值。

| 设置 | 当前值与单位 | 文件、作用及验证 |
| --- | --- | --- |
| 机器人 PD stiffness | 手指 1000 N/m，机械臂 4000 N·m/rad | `robot/simulation.py:DualFR3Agent._setup_controllers`，影响位置响应；轨迹与接触抓持测试 |
| 机器人 PD damping | 手指 50 N·s/m，机械臂 200 N·m·s/rad | 同上，影响振荡和跟踪；与 stiffness、控制频率一起验证 |
| 默认 drive force_limit | 手指 min(40 N, URDF effort)，机械臂取 URDF effort | 同上；动作执行可再次设置夹爪力，检查 `/maniskill/forces` |
| 普通机器人 scene contact_offset / solver_iterations | .001 m / 20 次 | `robot/simulation.py:_get_default_scene_config`；线缆/插入场景另有覆盖，不能视为全局有效值 |
| 插入 PCM / 最少迭代 | PCM 开启，位置至少 200、速度至少 10 次 | `scenes/usb_cable.py`，插入启用时生效；看物理子步与接触探针 |
| 普通显示相机 | eye=[2,2.3,1.8] m，target=[.45,.7,.25] m（world），1280×960，fovy=1 rad，near=.01 m、far=10 m | `robot/simulation.py:_register_render_cameras/_setup_viewer`；仅查看器，不是 sensing/camera.py 的 RGB-D 相机 |
| shader_dir | robot 默认 ibl；场景可为 MPM 点显示另选 shader | `robot/simulation.py:DualFR3Env`、`cable/backends.py`；仅渲染设置，检查后端加载 |
| USB TIP / SHOULDER | [0,.0179,0] m / .0059 m（USB CAD） | `usb/geometry.py`；金属长度=TIP.y−SHOULDER，几何报告及原生探针 |
| STEM_CROSS_SECTION_M / DEPTH | [.00445,.012] m（插座 YZ）/ .011 m | 同上；孔尺寸、target_depth/end_clearance 的共同依据 |
| HOLE / DEFAULT_CLEARANCE_YZ_M | [0,.0175,.0686] m / [.0004,.0004] m 每侧 | 同上；YAML 可覆盖目标孔中心/间隙，不能修改 CAD 分选参考中心来移动孔 |
| USB_IN_SOCKET | USB +Y→插座 −X，USB +X→插座 +Y，USB +Z→插座 +Z | 同上；目标和测量共用旋转，验证 tcp_goal 与 measure |

插座固定物性、几何选择边界、IK、ready/静止门限、heartbeat 等插入固定值见 [插入参数说明](insertion_parameters.md)。
测试/诊断脚本的输出路径和测试初态参数通过各脚本 `--help` 查阅；它们不覆盖运行场景默认值。

维护参数时运行：

```bash
python3 src/dual_fr3_maniskill/scripts/check_parameter_docs.py
# 修改了声明/配置后先更新含义表，再重建默认值附录：
python3 src/dual_fr3_maniskill/scripts/check_parameter_docs.py --write-defaults
```

检查器从源码静态读取声明、场景 YAML、xacro arg 和指定数据结构；不导入 ROS 或物理引擎。
新参数漏写作用表、默认值附录过期都会返回非零。动态新增声明方式需同时扩展扫描器。
