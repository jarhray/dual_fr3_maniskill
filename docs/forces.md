# 末端与手指作用力采集

仿真桥接默认启用只读力采集，不改变关节控制器、夹持约束、接触参数或 MTC 动作。
普通机器人、独立 USB 和 MTC 延迟生成线缆场景共用接口。数据用于受力观察和实验记录，
同时记录 USB 接触夹持、稳定性和滑移/脱落监测。监测结合仿真真值相对位姿与接触，
不实现连续闭环力调节、自动增力、自动恢复或插入成功判定。

## 输出形式

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/maniskill/forces` | `std_msgs/msg/String`，JSON | 完整时间窗、各传感位置的力/力矩、法向载荷、峰值、接触对象、可用性及缺失分量 |
| `/maniskill/forces/{left,right}/cable/wrench` | `geometry_msgs/msg/WrenchStamped` | 线缆直接作用于该手的载荷（含 MPM 理想导向反力），归算到 TCP |
| `/maniskill/forces/{left,right}/fixtures/wrench` | 同上 | 已加载静态结构直接作用于该手的载荷 |
| `/maniskill/forces/{left,right}/usb_base/wrench` | 同上 | 上一项中属于指定 USB 插接基座的部分 |
| `/maniskill/forces/{left,right}/usb/wrench` | 同上 | USB 直接作用于该手的法向接触载荷，归算到 TCP；相向夹力可能抵消 |
| `/maniskill/forces/fingers/<link>/{cable,fixtures,usb}/wrench` | 同上 | 各手指直接接触相应对象的载荷，归算到手指坐标系原点 |
| `/maniskill/forces/usb/{fixtures,usb_base,cable}/wrench` | 同上 | 对应对象作用于 USB 的载荷；`cable` 包含 USB—线缆连接反力，归算到 USB 原点 |
| `/maniskill/forces/usb/fingers/<link>/wrench` | 同上 | 每个真实手指分别作用于 USB 的法向接触载荷，归算到 USB 原点 |
| 上述各传感位置下的 `/normal_load` | `std_msgs/msg/Float64` | 可获得时输出法向载荷，单位 N；时间戳和完整含义以 JSON 为准 |

手指 link 为 `left_fr3_leftfinger`、`left_fr3_rightfinger`、
`right_fr3_leftfinger`、`right_fr3_rightfinger`。每个存在的手指单独统计，
两指相反方向的夹持力不会因为腕部合力抵消而丢失。
`fingers/<link>/usb` 与 `usb/fingers/<link>` 是同一接触的作用力/反作用力，
表达坐标系和力矩参考点不同；两者不能相加当作独立载荷。

`WrenchStamped` 仅在该信号可用时发布。消费方必须检查消息时间戳和 JSON 中的
`available`、`scope`、`missing_components`，不能将保留的旧消息视为当前读数。
部分可用的法向接触力仍可画曲线，其 `scope` 为 `partial_interaction_wrench`。

### 坐标、符号和采样

- 力单位 N，力矩单位 N·m，时间为 `/clock` 对应的仿真秒。
- 正负号表示外部对象**作用在传感位置所属刚体上**的力。
- 末端话题使用对应 `*_fr3_hand_tcp` 坐标系；手指和 USB 话题使用自身坐标系。
  力矩包含从接触位置到该坐标系原点的力臂。
- 每个实际物理子步采集一次，控制周期汇总并发布一次。默认线缆场景为 50 Hz，
  不受 `/joint_states` 的 25 Hz 发布频率或原有诊断的墙上时间限频影响。
- 本轮接触夹持验收使用 `rope_actor`；MPM 的动态释放/脱落耦合延期，保留原有物理步长和失稳检查。
- 平均值为 `Σ冲量 / Σ实际物理步长`。每个子步先转换到当时的传感坐标系，
  因此是随传感位置转动的局部轴平均值。`frame_poses_world` 记录时间窗末端位姿。
- `peak_substep_force_N` 是窗口内单个物理子步的合力模长峰值，不能解释为连续时间的真实冲击峰值。
- `normal_load_N` 是接触点法向冲量绝对值之和除以时间窗长度，不是合力模长，
  也不是用户设置的夹爪驱动力上限。
- 法向接触产生的力矩仅包含已报告法向力的力臂项，不能标成完整摩擦力矩、
  实际夹爪电机力矩或六维力传感器完整读数。JSON 同时提供 `torque_reference`、
  `torque_interpretation`、`force_unit` 和 `torque_unit`。

## 数据来源与范围

1. **原生刚体接触**：SAPIEN/PhysX 4 报告的法向接触冲量。保留接触对象，
   将 Rope-Actor 手指代理映射回原手指，并且只计一次。
   切向摩擦冲量未由当前报告接口提供，`missing_components` 标记
   `tangential_contact_impulses`；这不意味着物理求解没有摩擦。
2. **Rope-Actor 固定端**：通过 `_rope_physx.read_pair_constraint` 读取线缆首段
   与 USB 之间的约束反力，包含牵引载荷，而非仅统计 USB 表面碰撞。
   PhysX 的约束报告不包含关节驱动项产生的扭矩；柔性根部的弯曲驱动扭矩不能当作已读取。
   缺少扩展、约束断开或约束报告休眠时，对应信号标为不可用。
   输出位置是 `usb/cable`；它不等同于实际传到手指的载荷，也不会再隐式加入 `left/cable`。
3. **MPM**：读取现有固定端、刚体接触与理想滑孔代码实际施加的反作用冲量，
   保留力矩，合并当前求解周期内各次耦合反作用。MPM 不额外进行 GPU 接触查询。
   这些是模型施加的反作用载荷；接触汇总未提供逐点法向载荷，因此该标量为 `null`。
4. **MPM 右侧理想滑孔**：约束反力作用在 TCP，不能唯一分配到两根手指。
   右手指通道只含实际接触求解的部分，并标记 `ideal_guide_load_not_resolved_per_finger`。

这些信号按作用来源划分，并非完整腕部六维传感器读数。USB 自重和自身惯性不作为
单独的腕部载荷加入；线缆重力与运动引起的反力会体现在固定端载荷中。
当前场景没有 USB 插接基座，`usb_base` 继续为 `available=false`、数值 `null`，
不代表测得插入力为零。临时世界固定约束的支撑载荷未作为手指载荷输出，
`usb_grasp.external_support=true` 时不能声称 USB 已由手指独立夹稳。
顶层 `temporary_support` 输出支撑是否仍生效、世界约束个数、MPM 临时支撑粒子数，
`count` 为这两项总和；`material_intervals_m` 记录从 USB 出线点起算的局部支撑弧长区间。
`initial_layout` 保存创建时左右孔的径向/轴向偏差和直线覆盖检查，
是初始几何证据，不代表释放后仍维持初始直线。

USB 的存在独立于线缆对象判断。`load_cable:=false` 时仍采集 USB—真实手指接触；
所有线缆通道以 `cable_disabled` 标记并输出 `null`，顶层 `cable.enabled=false`。
USB 所受连接力、直接接触力各自保留，不依据 `mount_drive` 推断持有者或传力路径。

## 夹持状态和相对位姿

JSON/JSONL 的 `usb_grasp` 记录：

| 状态 | 含义 |
| --- | --- |
| `not_created` | USB 未创建 |
| `supported` | 世界临时定位仍生效，等待持续双侧真实手指接触 |
| `contact_ready` | 双侧接触满足力与持续时间门限，仍受外部支撑，允许解除定位 |
| `verifying` | 世界约束与局部临时支撑已移除，记录释放基准并等待验证 |
| `stable` | 释放后的相对位姿变化、双侧接触与持续时间满足门限 |
| `slipping` | 相对位姿漂移或双侧接触不足持续超过门限 |
| `dropped` | USB 离开夹持位姿范围，或两侧接触持续消失；需重置后开始新试验 |
| `failed` | 超时、解除定位失败或仿真异常；`reason` 给出原因 |

`stable` 只有释放定位后才可能为真。闭合命令结束、预设开度或电机停滞均不作为成功证据。
`ready_to_release` 根据每个刚体子步的真实手指接触判断，不包含 Rope-Actor 缆接触代理。
`reason`、`transitions`（最近 32 次变更）与释放时刻均随每个记录窗输出。

`relative_pose = inverse(T_tcp) * T_usb`，平移单位 m，四元数顺序 w/x/y/z，
TCP 使用 `left_fr3_hand_tcp`。`release_baseline` 在真正解除定位后仅记录一次；
`relative_translation_m`、`relative_rotation_rad` 为相对此基准的位移模长与最短转角。
`relative_linear_velocity_m_s` 和 `relative_angular_speed_rad_s` 为相邻子步相对变换差分，
不是 USB 世界速度。机械臂与 USB 整体平移、旋转不会因此误报滑移。
**这是仿真真值辅助检测，不代表仅靠法向接触力就能观测全部滑移。**

`cable_config` 的 `grasp` 节可设置以下正数；持续时间全部使用仿真秒，支持两种后端及 USB-only：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `contact_min_force_N` | 0.10 N | 每指有效接触的法向载荷下限 |
| `contact_hold_s` | 0.10 s | 释放前双侧接触连续成立的时间 |
| `close_timeout_s` | 5.0 s | 开始闭合后等不到持续接触即失败 |
| `verification_min_s` | 0.35 s | 释放后的最短观察时间 |
| `stable_hold_s` | 0.25 s | 接触与相对位姿正常的连续时间 |
| `verification_timeout_s` | 3.0 s | 始终无法完成初次验证时的超时 |
| `slip_translation_m` | 0.002 m | 相对位移滑移门限 |
| `slip_rotation_rad` | 0.14 rad（约 8°） | 相对转角滑移门限 |
| `slip_hold_s` | 0.10 s | 漂移或双侧接触不足的去抖时间 |
| `drop_translation_m` | 0.035 m | 离开夹持区域的相对位移门限 |
| `drop_rotation_rad` | 1.05 rad（约 60°） | 离开夹持姿态的转角门限 |
| `drop_hold_s` | 0.10 s | 离开夹持区域的去抖时间 |
| `contact_loss_hold_s` | 0.25 s | 两侧接触同时消失后判定脱落的持续时间 |

这些是可调实验判据，不是经真实 USB 标定的物理常数。短暂接触抖动不会立即判为脱落；
姿态恢复且重新满足稳定持续时间后可以从 `slipping` 回到 `stable`，
但释放基准不会重新跟随 USB，持续位移不能被自动抹去。

## 配置和记录

在传给 `maniskill_config` 的 YAML 中设置以下 ROS 参数；其余参数沿用所选场景的配置：

```yaml
dual_fr3_maniskill:
  ros__parameters:
    force_enabled: true
    force_usb_base_names: ""  # 场景中静态 actor 的精确名称；多个名称用逗号分隔
    force_record_path: ""     # 可选，例如 /tmp/forces_run_001.jsonl
```

上述参数在启动时读取，修改后重启桥接。`force_enabled` 默认 `true`。记录路径默认空，避免自动生成大文件；非空时使用独占创建，
已有记录不会被覆盖。记录每个控制周期的 JSON，包含完整元数据、均值、峰值和
`activity`（各臂执行轨迹/保持、夹爪运动/保持）。`activity` 不是 MTC 阶段编号。
每行立即刷新；运行中磁盘写入失败会明确报错并停止文件记录，ROS 发布继续。

`force_usb_base_names` 使用 SAPIEN 场景 actor 名称，不是 STL 路径。
基座必须已经通过场景描述加载为静态结构；采集配置不会自行放置 mesh。
未配置或没有任何匹配的已加载基座时，基座通道标为不可用；
`configured_usb_bases` / `loaded_usb_bases` 可用于核对。
其余已加载结构仍由 `fixtures` 通道统计。无接触且传感通道有效时输出零；
未生成/禁用线缆、未生成 USB 或缺失基座均使用 `null`。

启动原有 ManiSkill/MTC 流程后可查看：

```bash
ros2 topic echo /maniskill/forces/left/cable/wrench
ros2 topic echo /maniskill/forces/fingers/right_fr3_leftfinger/cable/wrench
ros2 topic echo /maniskill/forces/usb/fingers/left_fr3_leftfinger/wrench
ros2 topic echo /maniskill/forces/usb/fingers/left_fr3_rightfinger/wrench
ros2 topic echo /maniskill/forces
```

`WrenchStamped.wrench.force.{x,y,z}` 和 `torque.{x,y,z}` 可直接供 ROS 绘图工具使用。
结构化记录也可以通过 rosbag 保存：

```bash
ros2 bag record -o /tmp/force_bag_001 /clock /maniskill/forces \
  /maniskill/left_tcp_pose /maniskill/right_tcp_pose
```

数值故障导致线缆仿真暂停时，结构化话题发布不可用状态，不继续发布旧的力消息。

## 验证

加载 ROS 和工作区后：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test/test_forces.py \
  src/dual_fr3_maniskill/test/test_usb_grasp.py \
  src/dual_fr3_maniskill/test/test_usb_only.py
```

测试覆盖不等物理步长的平均力与峰值、坐标变换和力矩参考点、接触对顺序、
两指相向受力、代理归属、不可用数据、MPM 空间冲量转换，以及真实 PhysX 中
已知外力/外力矩的约束反力和手指受到已知载荷时的基座接触力。
状态测试覆盖接触持续时间、支撑未解除、相对变换、整体运动不误判、滑移/脱落去抖、超时、
手动释放、重置和 USB-only 力输出；算法测试不等于实际机器人夹持成功。
USB-only 生命周期测试以原生 PhysX/USB mesh 执行创建、世界定位、释放和自由落体，
禁止调用线缆工厂、后端初始化和 guide 构造，并检查重复创建、释放失败与清理。
该测试替换了机器人加载器与渲染器，不能单独证明真实手指网格上的稳定夹持。
这些验证不等同于完整双臂走线或真实 USB 插接力学的标定。

实际场景的短时采集、ROS 订阅和 JSONL 写入可用以下脚本验证（需要正常的 Vulkan/CUDA 环境）：

```bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 .venv/bin/python \
  src/dual_fr3_maniskill/test/check_forces_simulation.py \
  --solver rope_actor --ros --output /tmp/force_check_rope_001.jsonl
```

`--solver mpm` 验证另一后端。脚本使用记录的准备位姿，检查生成前一次控制周期和
生成后三次控制周期；只验证短时采集链路，不执行完整走线。输出路径必须尚不存在。
增加 `--usb-only` 可验证无缆的真实场景力输出；左夹爪保持张开、USB 仍临时定位，
因此此脚本也不会把接触零值或外部支撑误记为抓取成功。

MPM 临时支撑的 CUDA kernel 与图缓存释放可单独验证，无需渲染或加载机器人：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_mpm_support_cuda.py
```

它检查局部支撑覆盖、两份状态图的重用/销毁、解除支撑瞬间粒子位置与速度保持、
重新构图后原支撑粒子可以运动，以及渲染粒子实体重复关闭时只删除一次。
这是支撑生命周期专项检查，不代替完整 MPM 场景接触夹持验证。
支撑仍生效时直接调用 `MPMCable.reset()` 会拒绝；使用场景重置服务统一删除 USB、
线缆与支撑，避免保留旧世界定位目标。

独立状态话题 `/maniskill/usb/grasp_state` 发布同一 `usb_grasp` JSON 对象，支持关闭力采集后的状态观察。`creation_world_pose` / `world_displacement_m` 描述创建以来的世界移动；`release_baseline` / `relative_displacement_m` 描述夹爪内相对滑移，分类只使用后者及接触/时长条件。可运行 `ros2 run dual_fr3_maniskill watch_usb_grasp.py` 显示毫米、角度和每指 N 载荷。
