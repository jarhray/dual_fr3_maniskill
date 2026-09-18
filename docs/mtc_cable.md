# MTC 的 USB 接触夹持与可选线缆

本页说明 USB 端头接触夹持、接触载荷采集，以及稳定、滑移和脱落监测。当前 MTC 入口还默认开启已知孔位的终末插入，包含阻力反馈、固定和释放回位，见 [USB 插入](usb_insertion.md)。插座安装反力在满足可辨识条件时可用；未加载插座的场景仍为 unavailable。接触夹持不自动增力或重新抓取。

历史接触夹持记录采用 **Rope-Actor + USB-only**。当前 MTC 入口默认 `rope_actor` 并加载线缆；其他独立物理入口保留 `mpm` 默认，历史结果不等同于本轮带线缆验证。

## 启动

在工作区根目录加载环境后，两种模式可直接运行；不传 `cable_config` 时统一加载 `trunking_cable_simplified_2mm.yaml`，其中已显式列出 `usb` 和 `grasp` 的夹持参数：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 正常模式：USB + Rope-Actor 线缆。
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill load_cable:=true cable_solver:=rope_actor \
  execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"

# 快速调试：仅 USB；夹持验证后继续原双臂下降和后续 MTC 轨迹。
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill load_cable:=false cable_solver:=rope_actor execute:=true \
  preparation_interactive:=false maniskill_python:="$PWD/.venv/bin/python"
```

`load_cable:=false` 从 launch 传到 ROS 节点和仿真构造，不创建线缆后端、粒子、胶囊链、连接关节、代理、guide，也不初始化线缆 Warp/CUDA 内核或逐帧求解。机器人自身渲染初始化仍保留。USB、实际姿态 TF、世界临时定位、手指碰撞、载荷和监测全部保留，独立创建 USB 不依赖右臂；MTC 入口仍执行原双臂准备，以保持后续轨迹一致。`maniskill_cable:=false` 是旧的**关闭整个 USB/线缆场景**开关，与 USB-only 不同；USB-only 要保留 `maniskill_cable:=true`。

也可只启动物理和 MoveIt，手动调试：

```bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py load_cable:=false cable_solver:=rope_actor \
  maniskill_python:="$PWD/.venv/bin/python"
```

独立 `usb_cable` 自动创建世界临时定位的动态 USB；MTC `trunking_cable` 延迟至 `/maniskill/cable/spawn` 创建。`execute:=false` 仅规划，不会替 MTC 创建物体。默认交互确认仍可保留；无终端时使用上面明确的自动执行设置。

## 准备和执行顺序

正常模式完整预检后：两侧张开 → 按关键点准备目标 `spawn` 并固定 → 张开接近准备位姿 → 左夹爪接触闭合 → 右孔闭合 → `release` → `verify` → 更新实测规划附着体 → 重规划剩余运输及孔前接近 → 只等待一次 Enter → 下降/布线。USB-only 保留同一套双臂准备、下降和后续 MTC 轨迹，只禁用线缆创建/求解；日志明确标记为 USB 轨迹调试，不声称完成真实线缆布线。

左夹爪闭合目标为每指 0 m，使用用户夹爪 profile 的有限驱动力上限；真实物体阻挡手指。`stalled` 或 `reached_goal` 仅表示动作终止，不能证明抓持成功。`release` 等待两侧实际接触持续满足门限，移除世界约束和所有局部临时线缆支撑；`verify` 再等待无外部支撑的稳定观察窗口。失败、超时、缺少服务时停止后续搬运。ManiSkill 夹爪结果按 profile 的仿真秒数等待，另有 `max(120, 30 × profile.timeout)` 墙钟秒上限，避免慢速 Rope-Actor 被原 10 秒墙钟等待提前取消，也防止仿真停钟后无限等待。每个下降或正式搬运阶段前还会检查 `/maniskill/usb/status`，非 `stable` 不继续。

USB 使用实际 STL 的凸包碰撞、15 g 质量，材料摩擦系数 `usb.friction` 默认为 `0.5`；`usb.contact_offset` 为 `0.0002 m`，`usb.finger_contact_offset` 为 `0.0001 m`；静止偏移不变，没有屏蔽其碰撞。USB 距离用于提前检测下降过程中的 USB—线缆接触，未放宽 `0.1 mm` 穿透保护阈值。`usb.open_finger_position` 默认每指 `0.02 m`，驱动力上限采用 profile（本次验证为每指 10 N）。这些参数未按实物标定。

创建位置来自 `initial_leader_index` / `initial_follower_index` 指定的关键点、`preparation_height`、路径朝向和 `tool_roll/tool_pitch`，与准备 MoveTo 使用同一个目标。MTC 在接近前通过参数 `usb_preparation_poses` 将左右 TCP 目标送到仿真；spawn 在仿真步边界将关键点坐标系变换到 world，再加 USB 抓取偏移和各孔中心偏移。不会从尚未到位的当前 TCP 猜测创建位置。规划世界物体同样在接近前加入。

当前 `guide.routing: usb_to_right`：USB 出线后平滑接到右孔，不穿左孔，也不创建左孔临时支撑。提前定位期间，右孔附近的短线段由世界支撑保持；右臂尚未接近时，后端将“已经穿入真实右孔”的断言标为等待，而所有真实碰撞保持开启。正常释放前确认右孔已经对准定位线缆；释放后恢复真实孔内检查。此提前定位路径目前仅验收 `rope_actor`，带线缆的 MPM 路径会明确拒绝。

USB 始终是动态刚体，世界定位只在创建时计算一次，不随机械臂运动。解除定位只移除约束，不瞬移、不清速度、不焊回 TCP。USB—线缆物理连接继续保留。允许重新张开夹爪并观察滑移/脱落。

MoveIt `attached_collision_object` 只在 release/verify 成功后提交，它是规划表示，不是 SAPIEN 固定约束。生成阶段使用 status 提供的实际 `world_pose` 创建世界碰撞物体，因此重复 spawn 后即使 TCP 已移动，规划物体仍留在真实位置。确认抓持时使用实测 `relative_pose` 附着；无效或失稳状态阻止规划更新。重试生成先移除旧规划附着体。完整预检可预测后续附着几何，但不会提前执行物理附着。

## 服务和手动调试

以下接口均为 `std_srvs/srv/Trigger`，操作在单线程仿真步边界串行执行。等待条件使用仿真时间；物理慢于实时时，墙上耗时可能更长。

| 服务 | 行为 |
| --- | --- |
| `/maniskill/cable/spawn` | 张开且空闲时创建 USB/可选线缆；已创建时幂等返回，不重复生成 |
| `/maniskill/usb/release` | 等待持续双指接触后解除定位；接触不足/超时返回失败 |
| `/maniskill/usb/release_manual` | 调试用强制解除定位；不把无接触释放视为成功抓持 |
| `/maniskill/usb/verify` | 等待释放后的稳定窗口；仍受支撑、滑移/脱落/超时报失败 |
| `/maniskill/usb/status` | `message` 为 JSON 快照；只有 `stable` 时 `success=true` |
| `/usb_cable_demo/reset` | 空闲时清理 USB、线缆、全部支撑和观察基准；重新张开后再 spawn |

```bash
ros2 service call /maniskill/usb/status std_srvs/srv/Trigger '{}'
ros2 service call /maniskill/usb/release_manual std_srvs/srv/Trigger '{}'
ros2 action send_goal /left_franka_gripper/gripper_cmd control_msgs/action/GripperCommand \
  '{command: {position: 0.02, max_effort: 10.0}}'
ros2 topic echo /maniskill/forces
```

## 状态与阈值

| 状态 | 含义 |
| --- | --- |
| `not_created` | 尚未创建或已 reset |
| `supported` | 世界定位/局部临时支撑仍存在 |
| `contact_ready` | 持续双指接触满足释放条件，仍有外部支撑 |
| `verifying` | 已解除定位，正在观察 |
| `stable` | 无外部支撑，接触与相对位姿满足稳定时长 |
| `slipping` | 相对位置/朝向持续偏离基准或接触不足 |
| `dropped` | 持续显著位姿偏离或丢失接触 |
| `failed` | 操作或验证失败；`reason` 指出原因 |

`grasp` YAML 段的全部量均采用 SI 单位。默认双指门限每指 `contact_min_force_N=0.10 N`，持续 `contact_hold_s=0.10 s`；闭合超时 `close_timeout_s=5.0 s`。验证最少 `verification_min_s=0.35 s`、稳定连续 `stable_hold_s=0.25 s`，超时 `verification_timeout_s=3.0 s`。滑移门限 `slip_translation_m=0.002 m` / `slip_rotation_rad=0.14 rad`，持续 `slip_hold_s=0.10 s`；脱落门限 `drop_translation_m=0.035 m` / `drop_rotation_rad=1.05 rad`，持续 `drop_hold_s=0.10 s`，接触丢失去抖 `contact_loss_hold_s=0.25 s`。

释放后记录 `inverse(T_tcp) * T_usb` 基准，因此共同刚体运动不会直接算滑移。相对位姿和速度来自仿真真值，不能声称仅凭力反馈就检测全部滑移。状态、各手指载荷与关键位姿观测复用 `/maniskill/forces` JSON 和可选 JSONL，保留原有 WrenchStamped；法向接触、表达坐标系和力矩参考点详见[力接口](forces.md)。USB-only 的线缆分量显式 disabled/unavailable，不伪填有效零值。

## 几何和验证边界

当前默认 `trunking_cable_simplified_2mm.yaml` 使用新版 304 面简化碰撞线槽及原始 CAD 显示、1.5 m 线长和 **2 mm 线径**；默认后端仍为 MPM，本阶段命令显式指定 Rope-Actor；旧文档中的 3 mm 是历史实验描述。USB 夹持中心、出线点、左右孔中心分开建模，当前 TCP 夹持偏移为 `[0, 0, 0.012] m`。当前默认直连右孔，右孔内部和余量内的中心线沿真实孔轴，弯曲在孔外；`guide.routing: both_guides` 可显式恢复旧双孔回弯布局；完整 CAD 计算、左侧绕行及覆盖检查见 [USB 几何说明](usb_cable.md)。最小范围的局部线缆支撑只维持准备初态，随定位一起释放，后续形态由物理解算。

离线回归覆盖参数传递、左右布局、MTC 顺序、服务失败不附着/不搬运、状态判断和无后端路径；这不等同真实抓持成功。早期仅执行五阶段的 USB-only 历史结果见[本次验证记录](usb_grasp_validation.md)，复现使用 `test/check_usb_grasp_simulation.py` 和 `test/check_usb_grasp_ros.py`。不能用旧固定夹持完成整套走线的历史记录代替新流程的验收。未添加插接基座，也未验证插入或连续闭环力控制。

## 实时观察两种偏移

```bash
# 简洁终端显示：默认每 0.5 秒一行，状态变化立即显示。
ros2 run dual_fr3_maniskill watch_usb_grasp.py
# 完整 JSON 状态流；即使 force_enabled=false 仍发布。
ros2 topic echo /maniskill/usb/grasp_state
```

`/maniskill/usb/grasp_state` 是 `std_msgs/String`，内容与既有 `/maniskill/forces` 的 `usb_grasp` 对象相同，按控制周期发布；后者的 JSONL 仍记录这些观测。状态变化同时写入节点日志。现有 WrenchStamped 不变。

| 字段 | 基准和用途 |
| --- | --- |
| `creation_world_pose` | spawn 时 USB 的世界位姿，创建后固定不变 |
| `world_displacement_m` / `world_translation_m` | 当前 USB 减创建位置的 world XYZ 向量 / 长度，观察搬运总位移，不参与滑移判断 |
| `world_rotation_rad` | 相对创建世界朝向的转角，不参与滑移判断 |
| `release_baseline` | 解除定位时 `inverse(T_tcp) * T_usb`，滑移基准 |
| `relative_displacement_m` / `relative_translation_m` | 当前与释放基准在 TCP 坐标中的相对位置差向量 / 长度，参与滑移判断 |
| `relative_rotation_rad` | 相对释放基准的夹爪内转角，参与滑移判断 |
| `finger_normal_loads_N` | 两根真实手指的独立 USB 法向载荷 |

所以机械臂搬运 USB 100 mm 可以同时满足 `world_translation_m≈0.1`、`relative_translation_m≈0`，状态仍为 stable。位移/转角越限还必须持续达到门限；接触丢失也有去抖。基准未建立时字段为 null，仍受世界支撑时不会认定抓稳。

提前定位、正常 MTC 衔接和状态流的实际结果见[本次增量验证](usb_preposition_validation.md)。当前 simplified 默认场景已通过提前定位和释放后的稳定夹持，但完整原路径在双臂下降时触发线缆—线槽数值速度保护；正式布线未通过，不能把启动命令理解为完整任务成功保证。

夹持位姿精度和固定驱动力/刚度的真实对照见 [USB 夹持精度测量](usb_grasp_precision.md)。当前 stable 不等于满足精密定位公差；精度阈值按用户要求暂不修改。


## USB-only 原 MTC 轨迹验证（2026-09-16 更新）

根据用户澄清，`load_cable:=false` 现在只关闭线缆物理，不截断 MTC：仍执行双臂准备、USB 接触夹持/释放验证、双臂下降以及全部原正式运动阶段。无需额外开关，重新启动同一条命令即可。早期“五阶段结束”的记录是已替换行为。

实际无界面 ROS/ManiSkill 运行通过：当前关键点任务执行至 stage 13，所有缓存运动完成。末态 USB 为 stable，世界位移约 316.882 mm，相对释放基准的 TCP 内位移约 0.0600 mm、转角约 1.657°。这些数值不代表满足精密定位公差。末态两指法向载荷各约 3.62 N；线缆输出 disabled/null、缺失基座 unavailable。随后开爪检测到 dropped，重置到 not_created。

218 项 MTC 回归及 65 项无后端/启动集成检查通过。回归明确比较 true/false 生成的完整 MTC 阶段完全相同，同时保留物理层 USB-only 不初始化线缆后端的独立检查。原始记录位于 `artifacts/usb_only_motion_20260916/`。

当前直连右孔布局及初态对比见 [USB 直连右孔](usb_direct_routing.md)。

末端插入现可通过 `insertion_enabled:=true` 启用；几何、反馈、保持、双臂释放回位、参数及实际验收边界见 [USB 插入说明](usb_insertion.md)。早期验证记录中“未实现插入”的说明仅适用于当时版本。
