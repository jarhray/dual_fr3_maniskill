# 已知孔位 USB 插入（仿真）

`insertion_enabled:=true` 在原双臂 MTC 全部运动成功结束后执行末端插入。关闭时保留原流程和原 USB 整体凸包。默认仍使用 `trunking_cable_simplified_2mm.yaml`；带线缆须显式选择 `rope_actor`，其他场景默认求解器没有改变。USB-only 不导入、初始化或步进线缆后端。

本功能是已知孔位、低速刚体装配；不模拟 USB 弹片、卡扣、电气连接，不自动搜孔、增大夹持力或重新抓取。保持约束是显式仿真机构，不是自然卡紧的证据。实际验收结果见 [验证记录](usb_insertion_validation.md)，启动命令本身不表示该工况已通过。

## 几何依据

两个原始 STL 已原样纳入 `meshes/usb_base_visual.stl`、`meshes/usb_base_collision.stl`，由 ament 安装；运行不依赖 `temp/`。单位为 m，无缩放。视觉包围盒为 X `[-0.1043,0]`、Y `[0,0.03]`、Z `[0,0.0888]`；碰撞包围盒 X 为 `[-0.0748,0]`，YZ 相同。原点位于底面与孔口平面交线上，并非几何中心。

用户更正 XY 为 `[0.37,-0.07] m`，位于 `left_fr3_link0`，高度在线槽坐标系 Z=10 mm，方向服从实际 CAD。配置采用原 CAD 姿态（单位四元数），在当前 URDF 中左臂原点 world `[0.175,0.350,0]`，trunking 原点 `[0.809,1.123,0]`。所以基座原点为 `[0.545,0.280,0.010]`，给定孔口中心为 world `[0.545,0.2975,0.0786]`。这里的 10 mm 是高于线槽坐标平面；不是高于简化槽壁最高处 65 mm。模型孔腔朝局部 -X 延伸，**外法向 +X、插入方向 -X**。

| 项目 | 尺寸/数值 |
| --- | --- |
| CAD 目标孔口中心 | `[0, 0.01745, 0.06856] m` |
| 使用的给定孔中心 | `[0, 0.0175, 0.0686] m` |
| 原孔 Y/Z 范围 | `[15,19.9] / [62.46,74.66] mm` |
| 原孔横截面、深度 | `4.9 × 12.2 mm`、`11 mm` |
| USB 金属段横截面 | `4.45 × 12 mm` |
| 原始每侧间隙，以给定中心计算 | Y `0.275 / 0.175 mm`；Z `0.14 / 0.06 mm` |
| 调整后孔横截面 | `5.25 × 12.8 mm` |
| 调整后每侧间隙 | Y/Z 均 `0.4 mm` |
| USB 前端与肩部，USB 局部坐标 | 前端 `[0,17.9,0] mm`；肩部 Y=`5.9 mm` |
| 金属段长度 | `12 mm` |
| 目标前端深度 | `11 mm - 1 mm` 孔底余量 = `10 mm` |
| 目标时塑料肩部至孔口 | `2 mm` |

孔碰撞体使用 CAD 孔口平面三角剖分：36 个三角柱加 1 个实心后部，共 37 个凸体，保持其他孔、孔口和 11 mm 孔底。只移动目标孔壁顶点，不整体放大基座，不把凹腔导入为单一凸包。`test_insertion.py` 检查连续深度上的四个金属段角点及孔底、孔壁；`check_insertion_physx.py` 另检查 PhysX 实际导入形状。

USB 启用插入时拆为塑料壳凸体和金属段凸体，保留视觉网格、塑料壳夹持面、质量和惯量。原整体凸包在肩部和金属前端之间形成实际 CAD 没有的斜面；仅扩大插座会掩盖这项干涉。关闭插入时保留原 USB 碰撞构造。

几何间隙与接触参数分开：USB `contact_offset=0.2 mm`，插座 `0.1 mm`，总检测外壳 0.3 mm；两者 `rest_offset=0`，不允许额外穿透。0.4 mm 每侧几何间隙大于总检测外壳。摩擦保持 0.5，没有用高摩擦代替卡扣。插入模式刚体使用 1 ms 子步、至少 200 位置迭代/10 速度迭代，改善小物体的接触微振动；USB-only 的这些是刚体物理子步，线缆后端仍完全禁用。

位姿计算使用实际抓持：`T_TCP_USB = inverse(T_world_TCP) * T_world_USB`，再计算 `T_world_TCP_goal = T_world_USB_goal * inverse(T_TCP_USB)`。USB +Y 对齐孔腔 -X、USB +X 对齐基座 +Y。实测前端转换到孔坐标后 `depth=-tip.x`，横向误差为 `norm(tip.yz)`，姿态误差为完整 SO(3) 旋转角；这些是仿真真值辅助观测，不是仅凭力估计。`stable` 不代替这些精度条件。

根据用户授权，插座与固定安装件 `trunking`、`plate` 之间的碰撞在 MoveIt 和 PhysX 中按物体对豁免，允许其简化几何重叠。PhysX 使用每对独立的过滤位并在清理时恢复；不创建附加固定约束。USB—插座、机械臂—插座、线缆及其他物体的碰撞仍保留。基座高度是在线槽坐标系中的 10 mm，不是在线槽顶面之上再增加 10 mm。

## 控制、载荷和保持

原 MTC 完成 → 检查左臂真实抓持 → 右爪打开并验证两指开度 → 右臂沿 TCP -Z 退出 50 mm → 右臂回 `ready` 并验证停稳 → 重新检查左臂抓持 → 左臂 MoveIt 接近 → 按实测夹持再次对齐 → MoveIt 预检完整插入直线 → 交接给局部控制器。预检只在任务自己的规划场景中允许 USB—插座这一对预期接触；不放开手指—插座等碰撞；该预检本身不修改 PhysX 碰撞组。

孔前规划、回位使用最多 10 次独立任务重建/求解，以处理 MoveIt 随机采样失败；只有有效解才执行。执行失败不会盲目重放一段已经部分执行的轨迹。

局部控制器独占左臂现有 `sim.target` / 关节 PD 驱动；接管前要求没有活动/待处理轨迹及夹爪动作。接管期间拒绝外部机械臂和夹爪 action。连续推进速度为 `max(0, v_nominal - gain * filtered_resistance)`，有加减速和每关节指令变化限制；零接触允许前进，临近目标减速，达到深度即保持。不直接写 USB 位姿。横向/姿态越限、原始子步峰值过载、夹持失效、实际深度停滞、超程或超时均停止，保持夹爪，归还控制权。

所有控制时长、过滤周期、成功保持时间使用仿真时间；控制客户端超过 10 s 墙钟未更新专用 heartbeat 租约会停止；只读 status 和观察器不会续租。物理步若阻塞，恢复后先检查失联再允许下一条推进指令。取消和节点退出归还控制权并保持当前关节目标。规划和动作执行均在局部控制器退出之后进行。

原 `/maniskill/forces/usb/usb_base/wrench` 保留**法向接触分量**。新增 `/maniskill/forces/usb/socket_reaction/wrench` 来自基座—世界固定安装约束，包含接触摩擦分量；不用手指夹持力或线缆连接力充当插入力。基座是质量 0.1 kg 的安装刚体，明确禁用基座重力；这是载荷测量的去偏置条件，USB 和线缆仍有重力。只有 USB 接触基座、基座保持准静态时，此反力才可辨认为 USB—插座接触载荷；其他物体加载基座或基座速度越限时输出 unavailable 并停止。保持约束创建后也不再把该信号解释为插入接触载荷。没有把原生法向通道和安装反力重复相加。

新增 wrench 表达在 `usb_socket_hole`（基座轴向，原点在孔口），力为**基座作用在 USB 上**的 N，力矩参考孔中心、单位 N·m；+X 分量为反抗 -X 插入的阻力。无接触且传感器可用时为有效零值，传感器不可辨识时为 `available=false` 和 null。JSON 同时保留法向对照、完整反力、子步峰值及来源描述。

成功必须满足实际前端深度、横向误差、姿态误差、有效反馈、仍有效的真实夹持、无过载，并在足够低的相对速度下持续 0.3 s。先记录 `inserted_unretained` 和各判据，再在仿真步边界按当时相对位姿创建 USB—基座六自由度锁定；不移动物体，不显式清零速度，不增加 USB—TCP 约束。记录创建前后位姿、线速度、角速度和成功观测。固定失败保留闭爪状态。

固定后抓取观察器切换为 `socket_supported`，无手指接触后为 `gripper_released`，并保留固定前的完整抓取快照，避免误报脱落。USB—线缆物理连接继续存在。

## 释放和回位

收尾顺序为先右臂释放回位，再左臂插入、固定和释放回位。右爪在插入前使用已有 `open` profile；保持确认后，MoveIt 解除机器人上的 USB 规划附着，把唯一 USB 世界物体更新到实测位姿，再用同一 profile 打开左爪。左侧要求两根实际手指至少 35 mm 半开度；右侧目标总开度 30 mm，要求两指至少 14 mm。右臂原 MTC 末态靠近槽壁，直接要求 80 mm 总开口会使第二根手指受限在约 18.34 mm。CAD 内侧最小面在 TCP Y=-1.022 mm（未叠加开度），14 mm 半开度仍给对称孔中心留出超过 25.9 mm 的通道，足以使 2 mm 线缆与两半孔分离。以实际开度校验，不能单凭 action 的第一根手指 reached/stalled 判定两爪已释放。

研究夹爪的引导孔由两根手指上的半孔组成，手指沿 TCP ±Y 分开，完全张开后不再是闭合圆环。右臂在插入前、左臂在固定后，分别沿本侧 TCP -Z 直线退出 50 mm，然后通过 MoveIt 回到已有 SRDF `ready`，不是全零关节角。左臂退出期间保留 USB 与原夹持 touch links 的规划接触许可；退出后恢复此前 ACM。除上述两个固定安装碰撞对外，真实碰撞一直启用。右臂进入 `right_releasing` 后实际开始张开时，只取消“线缆必须持续穿过闭合孔”的几何断言；分体孔碰撞与 USB—线缆连接保留，输出明确标注孔已在打开/释放。带线缆的动态拉扯仍须通过实际 rope_actor 验证，静态半孔可分离不能证明任意线形下的回位必然可行。

`right_gripper_released`、`right_return_complete` 单独记录右臂的先行释放和回位；右臂回位失败或左臂夹持失效时，不开始插入。`insertion_success`、`retention_active`、`grippers_released`、`return_complete` 分开记录；插入或保持成功不代表回位完成。插入阶段取消、过载等失败不自动打开左爪；右臂此时已经释放回位。重复 start 在执行中或成功后幂等；失败须 reset；已经完成的终末流程不重复执行。reset 清理保持/安装约束、USB/线缆、接触窗口和状态，异步清除 MoveIt 世界/附着物；清理失败或未确认前拒绝再次 spawn。

## 状态与参数

状态依次为 `not_started`、`right_releasing`、`right_returning`、`right_ready`、`approach`、`aligned`、`feedback_advance`、`success_verification`、`inserted_unretained`、`retained`、`grippers_released`、`returning`、`complete`。失败为 `right_release_failed`、`right_return_failed`、`blocked`、`overload`、`slip_or_drop`、`timeout`、`feedback_unavailable`、`cancelled`、`retention_failed`、`release_failed`、`return_failed`，详因在 `reason`。

以下字段均位于默认 YAML 的 `insertion` 段；ROS 开关 `insertion_enabled` 控制是否启用。孔中心与 CAD 孔口 X=0 一致，配置只允许有限局部间隙，不接受非零 rest offset。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `visual_mesh`, `collision_mesh` | 上述包内路径 | 安装资源 |
| `xy_frame`, `xy_m` | left_fr3_link0, `[.37,-.07]` | 基座 XY |
| `height_frame`, `height_m` | trunking, `.010` | 底面/原点高度 m |
| `quaternion_wxyz` | `[1,0,0,0]` | 基座朝向 |
| `hole_center_m` | `[0,.0175,.0686]` | 局部孔中心 m |
| `clearance_yz_m` | `[.0004,.0004]` | 每侧几何间隙 m |
| `contact_offset_m`, `rest_offset_m` | `.0001`, `0` | 接触检测/静止偏移 m |
| `preinsert_m`, `target_depth_m` | `.008`, `.010` | 孔前距离/前端目标深度 m |
| `end_clearance_m`, `max_advance_m` | `.001`, `.022` | 孔底余量/最大推进 m |
| `speed_m_s`, `acceleration_m_s2` | `.001`, `.002` | 速度/变化限制 |
| `resistance_gain_m_N_s`, `filter_tau_s` | `.0004`, `.04` | 阻力减速增益/滤波秒 |
| `axial_limit_N`, `lateral_limit_N`, `torque_limit_Nm` | `5`, `2`, `.04` | 子步峰值硬停止 |
| `lateral_tolerance_m`, `angle_tolerance_rad` | `.00025`, `.025` | 位姿门限 |
| `depth_tolerance_m`, `hold_s` | `.00015`, `.3` | 深度容差/连续成功时长 |
| `retention_speed_m_s`, `retention_angular_rad_s` | `.001`, `.03` | 建立保持的实测低速条件 |
| `blocked_s`, `progress_epsilon_m` | `1.5`, `.0001` | 停滞时间/实际进展门限 |
| `timeout_s`, `wall_watchdog_s` | `45`, `10` | 仿真超时/墙钟失联保护 |
| `joint_speed_rad_s`, `tracking_limit_m` | `.1`, `.001` | IK 指令变化/跟踪偏差保护 |
| `retain_after_success`, `release_after_retention`, `return_after_release` | 均 true | 插入后固定、左爪释放、左臂回位开关；右臂先行释放回位始终执行 |
| `retreat_m`, `approach_lift_m` | `.05`, `0` | 开爪退出距离/可选孔上方中间点 |
| `planning_attempts`, `planning_timeout_s` | `10`, `15` | 独立规划次数/每次超时 |
| `rigid_frequency_hz` | `1000` | 插入模式的刚体子步频率 |
| `release_width_m` | left `.08`, right `.03` | 总开口 m |
| `release_min_half_width_m` | left `.035`, right `.014` | 每根手指最低实测开度 m |

## 启动与观测

在工作区根目录，先构建并加载：

```bash
source /opt/ros/humble/setup.bash
source /home/jerry/ws_moveit/install/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select dual_fr3_maniskill dual_fr3_moveit_config dual_fr3_trunking_mtc
source install/setup.bash
```

带 rope_actor 的完整原 MTC + 插入 + 保持 + 双臂回位（任何阶段失败即停止）：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=true \
  insertion_enabled:=true preparation_interactive:=false \
  maniskill_viewer:=true use_rviz:=true
```

USB-only 同样执行全部 MTC 和末端操作，仅禁用线缆后端：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=false \
  insertion_enabled:=true preparation_interactive:=false \
  maniskill_viewer:=true use_rviz:=true
```

```bash
ros2 run dual_fr3_maniskill watch_usb_insertion.py
# 记录 /forces 的完整 JSONL；目标文件必须不存在。
ros2 run dual_fr3_maniskill watch_usb_insertion.py --record /tmp/usb_insertion_new.jsonl
ros2 topic echo /maniskill/usb/insertion_state
ros2 topic echo /maniskill/forces/usb/socket_reaction/wrench
ros2 topic echo /maniskill/usb/grasp_state
ros2 service call /maniskill/usb/insertion/status std_srvs/srv/Trigger '{}'
ros2 service call /maniskill/usb/insertion/cancel std_srvs/srv/Trigger '{}'
```

`/maniskill/usb/insertion/{right_release,right_released,right_returned,target,start,status,heartbeat,cancel,retained,released,returning,returned}` 使用 Trigger；通常由 MTC 终末执行器调用，不要在有轨迹运行时手动 start。独立状态流不依赖 `force_enabled`；力/JSONL 仍使用既有输出体系。
