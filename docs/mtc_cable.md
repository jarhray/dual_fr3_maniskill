# MTC 准备阶段的线缆

先按[环境安装与验证](setup.md)安装依赖并构建任务包。加载 ROS 和工作区后，在工作区根目录启动：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"
```

MTC 仍先预检准备阶段和正式任务的完整规划。`execute:=false` 只做规划，
不会在仿真里生成线缆。`preparation_interactive` 保留原有的三个确认点。
Rope-Actor 当前为实验后端。用户已确认“简化线槽 + 2 mm 线缆”完成整套任务；
录制 `run_20260915T113808Z_3scs5omg` 共 3295 个控制步、65.9 s，未记录失败。
该基线保存在 `config/trunking_cable_simplified_2mm.yaml`。
当前默认改为“原始线槽 + 实测 3 mm 线径”的新试验，验证范围见
[原始线槽与 3 mm 线缆](debugging_summary.md#original-3mm)。

执行顺序为：两臂到初始关键点上方 → 左夹爪闭合 → 右夹爪闭合 →
`SimulationCable` 阶段生成 USB/线缆 → 双臂同步下降 → 正式任务。
创建失败时停止下降；后续动作重规划只处理未完成阶段，不重复生成线缆。

仅 `simulation_backend:=maniskill` 且 `maniskill_cable:=true`（默认）启用。
关闭线缆可设置 `maniskill_cable:=false`。启用时要求准备阶段、左右 Franka
手爪和默认的 leader=left / follower=right TCP 映射。

## 两端约束和模型

- 左手：准备宽度使用 `2 * usb.finger_position`，默认总开口 7.4 mm。
  闭合后生成独立 USB 刚体，用六自由度锁定的 SAPIEN drive 固定到左 TCP。
  STL 的 +Y 插头方向由 `leader_orientation_direction` 决定：`forward` 对应 TCP +X，
  `reverse`（默认）对应 TCP -X，使插头始终朝向路径前进方向。
  抓取中心通过 `usb.tcp_grip_offset: [0, 0, 0.0075]` 放在 TCP 局部 +Z 方向
  7.5 mm 处；夹爪向下（roll=π）时即孔中心下方。偏移随夹爪旋转，TCP 定义不变。
  仿真刚体、线缆固定端和 MoveIt 附着碰撞体共用这一安装位姿。
- 右手：总开口为 0，TCP X 为孔轴方向。当前 CAD 孔内的宽敞位置位于既有 TCP
  的局部 +Z 方向约 2.4 mm，使用 `guide.center_offset: [0, 0, 0.0024]`。
  偏移随手爪旋转，只改变初始穿线与孔口观测参考点；TCP 定义、机器人路径和手指 CAD 不变。
  Rope-Actor 仅靠真实孔壁接触、摩擦约束线缆，没有人工居中力。
  MPM 才使用理想滑孔约束，`guide.half_length` 指定其直线段半长，默认 12 mm。
- 初始布局：从 USB 尾部直线出线，随后弯向右孔；穿孔段和自由尾段沿孔轴
  直线伸出。此曲线只用于生成和重置，后续由所选模型及其接触求解。
  Rope-Actor 默认 `root_joint: spherical`，直接连接柔性线缆：出线点固定，
  第一段 7 mm 可以弯转，不再作为刚性胶套。MPM 保留原有固定段模型。
- 保留现有 `research_finger/finger1.STL`、TCP、URDF 和 SRDF。
  MPM 对四个指尖使用原始碰撞网格，避免 SAPIEN 关节碰撞凸包填平孔洞。
  CAD 中闭合壳体共边的情况用有向边平衡验证，不补孔、不忽略整只右手的碰撞。

MTC 默认配置是 `dual_fr3_maniskill/config/trunking_cable.yaml`，线长 1.5 m、
线径 3 mm、MPM 采样间隔 1 mm。旧 TCP 轴线到孔壁最小距离约为 1.27 mm；
上述偏移后，沿 24 mm 轴向采样的最小距离约为 3.537 mm，3 mm 线缆表面余量约 2.037 mm。
线密度仍为 15 g/m 的现有估算，线身质量为 22.5 g。
可用 `cable_config:=/绝对路径/config.yaml`
同时传入 MTC 和仿真。线长不足或初始线缆穿入刚体时，创建会返回失败。

默认 YAML 中 `scene.trunking_mesh: original` 为原始有齿缝网格；`simplified` 为简化网格。
最终 URDF 的显示与碰撞一起切换，并共用于 MoveIt、MTC、RViz 和 ManiSkill。
原始线槽测得一段侧壁齿缝约 8 mm，3 mm 线径仍可能进入，不能用加粗代替避开卡口的路径验证。
回到已完成的基线，只需在原启动命令中增加：

```bash
cable_config:="$PWD/src/dual_fr3_maniskill/config/trunking_cable_simplified_2mm.yaml"
```

`cable_solver:=rope_actor` 使用胶囊/三轴关节链；`cable_solver:=mpm` 或省略参数使用原有 MPM。
两者保留相同的准备和生成时序。Rope-Actor 的右孔由原始手指网格碰撞与摩擦实现，不锁定线缆位置和朝向；
MTC 默认使用内接凸胶囊参与碰撞，修正捕获的孔口及线槽边缘漏检；线缆理想外形和验收容差保持原值。
穿孔观察器按真实孔口检查偏心运动，
具体物理参数和模型差异见[线缆建模方式](cable_backends.md)。原有 MPM 参数和性能基线保留。

## 接口和行为边界

- `/maniskill/cable/spawn`：`std_srvs/Trigger`，必须两次闭合动作成功且没有活动动作。
  重复调用保持现有线缆；首次构造同步完成后才返回。
- `/apply_planning_scene`：MTC 执行端在仿真成功创建后添加 USB 附着碰撞体。
  规划阶段也包含同一网格、同一 TCP 相对位姿和允许接触的左手链接。
- `/usb_cable_demo/markers`、`/usb_cable_demo/diagnostics`：沿用已有线缆反馈。
  新诊断包含 `guide_material_coordinate` 和 `guide_radial_error_m`。
- `/usb_cable_demo/reset`：在当前两端位姿重新生成线缆，不移动双臂。
  准备前不能用此服务绕过闭合顺序。

左端采用固定夹持；右孔在 MPM 中为理想滑孔，在 Rope-Actor 中为孔壁碰撞与摩擦。
尚不支持执行中松开任一夹爪、交接 USB 或重新穿孔。
柔性线缆不作为 MoveIt 的刚性碰撞物；MTC 检查机器人及 USB，线缆接触由仿真检查。
材料和力并未标定，MPM 运行速度可能显著慢于墙上时间。数值异常会暂停仿真并使
正在执行的动作失败，不会继续执行后面的下降或走线。

## 验证入口

离线检查重点是 `test_threading.py`、`test_simulation_cable.py` 和场景启动测试；GPU 接触及完整 MTC 运行方法见[环境安装与验证](setup.md)。`validate_integration.sh --mode mtc` 会自动执行当前任务并检查完成日志。

单独生成线缆或下降若干物理步只能验证局部行为，不能替代完整走线执行。材料、接触和滑孔参数仍需按实际任务验证。
