# MTC 准备阶段的线缆

先按[环境安装与验证](setup.md)安装依赖并构建任务包。加载 ROS 和工作区后，在工作区根目录启动：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"
```

MTC 仍先预检准备阶段和正式任务的完整规划。`execute:=false` 只做规划，
不会在仿真里生成线缆。`preparation_interactive` 保留原有的三个确认点。

执行顺序为：两臂到初始关键点上方 → 左夹爪闭合 → 右夹爪闭合 →
`SimulationCable` 阶段生成 USB/线缆 → 双臂同步下降 → 正式任务。
创建失败时停止下降；后续动作重规划只处理未完成阶段，不重复生成线缆。

仅 `simulation_backend:=maniskill` 且 `maniskill_cable:=true`（默认）启用。
关闭线缆可设置 `maniskill_cable:=false`。启用时要求准备阶段、左右 Franka
手爪和默认的 leader=left / follower=right TCP 映射。

## 两端约束和模型

- 左手：准备宽度使用 `2 * usb.finger_position`，默认总开口 7.4 mm。
  闭合后生成独立 USB 刚体，用六自由度锁定的 SAPIEN drive 固定到左 TCP。
  STL 的 +Y 插头方向对应 TCP +X，与 `ORIENTATION_DIRECTIONS` 产生的朝向一致。
- 右手：总开口为 0；TCP 原点为孔中心，TCP X 为孔轴。约束跟随真实 TCP 位姿，
  只修正横向位置和相对速度，每次重新寻找穿孔的材料截面，允许轴向滑动。
  `guide.half_length` 指定孔内保持直线的半长，默认 12 mm；孔外有平滑过渡。
- 初始布局：USB 尾部保留固定胶套段，随后弯向右孔；穿孔段和自由尾段沿孔轴
  直线伸出。此曲线只用于生成和重置，后续由 MPM、轴向纤维和接触求解。
- 保留现有 `research_finger/finger1.STL`、TCP、URDF 和 SRDF。
  MPM 对四个指尖使用原始碰撞网格，避免 SAPIEN 关节碰撞凸包填平孔洞。
  CAD 中闭合壳体共边的情况用有向边平衡验证，不补孔、不忽略整只右手的碰撞。

MTC 默认配置是 `dual_fr3_maniskill/config/trunking_cable.yaml`，线长 1.5 m、
线径 2 mm、采样间隔 1 mm。闭合研究手指的轴线到孔壁最小距离约为 1.27 mm，
独立演示用的 3.5 mm 线径不适用。可用 `cable_config:=/绝对路径/config.yaml`
同时传入 MTC 和仿真。线长不足或初始线缆穿入刚体时，创建会返回失败。

## 接口和行为边界

- `/maniskill/cable/spawn`：`std_srvs/Trigger`，必须两次闭合动作成功且没有活动动作。
  重复调用保持现有线缆；首次构造同步完成后才返回。
- `/apply_planning_scene`：MTC 执行端在仿真成功创建后添加 USB 附着碰撞体。
  规划阶段也包含同一网格、同一 TCP 相对位姿和允许接触的左手链接。
- `/usb_cable_demo/markers`、`/usb_cable_demo/diagnostics`：沿用已有线缆反馈。
  新诊断包含 `guide_material_coordinate` 和 `guide_radial_error_m`。
- `/usb_cable_demo/reset`：在当前两端位姿重新生成线缆，不移动双臂。
  准备前不能用此服务绕过闭合顺序。

这一步采用固定夹持和理想滑孔，尚不支持执行中松开任一夹爪、交接 USB 或重新穿孔。
柔性线缆不作为 MoveIt 的刚性碰撞物；MTC 检查机器人及 USB，线缆接触由仿真检查。
材料和力并未标定，MPM 运行速度可能显著慢于墙上时间。数值异常会暂停仿真并使
正在执行的动作失败，不会继续执行后面的下降或走线。

## 验证入口

离线检查重点是 `test_threading.py`、`test_simulation_cable.py` 和场景启动测试；GPU 接触及完整 MTC 运行方法见[环境安装与验证](setup.md)。`validate_integration.sh --mode mtc` 会自动执行当前任务并检查完成日志。

单独生成线缆或下降若干物理步只能验证局部行为，不能替代完整走线执行。材料、接触和滑孔参数仍需按实际任务验证。
