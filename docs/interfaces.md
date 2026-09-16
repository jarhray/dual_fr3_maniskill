# ROS 接口与仿真模型

使用方法见 [README](../README.md)，环境与验证见[安装说明](setup.md)。

本阶段接触夹持验收聚焦 `cable_solver:=rope_actor` 与 `load_cable:=false` 的 USB-only；MPM 延后完善，全局默认 `mpm` 保持不变。推荐双模式命令见 [MTC 使用说明](mtc_cable.md#启动)。

## 场景与配置

| 场景 | 入口 | 默认配置 |
| --- | --- | --- |
| `robot` | MoveIt 的 `maniskill.launch.py` | `config/simulation.yaml` |
| `usb_cable` | MoveIt 的 `usb_cable.launch.py` | `config/simulation_usb_cable.yaml`、`config/usb_cable.yaml` |
| `trunking_cable` | MTC 的 `mtc_prototype.launch.py simulation_backend:=maniskill` | `config/simulation_usb_cable.yaml`、`config/trunking_cable_simplified_2mm.yaml` |

`maniskill_config` 覆盖桥接参数，`cable_config` 覆盖 USB/线缆配置。`load_cable=false` 完全跳过线缆后端初始化和逐帧求解，但保留 USB 创建、接触载荷、TF 和状态监测。完整启动编排选择一个场景、一个物理桥接；本包 `sim.launch.py` 只提供桥接，要求调用者传入最终 URDF/SRDF。

## 机械臂与夹爪接口

下表的 `{left,right}` 表示分别存在左右两个接口。

| 接口 | 类型 / 含义 |
| --- | --- |
| `/{left,right}_fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/{left,right}_franka_gripper/gripper_cmd` | `control_msgs/action/GripperCommand` |
| `/joint_states` | `sensor_msgs/msg/JointState`，实际位置和速度 |
| `/{left,right}_fr3_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState`，目标、实际值和误差 |
| `/clock` | `rosgraph_msgs/msg/Clock`，按物理步数生成 |
| `/maniskill/{left,right}_tcp_pose` | `geometry_msgs/msg/PoseStamped`，物理引擎实测 TCP，坐标系 `world` |
| `/maniskill/forces` | `std_msgs/msg/String`，控制周期内的作用力、峰值、来源与可用性，见[力采集说明](forces.md) |
| `/maniskill/forces/.../wrench` | `geometry_msgs/msg/WrenchStamped`，末端和各手指的局部作用力/力矩，见[话题列表](forces.md) |

默认带夹爪模型发布 14 个机械臂关节和 4 个手指关节，不用目标值冒充实测反馈。机器人 TF 由 `robot_state_publisher` 发布；动态 USB 的 world TF 由物理桥接按实测刚体位姿发布，没有 URDF TCP 固定关节。此后端不启动 `controller_manager`，因此用 action 和状态话题检查桥接，而非 `ros2 control list_controllers`。

轨迹必须完整包含对应臂的 7 个关节，名称顺序可变。位置点做线性插值，带速度用三次插值，带速度和加速度用五次插值。支持未来开始时间、反馈、取消及路径 / 终点容差。同一臂忙时拒绝新目标，左右臂可并发；取消或失败后保持实际当前位置。

夹爪 `position` 是单指位移，总开口约为两倍。`max_effort=0` 使用配置默认力，正值受上限约束。`reached_goal` 根据实测位置和速度判断；`stalled` 表示目标未达到且持续停止运动，并不直接证明抓住了线缆。

两个 USB 场景均允许重新张开。动态 USB 的世界定位、接触门控释放、稳定验证和状态查询分别由 `/maniskill/cable/spawn`、`/maniskill/usb/release`、`/maniskill/usb/verify`、`/maniskill/usb/status`（均 Trigger）提供；`release_manual` 可调试强制释放。`reset` 清理物体与支持后需重新张开、spawn。只有 status 为 stable 时 success=true；message 为 JSON 状态及 reason。场景限制见 [USB 说明](usb_cable.md)和 [MTC 说明](mtc_cable.md)。

## 模型与物理

上层用同一份最终描述供仿真、MoveIt、TF 和 RViz 使用。`assets.py` 解析 package URI，把 STL 转为保留硬边法线的 GLB；临时 URDF 与 PhysX 缓存存放在进程临时目录，退出时清理。

双臂位于同一固定基座 articulation，工作台、安装板和线槽作为静态刚体。线槽使用静态三角网格保留凹槽；SRDF 的碰撞排除由 `collision.py` 编码到 PhysX 碰撞位掩码。关节驱动使用 URDF 力矩上限，并施加被动力补偿。

普通场景的物理 / 控制 / 发布频率为 500 / 100 / 50 Hz；线缆场景为 500 / 50 / 25 Hz。线缆求解另由 `mpm.frequency` 控制，默认 5000 Hz。仿真可能慢于实时，ROS 节点应使用统一 `/clock`。

## 模块分工与边界

| 模块 | 职责 |
| --- | --- |
| `simulation.py`、`ros_bridge.py`、`trajectory.py` | 刚体、动作与轨迹采样 |
| `assets.py`、`collision.py` | 模型转换与碰撞配置 |
| `scenes/` | 普通机器人、独立 USB 和 MTC 延迟线缆场景 |
| `cable/model.py`、`cable/threading.py` | 线缆参数、几何和穿孔初态 |
| `cable/mpm_cable.py`、`cable/solver.py`、`cable/fibers.py` | MPM 与轴向纤维 |
| `cable/contacts.py`、`cable/mesh_contacts.py`、`cable/guide.py` | 刚体接触与滑孔约束 |
| `cable/ros_bridge.py`、`cable/trunking_bridge.py` | 线缆反馈、重置和延迟生成服务 |
| `cable/planning_scene.py` | USB 附着碰撞体 |

运行时新增的 MoveIt 障碍物不会自动变为 SAPIEN 刚体。柔性线缆只在物理场景中处理接触，不作为 MoveIt 避障物。材料未标定、无视觉或力控闭环，不能把短时接触夹持或无穿透测试当作完整走线验收。
