# 关键点提前定位与偏移状态流（2026-09-16）

默认配置已按要求切换为 `trunking_cable_simplified_2mm.yaml`；显式列出 USB 接触外壳、摩擦、开度和 `grasp` 阈值，与原始线槽对照配置同步。简化 CAD 用于碰撞，原始 CAD 用于显示。

本次将定位从“到位后、闭爪前”前移到“机械臂接近前”。正常模式仍执行原关键点任务：张开 → 按准备目标创建并世界固定 → 双臂接近 → 接触闭合 → 解除定位/稳定验证 → 双臂下降 → 原正式 MTC 阶段。使用 `load_cable:=true cable_solver:=rope_actor`；USB-only 现已按用户澄清保留全部原 MTC 运动，仅禁用线缆；最新通过记录见 [USB-only 原轨迹验证](mtc_cable.md#usb-only-原-mtc-轨迹验证2026-09-16-更新)。

## 定位依据

左右目标来自各自初始关键点索引、准备高度、路径方向和工具 roll/pitch，和 MoveTo 使用相同目标。参数 `usb_preparation_poses` 保留目标坐标系和位姿，spawn 时一次性变换到 world。USB 抓取中心、USB 出线点和两孔中心继续使用各自几何变换；不采用当前尚未到位的 TCP 作为目标。

Rope-Actor 提前定位时，真实右手还未到孔位，因此“当前线缆已穿入右孔”的几何断言等待接近完成；真实碰撞、穿透检查、关节连续性和速度检查始终生效。正常解除支撑前检查实际右孔对齐，解除后恢复孔内检查。局部线缆支撑与 USB 世界约束一起释放。

## 输出

独立 `/maniskill/usb/grasp_state` 使用既有 `usb_grasp` JSON 字段，并增加创建世界基准和 XYZ 偏移。`/maniskill/forces` 和原 JSONL 也包含相同观测；WrenchStamped 不变。

- `world_displacement_m`：相对创建世界位置的 XYZ 位移，包含正常搬运。
- `relative_displacement_m`：`inverse(T_tcp)*T_usb` 相对解除定位时基准的位置变化，用于滑移判断。
- 转角、两指载荷及持续时长共同决定状态；外部支撑未撤除时不判定抓稳。
- 基准不存在时使用 null；reset 清理所有基准。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dual_fr3_maniskill watch_usb_grasp.py
```

观察器显示毫米、角度和每指 N 载荷，每 0.5 秒最多一行，状态变化立即显示。JSON 中保持 m、rad、N。

## 验证

- 三包构建通过；最新全量 **479 项测试通过**，另补充“正常模式保留正式阶段”的断言通过。
- 真实 USB-only MTC：创建先于左臂接近，随后接触闭合、释放和稳定验证全部通过。ROS 检查确认新状态话题包含 stable/dropped/not_created 和两种偏移，重新张开/重置通过。
- 真实 Rope-Actor 独立验证：两臂在目标以外时提前创建，接近期间 USB 不动，闭合、释放、短暂停留、共同运动、开爪滑移/脱落、清理均通过。
- 原关键点正常 MTC 已完成提前创建、双臂接近、两爪闭合和稳定验证，实际进入双臂下降。下降记录中，USB 世界位移约 40.2 mm、夹爪内相对位移约 0.069 mm，状态仍为 stable。随后 USB—线缆接触穿透 0.156 mm，触发保护，实际执行停止在下降阶段。该次不能视为完整正式布线通过。
- 将 USB 接触提前检测距离从 0.1 mm 调整为 0.2 mm 后，真实模型短程回归通过：稳定夹持、双臂下降 50 mm、开爪滑移/脱落和清理。摩擦、静止偏移和穿透保护阈值未改。原始网格的第二次完整复测在下降阶段由本次配置切换主动停止，不能计为完成。

记录位于工作区 `artifacts/usb_preposition_20260916/`；当前启动命令、状态和阈值见 [MTC 使用说明](mtc_cable.md)。MPM 动态夹持仍按约定延期。

夹爪超时回归：简化配置首测已经达到 contact_ready，但旧客户端按 10 秒墙钟等待，先于慢速仿真的动作结果取消。修正为 profile 的仿真时间超时，并保留 `max(120, 30 × timeout)` 秒墙钟保护；慢时钟、正常到期、停钟三个回归通过。

## 当前默认 simplified 配置的最终实测结果

479 项自动检查通过；ROS 启动实参确认 `cable_config` 指向 `trunking_cable_simplified_2mm.yaml`、`cable_solver=rope_actor`、`load_cable=true`。保持原关键点、准备高度和 MTC 轨迹。

- 机械臂接近前创建；两臂接近期间 USB 相对创建世界位置的最大偏移为 0。
- 两爪闭合、持续双指接触、释放全部外部定位、稳定验证均通过。修正仿真时间等待后，未再次出现客户端的 10 秒墙钟误取消。
- MTC 自动进入原双臂下降阶段。在约 38.7 mm 世界位移时，USB 相对夹爪位移约 0.107 mm；尚未达到滑移阈值。
- **下降未完成**：Rope-Actor 的线缆—线槽接触产生数值速度突增（segment 10，12.276 m/s，超过 10 m/s 保护；最大接触冲量来自 rope_actor_5 与 rope_fixture_trunking）。状态明确发布 failed 并终止双臂动作。正式布线阶段未执行，不能声称完整后续 MTC 已跑通。

CAD 辅助探针：若将创建时的线形刚性下移至原目标 TCP Z=0.05 m，局部回弯最低表面到达约 Z=0.0066 m，且部分线段与简化槽壁相交。这个静态探针不代表自由线缆必然穿透；它说明下降期间必须发生接触与变形，初态双孔直线检查不能替代完整运动验收。旋转回弯方向的离线探针仍存在槽壁交叠，因此未将未经物理验证的回弯修改写入默认配置。

目前通过的是提前定位、USB 接触夹持、载荷采集和稳定性/滑移/脱落监测相关验证；完整原路径的线缆动态接触仍有上述已复现问题。未关闭碰撞、放宽保护、重新固定 USB 或加入隐藏支撑绕过它。

记录：`simplified_final_mtc.log`、`simplified_final_mtc_grasp_state.jsonl`、`simplified_final_mtc.summary.json`；首次墙钟超时保存在 `simplified_before_timeout_fix.log`。原始场景短程下降通过的记录不能替代本默认场景的失败结果。
