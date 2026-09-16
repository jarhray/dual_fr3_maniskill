# USB 直接连接右引导孔

根据用户对初态的最新要求，默认去掉 USB 附近的回环：线缆从真实 USB 出线点出发，短距离沿出线方向离开插头，再平滑转向右孔。**不再穿过左孔**。USB 抓取位置、朝向、夹持参数和 MTC 运动流程保持原值。

两份 trunking YAML 已同步：默认 `trunking_cable_simplified_2mm.yaml` 和原始网格对照 `trunking_cable.yaml` 均设置：

```yaml
guide:
  routing: usb_to_right
```

`both_guides` 可显式恢复历史双孔回环布局；省略此字段的旧自定义配置仍按双孔解释。`left_loop_clearance` 只在 `both_guides` 中生效。

## 几何与支撑

- 当前准备位姿下，USB 出线点至右孔的直线距离约 0.6004 m；新曲线约 0.6258 m，旧双孔曲线约 0.8818 m。
- 右孔内部及两侧余量保持沿孔轴共线，平滑转向发生在孔外。151 段离散链的最大初始弯角约 36.21°，小于 85° 限位。
- 原始手指 CAD 检查：线缆表面到左指最小余量约 11.93 mm，到右指孔壁约 2.537 mm；自由线缆到 USB 凸包约 7.50 mm。
- 不创建左孔局部支撑；只有右孔附近约 49 mm 材料区间的临时支撑，随 USB 世界定位一起释放。
- `initial_layout` 和 `current_guide_alignment` 中，左孔输出 `available=false`、`required=false`、`passed=null`、原因 `disabled_usb_to_right_routing`；不伪报穿过左孔。右孔继续输出径向、角度及覆盖检查。
- 总线长仍为 1.5 m；少用的回环长度留给右孔后的自由端。初态不是张紧控制，解除定位后的下垂和接触由物理解算，不逐帧改写位置。

创建曲线对比保存在工作区 `artifacts/usb_direct_routing_20260916/routing_comparison.png`（橙色为新布局，灰色为旧布局）。这些 CAD 数值为创建时几何检查，不等同于动态全过程无碰撞。

## 使用与验证

重新创建物体或重新启动才能应用新布局；正在运行的旧线缆不会被瞬移成新曲线。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor \
  load_cable:=true execute:=true
```

`load_cable:=false` 仍为无线路径调试，执行原双臂 MTC 轨迹。

自动回归：482 项通过，包含直连布局、双孔兼容、左右朝向、刚性链长度、唯一右孔支撑区间和非法配置拒绝。实际 Rope-Actor 验证使用：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --load-cable --solver rope_actor --preposition-before-approach \
  --output /tmp/direct_usb_rope.jsonl
```

真实 Rope-Actor 验证通过：提前创建后世界定位保持、双臂接近、双指接触、移除全部临时支撑且保留位置/速度、稳定验证、小幅共同运动、开爪滑移/脱落、重复清理均通过。初态只生成右孔支撑区间。本次未重新验收完整布线轨迹，重点为新的创建布局和抓持流程。原始 JSONL 和 summary 保存在上述 artifacts 目录。
