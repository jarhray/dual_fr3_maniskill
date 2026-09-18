# 默认 MTC 工作流与独立插入入口（2026-09-18）

本轮接续 09-17 的职责整理，按用户新增要求修改默认启动、插入前规划衔接、Enter 时机和独立 ROS 调用。历史结果仍在 [原重构记录](insertion_refactor_validation.md)，不将它们改写为本轮结果。

## 修改及参数边界

- MTC 入口默认 `simulation_backend=maniskill`、`cable_solver=rope_actor`、`load_cable=true`、`execute=true`，解释器仍由 `MANISKILL_PYTHON` 或当前目录 `.venv/bin/python` 决定。显式参数优先；demo 及独立物理入口保留原后端默认。
- 准备候选必须同时通过运输和连续末端接近规划。末端 MTC Task 包含右爪开度预览、右臂退出/回位、左臂接近和完整插入直线碰撞预检；按完整解 ID 缓存相连轨迹，预检段不执行。
- 真实闭爪、解除世界固定、稳定验证后，按实测 USB 附着重规划剩余路径，保留所选锚点的关节末态和笛卡尔终点。通过后，ManiSkill 接触夹持模式只在下降前等待一次 Enter，等待结束再次检查抓持。其他后端保留原闭爪确认。
- 不再在孔前临时规划微量对齐；start 仍用实际 USB 位姿执行原对齐校验。超过原阈值停止，不以缓存目标代替测量。
- `run_insertion=false` 只跳过运输后自动调用，保留物理插座和规划检查。`insertion_skill.launch.py` 连接已有场景，更新实测附着，从当前状态规划并复用同一终末流程，不创建另一套仿真或重新抓取。
- 保留用户已有 `insertion.max_path_length_ratio=3.0`；代码 fallback 仍为 1.5。物理步长、摩擦、碰撞、力反馈和成功阈值未调参，服务名称/类型/JSON/错误语义未修改。

修改位置和单位/坐标/默认值见 [参数说明](insertion_parameters.md)；启动及单独调用见 [USB 插入](usb_insertion.md#独立调用)。

## 检查记录

原始输出位于工作区 `artifacts/insertion_workflow_20260918/`。

| 检查 | 结果 |
| --- | --- |
| 本轮编辑前相关基线 | 154 passed，`baseline.log` |
| 默认、确认、服务及新入口专项 | 200 passed，`batch3.log` |
| 保留锚点关节末态专项 | 101 passed，`anchor_refresh.log` |
| 中间全量回归 | 590 passed / 1 failed（`full.log`）；随后 599 passed / 1 failed（`final_full.log`） |
| 最终全量回归 | **604 passed / 1 failed**，`final_verified.log`，仅上述历史样本失败 |
| 感知包 | 33 passed，`perception.log`，源码未修改 |
| 几何等价 | 与重构前 160 个数组逐值相同，`after/equivalence.json` |
| 配置等价 | 12 组解析结果仅含用户先前的路径比例 2.0→3.0，其他值相同 |
| 推荐启动与默认 | 测试比较 launch 全部解析参数，默认与推荐显式命令一致（环境变量优先级保留） |
| 构建 | 两包 symlink build 成功；系统 Python 缺 pytest 的 ament 警告保留，实际测试使用项目 venv |

唯一既有断言失败仍为 `test_rope_finger_contact.py::test_recorded_rim_overlap_generates_contact_with_correct_depth[rope_trunking_rim_20260915-convex_capsule]`。当前 CAD 深度约 0.003760325567 m，历史样本约 0.000159900599 m。没有改样本、删测试或放宽原阈值。

中间集成运行保留在 `usb_only_1/2/3` 和 `cable_1`：最初两轮发现并修复了新增模块的 C++ 所有权转移后访问和 YAML list 转 ndarray 问题；第三轮在初始搜索预算内没有整段可行解，未执行运动。带线缆第一轮完成抓持/解除固定，但重新按位姿采样的锚点路径失败，未开始运输。随后增加保留所选锚点关节末态，未调整任何规划阈值。

## 独立 ROS 入口的真实场景结果

`standalone_1` **完整通过**。这是 USB-only（`load_cable=false`）场景：原 MTC 完成准备和运输后退出，另一个 ROS launch 进程调用 `insertion_skill.launch.py`，从当前抓姿规划全部接近动作，然后完成插入、固定、释放和双臂回位。

- `insertion_success`、`retention_active`、`grippers_released`、`return_complete`、`right_return_complete` 均 true；`local_controller_owns_left_arm=false`。
- 状态记录明确先 `inserted_unretained`，再 `retained`，最后释放和回位；未把保持约束创建作为插入成功的依据。
- 重复原 start 服务维持 complete，未新增状态转换。reset 成功，MoveIt 世界和附着物中的 USB/插座全部清除。
- ROS 节点全部 28 项有效参数与 09-17 原版本成功的同工况一致，见 `effective_parameters.json`；YAML 路径比例的用户变更单独记录，不属于 ROS 参数。
- 这是独立入口真实集成结果，不是仅单元测试。此次未操作 GUI 人工 Enter；单次确认的位置、顺序及等待后复查由测试覆盖。

## 带线缆复跑与未完成项

最终代码的 `cable_2`（`load_cable=true`、`run_insertion=true`）未完成全程。初始第一组候选通过运输、孔前接近及插入碰撞预检；双爪闭合、解除固定、稳定验证均成功。随后实测抓姿重规划的 `step_0_move_anchor_leader_plan_to_entry_5` 在 10 次尝试内失败，10 次均报告 OMPL `FAILURE`，没有到达路程检查的有效轨迹，程序退出码 3。此处不能将所有失败归因为路径比例；上一轮 `cable_1` 才另有路径超限记录。

此轮没有执行下降/运输，也没有进入右臂释放或反馈插入，停止在稳定夹持状态；没有为通过场景而增加重试次数、放宽路径比率或修改物理参数。新前置检查把该失败暴露在任务开始前，但**不能宣称当前默认带线缆命令已通过完整场景，也不能保证任意初态都可规划**。保留该可重复性问题供后续独立定位；USB-only 独立入口的成功不能替代带线缆运输、插入和回位验证。

人工 Enter 的 GUI/终端交互未在本轮自动回放中实际输入；测试已验证只有夹持解除固定后的一处确认，失败不显示开始确认，等待后失稳不继续。没有修改或验证新的感知闭环、压线技能或 MPM 路径。

## 复现

在工作区根目录：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/insertion_workflow_tests
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
.venv/bin/python -m pytest -q src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test
.venv-perception/bin/python -m pytest -q src/dual_fr3_cable_perception/test

# 完整带线缆；报告目录必须为新目录。
CHECK_LOAD_CABLE=true ROS_DOMAIN_ID=87 ROS_LOG_DIR=/tmp/insertion_workflow_sim \
  /usr/bin/python3 artifacts/insertion_workflow_20260918/run_scene.py /tmp/insertion_cable_new
# 先运输再由独立进程调用插入；USB-only 工况。
CHECK_LOAD_CABLE=false CHECK_STANDALONE=true ROS_DOMAIN_ID=88 ROS_LOG_DIR=/tmp/insertion_skill_sim \
  /usr/bin/python3 artifacts/insertion_workflow_20260918/run_scene.py /tmp/insertion_skill_new
```

驱动仅关闭 Viewer/RViz、显式取消人工等待，并设置独立 ROS 域。通过原 launch、真实物理和原服务执行，不替换观测或调宽阈值；末尾检查完成标志、重复 start 和 reset，清理自己启动的进程。脚本延用 `usb_only_*` 文件名，实际工况以 command JSON 中的 `load_cable` 为准。
