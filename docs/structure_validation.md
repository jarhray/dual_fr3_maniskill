# 目录与参数文档整理验证（2026-09-18）

本记录只描述这轮目录归组和文档整理。之前的
[初轮重构](insertion_refactor_validation.md)、[工作流调整](insertion_workflow_validation.md)
及用户实测记录保留原始范围与配置，不作为本轮完整场景通过的证据。

## 修改与保持范围

- MTC 16 个根模块移入 `task/`、`execution/`、`insertion_task/`、`nodes/`；
  主入口的原参数解析函数移到 `runtime/arguments.py`。
- ManiSkill 16 个根模块移入 `robot/`、`ros/`、`usb/`、`sensing/`、`engine/`。
  `cable/` 求解、接触与 `scenes/` 生命周期只调整导入，不重排物理步骤。
- `_compat/` 保留旧导入，与新路径指向同一模块对象；`python -m dual_fr3_maniskill.warp_setup`
  也保留命令行入口。新增代码使用新路径，已有测试继续使用旧路径验证兼容。
- 四包新增参数作用索引和自动生成的默认值/来源附录。扫描覆盖 480 个包内参数名
  （跨包同名会重复计数），含 YAML、直接 CLI/launch/ROS 声明、指定缺省结构及 xacro arg。
  另手工列出可选字段、关键代码固定值和共享转发选项。扫描器不保证识别任意新动态声明方式。
- 以本轮开始时源码和配置为基准，保留 `cable.penetration_tolerance=.0008`、
  `insertion.max_path_length_ratio=3.0`。18 份配置逐字节相同，未改物理数值、阈值或历史样本。
  文档中的 cable_tip 开口修正为配置实际值 .0045 m。

工作区根目录不是 Git 仓库；逐包检查并保留已有修改。未发现适用 AGENTS.md。
本轮开始时的 257 份源码/配置/文档快照及 SHA256 位于工作区
`artifacts/structure_parameters_20260918/before/`，不参与 colcon 包发现。

## 检查结果

所有日志和 JSON 均在 `artifacts/structure_parameters_20260918/`。

| 检查 | 本次结果 | 证据 |
| --- | --- | --- |
| 修改前现有测试基线 | 604 passed、1 failed | `baseline.log` |
| 第一批目录移动/入口拆分 | 432 passed、8 failed；定位为入口遗漏 Path 导入 | `batch1.log` |
| 补回 Path 后对应执行测试 | 33 passed | `batch1_fix.log` |
| 全量现有测试及初版兼容测试 | 637 passed、1 failed | `final_tests.log` |
| 最后补充旧 Warp CLI 兼容后专项检查 | 34 passed（与全量有重叠，不相加） | `compat_final.log` |
| 感知包 | 33 passed | `perception.log` |
| 两包 colcon build | 成功；系统 Python 缺 pytest 的原有提示保留，实际测试使用项目 venv | `build_final.log` |
| 几何 | 160 组顶点、三角形、变换和测量数组逐元素完全相同 | `geometry_before/geometry.npz`、`geometry_after/geometry.npz` |
| 解析场景配置 | 4 YAML × 3 solver 选择，共 12 组 JSON 完全相同 | 两个 geometry 目录的 `resolved.json` |
| Python 逻辑 | 68 个已有模块去掉 import 后 AST 一致；CLI 函数提取保持原 AST，源树相对路径多一层作对应修正 | `equivalence.json` |
| launch 参数 | MTC 71 项、独立 skill 2 项、sim 19 项，共 92 项前后相同 | `launch_defaults_equal.json` |
| 参数文档覆盖/附录同步 | 四包共 480 项，0 项漏写 | `parameters_check.log` |
| 原生 PhysX 探针 | 6 项通过，包括 37 凸体、孔底反力、摩擦反力、孔中心力矩与载荷来源检查 | `physx.json` |
| 实际机器人局部插入 | 插入成功、保持约束成立、开爪成功；USB 位移约 7.45e-8 m | `local_insertion.summary.json` |

单元测试唯一保留失败为
`test_rope_finger_contact.py::test_recorded_rim_overlap_generates_contact_with_correct_depth[rope_trunking_rim_20260915-convex_capsule]`。
当前 CAD 深度 .003760325567 m，历史样本 .000159900599 m，超过原 3 µm 对照阈值。
本轮基线和全量复跑均失败，未修改测试、样本或容差。

## 完整场景尝试与未验证范围

实际 ROS/GPU 环境可运行，进行了两次全程启动：

1. `standalone_usb/`：`load_cable=false`、`run_insertion=false`，准备运输完成后再启动独立插入入口。
   全程候选规划未成功，状态仅到 `not_started`，独立入口没有获得调用机会。
2. `full_cable/`：推荐的 `load_cable=true`、Rope-Actor、自动终末插入。
   同样在准备候选/连续插入接近预检中耗尽搜索预算，未进入执行。

两者均保留原规划参数；日志包含换锚点路径比例/高度约束、Cartesian 路径及连续接近规划拒绝。
没有为得到通过结果增加预算、放宽约束或修改初态。AST 与配置等价检查未发现本轮动作逻辑或参数变化，
但本轮没有重现历史完整成功，不能把单元测试、局部物理探针描述成完整场景通过。
**本轮实际全流程的双臂回位、独立 ROS skill 成功闭环及完成后的重复 start/reset 未验证到；**
这些仍有现有编排/服务单元测试覆盖，历史成功记录独立保留。

局部插入探针从原测试定义的孔前初态开始，只验证接触、反馈推进、成功保持、固定和开爪；
没有完成全程运输和回位。两次 ROS driver 均清理自己启动的进程。

## 可复现命令

先在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select dual_fr3_maniskill dual_fr3_trunking_mtc
source install/setup.bash
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/structure-tests \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test
.venv-perception/bin/python -m pytest -q src/dual_fr3_cable_perception/test
python3 src/dual_fr3_maniskill/scripts/check_parameter_docs.py
.venv/bin/python src/dual_fr3_maniskill/test/check_insertion_physx.py --output /tmp/physx_structure_new.json
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_insertion_simulation.py --output /tmp/local_structure_new.jsonl
```

新增文件后必须重建包，否则旧 install 不能发现新目录。局部探针输出要求新路径。
最终全量日志在最后增加 Warp CLI 测试之前产生；重跑上述命令会额外收集这个已单独通过的测试。

推荐交互启动保持：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=true execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"
```

本轮自动场景 driver 追加 `preparation_interactive=false`、`use_rviz=false`、`maniskill_viewer=false`、
`mtc_keep_alive_sec=.1`；这是自动验证设置，不改推荐命令或默认值。完整命令保存在两个场景目录的 summary JSON。

```bash
CHECK_LOAD_CABLE=true ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/structure-cable /usr/bin/python3 \
  artifacts/insertion_workflow_20260918/run_scene.py /tmp/structure_cable_new
CHECK_LOAD_CABLE=false CHECK_STANDALONE=true ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/structure-usb /usr/bin/python3 \
  artifacts/insertion_workflow_20260918/run_scene.py /tmp/structure_usb_new
```

后续处理重点是完整 MoveIt 候选规划的稳定性，以及已知 CAD 历史样本差异；它们没有通过本轮结构整理被隐式修复。
