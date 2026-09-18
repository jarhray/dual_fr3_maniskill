# 插入职责整理与验证（2026-09-17）

本记录仅说明这轮小范围重构。此前用户完成的插入实验及其配置保持原记录，
没有将历史成功改写为本轮结果。代码和实际 YAML 数值优先于旧说明；本轮没有改变运行参数。

## 变更与基线

工作区根目录不是可用的包 Git 仓库；分别检查了四个包，初始均无未提交修改，未找到适用的 AGENTS.md。

| 包 | 修改前 commit |
| --- | --- |
| dual_fr3_maniskill | `d41224402d3b01284eba356566068c16340f6965` |
| dual_fr3_trunking_mtc | `b3fe0cd8ec6775c4182444ce4fd93dba243e9d62` |
| dual_fr3_moveit_config | `281b02b77d8c9bc33bf80342042ce46d33311c74` |
| dual_fr3_cable_perception | `e11e91d6b2704278a53111aae34bf086b6dd4687` |

代码修改仅在 ManiSkill 与 MTC：提取公共孔壁计算、MoveIt 场景消息和 MTC 运动辅助函数，
服务分支拆成 handle_*，主流程保留原动作顺序及失败/清理语义。
策略更新顺序、PhysX 子步、接触算法、保持创建和资源清理实现不重写。
文档补齐了入口默认值、参数覆盖和调参位置，详见 [参数说明](insertion_parameters.md)。

原始输出放在工作区 `artifacts/insertion_refactor_20260917/`。
`before/manifest.json` 记录四包提交及配置/launch 文件 SHA256；`before/resolved.json`
保存四种 YAML × 三种 solver 解析结果；`before/geometry.npz` 在编辑前保存网格和变换。
`original_python/` 从上述提交复制 Python 源码，原生扩展链接至同一已构建 `_rope_physx.so`，
供本机基线和原版本场景对照使用，没有切换或重置工作包。

## 自动检查

| 检查 | 本次结果 / 日志 |
| --- | --- |
| 原版本相关子集 | **145 passed、1 failed**，`baseline-targeted-native.log` |
| 原版本完整离线回归 | **550 passed、1 failed**，`baseline-full-host.log` |
| 几何/职责提取后既有专项 | **28 passed**，`batch2.log` |
| 服务、几何和编排专项，含新增检查 | **55 passed**，`batch3.log` |
| 重构后完整离线回归 | **577 passed、1 failed**，`final-full-host.log`，新增 27 项全部通过 |
| 感知包 | 修改前后均 **33 passed**，专用 `.venv-perception`，源码未修改 |
| 几何等价性 | 默认及偏置孔中心/非对称间隙两组，160 个数组逐值相等（顶点、面、目标变换、关键点），尺寸报告相等 |
| 配置等价性 | 12 组解析配置相等，四包记录的配置与 launch 校验和不变；原版本与重构版仿真节点全部 28 项 ROS 有效参数（含模型）相同 |
| 规划辅助函数 | 用显式参数替换 self 后，motion/withdraw AST 相同，含数值、重试及执行顺序 |
| 服务拆分差分 | 22,848 组 state/flags/operation 的返回值、错误文本、状态快照、控制权、预留与 heartbeat 相同 |
| 原生 PhysX 组件探针 | 6 项全部通过，`physx.json`；含孔腔、孔底反力、摩擦、力矩、外部载荷不可辨识与 37 凸体 |
| 构建 | 两个修改包 colcon symlink build 成功，新运动/场景辅助模块可从 install 导入 |

唯一持续失败是 `test_rope_finger_contact.py::test_recorded_rim_overlap_generates_contact_with_correct_depth`
的 `rope_trunking_rim_20260915-convex_capsule`：CAD 复算深度约 **0.003760325567 m**，
历史样本为 **0.000159900599 m**。未删除测试、修改样本或放宽原 3 µm 阈值。

第一轮全量尝试在受限沙箱中遭遇 ROS 环境相关失败，并在
`test_path_length.py::test_real_mtc_cartesian_gate_blocks_detours` 原生调用处段错误，日志为 `baseline.log`。
本机独立 ROS 域重跑得到上表结果。中途直接将源目录放入 PYTHONPATH 的尝试未包含已编译扩展，
以及误用仿真 venv 运行感知遇到缺少 skimage，均作为测试环境诊断保留，未当作代码回归。
构建不代替测试：系统 Python 缺少 pytest，ament 的警告保留，pytest 实际由项目 venv 执行。

可复现测试命令（工作区根目录）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_LOG_DIR=/tmp/insertion_refactor_tests
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
.venv/bin/python -m pytest -q src/dual_fr3_maniskill/test \
  src/dual_fr3_moveit_config/test/test_maniskill_scene_launch.py \
  src/dual_fr3_trunking_mtc/test
.venv-perception/bin/python -m pytest -q src/dual_fr3_cable_perception/test
```

基线运行在原版本测试尚未添加新用例时完成，使用
`PYTHONPATH="$PWD/artifacts/insertion_refactor_20260917/original_python:$PYTHONPATH"`。
几何/参数快照可用该目录内 `capture.py before|after` 重现；服务差分为 `compare_services.py`。
保留报告文件后再跑应改用新目录，勿覆盖本次证据。

## 仿真复跑与边界

本机 RTX 3050、驱动 595.91.07 可用；沙箱内 nvidia-smi 不可用，本次物理场景改在本机运行。
原生组件探针无需渲染器，局部/完整机器人场景使用现有仿真环境。

局部真实机器人场景 `check_usb_insertion_simulation.py` **通过**：按既有孔前测试初态，
真实闭爪、支撑释放、实测对齐、连续插入、固定、开爪。`local.summary.json`
记录 `insertion_success=true`、`retention_active=true`，开爪后 USB 位移
**7.45058e-8 m**，抓取状态 `gripper_released`。此脚本不包含完整 MTC 回位。

既有真实障碍用例 `--obstacle-depth .006` 也**通过**（`obstacle.summary.json`）：
反馈推进最终以 `blocked / no_measured_tip_progress` 停止，`insertion_success=false`、
`retention_active=false`，没有将指令推进或孔内阻挡误判为成功。测试使用实际刚体障碍，未替换测量数据。

本轮完整 USB-only 复跑使用原推荐配置、`insertion_enabled=true`、`cable_solver=rope_actor`、
`load_cable=false`，仅关闭 Viewer/RViz，并使用独立 ROS 域 87。
原 MTC 运动与右臂释放/退出/回位已完成；孔前接近在 10 次独立求解中都被已有 TCP 路程限制拒绝，
以 `cancelled / client_cancel_or_shutdown` 收尾，控制权未残留。日志末次路径为约
**0.175976 m**，允许上限 **0.122792 m**（2 × 0.061396 m）。没有开始反馈插入或建立保持约束。
日志：`usb_only_launch.log`、`usb_only_summary.json`、`usb_only_states.jsonl`。

随后使用 `original_python/` 的原版本源码、相同命令和配置复跑，**完整通过**，结果在
`original_usb_only/`。孔前接近第一次路径 .146290 m 超限，第二次 .112388 m 满足 .122818 m 上限；
最终四项结果均 true，控制权已释放，重复 start 维持 complete 且不新增状态转换，
reset 后 MoveIt 世界及附着物中的 USB/插座均清除。这说明原场景可运行，但规划仍有随机性；
不把原版本的成功计作重构版成功。

第二轮重构版在原 MTC 运动完成后，右爪已经释放，但右臂笛卡尔退出 10 次规划均报告
`failed to move full distance`，因此状态为 `right_return_failed`，未执行插入。
记录在 `usb_only_repeat/`；该轮 ROS 参数 dump 与原版本成功轮的全部 28 项参数相等。
保留该可执行性问题，不增加退出动作或放宽检查。

第三轮重构版完成右臂释放回位，孔前接近在第 2 次求解得到 .110189 m 的合格路径
（上限 .122615 m）并执行成功。随后 `socket_align_actual_usb` 的 10 次规划均未满足
`CartesianPath.min_fraction=1.0`（日志含 .454545、.636364 等实际 fraction），以 cancelled 收尾。
记录在 `usb_only_final/`，全部 ROS 参数仍与原版本相同。

以上是本轮新增的集成观察，不等同于已知历史 CAD 断言失败。虽然几何、运动函数结构、参数及服务差分均等价，
**重构版三轮完整场景未通过，而原版本对照一轮通过**，因此不能宣称完整场景无回归；
随机规划初态/采样与微量对齐的可重复性仍需单独定位。本轮未调整动作、求解器或阈值来绕过失败。

本轮完整场景尚未验证插入后的左臂退出/回位，以及 complete 后重复 start 和 ROS reset；
这些路径的策略/服务/编排及原生清理测试通过，不能代替完整场景通过。
带线缆运输、插入与回位不在此次 USB-only 复跑中；既有实验结论保持原记录。
路径比例、接近策略和物理数值均未为获得成功而改变。

复跑入口（先停止其他仿真，报告用新路径）：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_insertion_physx.py --output /tmp/refactor_physx_new.json
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_insertion_simulation.py --output /tmp/refactor_local_new.jsonl
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/refactor_usb_ros \
  /usr/bin/python3 artifacts/insertion_refactor_20260917/run_usb_only.py /tmp/refactor_usb_new
```

最后一项是本轮报告目录中的驱动脚本，运行原 `mtc_prototype.launch.py`，保存命令、状态与日志，
若完成则自动检查重复 start、规划场景及 reset；无论结果如何都会清理自己创建的仿真进程。
它没有修改规划器、超时阈值或动作逻辑。交互启动命令见 [插入说明](usb_insertion.md#启动与观测)。
