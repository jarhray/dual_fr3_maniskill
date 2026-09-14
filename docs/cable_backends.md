# 线缆后端交接说明

## 当前交接点

截至 2026-09-14，只有 MPM 线缆实现。用户决定暂停 MPM 优化，先检查并自行备份，再由后续 agent 加入细杆模型和参数选择。
本次整理保留当前物理算法和默认值，没有加入细杆、空占位后端或尚未生效的启动参数。

MTC 基线：1.5 m 长、2 mm 直径、1 mm 采样，每截面 7 个体积材料点，共 10507 粒子。
控制/刚体/MPM 频率 50/500/5000 Hz，轴向 6 次、接触 2 次迭代；Graph 与 GPU 网格检查启用。
4 次迭代已因约 16.3 mm 运动末态差被排除。独立 USB 场景默认直径是 3.5 mm，不能混用两个场景的几何默认值。

性能和失败项见 [MPM 性能与验证](mpm_performance.md)、[归档基线](benchmarks/mpm_20260912/README.md)。
材料未标定；严格全程 0.16 mm 对照曾未通过；这轮没有验收完整 MTC。MPM 不能作为真实线缆误差的绝对真值。

## 代码分工

下列路径以本仓库的 `dual_fr3_maniskill/` Python 包为起点。

| 模块 | 职责 |
| --- | --- |
| `scenes/__init__.py`、`launch_support.py` | 场景、默认配置、桥接节点；导入时不需要 GPU |
| `scenes/usb_cable.py` | 启动即生成线缆；刚体/线缆步进和控制步末检查 |
| `scenes/trunking_cable.py` | 准备后生成 USB、安装 drive、穿孔初态和线缆；失败时清理临时对象 |
| `cable/model.py`、`threading.py` | YAML 校验、USB 模型、7 点截面采样、初始布局、穿孔几何、MPM 网格预算 |
| `cable/mpm_cable.py` | MPM 生命周期、夹持、双向耦合、重置、渲染与诊断 |
| `cable/solver.py` | 带 padding 的移动网格，调用配套 ManiSkill2 MPM 内核 |
| `cable/fibers.py` | 七条纵向 XPBD 纤维，每次迭代后执行导向/碰撞 |
| `cable/guide.py`、`guide_cuda.py` | 理想右 TCP 滑孔、速度修正和反作用冲量 |
| `cable/contacts.py`、`mesh_contacts.py` | 有限半径扫掠、摩擦、APIC 修正、保守缓存、独立穿透审计、原始指尖网格 |
| `cable/execution.py` | GPU 边界摘要、Graph 缓存与释放，不定义材料模型 |
| `cable/ros_bridge.py`、`trunking_bridge.py` | 动作边界、spawn/reset、标记、诊断和 USB TF |
| `cable/planning_scene.py` | MoveIt 的 USB 附着对象；柔性线缆本身不是 MoveIt 刚性碰撞物 |
| `sapien_compat.py`、`warp_setup.py` | Vulkan 环境保留、配套 Warp 安装/编译兼容 |

实现入口：[MPMCable](../dual_fr3_maniskill/cable/mpm_cable.py)、[USB 场景](../dual_fr3_maniskill/scenes/usb_cable.py)、
[MTC 场景](../dual_fr3_maniskill/scenes/trunking_cable.py)。测试参考源码在 `test/reference/`，不属于运行时依赖。

## 生成与耦合时序

MTC：完整任务预检 → 双臂准备 → 两侧夹爪闭合成功 → `/maniskill/cable/spawn` → 双臂下降 → 走线。
生成要求动作已结束、实际夹爪位置符合准备宽度；重复调用不重复创建。
左端通过 USB 与左 TCP 固定连接；右 TCP 是允许轴向滑动的孔约束，不能固定到某个材料粒子。

每个刚体步的顺序：

```text
agent.before_simulation_step()
cable.step(rigid_dt)       # 更新刚体姿态，推进线缆，提交反作用力
scene.step()              # PhysX 消耗该步力，推进机器人和 USB
cable.follow_plug()       # 使用新刚体姿态修正约束，力供下一步消耗
```

每个控制步结束检查穿透、连续性和机器人状态，随后按发布/显示频率输出。
重置保留机械臂位姿，先验证新布局再更新线缆；拒绝的重置不能覆盖旧粒子。
异常会暂停仿真并结束相关动作，不能让 MTC 继续下降或走线。

## 参数选择方案（待实现）

建议用单一启动/ROS 参数 **`cable_solver`**，候选值 `mpm`、`rod`，缺省为 `mpm`。
它与现有参数的层次如下；前两项已实现，第三项尚未实现：

| 参数 | 选择内容 |
| --- | --- |
| `simulation_backend` | `maniskill`、`gazebo`、`fake`、`real` 环境 |
| `maniskill_scene` | `robot`、`usb_cable`、`trunking_cable` 场景 |
| 拟新增 `cable_solver` | ManiSkill 线缆场景中的 `mpm` 或 `rod` 求解器 |

当前不能传入 `cable_solver:=rod` 使用细杆。后续实现应拒绝未知值，不能悄悄退回 MPM。
省略参数时保持本次基线，避免 YAML 再出现优先级不明的同义开关。

参数需贯通：MTC `mtc_prototype.launch.py` → MoveIt `demo.launch.py` / `maniskill.launch.py` →
本包 `launch/sim.launch.py`、`launch_support.create_bridge_node()` → 两个线缆 bridge → Simulation → 统一创建工厂。
独立验证脚本、显式 `cable_config`、按场景选取的默认配置和诊断都应确认实际求解器。
MTC 的 `simulation_cable.py`、`preparation.py` 和本包 `planning_scene.py` 还读取配置确定夹爪宽度和 USB 几何，
这些几何调用不能因选择 rod 而强制导入 MPM 或初始化其 CUDA 环境。

## 共同接口与需要拆分的假设

可由工厂返回共同接口；下表列出当前场景/桥接需求，不是已经存在的抽象基类：

| 接口 | 用途 |
| --- | --- |
| `step(rigid_dt)`、`follow_plug()` | 保持双向刚体耦合时序 |
| `centerline` | 世界坐标系下、按材料顺序的 `N×3` 数组 |
| `plug` | USB 刚体，供安装、TF 和力反馈使用 |
| `reset()`、`check_contacts()` | 合法状态重置、有限线径穿透检查 |
| `diagnostics()`、`update_render()` | 可比较的物理量和实际状态显示 |
| 拟统一 `close()` | 释放资源；当前场景直接清理 MPM 的 `constraint_graphs`，需一起抽离 |

接入时重点处理：

- `model.load_config()` 要求 `mpm` 段并限制其粒子采样间距，应拆分公共几何与后端校验，同时保持旧 MPM YAML 有效。
- 两个 Simulation 构造函数直接检查 `mpm.frequency`；MTC 环境提前初始化 Warp，USB 环境的导入和 shader 也绑定 MPM。应按后端延迟导入、初始化和校验。
- `SlidingGuide`、`CableContacts`、`threaded_positions()` 多处假定每截面 7 粒子。可复用几何含义，不能要求 rod 伪造 `states[0].struct`。
- 当前 `guide_material_coordinate` 是可带小数的截面索引，不是米。新模型改用弧长时应标明单位，不能静默改变字段语义。
- `marker_stride` 位于 `mpm` 段；诊断名、错误文本、Marker namespace 也含 MPM。后续应提供公共显示设置和实际后端标识，兼容现有订阅话题。
- 连续性检查以粒子间隔两倍为上限，rod 应按其材料段长检查；不同采样密度不能直接套用 MPM 阈值。
- GPU 粒子数组、Graph、`particle_F`/`particle_C` 是 MPM 内部细节。保留 MPM 专项测试，另建 rod 验证，不向共享接口暴露这些假设。

## 后续对照要求

保留现有 ROS 动作、spawn/reset、TF、夹爪完成检查、USB 附着和研究手指孔洞。
有限线径接触须覆盖节点之间的线段及步间运动，不能只检查节点或以视觉平滑掩盖穿墙。
当前 MPM 没有显式线段自碰撞求解，紧密缠绕/打结不属于已验证基线。

先在同一准备姿态、场景与双臂运动下比较步进耗时、接触阶段耗时、固定端/滑孔误差、长度/拉伸、弯曲响应、穿透和反作用力，
再验证完整 MTC。0.16 mm 的独立运行位置差不能直接变成新模型所有物理量的统一容差，需按任务评价新旧模型差异。

本次不再追加解析线槽碰撞、距离场、MPM 频率调整或其他性能试验。
后续 agent 可从这个归档基线继续，无需依赖本次聊天或仓库外的旧参考源码。
