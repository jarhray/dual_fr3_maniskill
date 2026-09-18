# ManiSkill 模块职责与修改路径

启动命令见 [README](README.md)。全部配置项见 [参数索引](docs/parameters.md)，
各预设和代码缺省值见 [默认值来源](docs/parameter_defaults.md)。代码和所选配置的数值为准。

```text
dual_fr3_maniskill/
├── robot/       URDF 资产、机器人场景、轨迹采样、碰撞工具
├── ros/         通用 ROS action 桥接及 launch 装配
├── usb/         USB 夹持、插入几何、策略、物理适配、ROS 服务
├── sensing/     力观测、力输出、RGB-D 相机
├── engine/      SAPIEN 版本适配、Warp 构建
├── scenes/      robot / usb_cable / trunking_cable 场景装配及生命周期
├── cable/       线缆配置、后端、接触、子步、诊断与线缆 ROS 服务
└── _compat/     旧根模块导入兼容别名，不放业务逻辑
```

目录按责任归组。`cable/` 的核心求解器保持原文件边界、子步顺序和资源生命周期，
`scenes/` 负责把机器人、线缆、夹持及插座组合起来；本轮不把物理大循环拆成新的调度框架。

| 要调整的行为 | 代码位置 | 验证入口 |
| --- | --- | --- |
| 机器人资产/场景步进 | `robot/assets.py`、`robot/simulation.py` | assets、trajectory 测试及 ROS 执行场景 |
| 轨迹、取消、忙状态、夹爪 action | `ros/bridge.py` | 轨迹、夹持服务测试及真实 action |
| 选择场景和参数覆盖 | `ros/launch.py`、`scenes/__init__.py` | MoveIt 包的 `test_maniskill_scene_launch.py` |
| 初始 USB 与线缆放置 | `cable/model.py`、`cable/threading.py` | 初始抓姿/穿孔几何测试 |
| 闭爪接触、释放后的稳定/滑移/掉落 | `usb/grasp.py`、`scenes/usb_cable.py` | `test_usb_grasp.py`、`check_usb_grasp_simulation.py` |
| 孔大小、USB 关键点、坐标变换 | `usb/geometry.py` | 几何报告、MoveIt/PhysX 顶点对照、原生凸体探针 |
| 推进、阻力滤波、停滞、成功保持 | `usb/insertion.py` | `test_insertion.py`，再局部插入/障碍测试 |
| 服务状态、控制权交接、heartbeat | `usb/bridge.py` | `test_insertion_services.py`、取消/重复/reset |
| 插座实体、支撑反力和固定约束 | `usb/scene.py` | `check_insertion_physx.py`、固定后开爪与 reset |
| 接触力的参考坐标和记录 | `sensing/forces.py`、`sensing/force_output.py` | `test_forces.py` 与实测 wrench |
| 相机内参、视角及话题 | `sensing/camera.py` | 相机测试和 RGB-D/TF 对照 |

插入依赖方向为 `usb/geometry.py ← usb/insertion.py ← usb/scene.py / usb/bridge.py`。
几何与状态策略不依赖 ROS、SAPIEN、ManiSkill 或 CUDA 初始化；共享几何由 MTC 直接使用。
ROS 桥接在串行 tick 边界处理实体修改，场景拥有实体与约束，策略只记录观测和状态。
仿真时间负责滤波、推进、成功保持；墙钟负责 heartbeat 租约。status 不续租。
必须先确认 `inserted_unretained`，再建立固定约束，MTC 确认固定后才开左爪。

兼容目录被包的 `__path__` 作为最后一项搜索。旧 `dual_fr3_maniskill.insertion_geometry`
与新 `dual_fr3_maniskill.usb.geometry` 返回同一模块对象，保留 monkeypatch、共享状态和既有导入。
普通包导入不急切载入引擎。新增实现只使用新路径，不向兼容目录增加业务代码。
新增/移动文件后须重新 `colcon build --symlink-install`，不能只依赖旧 install 的符号链接。

历史记录保留原始模块名和当时配置，旧名字可通过 `_compat/` 找到当前实现。
本轮检查及未验证范围见 [结构整理验证](docs/structure_validation.md)。
